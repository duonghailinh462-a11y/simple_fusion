"""
雷达数据源抽象层 - 统一的数据源接口

设计目标：
1. 定义统一的雷达数据源接口（BaseRadarSource）
2. 支持多种数据源实现（文件、流、数据库、实时 API 等）
3. 提供装饰器模式实现的功能扩展（过滤、转换、缓存）
4. 使用工厂模式创建数据源实例
5. 完全解耦业务逻辑与数据来源

架构：
    
    ┌─────────────────────┐
    │ RadarSourceFactory  │  (工厂模式)
    └──────────┬──────────┘
               │
       ┌───────┴────────┬────────────────┐
       │                │                │
    ┌──▼───────┐  ┌─────▼────┐  ┌──────▼──────┐
    │ JSONL文件│  │流式读取   │  │多摄像头映射 │
    │  数据源  │  │  数据源   │  │   数据源    │
    └──────────┘  └───────────┘  └─────────────┘
       │                │                │
       └───────┬────────┴────────────────┘
               │
       ┌───────▼────────────────┐
       │ BaseRadarSource        │
       │ (抽象基类)             │
       └────────────────────────┘
           △
           │ 实现
           │
       ┌───┴──────────────────┐
       │  装饰器模式           │
       │ (功能扩展)            │
       └───────────────────────┘
"""

import json
import math
import time
import logging
from abc import ABC, abstractmethod
from datetime import datetime
from typing import List, Dict, Tuple, Optional, Generator, Any
from collections import deque, defaultdict


# 获取日志记录器
logger = logging.getLogger('RadarSourceAbstraction')


# ==========================================
# 常量定义
# ==========================================
VALID_RADAR_TYPES = {1}

RADAR_IP_TO_CAMERA = {
    '44.30.142.88': 1,  # C1
    '44.30.142.85': 2,  # C2
    '44.30.142.87': 3,  # C3
}


# ==========================================
# 工具函数
# ==========================================
def parse_time(ts_str):
    """解析时间戳字符串"""
    if not ts_str:
        return 0.0
    
    if isinstance(ts_str, (int, float)):
        return float(ts_str)
    
    clean_ts = ' '.join(str(ts_str).split())
    
    formats_to_try = [
        ('%Y-%m-%d %H:%M:%S.%f', False),
        ('%Y-%m-%d %H:%M:%S', False),
    ]
    
    for fmt, needs_padding in formats_to_try:
        try:
            if needs_padding and '.' in clean_ts:
                parts = clean_ts.split('.')
                if len(parts) == 2 and len(parts[1]) < 6:
                    clean_ts = f"{parts[0]}.{parts[1].ljust(6, '0')}"
            
            dt = datetime.strptime(clean_ts, fmt)
            return dt.timestamp()
        except ValueError:
            if not needs_padding and '.' in clean_ts and '%f' in fmt:
                parts = clean_ts.split('.')
                if len(parts) == 2 and len(parts[1]) < 6:
                    try:
                        clean_ts_padded = f"{parts[0]}.{parts[1].ljust(6, '0')}"
                        dt = datetime.strptime(clean_ts_padded, fmt)
                        return dt.timestamp()
                    except ValueError:
                        continue
            continue
    
    return 0.0


# ==========================================
# 数据结构
# ==========================================
class RadarObject:
    """雷达目标对象"""
    def __init__(self, radar_id, latitude, longitude, speed=0.0, azimuth=0.0, 
                 lane=None, timestamp_str=None, source_ip=None):
        self.id = radar_id
        self.latitude = latitude
        self.longitude = longitude
        self.speed = float(speed or 0)
        self.azimuth = float(azimuth or 0)
        self.lane = lane
        self.timestamp_str = timestamp_str
        self.source_ip = source_ip
    
    def to_dict(self):
        """转换为字典"""
        return {
            'id': self.id,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'speed': self.speed,
            'azimuth': self.azimuth,
            'lane': self.lane,
            'timestamp_str': self.timestamp_str,
            'source_ip': self.source_ip,
        }


class RadarFrame:
    """雷达数据帧 - 一个时间戳对应的所有雷达目标"""
    def __init__(self, timestamp: str, objects: List[RadarObject], metadata: Dict = None):
        self.timestamp = timestamp
        self.objects = objects
        self.metadata = metadata or {}
        self.timestamp_numeric = parse_time(timestamp)
    
    @property
    def camera_id(self) -> Optional[int]:
        """从第一个对象推断摄像头ID"""
        if self.objects and hasattr(self.objects[0], 'source_ip'):
            return RADAR_IP_TO_CAMERA.get(self.objects[0].source_ip)
        return self.metadata.get('camera_id')
    
    def filter_by_camera(self, camera_id: int) -> 'RadarFrame':
        """按摄像头ID过滤"""
        filtered = [
            obj for obj in self.objects
            if RADAR_IP_TO_CAMERA.get(obj.source_ip) == camera_id
        ]
        return RadarFrame(self.timestamp, filtered, self.metadata)
    
    def filter_by_lane(self, lane: str) -> List[RadarObject]:
        """按车道过滤"""
        return [obj for obj in self.objects if obj.lane == lane]
    
    def get_object_count(self) -> int:
        """获取目标数量"""
        return len(self.objects)


# ==========================================
# 抽象基类
# ==========================================
class BaseRadarSource(ABC):
    """
    雷达数据源抽象基类
    
    定义所有数据源必须实现的接口
    """
    
    @abstractmethod
    def initialize(self) -> bool:
        """初始化数据源"""
        pass
    
    @abstractmethod
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """获取指定时间戳的数据帧"""
        pass
    
    @abstractmethod
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成数据帧"""
        pass
    
    @abstractmethod
    def get_all_timestamps(self) -> List[str]:
        """获取所有时间戳"""
        pass
    
    @abstractmethod
    def close(self) -> None:
        """关闭数据源"""
        pass
    
    @abstractmethod
    def get_stats(self) -> Dict:
        """获取统计信息"""
        pass


# ==========================================
# 具体实现：JSONL 文件数据源
# ==========================================
class JSONLRadarSource(BaseRadarSource):
    """
    从 JSONL 文件读取雷达数据
    
    特点：
    - 将整个文件加载到内存
    - 支持快速随机访问
    - 适合中等规模数据集
    """
    
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.frames = {}  # timestamp -> RadarFrame
        self.timestamps = []
        self.is_initialized = False
        self.stats = {
            'frames_loaded': 0,
            'objects_loaded': 0,
            'load_time_ms': 0,
        }
    
    def initialize(self) -> bool:
        """加载 JSONL 文件"""
        start_time = time.time()
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    try:
                        obj = json.loads(line)
                        timestamp = obj.get('time', '')
                        if not timestamp:
                            continue
                        
                        objects = self._parse_radar_objects(obj)
                        if objects:
                            frame = RadarFrame(timestamp, objects)
                            self.frames[timestamp] = frame
                            self.timestamps.append(timestamp)
                            self.stats['objects_loaded'] += len(objects)
                    
                    except (json.JSONDecodeError, KeyError):
                        continue
            
            self.timestamps.sort(key=parse_time)
            self.stats['frames_loaded'] = len(self.frames)
            self.stats['load_time_ms'] = (time.time() - start_time) * 1000
            self.is_initialized = True
            
            logger.info(f"✅ JSONLRadarSource 初始化完成: {self.stats['frames_loaded']} 帧, "
                       f"{self.stats['objects_loaded']} 个目标, {self.stats['load_time_ms']:.2f}ms")
            return True
        
        except Exception as e:
            logger.error(f"❌ JSONLRadarSource 初始化失败: {e}")
            return False
    
    def _parse_radar_objects(self, json_obj: Dict) -> List[RadarObject]:
        """从 JSON 对象解析雷达目标"""
        objects = []
        timestamp = json_obj.get('time', '')
        source_ip = json_obj.get('source_ip', '')
        
        for item in json_obj.get('locusList', []):
            if item.get('objType') not in VALID_RADAR_TYPES:
                continue
            
            radar_lane = item.get('lane')
            lane_str = f'lane_{radar_lane}' if radar_lane is not None else None
            
            azimuth = self._safe_float(item.get('azimuth', 0.0))
            
            obj = RadarObject(
                radar_id=item.get('id', ''),
                latitude=self._safe_float(item.get('latitude', 0.0)),
                longitude=self._safe_float(item.get('longitude', 0.0)),
                speed=self._safe_float(item.get('speed', 0.0)),
                azimuth=azimuth,
                lane=lane_str,
                timestamp_str=timestamp,
                source_ip=source_ip
            )
            objects.append(obj)
        
        return objects
    
    @staticmethod
    def _safe_float(val):
        """安全地转换浮点数"""
        try:
            f = float(val or 0.0)
            if math.isnan(f) or math.isinf(f):
                return 0.0
            return f
        except (ValueError, TypeError):
            return 0.0
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """获取指定时间戳的数据帧"""
        return self.frames.get(timestamp)
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成数据帧"""
        for timestamp in self.timestamps:
            yield self.frames[timestamp]
    
    def get_all_timestamps(self) -> List[str]:
        """获取所有时间戳"""
        return self.timestamps.copy()
    
    def close(self) -> None:
        """关闭数据源"""
        self.frames.clear()
        self.timestamps.clear()
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return self.stats.copy()


# ==========================================
# 具体实现：流式数据源
# ==========================================
class StreamingRadarSource(BaseRadarSource):
    """
    流式雷达数据源 - 逐帧读取，不一次性加载全部
    
    特点：
    - 低内存占用
    - 支持处理大型数据集
    - 不支持随机访问
    """
    
    def __init__(self, file_path: str, buffer_size: int = 100):
        self.file_path = file_path
        self.buffer_size = buffer_size
        self.file_handle = None
        self.buffer = deque(maxlen=buffer_size)
        self.is_initialized = False
        self.stats = {
            'frames_streamed': 0,
            'objects_streamed': 0,
        }
    
    def initialize(self) -> bool:
        """打开文件（不完全加载）"""
        try:
            self.file_handle = open(self.file_path, 'r', encoding='utf-8')
            self.is_initialized = True
            logger.info(f"✅ StreamingRadarSource 初始化完成: {self.file_path}")
            return True
        except Exception as e:
            logger.error(f"❌ StreamingRadarSource 初始化失败: {e}")
            return False
    
    def _parse_radar_objects(self, json_obj: Dict) -> List[RadarObject]:
        """从 JSON 对象解析雷达目标"""
        objects = []
        timestamp = json_obj.get('time', '')
        source_ip = json_obj.get('source_ip', '')
        
        for item in json_obj.get('locusList', []):
            if item.get('objType') not in VALID_RADAR_TYPES:
                continue
            
            radar_lane = item.get('lane')
            lane_str = f'lane_{radar_lane}' if radar_lane is not None else None
            
            try:
                azimuth = float(item.get('azimuth', 0.0) or 0.0)
                if math.isnan(azimuth) or math.isinf(azimuth):
                    azimuth = 0.0
            except (ValueError, TypeError):
                azimuth = 0.0
            
            obj = RadarObject(
                radar_id=item.get('id', ''),
                latitude=float(item.get('latitude', 0)),
                longitude=float(item.get('longitude', 0)),
                speed=float(item.get('speed', 0)),
                azimuth=azimuth,
                lane=lane_str,
                timestamp_str=timestamp,
                source_ip=source_ip
            )
            objects.append(obj)
        
        return objects
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """不支持随机访问"""
        # 仅返回缓冲区中的数据
        for frame in self.buffer:
            if frame.timestamp == timestamp:
                return frame
        return None
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成数据帧"""
        if not self.is_initialized or self.file_handle is None:
            return
        
        # 重置文件指针
        self.file_handle.seek(0)
        
        for line in self.file_handle:
            try:
                obj = json.loads(line)
                timestamp = obj.get('time', '')
                if not timestamp:
                    continue
                
                objects = self._parse_radar_objects(obj)
                if objects:
                    frame = RadarFrame(timestamp, objects)
                    self.buffer.append(frame)
                    self.stats['frames_streamed'] += 1
                    self.stats['objects_streamed'] += len(objects)
                    yield frame
            
            except (json.JSONDecodeError, KeyError):
                continue
    
    def get_all_timestamps(self) -> List[str]:
        """无法获取所有时间戳（流式数据源）"""
        return [frame.timestamp for frame in self.buffer]
    
    def close(self) -> None:
        """关闭数据源"""
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
        self.buffer.clear()
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return self.stats.copy()


# ==========================================
# 具体实现：多摄像头映射数据源
# ==========================================
class MultiCameraRadarSource(BaseRadarSource):
    """
    多摄像头映射数据源 - 将单一数据源按摄像头分类
    
    特点：
    - 自动按摄像头分离数据
    - 支持按摄像头获取数据
    - 简化多摄像头融合逻辑
    """
    
    def __init__(self, base_source: BaseRadarSource):
        self.base_source = base_source
        self.camera_frames = defaultdict(list)  # camera_id -> [frames]
        self.camera_timestamps = defaultdict(set)
        self.is_initialized = False
        self.stats = {
            'cameras_detected': 0,
            'frames_per_camera': {},
            'objects_per_camera': {},
        }
    
    def initialize(self) -> bool:
        """初始化基础数据源并按摄像头分类"""
        if not self.base_source.initialize():
            return False
        
        # 遍历所有帧并按摄像头分类
        for frame in self.base_source.stream_frames():
            if frame.camera_id:
                camera_id = frame.camera_id
                self.camera_frames[camera_id].append(frame)
                self.camera_timestamps[camera_id].add(frame.timestamp)
                
                # 统计
                if camera_id not in self.stats['frames_per_camera']:
                    self.stats['frames_per_camera'][camera_id] = 0
                    self.stats['objects_per_camera'][camera_id] = 0
                
                self.stats['frames_per_camera'][camera_id] += 1
                self.stats['objects_per_camera'][camera_id] += frame.get_object_count()
        
        self.stats['cameras_detected'] = len(self.camera_frames)
        self.is_initialized = True
        
        logger.info(f"✅ MultiCameraRadarSource 初始化完成: {self.stats['cameras_detected']} 个摄像头")
        for camera_id, count in self.stats['frames_per_camera'].items():
            logger.info(f"   C{camera_id}: {count} 帧, {self.stats['objects_per_camera'][camera_id]} 个目标")
        
        return True
    
    def get_frame_by_camera(self, camera_id: int, timestamp: str) -> Optional[RadarFrame]:
        """获取指定摄像头和时间戳的数据帧"""
        for frame in self.camera_frames.get(camera_id, []):
            if frame.timestamp == timestamp:
                return frame
        return None
    
    def stream_frames_by_camera(self, camera_id: int) -> Generator[RadarFrame, None, None]:
        """按摄像头流式生成数据帧"""
        for frame in self.camera_frames.get(camera_id, []):
            yield frame
    
    def get_timestamps_by_camera(self, camera_id: int) -> List[str]:
        """获取指定摄像头的所有时间戳"""
        return sorted(self.camera_timestamps.get(camera_id, set()), key=parse_time)
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """获取指定时间戳的所有摄像头数据"""
        for frames in self.camera_frames.values():
            for frame in frames:
                if frame.timestamp == timestamp:
                    return frame
        return None
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成所有数据帧"""
        for frames in self.camera_frames.values():
            for frame in frames:
                yield frame
    
    def get_all_timestamps(self) -> List[str]:
        """获取所有时间戳"""
        all_timestamps = set()
        for timestamps in self.camera_timestamps.values():
            all_timestamps.update(timestamps)
        return sorted(all_timestamps, key=parse_time)
    
    def close(self) -> None:
        """关闭数据源"""
        self.base_source.close()
        self.camera_frames.clear()
        self.camera_timestamps.clear()
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return self.stats.copy()


# ==========================================
# 装饰器：缓存装饰器
# ==========================================
class CachedRadarSource(BaseRadarSource):
    """
    缓存装饰器 - 缓存频繁访问的帧
    
    使用方式：
    ```python
    base_source = JSONLRadarSource('radar.jsonl')
    cached_source = CachedRadarSource(base_source, cache_size=500)
    ```
    """
    
    def __init__(self, source: BaseRadarSource, cache_size: int = 500):
        self.source = source
        self.cache = {}
        self.cache_size = cache_size
        self.cache_hits = 0
        self.cache_misses = 0
    
    def initialize(self) -> bool:
        return self.source.initialize()
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """获取帧（优先使用缓存）"""
        if timestamp in self.cache:
            self.cache_hits += 1
            return self.cache[timestamp]
        
        frame = self.source.get_frame(timestamp)
        if frame:
            self.cache_misses += 1
            if len(self.cache) >= self.cache_size:
                # 清理最旧的缓存
                self.cache.pop(next(iter(self.cache)))
            self.cache[timestamp] = frame
        
        return frame
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成帧（不使用缓存）"""
        for frame in self.source.stream_frames():
            yield frame
    
    def get_all_timestamps(self) -> List[str]:
        return self.source.get_all_timestamps()
    
    def close(self) -> None:
        self.source.close()
        self.cache.clear()
    
    def get_stats(self) -> Dict:
        stats = self.source.get_stats()
        stats['cache_hits'] = self.cache_hits
        stats['cache_misses'] = self.cache_misses
        stats['cache_size'] = len(self.cache)
        if self.cache_hits + self.cache_misses > 0:
            stats['cache_hit_rate'] = self.cache_hits / (self.cache_hits + self.cache_misses)
        return stats


# ==========================================
# 工厂模式
# ==========================================
class RadarSourceFactory:
    """
    雷达数据源工厂 - 创建合适的数据源实例
    """
    
    @staticmethod
    def create_jsonl_source(file_path: str, cached: bool = True) -> BaseRadarSource:
        """创建 JSONL 文件数据源"""
        source = JSONLRadarSource(file_path)
        if cached:
            source = CachedRadarSource(source)
        return source
    
    @staticmethod
    def create_streaming_source(file_path: str, buffer_size: int = 100) -> BaseRadarSource:
        """创建流式数据源"""
        return StreamingRadarSource(file_path, buffer_size)
    
    @staticmethod
    def create_multi_camera_source(file_path: str, use_streaming: bool = False) -> MultiCameraRadarSource:
        """创建多摄像头数据源"""
        if use_streaming:
            base_source = RadarSourceFactory.create_streaming_source(file_path)
        else:
            base_source = RadarSourceFactory.create_jsonl_source(file_path)
        
        return MultiCameraRadarSource(base_source)
    
    @staticmethod
    def create_auto(file_path: str) -> BaseRadarSource:
        """自动选择合适的数据源"""
        import os
        file_size = os.path.getsize(file_path)
        
        # 根据文件大小选择数据源
        if file_size > 100 * 1024 * 1024:  # > 100MB，使用流式
            logger.info(f"📊 文件大小 {file_size / 1024 / 1024:.1f}MB，使用 StreamingRadarSource")
            return RadarSourceFactory.create_streaming_source(file_path)
        else:
            logger.info(f"📊 文件大小 {file_size / 1024 / 1024:.1f}MB，使用 JSONLRadarSource")
            return RadarSourceFactory.create_jsonl_source(file_path)


# ==========================================
# 示例和测试
# ==========================================
if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.INFO)
    
    # 示例 1：基本用法
    print("示例 1：基本用法")
    print("-" * 60)
    
    # 使用工厂创建数据源
    source = RadarSourceFactory.create_jsonl_source('radar_data.jsonl')
    if source.initialize():
        # 获取所有时间戳
        timestamps = source.get_all_timestamps()
        print(f"总共 {len(timestamps)} 个时间戳")
        
        # 获取第一帧
        if timestamps:
            first_frame = source.get_frame(timestamps[0])
            print(f"第一帧: {first_frame.timestamp}, 目标数: {first_frame.get_object_count()}")
        
        # 获取统计信息
        stats = source.get_stats()
        print(f"统计信息: {stats}")
        
        source.close()
    
    # 示例 2：多摄像头数据源
    print("\n示例 2：多摄像头数据源")
    print("-" * 60)
    
    multi_source = RadarSourceFactory.create_multi_camera_source('radar_data.jsonl')
    if multi_source.initialize():
        stats = multi_source.get_stats()
        print(f"检测到 {stats['cameras_detected']} 个摄像头")
        
        for camera_id in range(1, 4):
            timestamps = multi_source.get_timestamps_by_camera(camera_id)
            if timestamps:
                print(f"C{camera_id}: {len(timestamps)} 帧")
        
        multi_source.close()

