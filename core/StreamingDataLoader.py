"""
流式数据加载器 - 替代批量加载的高效方案
设计目标：按时间顺序流式加载数据，自动清理过期数据，缓冲区始终保持小尺寸

核心特性：
1. StreamingRadarLoader - 逐帧读取雷达数据，不一次性加载所有数据
2. 与 RadarVisionFusionProcessor 配合使用
3. 减少内存占用和时间戳匹配耗时
"""

import json
import math
import time
from datetime import datetime
from typing import List, Dict, Tuple, Generator, Optional
from collections import deque


# ==========================================
# 常量定义
# ==========================================
VALID_RADAR_TYPES = {1}
RADAR_IP_TO_CAMERA = {
    '192.168.1.1': 1,
    '192.168.1.2': 2,
    '192.168.1.3': 3,
}


# ==========================================
# 工具函数
# ==========================================
def parse_time(ts_str):
    """解析时间戳字符串，支持多种格式
    
    支持格式：
    - "2025-11-21 11:59:10.171" (3位毫秒)
    - "2025-11-21 11:59:10.171000" (6位微秒)
    - "2025-11-21 11:59:10"
    
    Args:
        ts_str: 时间戳字符串或浮点数
        
    Returns:
        Unix时间戳（浮点数）
    """
    if not ts_str:
        return 0.0
    
    # 如果已经是浮点数，直接返回
    if isinstance(ts_str, (int, float)):
        return float(ts_str)
    
    clean_ts = ' '.join(str(ts_str).split())
    
    # 优先尝试带毫秒的格式
    formats_to_try = [
        ('%Y-%m-%d %H:%M:%S.%f', False),  # 6位微秒
        ('%Y-%m-%d %H:%M:%S', False),      # 不带毫秒
    ]
    
    for fmt, needs_padding in formats_to_try:
        try:
            # 如果格式需要6位微秒但只有3位毫秒，需要补充到6位
            if needs_padding and '.' in clean_ts:
                parts = clean_ts.split('.')
                if len(parts) == 2 and len(parts[1]) < 6:
                    clean_ts = f"{parts[0]}.{parts[1].ljust(6, '0')}"
            
            dt = datetime.strptime(clean_ts, fmt)
            return dt.timestamp()
        except ValueError:
            # 如果不需要补充但失败，尝试补充后重试
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
    
    # 如果都失败，返回0.0
    return 0.0


def format_ts(ts):
    """格式化时间戳"""
    if not isinstance(ts, (int, float)):
        return str(ts)
    return datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


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


# ==========================================
# 核心类：流式雷达数据加载器
# ==========================================
class StreamingRadarLoader:
    """
    流式雷达数据加载器 - 逐帧读取，不一次性加载全部数据
    
    相比 RadarDataLoader.load()：
    - ❌ 批量加载所有数据到内存（1000+ 帧）
    - ✓ 按需流式读取（每次生成一帧）
    - ✓ 减少内存占用
    - ✓ 允许处理任意大小的数据文件
    """
    
    def __init__(self, radar_file_path):
        """
        初始化流式加载器
        
        Args:
            radar_file_path: 雷达数据文件路径 (JSONL)
        """
        self.radar_file_path = radar_file_path
    
    def stream_radar_frames(self) -> Generator[Tuple[str, List[RadarObject]], None, None]:
        """
        按时间顺序逐帧生成雷达数据
        
        这是一个生成器函数，每次调用 next() 时才读取一帧数据，
        而不是一次性加载所有数据到内存。
        
        Yields:
            (timestamp: str, radar_objects: List[RadarObject])
            
        Example:
            >>> loader = StreamingRadarLoader('radar.jsonl')
            >>> for timestamp, objects in loader.stream_radar_frames():
            ...     print(f"Radar frame at {timestamp}: {len(objects)} objects")
        """
        try:
            with open(self.radar_file_path, 'r', encoding='utf-8') as f:
                frame_count = 0
                for line in f:
                    try:
                        obj = json.loads(line)
                        
                        # 提取时间戳
                        time_str = obj.get('time', '')
                        if not time_str:
                            continue
                        
                        # 提取所有有效的雷达对象
                        locus = []
                        for x in obj.get('locusList', []):
                            if x.get('objType') in VALID_RADAR_TYPES:
                                # 处理 lane 格式转换
                                radar_lane = x.get('lane', None)
                                lane_str = f'lane_{radar_lane}' if radar_lane is not None else None
                                
                                # 处理 azimuth
                                azimuth_val = x.get('azimuth')
                                if azimuth_val is None:
                                    azimuth_val = 0.0
                                else:
                                    try:
                                        azimuth_val = float(azimuth_val)
                                        if math.isnan(azimuth_val) or math.isinf(azimuth_val):
                                            azimuth_val = 0.0
                                    except (ValueError, TypeError):
                                        azimuth_val = 0.0
                                
                                # 创建雷达对象
                                radar_obj = RadarObject(
                                    radar_id=x.get('id', ''),
                                    latitude=float(x.get('latitude', 0)),
                                    longitude=float(x.get('longitude', 0)),
                                    speed=float(x.get('speed', 0)),
                                    azimuth=azimuth_val,
                                    lane=lane_str,
                                    timestamp_str=time_str,
                                    source_ip=obj.get('source_ip', '')
                                )
                                locus.append(radar_obj)
                        
                        # 仅当有有效对象时才生成
                        if locus:
                            frame_count += 1
                            yield time_str, locus
                    
                    except json.JSONDecodeError as e:
                        # 跳过无效的JSON行
                        continue
                    except Exception as e:
                        # 其他错误也继续处理下一行
                        continue
        
        except Exception as e:
            print(f"❌ 读取雷达文件失败: {e}")
            return


# ==========================================
# 融合处理器的辅助类
# ==========================================
class StreamingBufferManager:
    """
    流式缓冲区管理器 - 自动管理雷达数据缓冲区大小
    
    功能：
    1. 维护雷达数据缓冲区
    2. 自动清理过期数据
    3. 提供诊断信息
    """
    
    def __init__(self, max_age=2.0, cleanup_interval=1.0):
        """
        初始化缓冲区管理器
        
        Args:
            max_age: 最大保留时间（秒），超过此时间的数据会被清理
            cleanup_interval: 清理间隔（秒）
        """
        self.radar_buffer = {}  # timestamp -> List[RadarObject]
        self.max_age = max_age
        self.cleanup_interval = cleanup_interval
        self.last_cleanup_time = time.time()
        
        # 统计信息
        self.stats = {
            'total_added': 0,
            'total_cleaned': 0,
            'max_buffer_size': 0,
            'cleanup_count': 0,
        }
    
    def add_radar_data(self, timestamp, radar_objects: List[RadarObject]) -> None:
        """
        添加雷达数据到缓冲区
        
        Args:
            timestamp: 时间戳（字符串或浮点数）
            radar_objects: 雷达目标列表
        """
        self.radar_buffer[timestamp] = radar_objects
        self.stats['total_added'] += len(radar_objects)
        
        # 更新最大缓冲区大小
        current_size = len(self.radar_buffer)
        if current_size > self.stats['max_buffer_size']:
            self.stats['max_buffer_size'] = current_size
        
        # 检查是否需要清理
        current_time = time.time()
        if current_time - self.last_cleanup_time > self.cleanup_interval:
            self._cleanup_old_data(timestamp)
    
    def _cleanup_old_data(self, current_timestamp) -> int:
        """
        清理过期的雷达数据
        
        Args:
            current_timestamp: 当前时间戳
            
        Returns:
            清理的数据条数
        """
        current_numeric = parse_time(current_timestamp)
        
        # 找出需要清理的时间戳
        old_timestamps = []
        for ts in self.radar_buffer.keys():
            ts_numeric = parse_time(ts)
            if current_numeric - ts_numeric > self.max_age:
                old_timestamps.append(ts)
        
        # 执行清理
        for ts in old_timestamps:
            removed_count = len(self.radar_buffer.pop(ts, []))
            self.stats['total_cleaned'] += removed_count
        
        self.stats['cleanup_count'] += 1
        self.last_cleanup_time = time.time()
        
        return len(old_timestamps)
    
    def get_buffer_info(self) -> Dict:
        """
        获取缓冲区信息
        
        Returns:
            包含缓冲区统计的字典
        """
        return {
            'current_size': len(self.radar_buffer),
            'max_size': self.stats['max_buffer_size'],
            'total_added': self.stats['total_added'],
            'total_cleaned': self.stats['total_cleaned'],
            'cleanup_count': self.stats['cleanup_count'],
        }
    
    def diagnose(self) -> None:
        """输出诊断信息"""
        info = self.get_buffer_info()
        print(f"""
[缓冲区诊断]
  当前大小: {info['current_size']} 帧
  最大大小: {info['max_size']} 帧
  已添加: {info['total_added']} 个对象
  已清理: {info['total_cleaned']} 个对象
  清理次数: {info['cleanup_count']}
        """)


# ==========================================
# 集成工具：融合处理管线
# ==========================================
class StreamingFusionPipeline:
    """
    流式融合处理管线 - 集成雷达数据加载和融合处理
    
    使用示例：
    
    ```python
    # 创建融合处理器（每个摄像头一个）
    processors = {
        1: RadarVisionFusionProcessor(camera_id=1),
        2: RadarVisionFusionProcessor(camera_id=2),
        3: RadarVisionFusionProcessor(camera_id=3),
    }
    
    # 创建融合管线
    pipeline = StreamingFusionPipeline('radar.jsonl', processors)
    
    # 处理视觉帧
    for camera_id, vision_timestamp, vision_objects in vision_frame_source:
        results = pipeline.process_vision_frame(camera_id, vision_timestamp, vision_objects)
        # 使用 results
    ```
    """
    
    def __init__(self, radar_file_path, fusion_processors: Dict):
        """
        初始化融合处理管线
        
        Args:
            radar_file_path: 雷达数据文件路径
            fusion_processors: {camera_id -> RadarVisionFusionProcessor}
        """
        self.loader = StreamingRadarLoader(radar_file_path)
        self.processors = fusion_processors
        
        # 为每个处理器配置一个缓冲区管理器
        self.buffer_managers = {
            camera_id: StreamingBufferManager(max_age=2.0, cleanup_interval=1.0)
            for camera_id in fusion_processors.keys()
        }
        
        # 启动雷达数据加载线程
        self.radar_iterator = None
        self._init_radar_loader()
    
    def _init_radar_loader(self) -> None:
        """初始化雷达加载器迭代器"""
        self.radar_iterator = self.loader.stream_radar_frames()
    
    def add_radar_data_from_stream(self) -> None:
        """
        从雷达流中读取并添加数据到所有处理器
        
        这个方法应该在主处理循环中定期调用，
        或者在单独的线程中运行
        """
        if self.radar_iterator is None:
            return
        
        try:
            timestamp, radar_objects = next(self.radar_iterator)
            
            # 添加到所有处理器和缓冲区管理器
            for camera_id, processor in self.processors.items():
                processor.add_radar_data(timestamp, radar_objects)
                self.buffer_managers[camera_id].add_radar_data(timestamp, radar_objects)
        
        except StopIteration:
            # 雷达数据已全部读完
            self.radar_iterator = None
    
    def process_vision_frame(self, camera_id: int, vision_timestamp, vision_objects):
        """
        处理视觉帧
        
        Args:
            camera_id: 摄像头ID
            vision_timestamp: 视觉时间戳
            vision_objects: 视觉目标列表
            
        Returns:
            融合后的目标列表
        """
        processor = self.processors.get(camera_id)
        if processor is None:
            return vision_objects
        
        # 处理视觉帧
        results = processor.process_frame(vision_timestamp, vision_objects)
        
        # 清理过期数据
        processor.clear_old_radar_data(vision_timestamp, max_age=2.0)
        
        return results
    
    def diagnose_all(self) -> None:
        """诊断所有摄像头的缓冲区状态"""
        print("\n" + "="*60)
        print("融合管线诊断信息")
        print("="*60)
        
        for camera_id, manager in self.buffer_managers.items():
            print(f"\n[摄像头 {camera_id}]")
            manager.diagnose()


# ==========================================
# 示例和测试
# ==========================================
if __name__ == "__main__":
    # 示例 1：基本的流式加载
    print("示例 1：基本的流式加载")
    print("-" * 60)
    
    loader = StreamingRadarLoader('radar_data.jsonl')
    count = 0
    for timestamp, objects in loader.stream_radar_frames():
        count += 1
        if count <= 3:  # 只打印前3帧
            print(f"Radar frame {count}: timestamp={timestamp}, objects={len(objects)}")
        
        if count >= 100:  # 只演示100帧
            break
    
    print(f"✅ 总共处理了 {count} 帧雷达数据")
    
    # 示例 2：使用缓冲区管理器
    print("\n示例 2：使用缓冲区管理器")
    print("-" * 60)
    
    manager = StreamingBufferManager(max_age=1.0, cleanup_interval=0.1)
    
    loader = StreamingRadarLoader('radar_data.jsonl')
    for timestamp, objects in loader.stream_radar_frames():
        manager.add_radar_data(timestamp, objects)
        
        if manager.stats['cleanup_count'] >= 5:
            break
    
    manager.diagnose()

