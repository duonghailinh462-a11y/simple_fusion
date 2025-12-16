"""
文件数据源 - 从JSONL文件加载雷达数据（测试模式）
职责：
  1. 加载JSONL格式的雷达数据文件
  2. 按时间戳组织数据
  3. 支持按摄像头过滤
  4. 提供帧迭代接口
"""

import json
import math
import time
import logging
from typing import List, Optional, Dict, Any
from collections import defaultdict

from .base import IRadarSource, RadarDataFrame

# 导入RadarObject
try:
    from core.RadarVisionFusion import RadarObject
except ImportError:
    # 如果无法导入，定义一个简单的占位符
    class RadarObject:
        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)

# 导入日志
try:
    from core.logger_config import get_logger
except ImportError:
    logging.basicConfig(level=logging.INFO)
    get_logger = logging.getLogger

logger = get_logger('FileRadarSource')


class FileRadarSource(IRadarSource):
    """
    文件数据源 - 从JSONL文件读取雷达数据
    
    特点：
      1. 支持离线测试
      2. 可重复读取
      3. 支持按摄像头过滤
      4. 提供帧迭代和批量获取
    
    用法：
        source = FileRadarSource()
        source.initialize(file_path='/path/to/radar.jsonl')
        source.start()
        
        while True:
            frame = source.get_next_frame()
            if frame is None:
                break
            # 处理frame
        
        source.stop()
    """

    # 雷达IP到摄像头ID的映射
    RADAR_IP_TO_CAMERA = {
        '44.30.142.85': 2,  # C2
        '44.30.142.88': 1,  # C1
        '44.30.142.87': 3,  # C3
    }

    def __init__(self):
        """初始化文件数据源"""
        self.file_path = None
        self.radar_data = {}  # 时间戳 -> 雷达目标列表
        self.timestamps_list = []  # 排序的时间戳列表
        self.current_index = 0  # 当前帧索引
        self.camera_filter = None  # 摄像头过滤器
        
        # 统计信息
        self.stats = {
            'frames_loaded': 0,
            'frames_processed': 0,
            'objects_loaded': 0,
            'errors': 0,
            'load_time_ms': 0.0,
        }
        
        self.is_initialized = False
        self.is_running = False

    def initialize(self, **kwargs) -> bool:
        """
        初始化文件数据源
        
        Args:
            file_path: 雷达数据文件路径 (JSONL格式)
            
        Returns:
            bool: 初始化成功返回 True
        """
        try:
            self.file_path = kwargs.get('file_path')
            if not self.file_path:
                logger.error("❌ 文件路径未指定")
                return False
            
            logger.info(f"📂 初始化文件数据源: {self.file_path}")
            self.is_initialized = True
            return True
        
        except Exception as e:
            logger.error(f"❌ 初始化失败: {e}")
            self.stats['errors'] += 1
            return False

    def start(self) -> bool:
        """
        启动数据源（加载JSONL文件）
        
        Returns:
            bool: 启动成功返回 True
        """
        if not self.is_initialized:
            logger.error("❌ 数据源未初始化")
            return False
        
        try:
            load_start = time.time()
            
            with open(self.file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    try:
                        obj = json.loads(line)
                        source_ip = obj.get('source_ip', '')
                        camera_id = self.RADAR_IP_TO_CAMERA.get(source_ip)
                        
                        if camera_id is None:
                            continue
                        
                        # 如果设置了摄像头过滤，检查
                        if self.camera_filter and camera_id != self.camera_filter:
                            continue
                        
                        time_str = obj.get('time', '')
                        if not time_str:
                            continue
                        
                        # 解析雷达对象
                        locus = []
                        for x in obj.get('locusList', []):
                            if x.get('objType') in {1}:  # VALID_RADAR_TYPES
                                radar_lane = x.get('lane', None)
                                lane_str = f'lane_{radar_lane}' if radar_lane is not None else None
                                
                                azimuth_val = x.get('azimuth', 0.0)
                                if azimuth_val is None:
                                    azimuth_val = 0.0
                                else:
                                    try:
                                        azimuth_val = float(azimuth_val)
                                        if math.isnan(azimuth_val) or math.isinf(azimuth_val):
                                            azimuth_val = 0.0
                                    except (ValueError, TypeError):
                                        azimuth_val = 0.0
                                
                                radar_obj = RadarObject(
                                    radar_id=x.get('id', ''),
                                    latitude=float(x.get('latitude', 0)),
                                    longitude=float(x.get('longitude', 0)),
                                    speed=float(x.get('speed', 0)),
                                    azimuth=azimuth_val,
                                    lane=lane_str,
                                    timestamp_str=time_str,
                                    source_ip=source_ip
                                )
                                locus.append(radar_obj)
                                self.stats['objects_loaded'] += 1
                        
                        if locus:
                            self.radar_data[time_str] = locus
                            self.stats['frames_loaded'] += 1
                    
                    except Exception as e:
                        logger.warning(f"⚠️ 解析数据行失败: {e}")
                        self.stats['errors'] += 1
                        continue
            
            # 排序时间戳
            self.timestamps_list = sorted(self.radar_data.keys())
            self.current_index = 0
            
            load_elapsed = (time.time() - load_start) * 1000
            self.stats['load_time_ms'] = load_elapsed
            
            logger.info(f"✅ 加载完成: {self.stats['frames_loaded']} 帧, "
                       f"{self.stats['objects_loaded']} 个对象, 耗时={load_elapsed:.2f}ms")
            
            self.is_running = True
            return True
        
        except Exception as e:
            logger.error(f"❌ 加载文件失败: {e}")
            self.stats['errors'] += 1
            return False

    def stop(self) -> None:
        """停止数据源"""
        self.is_running = False
        logger.info("✅ 文件数据源已停止")

    def get_next_frame(self, timeout: Optional[float] = None) -> Optional[RadarDataFrame]:
        """
        获取下一帧雷达数据
        
        Args:
            timeout: 超时时间（文件源不支持，忽略）
        
        Returns:
            RadarDataFrame 或 None
        """
        if not self.is_running:
            return None
        
        if self.current_index >= len(self.timestamps_list):
            return None
        
        try:
            timestamp = self.timestamps_list[self.current_index]
            radar_objects = self.radar_data[timestamp]
            self.current_index += 1
            self.stats['frames_processed'] += 1
            
            # 获取摄像头ID（从第一个雷达对象）
            camera_id = None
            if radar_objects:
                source_ip = getattr(radar_objects[0], 'source_ip', None)
                camera_id = self.RADAR_IP_TO_CAMERA.get(source_ip)
            
            return RadarDataFrame(
                timestamp=timestamp,
                radar_objects=radar_objects,
                source='file',
                camera_id=camera_id
            )
        
        except Exception as e:
            logger.error(f"❌ 获取帧失败: {e}")
            self.stats['errors'] += 1
            return None

    def get_all_frames(self) -> List[RadarDataFrame]:
        """
        获取所有雷达数据帧
        
        Returns:
            List[RadarDataFrame]: 所有数据帧列表
        """
        frames = []
        self.current_index = 0  # 重置索引
        
        while True:
            frame = self.get_next_frame()
            if frame is None:
                break
            frames.append(frame)
        
        return frames

    def is_ready(self) -> bool:
        """检查数据源是否就绪"""
        return self.is_running and len(self.timestamps_list) > 0

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        return self.stats.copy()

    def set_camera_filter(self, camera_id: int) -> None:
        """设置摄像头过滤器"""
        self.camera_filter = camera_id
        logger.info(f"📹 设置摄像头过滤: C{camera_id}")

    def reset(self) -> None:
        """重置数据源（用于重新读取）"""
        self.current_index = 0
        self.stats['frames_processed'] = 0
        logger.info("🔄 数据源已重置")

    def get_frame_count(self) -> int:
        """获取总帧数"""
        return len(self.timestamps_list)

