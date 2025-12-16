"""
RadarVisionFusionProcessor 与 server_multi.py 的集成适配器

功能：
1. 将 server_multi.py 的实时雷达数据流转换为 RadarFrame
2. 提供 StreamRadarSource 兼容的接口
3. 支持实时融合处理
4. 自动缓冲和队列管理

架构：
    server_multi.py (TCP服务器)
        │
        ├─ 保存到: radar_data.jsonl
        ├─ 发送到: multiprocessing.Queue
        │
        └─ server_wrapper.py (适配器)
            ├─ 转换为 RadarFrame
            ├─ 兼容 BaseRadarSource 接口
            └─ 支持 RadarVisionFusionProcessor 融合处理

使用示例：
    # 方式 1: 独立运行服务器和融合处理
    from radar.server_wrapper import RealtimeRadarServer
    
    server = RealtimeRadarServer(camera_id=1)
    server.start()  # 启动TCP服务器和融合处理
    
    # 方式 2: 作为 RadarSourceFactory 的输入
    from core.radar_source_abstraction import RadarSourceFactory
    from radar.server_wrapper import create_realtime_source
    
    source = create_realtime_source(camera_id=1, buffer_size=100)
    source.initialize()
    
    for frame in source.stream_frames():
        print(f"{frame.timestamp}: {frame.get_object_count()} 个目标")
"""

import json
import socket
import struct
import threading
import multiprocessing
import time
import queue
import logging
from typing import Dict, List, Optional, Generator
from datetime import datetime
from collections import OrderedDict, deque

try:
    import radar_pb2
    from google.protobuf.json_format import MessageToDict
    PROTOBUF_AVAILABLE = True
except ImportError:
    PROTOBUF_AVAILABLE = False
    # 不在导入时抛出错误，允许在不需要 protobuf 的测试中使用

from core.radar_source_abstraction import (
    BaseRadarSource, RadarFrame, RadarObject, parse_time, 
    RADAR_IP_TO_CAMERA, VALID_RADAR_TYPES
)


logger = logging.getLogger(__name__)


# ==========================================
# 实时雷达数据源
# ==========================================
class RealtimeRadarSource(BaseRadarSource):
    """
    实时雷达数据源 - 从 server_multi.py 获取数据
    
    特点：
    - 无限流式数据，不支持随机访问
    - 自动过期数据清理
    - 支持多摄像头过滤
    - 与 RadarVisionFusionProcessor 无缝集成
    """
    
    def __init__(self, data_queue: multiprocessing.Queue, 
                 camera_id: Optional[int] = None,
                 buffer_size: int = 100,
                 buffer_time_window: float = 30.0,
                 timeout: float = 1.0):
        """
        Args:
            data_queue: multiprocessing.Queue 来自 server_multi.run_server()
            camera_id: 摄像头ID过滤（None=不过滤）
            buffer_size: 缓冲区大小（帧数）
            buffer_time_window: 缓冲时间窗口（秒）
            timeout: 队列读取超时（秒）
        """
        super().__init__()
        self.data_queue = data_queue
        self.camera_id = camera_id
        self.buffer_size = buffer_size
        self.buffer_time_window = buffer_time_window
        self.timeout = timeout
        
        # 缓冲区
        self.frame_buffer = OrderedDict()  # timestamp -> RadarFrame
        self.timestamp_list = deque(maxlen=buffer_size)
        
        # 统计
        self.total_frames_received = 0
        self.total_frames_filtered = 0
        self.total_objects_received = 0
        self.start_time = None
        self.end_time = None
        
        # 线程管理
        self._reader_thread = None
        self._is_reading = False
        self._lock = threading.Lock()
    
    def initialize(self) -> bool:
        """初始化数据源 - 启动后台读取线程"""
        if self._is_reading:
            logger.warning("RealtimeRadarSource 已初始化")
            return True
        
        self._is_reading = True
        self._reader_thread = threading.Thread(
            target=self._read_from_queue,
            daemon=True
        )
        self._reader_thread.start()
        logger.info(f"RealtimeRadarSource 已初始化 (camera_id={self.camera_id})")
        return True
    
    def _read_from_queue(self):
        """后台线程：从队列读取数据并转换为 RadarFrame"""
        while self._is_reading:
            try:
                # 读取原始数据字典
                data_dict = self.data_queue.get(timeout=self.timeout)
                if data_dict is None:
                    break
                
                # 转换为 RadarFrame
                frame = self._convert_to_frame(data_dict)
                if frame is None:
                    continue
                
                # 过滤摄像头
                if self.camera_id is not None and frame.camera_id != self.camera_id:
                    self.total_frames_filtered += 1
                    continue
                
                # 加入缓冲区
                with self._lock:
                    self.frame_buffer[frame.timestamp_numeric] = frame
                    self.timestamp_list.append(frame.timestamp_numeric)
                    self.total_frames_received += 1
                    self.total_objects_received += len(frame.objects)
                    
                    if self.start_time is None:
                        self.start_time = frame.timestamp_numeric
                    self.end_time = frame.timestamp_numeric
                    
                    # 清理过期数据
                    self._cleanup_by_time_window()
                    
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"读取队列异常: {e}", exc_info=True)
                continue
    
    def _convert_to_frame(self, data_dict: Dict) -> Optional[RadarFrame]:
        """将原始数据字典转换为 RadarFrame"""
        try:
            # 提取时间戳
            timestamp_str = data_dict.get('time', '')
            if not timestamp_str:
                return None
            
            # 提取雷达类型
            radar_type = data_dict.get('radarType')
            if radar_type not in VALID_RADAR_TYPES:
                return None
            
            # 提取对象列表
            objects_data = data_dict.get('detectedObjLocus', [])
            if not isinstance(objects_data, list):
                return None
            
            # 转换对象
            objects = []
            for obj_data in objects_data:
                try:
                    obj = RadarObject(
                        radar_id=obj_data.get('radarId'),
                        latitude=float(obj_data.get('lat', 0)),
                        longitude=float(obj_data.get('lon', 0)),
                        speed=float(obj_data.get('speed', 0)),
                        azimuth=float(obj_data.get('azimuth', 0)),
                        lane=obj_data.get('lane'),
                        timestamp_str=timestamp_str,
                        source_ip=data_dict.get('source_ip')
                    )
                    objects.append(obj)
                except Exception as e:
                    logger.debug(f"转换对象失败: {e}")
                    continue
            
            # 创建帧
            frame = RadarFrame(
                timestamp=timestamp_str,
                objects=objects,
                metadata={
                    'source_ip': data_dict.get('source_ip'),
                    'camera_id': self.camera_id,
                    'radar_type': radar_type,
                }
            )
            return frame
            
        except Exception as e:
            logger.debug(f"转换帧失败: {e}")
            return None
    
    def _cleanup_by_time_window(self):
        """清理超出时间窗口的数据"""
        try:
            current_time = time.time()
            keys_to_remove = []
            
            for ts in self.frame_buffer.keys():
                if current_time - ts > self.buffer_time_window:
                    keys_to_remove.append(ts)
            
            for ts in keys_to_remove:
                del self.frame_buffer[ts]
                
        except Exception as e:
            logger.debug(f"清理缓冲区失败: {e}")
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成帧 - 等待新数据到达"""
        last_ts = -1
        
        while self._is_reading:
            try:
                with self._lock:
                    # 获取新时间戳
                    if len(self.timestamp_list) > 0:
                        new_timestamps = [ts for ts in self.frame_buffer.keys() 
                                        if ts > last_ts]
                        new_timestamps.sort()
                        
                        for ts in new_timestamps:
                            frame = self.frame_buffer[ts]
                            last_ts = ts
                            yield frame
                
                # 短暂等待，避免忙轮询
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"流式生成异常: {e}", exc_info=True)
                time.sleep(0.1)
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """获取特定时间戳的帧 - 实时源不支持随机访问"""
        try:
            ts_numeric = parse_time(timestamp)
            with self._lock:
                return self.frame_buffer.get(ts_numeric)
        except Exception as e:
            logger.error(f"获取帧失败: {e}")
            return None
    
    def get_all_timestamps(self) -> List[str]:
        """获取所有时间戳"""
        with self._lock:
            timestamps = []
            for frame in self.frame_buffer.values():
                timestamps.append(frame.timestamp)
            return timestamps
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        with self._lock:
            duration = 0.0
            if self.start_time and self.end_time:
                duration = self.end_time - self.start_time
            
            return {
                'source_type': 'RealtimeRadarSource',
                'total_frames_received': self.total_frames_received,
                'total_frames_filtered': self.total_frames_filtered,
                'total_objects_received': self.total_objects_received,
                'buffered_frames': len(self.frame_buffer),
                'buffer_size_limit': self.buffer_size,
                'duration_seconds': duration,
                'camera_id': self.camera_id,
                'buffer_time_window': self.buffer_time_window,
                'avg_objects_per_frame': (
                    self.total_objects_received / self.total_frames_received 
                    if self.total_frames_received > 0 else 0
                ),
            }
    
    def close(self):
        """关闭数据源"""
        self._is_reading = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        logger.info("RealtimeRadarSource 已关闭")


# ==========================================
# 实时雷达服务器集成
# ==========================================
class RealtimeRadarServer:
    """
    完整的实时雷达系统集成
    
    功能：
    1. 运行 TCP 服务器接收雷达数据
    2. 创建数据源以供融合处理使用
    3. 管理数据流的完整生命周期
    """
    
    def __init__(self, camera_id: int = 1, listen_port: int = 12400,
                 buffer_size: int = 100, save_filename: str = "radar_data.jsonl"):
        """
        Args:
            camera_id: 摄像头ID
            listen_port: TCP 监听端口
            buffer_size: 缓冲区大小
            save_filename: 保存的 JSONL 文件名
        """
        self.camera_id = camera_id
        self.listen_port = listen_port
        self.buffer_size = buffer_size
        self.save_filename = save_filename
        
        self.data_queue = multiprocessing.Queue(maxsize=1000)
        self.server_process = None
        self.source = None
    
    def start(self):
        """启动服务器"""
        if not PROTOBUF_AVAILABLE:
            raise ImportError("需要安装 protobuf: pip install protobuf google-protobuf-json-format")
        
        # 导入 server_multi
        from radar import server_multi
        
        # 重设保存文件名
        server_multi.SAVE_FILENAME = self.save_filename
        server_multi.LISTEN_PORT = self.listen_port
        
        # 启动服务器进程
        self.server_process = multiprocessing.Process(
            target=server_multi.run_server,
            args=(self.data_queue,),
            daemon=True
        )
        self.server_process.start()
        logger.info(f"TCP 服务器已启动 (端口: {self.listen_port})")
        
        # 创建数据源
        self.source = RealtimeRadarSource(
            data_queue=self.data_queue,
            camera_id=self.camera_id,
            buffer_size=self.buffer_size
        )
        self.source.initialize()
        logger.info(f"实时雷达源已初始化 (camera_id: {self.camera_id})")
    
    def get_source(self) -> RealtimeRadarSource:
        """获取数据源用于融合处理"""
        if self.source is None:
            raise RuntimeError("服务器未启动，请先调用 start()")
        return self.source
    
    def stop(self):
        """停止服务器"""
        if self.source:
            self.source.close()
        if self.server_process and self.server_process.is_alive():
            self.server_process.terminate()
            self.server_process.join(timeout=5.0)
        logger.info("实时雷达服务器已停止")


# ==========================================
# 工厂函数
# ==========================================
def create_realtime_source(camera_id: int = 1, buffer_size: int = 100,
                           listen_port: int = 12400) -> RealtimeRadarSource:
    """
    创建实时雷达数据源
    
    Args:
        camera_id: 摄像头 ID
        buffer_size: 缓冲区大小
        listen_port: TCP 监听端口
    
    Returns:
        RealtimeRadarSource 实例
    
    示例：
        source = create_realtime_source(camera_id=1)
        source.initialize()
        
        for frame in source.stream_frames():
            print(f"收到: {frame.timestamp} ({frame.get_object_count()} 个目标)")
    """
    if not PROTOBUF_AVAILABLE:
        raise ImportError("需要安装 protobuf: pip install protobuf google-protobuf-json-format")
    
    from radar import server_multi
    
    # 配置服务器
    server_multi.LISTEN_PORT = listen_port
    
    # 创建队列
    data_queue = multiprocessing.Queue(maxsize=1000)
    
    # 启动服务器线程
    server_thread = threading.Thread(
        target=server_multi.run_server,
        args=(data_queue,),
        daemon=True
    )
    server_thread.start()
    time.sleep(0.5)  # 等待服务器启动
    
    # 创建数据源
    source = RealtimeRadarSource(
        data_queue=data_queue,
        camera_id=camera_id,
        buffer_size=buffer_size
    )
    
    return source


# ==========================================
# 集成示例
# ==========================================
def example_realtime_fusion():
    """
    示例：实时雷达融合处理
    """
    from core.RadarVisionFusion import RadarVisionFusionProcessor
    
    # 创建实时服务器
    server = RealtimeRadarServer(camera_id=1)
    server.start()
    
    # 创建融合处理器
    processor = RadarVisionFusionProcessor(camera_id=1)
    
    # 创建数据源
    source = server.get_source()
    
    print("开始实时融合处理... (按 Ctrl+C 停止)")
    try:
        # 流式处理雷达数据
        for frame in source.stream_frames():
            # 添加到融合处理器
            processor.add_radar_data(frame.timestamp, frame.objects)
            
            # 获取统计信息
            stats = processor.get_buffer_stats()
            if processor.total_fused % 100 == 0:
                print(f"已融合: {processor.total_fused} 帧, "
                      f"雷达缓冲: {stats['radar_buffer_size']}, "
                      f"视觉缓冲: {stats['vision_buffer_size']}")
    
    except KeyboardInterrupt:
        print("\n停止处理")
    finally:
        source.close()
        server.stop()


if __name__ == '__main__':
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 运行示例
    example_realtime_fusion()

