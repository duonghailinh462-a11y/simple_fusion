"""
流数据源 - 从TCP流接收雷达数据（实际模式）
职责：
  1. 建立TCP连接到雷达服务器
  2. 接收并解析数据帧
  3. 支持缓冲区管理
  4. 提供错误恢复和重连机制
  
注意：
  目前为简化版本，不依赖server_multi.py
  完整的server_wrapper集成将在第4步进行
"""

import socket
import json
import math
import time
import threading
import logging
from typing import List, Optional, Dict, Any
from queue import Queue, Empty
from collections import deque

from .base import IRadarSource, RadarDataFrame

# 导入RadarObject
try:
    from core.RadarVisionFusion import RadarObject
except ImportError:
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

logger = get_logger('StreamRadarSource')


class StreamRadarSource(IRadarSource):
    """
    流数据源 - 从TCP连接接收雷达数据
    
    特点：
      1. 支持实时数据流
      2. 异步接收和处理
      3. 自动缓冲和队列管理
      4. 支持摄像头过滤
      5. 自动重连机制
    
    用法：
        source = StreamRadarSource()
        source.initialize(
            host='44.30.142.88',
            port=5000,
            camera_id=1
        )
        source.start()
        
        while True:
            frame = source.get_next_frame(timeout=1.0)
            if frame:
                # 处理frame
                pass
        
        source.stop()
    """

    # 雷达IP到摄像头ID的映射
    RADAR_IP_TO_CAMERA = {
        '44.30.142.85': 2,  # C2
        '44.30.142.88': 1,  # C1
        '44.30.142.87': 3,  # C3
    }

    def __init__(self):
        """初始化流数据源"""
        self.host = None
        self.port = None
        self.camera_id = None
        self.socket = None
        self.buffer = b''
        self.frame_queue = Queue(maxsize=100)  # 帧缓冲队列
        self.receive_thread = None
        self.running = False
        
        # 统计信息
        self.stats = {
            'frames_received': 0,
            'frames_processed': 0,
            'objects_received': 0,
            'errors': 0,
            'bytes_received': 0,
            'connection_time_ms': 0.0,
            'reconnect_count': 0,
        }
        
        # 连接参数
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0  # 秒
        self.socket_timeout = 5.0  # 秒
        
        self.is_initialized = False

    def initialize(self, **kwargs) -> bool:
        """
        初始化流数据源
        
        Args:
            host: TCP服务器地址 (如 '44.30.142.88')
            port: TCP服务器端口 (如 5000)
            camera_id: 摄像头ID (1, 2, 3) - 用于验证和过滤
            socket_timeout: Socket超时时间（秒）
            max_reconnect_attempts: 最大重连次数
            
        Returns:
            bool: 初始化成功返回 True
        """
        try:
            self.host = kwargs.get('host')
            self.port = kwargs.get('port')
            self.camera_id = kwargs.get('camera_id')
            
            if not self.host or not self.port:
                logger.error("❌ 服务器地址或端口未指定")
                return False
            
            self.socket_timeout = kwargs.get('socket_timeout', 5.0)
            self.max_reconnect_attempts = kwargs.get('max_reconnect_attempts', 5)
            
            logger.info(f"📡 初始化流数据源: {self.host}:{self.port}, "
                       f"camera_id={self.camera_id}")
            
            self.is_initialized = True
            return True
        
        except Exception as e:
            logger.error(f"❌ 初始化失败: {e}")
            self.stats['errors'] += 1
            return False

    def _connect(self) -> bool:
        """建立TCP连接"""
        try:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.socket_timeout)
            
            connect_start = time.time()
            self.socket.connect((self.host, self.port))
            connect_elapsed = (time.time() - connect_start) * 1000
            
            logger.info(f"✅ TCP连接成功: {self.host}:{self.port}, "
                       f"耗时={connect_elapsed:.2f}ms")
            
            self.stats['connection_time_ms'] = connect_elapsed
            self.buffer = b''
            
            return True
        
        except Exception as e:
            logger.error(f"❌ TCP连接失败: {e}")
            self.stats['errors'] += 1
            return False

    def _receive_data(self) -> None:
        """
        后台线程 - 持续接收数据并放入队列
        """
        reconnect_count = 0
        
        while self.running:
            try:
                # 尝试连接
                if not self.socket:
                    if reconnect_count < self.max_reconnect_attempts:
                        logger.warning(f"⚠️ 尝试重连 ({reconnect_count + 1}/{self.max_reconnect_attempts})...")
                        if not self._connect():
                            reconnect_count += 1
                            time.sleep(self.reconnect_delay)
                            continue
                        reconnect_count = 0
                        self.stats['reconnect_count'] += 1
                    else:
                        logger.error("❌ 达到最大重连次数，停止接收")
                        self.running = False
                        break
                
                # 接收数据
                data = self.socket.recv(4096)
                if not data:
                    logger.warning("⚠️ TCP连接已关闭")
                    self.socket = None
                    continue
                
                self.buffer += data
                self.stats['bytes_received'] += len(data)
                
                # 尝试解析完整的数据帧（按行分割，每行一个JSON）
                while b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    if not line:
                        continue
                    
                    try:
                        # 解析JSON
                        obj = json.loads(line.decode('utf-8'))
                        frame = self._parse_frame(obj)
                        
                        if frame:
                            # 放入队列（如果满则丢弃最早的帧）
                            try:
                                self.frame_queue.put_nowait(frame)
                                self.stats['frames_received'] += 1
                            except:
                                # 队列满，丢弃
                                try:
                                    self.frame_queue.get_nowait()
                                    self.frame_queue.put_nowait(frame)
                                except:
                                    pass
                    
                    except Exception as e:
                        logger.debug(f"⚠️ 解析数据行失败: {e}")
                        self.stats['errors'] += 1
                        continue
            
            except socket.timeout:
                logger.debug("⏱️ Socket超时，重新连接...")
                self.socket = None
                continue
            
            except Exception as e:
                logger.error(f"❌ 接收数据异常: {e}")
                self.stats['errors'] += 1
                self.socket = None
                time.sleep(self.reconnect_delay)
                continue

    def _parse_frame(self, obj: Dict) -> Optional[RadarDataFrame]:
        """
        解析单个数据帧
        
        Args:
            obj: JSON对象
            
        Returns:
            RadarDataFrame 或 None
        """
        try:
            source_ip = obj.get('source_ip', '')
            camera_id = self.RADAR_IP_TO_CAMERA.get(source_ip)
            
            if camera_id is None:
                return None
            
            # 如果设置了摄像头ID，检查是否匹配
            if self.camera_id and camera_id != self.camera_id:
                return None
            
            time_str = obj.get('time', '')
            if not time_str:
                return None
            
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
                    self.stats['objects_received'] += 1
            
            if locus:
                return RadarDataFrame(
                    timestamp=time_str,
                    radar_objects=locus,
                    source='stream',
                    camera_id=camera_id
                )
        
        except Exception as e:
            logger.debug(f"⚠️ 解析帧失败: {e}")
            self.stats['errors'] += 1
        
        return None

    def start(self) -> bool:
        """
        启动数据源（建立连接并启动接收线程）
        
        Returns:
            bool: 启动成功返回 True
        """
        if not self.is_initialized:
            logger.error("❌ 数据源未初始化")
            return False
        
        try:
            # 建立初始连接
            if not self._connect():
                return False
            
            # 启动接收线程
            self.running = True
            self.receive_thread = threading.Thread(
                target=self._receive_data,
                daemon=True
            )
            self.receive_thread.start()
            
            logger.info("✅ 流数据源已启动")
            return True
        
        except Exception as e:
            logger.error(f"❌ 启动失败: {e}")
            self.stats['errors'] += 1
            return False

    def stop(self) -> None:
        """停止数据源"""
        self.running = False
        
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        logger.info("✅ 流数据源已停止")

    def get_next_frame(self, timeout: Optional[float] = None) -> Optional[RadarDataFrame]:
        """
        获取下一帧雷达数据
        
        Args:
            timeout: 等待超时时间（秒）
                    - None: 无限等待
                    - 0: 非阻塞
                    - >0: 等待指定时间
        
        Returns:
            RadarDataFrame 或 None
        """
        if timeout is None:
            timeout = 1.0  # 默认等待1秒
        
        try:
            frame = self.frame_queue.get(timeout=timeout)
            self.stats['frames_processed'] += 1
            return frame
        
        except Empty:
            return None
        
        except Exception as e:
            logger.error(f"❌ 获取帧失败: {e}")
            self.stats['errors'] += 1
            return None

    def get_all_frames(self) -> List[RadarDataFrame]:
        """
        获取队列中所有当前可用的帧
        
        Returns:
            List[RadarDataFrame]: 所有可用数据帧
        """
        frames = []
        
        while True:
            try:
                frame = self.frame_queue.get_nowait()
                frames.append(frame)
                self.stats['frames_processed'] += 1
            except Empty:
                break
            except Exception as e:
                logger.error(f"❌ 获取帧失败: {e}")
                self.stats['errors'] += 1
                break
        
        return frames

    def is_ready(self) -> bool:
        """检查数据源是否就绪"""
        return self.running and self.socket is not None

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        return self.stats.copy()

    def set_camera_filter(self, camera_id: int) -> None:
        """设置摄像头过滤器"""
        self.camera_id = camera_id
        logger.info(f"📹 设置摄像头过滤: C{camera_id}")

