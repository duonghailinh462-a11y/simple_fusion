"""
数据源抽象层 - 定义IRadarSource接口
职责：
  1. 定义统一的雷达数据源接口
  2. 支持多种数据源（JSONL文件、TCP流等）
  3. 提供通用的初始化、启动、获取数据、关闭接口
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class RadarDataFrame:
    """
    雷达数据帧 - 统一的数据容器
    
    Attributes:
        timestamp: 时间戳字符串 (如 "2025-11-21 11:59:10.171")
        radar_objects: 雷达目标列表 (RadarObject对象)
        source: 数据源标识 (如 'file', 'tcp', 'rtsp')
        camera_id: 摄像头ID (1, 2, 3) - 用于按摄像头过滤
    """
    timestamp: str
    radar_objects: List[Any]  # 应该是RadarObject列表
    source: str
    camera_id: Optional[int] = None


class IRadarSource(ABC):
    """
    雷达数据源接口 - 所有雷达数据源必须实现此接口
    
    设计原则：
      1. 统一的数据源接口
      2. 支持同步和异步操作
      3. 提供缓冲区管理
      4. 支持多摄像头过滤
    """

    @abstractmethod
    def initialize(self, **kwargs) -> bool:
        """
        初始化数据源
        
        Args:
            **kwargs: 数据源特定的配置参数
            
        Returns:
            bool: 初始化成功返回 True，失败返回 False
        """
        pass

    @abstractmethod
    def start(self) -> bool:
        """
        启动数据源（可能涉及连接、加载等操作）
        
        Returns:
            bool: 启动成功返回 True，失败返回 False
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        停止数据源（关闭连接、释放资源）
        """
        pass

    @abstractmethod
    def get_next_frame(self, timeout: Optional[float] = None) -> Optional[RadarDataFrame]:
        """
        获取下一帧雷达数据
        
        Args:
            timeout: 等待超时时间（秒）
                    - None: 阻塞等待
                    - 0: 非阻塞，立即返回
                    - >0: 等待指定时间后超时
        
        Returns:
            RadarDataFrame: 雷达数据帧，如果没有数据返回 None
        """
        pass

    @abstractmethod
    def get_all_frames(self) -> List[RadarDataFrame]:
        """
        获取所有可用的雷达数据帧（用于文件源）
        
        Returns:
            List[RadarDataFrame]: 所有雷达数据帧列表
        """
        pass

    @abstractmethod
    def is_ready(self) -> bool:
        """
        检查数据源是否已准备好
        
        Returns:
            bool: 如果可以获取数据返回 True，否则返回 False
        """
        pass

    @abstractmethod
    def get_stats(self) -> Dict[str, Any]:
        """
        获取数据源统计信息
        
        Returns:
            dict: 包含统计信息的字典，如：
                {
                    'frames_received': int,
                    'frames_processed': int,
                    'errors': int,
                    'bytes_received': int,
                    'connection_time': float,
                    ...
                }
        """
        pass

    def set_camera_filter(self, camera_id: int) -> None:
        """
        设置摄像头过滤器（可选实现）
        
        Args:
            camera_id: 摄像头ID (1, 2, 3)
        """
        pass

    def get_source_name(self) -> str:
        """
        获取数据源名称
        
        Returns:
            str: 数据源名称 (如 'file_source', 'stream_source')
        """
        return self.__class__.__name__

