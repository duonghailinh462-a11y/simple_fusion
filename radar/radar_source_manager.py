"""
雷达数据源管理器
职责：
  1. 加载和解析配置文件
  2. 根据配置创建合适的数据源
  3. 管理数据源的生命周期
  4. 提供统一的数据获取接口
  5. 支持多摄像头过滤
"""

import os
import logging
import yaml
from typing import Optional, Dict, Any, List
from pathlib import Path

from radar.data_source import IRadarSource, FileRadarSource, StreamRadarSource, RadarDataFrame

# 导入日志
try:
    from core.logger_config import get_logger
except ImportError:
    logging.basicConfig(level=logging.INFO)
    get_logger = logging.getLogger

logger = get_logger('RadarSourceManager')


class RadarSourceManager:
    """
    雷达数据源管理器
    
    职责：
      1. 从配置文件读取雷达数据源配置
      2. 创建和管理数据源实例
      3. 提供统一的数据获取接口
      4. 处理多摄像头场景
    
    用法：
        manager = RadarSourceManager()
        manager.load_config('config/radar_source_config.yaml')
        manager.initialize()
        
        while True:
            frame = manager.get_next_frame(camera_id=1)
            if frame:
                # 处理frame
                pass
        
        manager.stop()
    """

    def __init__(self):
        """初始化数据源管理器"""
        self.config = {}
        self.data_source = None
        self.config_path = None
        self.is_initialized = False
        self.is_running = False
        
        # 缓冲数据（用于多摄像头支持）
        self.frame_buffer = {}  # camera_id -> [frames]
        self.last_cleanup_time = 0

    def load_config(self, config_path: str) -> bool:
        """
        加载配置文件
        
        Args:
            config_path: 配置文件路径 (YAML格式)
            
        Returns:
            bool: 加载成功返回 True
        """
        try:
            # 解决相对路径问题
            if not os.path.isabs(config_path):
                config_path = os.path.join(os.path.dirname(__file__), '..', config_path)
            
            config_path = os.path.abspath(config_path)
            
            if not os.path.exists(config_path):
                logger.error(f"❌ 配置文件不存在: {config_path}")
                return False
            
            with open(config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f) or {}
            
            self.config_path = config_path
            logger.info(f"✅ 配置文件已加载: {config_path}")
            
            # 打印配置信息
            radar_source_config = self.config.get('radar_source', {})
            mode = radar_source_config.get('mode', 'file')
            logger.info(f"📡 数据源模式: {mode}")
            
            return True
        
        except yaml.YAMLError as e:
            logger.error(f"❌ YAML解析错误: {e}")
            return False
        except Exception as e:
            logger.error(f"❌ 加载配置文件失败: {e}")
            return False

    def initialize(self) -> bool:
        """
        初始化数据源管理器和数据源
        
        根据配置文件创建合适的数据源实例
        
        Returns:
            bool: 初始化成功返回 True
        """
        try:
            if not self.config:
                logger.error("❌ 配置未加载")
                return False
            
            radar_source_config = self.config.get('radar_source', {})
            mode = radar_source_config.get('mode', 'file')
            
            logger.info(f"🔧 初始化数据源: {mode}")
            
            # 创建数据源
            if mode == 'file':
                self.data_source = self._create_file_source(radar_source_config)
            elif mode == 'stream':
                self.data_source = self._create_stream_source(radar_source_config)
            else:
                logger.error(f"❌ 未知的数据源模式: {mode}")
                return False
            
            if not self.data_source:
                logger.error("❌ 创建数据源失败")
                return False
            
            # 应用摄像头过滤
            camera_config = self.config.get('camera_filter', {})
            if camera_config.get('enabled', True):
                camera_id = camera_config.get('camera_id')
                if camera_id:
                    self.data_source.set_camera_filter(camera_id)
                    logger.info(f"📹 应用摄像头过滤: C{camera_id}")
            
            self.is_initialized = True
            logger.info("✅ 数据源管理器初始化完成")
            return True
        
        except Exception as e:
            logger.error(f"❌ 初始化失败: {e}")
            return False

    def _create_file_source(self, config: Dict[str, Any]) -> Optional[FileRadarSource]:
        """创建文件数据源"""
        try:
            file_config = config.get('file', {})
            file_path = file_config.get('path')
            
            if not file_path:
                logger.error("❌ 文件路径未指定")
                return None
            
            source = FileRadarSource()
            if not source.initialize(file_path=file_path):
                return None
            
            logger.info(f"✅ 文件数据源已创建: {file_path}")
            return source
        
        except Exception as e:
            logger.error(f"❌ 创建文件数据源失败: {e}")
            return None

    def _create_stream_source(self, config: Dict[str, Any]) -> Optional[StreamRadarSource]:
        """创建流数据源"""
        try:
            stream_config = config.get('stream', {})
            host = stream_config.get('host')
            port = stream_config.get('port')
            socket_timeout = stream_config.get('socket_timeout', 5.0)
            max_reconnect = stream_config.get('max_reconnect_attempts', 5)
            reconnect_delay = stream_config.get('reconnect_delay', 2.0)
            
            if not host or not port:
                logger.error("❌ 服务器地址或端口未指定")
                return None
            
            source = StreamRadarSource()
            if not source.initialize(
                host=host,
                port=port,
                socket_timeout=socket_timeout,
                max_reconnect_attempts=max_reconnect
            ):
                return None
            
            logger.info(f"✅ 流数据源已创建: {host}:{port}")
            return source
        
        except Exception as e:
            logger.error(f"❌ 创建流数据源失败: {e}")
            return None

    def start(self) -> bool:
        """
        启动数据源
        
        Returns:
            bool: 启动成功返回 True
        """
        if not self.is_initialized:
            logger.error("❌ 数据源未初始化")
            return False
        
        try:
            if not self.data_source.start():
                logger.error("❌ 数据源启动失败")
                return False
            
            self.is_running = True
            logger.info("✅ 数据源已启动")
            return True
        
        except Exception as e:
            logger.error(f"❌ 启动失败: {e}")
            return False

    def stop(self) -> None:
        """停止数据源"""
        if self.data_source:
            self.data_source.stop()
        
        self.is_running = False
        logger.info("✅ 数据源已停止")

    def get_next_frame(self, camera_id: Optional[int] = None, timeout: Optional[float] = None) -> Optional[RadarDataFrame]:
        """
        获取下一帧雷达数据
        
        Args:
            camera_id: 摄像头ID (1, 2, 3) - 如果为None则获取任何摄像头的数据
            timeout: 超时时间（秒）
        
        Returns:
            RadarDataFrame 或 None
        """
        if not self.is_running or not self.data_source:
            return None
        
        try:
            frame = self.data_source.get_next_frame(timeout=timeout)
            
            if frame and camera_id and frame.camera_id != camera_id:
                # 如果指定了摄像头但不匹配，继续获取
                return self.get_next_frame(camera_id=camera_id, timeout=0)
            
            return frame
        
        except Exception as e:
            logger.error(f"❌ 获取帧失败: {e}")
            return None

    def get_all_frames(self) -> List[RadarDataFrame]:
        """
        获取所有可用的雷达数据帧
        
        Returns:
            List[RadarDataFrame]: 所有雷达数据帧
        """
        if not self.data_source:
            return []
        
        try:
            return self.data_source.get_all_frames()
        except Exception as e:
            logger.error(f"❌ 获取所有帧失败: {e}")
            return []

    def is_ready(self) -> bool:
        """检查数据源是否就绪"""
        if not self.is_running or not self.data_source:
            return False
        
        return self.data_source.is_ready()

    def get_stats(self) -> Dict[str, Any]:
        """获取数据源统计信息"""
        if not self.data_source:
            return {}
        
        return self.data_source.get_stats()

    def print_stats(self) -> None:
        """打印统计信息"""
        if not self.data_source:
            logger.warning("⚠️ 无可用的数据源")
            return
        
        stats = self.get_stats()
        
        logger.info("📊 数据源统计信息:")
        for key, value in stats.items():
            if isinstance(value, float):
                logger.info(f"  {key}: {value:.2f}")
            else:
                logger.info(f"  {key}: {value}")

    def set_camera_filter(self, camera_id: Optional[int]) -> None:
        """设置摄像头过滤器"""
        if self.data_source and camera_id:
            self.data_source.set_camera_filter(camera_id)


# =============================================
# 工厂函数 - 快速创建管理器
# =============================================

def create_radar_source_manager(config_path: Optional[str] = None) -> Optional[RadarSourceManager]:
    """
    快速创建并初始化RadarSourceManager
    
    Args:
        config_path: 配置文件路径，如果为None则寻找默认路径
        
    Returns:
        RadarSourceManager 或 None
    """
    manager = RadarSourceManager()
    
    # 寻找配置文件
    if not config_path:
        # 尝试默认路径
        candidates = [
            'config/radar_source_config.yaml',
            '../config/radar_source_config.yaml',
            './config/radar_source_config.yaml',
        ]
        
        for candidate in candidates:
            if os.path.exists(candidate):
                config_path = candidate
                break
    
    if not config_path or not manager.load_config(config_path):
        logger.error("❌ 无法加载配置文件")
        return None
    
    if not manager.initialize():
        logger.error("❌ 无法初始化管理器")
        return None
    
    if not manager.start():
        logger.error("❌ 无法启动数据源")
        return None
    
    return manager

