# -*- coding: utf-8 -*-
"""
author： pony
date: 2025/03/21
action：使用rtsp协议读取视频流
"""

# rtsp_reader.py

import cv2
import threading
import time
import logging
from typing import Generator, Optional, Tuple

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RTSPStreamReader:
    """
    RTSP 视频流读取器，支持多线程缓冲帧，避免 OpenCV 读取延迟
    """

    def __init__(self, rtsp_url: str, buffer_size: int = 10):
        self.rtsp_url = rtsp_url
        self.buffer_size = buffer_size
        self.cap: Optional[cv2.VideoCapture] = None
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.connect_attempts = 0
        self.max_reconnect_attempts = 5

    def connect(self) -> bool:
        """尝试连接 RTSP 流"""
        if self.cap:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.rtsp_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)

        if not self.cap.isOpened():
            logger.error(f"无法打开 RTSP 流: {self.rtsp_url}")
            return False

        logger.info(f"成功连接到 RTSP 流: {self.rtsp_url}")
        return True

    def _reader(self):
        """后台线程：持续读取帧"""
        while self.running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.connect_attempts += 1
                    logger.warning(f"读取帧失败，尝试重连... ({self.connect_attempts}/{self.max_reconnect_attempts})")
                    if self.connect_attempts >= self.max_reconnect_attempts:
                        logger.error("重连次数超限，停止读取")
                        self.running = False
                        break
                    time.sleep(2)
                    self.connect()
                    continue
                else:
                    self.connect_attempts = 0  # 重置重连计数

                with self.lock:
                    self.latest_frame = frame.copy()
            except Exception as e:
                logger.error(f"读取帧时发生异常: {e}")
                time.sleep(1)

    def start(self) -> bool:
        """启动读取线程"""
        if self.running:
            return True

        if not self.connect():
            return False

        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        return True

    def read(self) -> Optional[Tuple[bool, any]]:
        """
        读取最新一帧
        返回: (success: bool, frame: np.ndarray or None)
        """
        with self.lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
            else:
                return False, None

    def stop(self):
        """停止读取"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.cap:
            self.cap.release()
        logger.info("RTSP 流读取已停止")

    def __iter__(self) -> Generator[Tuple[bool, any], None, None]:
        """
        支持 for 循环遍历帧
        示例: for success, frame in reader: ...
        """
        while self.running:
            yield self.read()


# 便捷函数：从配置文件加载并创建读取器
def create_reader_from_config(config_file: str, section: str = "Camera1") -> Optional[RTSPStreamReader]:
    """
    从配置文件创建 RTSP 读取器
    :param config_file: 配置文件路径
    :param section: 配置节名称
    :return: RTSPStreamReader 实例或 None
    """
    import configparser

    config = configparser.ConfigParser()
    try:
        config.read(config_file)
        if section not in config:
            logger.error(f"配置节 '{section}' 不存在")
            return None

        url = config[section]["rtsp_url"]
        enabled = config[section].getboolean("enabled", True)
        if not enabled:
            logger.info(f"摄像头 '{section}' 已禁用")
            return None

        return RTSPStreamReader(url)

    except Exception as e:
        logger.error(f"读取配置文件失败: {e}")
        return None





