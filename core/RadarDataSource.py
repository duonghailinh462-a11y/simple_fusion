#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达数据源抽象层 - 支持JSONL文件和UDP实时接收
统一接口：无论数据来自文件还是网络，都通过 get_latest_data() 获取
"""

import threading
import socket
import time
import json
import logging
from collections import deque
from abc import ABC, abstractmethod
from typing import Optional, Tuple, List

logger = logging.getLogger(__name__)

# 尝试导入 protobuf (确保你已编译 radar.proto)
try:
    import sys
    import os
    # 添加 radar_read 目录到 sys.path
    radar_read_path = os.path.join(os.path.dirname(__file__), '..', 'radar', 'radar_read')
    if radar_read_path not in sys.path:
        sys.path.insert(0, radar_read_path)
    import radar_pb2
    PROTO_AVAILABLE = True
except ImportError as e:
    PROTO_AVAILABLE = False
    logger.warning(f"无法导入 radar_pb2: {e}，实时模式将不可用")


class BaseRadarSource(ABC):
    """雷达数据源基类 (策略接口)"""
    
    @abstractmethod
    def start(self):
        """启动数据源 (打开文件或绑定端口)"""
        pass

    @abstractmethod
    def get_latest_data(self, current_time=None):
        """
        获取最新一帧雷达数据 (零阶保持)
        
        Args:
            current_time: 当前参考时间（可选，用于文件模式的快进）
        
        Returns:
            (timestamp, [RadarObject, ...]) 或 None
        """
        pass

    @abstractmethod
    def stop(self):
        """停止数据源"""
        pass


class FileRadarSource(BaseRadarSource):
    """测试模式: 读取JSONL文件 (封装 RadarDataLoader)"""
    
    def __init__(self, file_path):
        """
        初始化文件数据源
        
        Args:
            file_path: JSONL文件路径
        """
        from core.RadarVisionFusion import RadarDataLoader
        self.loader = RadarDataLoader(file_path)
        self.stream = None
        self.latest_frame = (0, [])
        self.pending_frame = None  # 待处理的帧（用于快进逻辑）
        self.running = False
        logger.info(f"✅ FileRadarSource 初始化成功: {file_path}")

    def start(self):
        """启动文件流"""
        self.stream = self.loader.stream_radar_data()
        self.running = True
        # 预读第一包数据
        try:
            self.latest_frame = next(self.stream)
            logger.info(f"✅ 预读雷达数据成功: ts={self.latest_frame[0]:.3f}, objs={len(self.latest_frame[1])}")
        except StopIteration:
            logger.warning("⚠️ 雷达数据流为空")
            self.latest_frame = (0, [])

    def get_latest_data(self, current_time=None):
        """
        获取最新雷达数据，支持快进逻辑
        
        这实现了零阶保持（Zero-Order Hold）的快进逻辑：
        - 如果雷达数据太老，丢弃并继续读下一帧
        - 如果雷达数据太新，保留给下一次循环
        - 如果时间匹配，返回该帧
        
        Args:
            current_time: 当前视觉基准时间戳
        
        Returns:
            (timestamp, radar_objs) 或 None
        """
        if not self.running or self.stream is None:
            return self.latest_frame if self.latest_frame[1] else None
        
        # 如果没有指定参考时间，直接返回最新帧
        if current_time is None:
            return self.latest_frame if self.latest_frame[1] else None
        
        # 快进逻辑：根据参考时间调整雷达数据
        while True:
            # 1. 确保手里有一帧数据待处理
            if self.pending_frame is None:
                try:
                    self.pending_frame = next(self.stream)
                except StopIteration:
                    logger.debug("雷达数据流已结束")
                    break
            
            # 2. 比较时间戳
            r_ts, r_objs = self.pending_frame
            
            # 情况 A: 雷达太老了 (比视觉慢了超过 0.1s) -> 丢弃，继续读下一帧 (快进)
            if r_ts < current_time - 0.1:
                self.latest_frame = self.pending_frame  # 记录下这帧作为"最新过去帧"
                self.pending_frame = None  # 清空手里的，准备读下一个
                continue
            
            # 情况 B: 雷达跑太快了 (比视觉还早) -> 停！保留这帧给下一次循环用
            elif r_ts > current_time:
                # 此时 self.latest_frame 保持的是最接近当前时刻的"过去帧"
                break
            
            # 情况 C: 正好撞上 (在误差范围内) -> 完美
            else:
                self.latest_frame = self.pending_frame
                self.pending_frame = None  # 用掉了，清空
                break
        
        return self.latest_frame if self.latest_frame[1] else None

    def stop(self):
        """停止文件流"""
        self.running = False


class RealtimeRadarSource(BaseRadarSource):
    """工程模式: UDP接收 + Proto解码"""
    
    def __init__(self, port=12400):
        """
        初始化实时雷达数据源
        
        Args:
            port: UDP监听端口
        """
        self.port = port
        self.sock = None
        self.running = False
        self.thread = None
        # 只保留最新一帧，自动丢弃旧数据
        self.buffer = deque(maxlen=1)
        logger.info(f"✅ RealtimeRadarSource 初始化成功: port={port}")

    def start(self):
        """启动UDP接收"""
        if not PROTO_AVAILABLE:
            logger.error("无法导入 radar_pb2，请先编译 proto 文件！")
            return
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 设置socket选项以允许地址重用
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 现场可能需要绑定特定IP，或者 0.0.0.0
            self.sock.bind(('0.0.0.0', self.port))
            self.running = True
            self.thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.thread.start()
            logger.info(f"✅ 实时雷达接收已启动，监听端口: {self.port}")
        except Exception as e:
            logger.error(f"❌ 启动UDP接收失败: {e}")
            self.running = False

    def _receive_loop(self):
        """后台接收线程 (参考 server_multi.py)"""
        while self.running:
            try:
                # 接收数据
                data, addr = self.sock.recvfrom(65535)
                
                # Proto 解码
                radar_frame = radar_pb2.ObjLocus()
                radar_frame.ParseFromString(data)
                
                # 转换为系统的 RadarObject 格式
                radar_objs = []
                for locus in radar_frame.locusList:
                    radar_obj = self._convert_proto_to_object(locus, addr[0])
                    radar_objs.append(radar_obj)
                
                # 存入缓冲区，使用系统当前时间
                timestamp = time.time()
                self.buffer.append((timestamp, radar_objs))
                
            except Exception as e:
                logger.warning(f"雷达接收异常: {e}")

    def _convert_proto_to_object(self, locus, source_ip):
        """
        将Protobuf对象转换为内部RadarObject
        
        Args:
            locus: Protobuf Locus 对象
            source_ip: 数据源IP地址
        
        Returns:
            RadarObject 实例
        """
        from core.RadarVisionFusion import RadarObject
        
        # 根据 radar.proto 的字段定义进行转换
        radar_obj = RadarObject(
            id=locus.id,
            latitude=locus.latitude,
            longitude=locus.longitude,
            speed=locus.speed,
            azimuth=locus.azimuth,
            lane=locus.lane if locus.lane > 0 else None,  # lane 字段存在
            timestamp_str=locus.time,  # 使用proto中的时间戳
            source_ip=source_ip  # 记录数据源IP
        )
        
        return radar_obj

    def get_latest_data(self, current_time=None):
        """
        获取最新的雷达数据
        
        Args:
            current_time: 忽略（实时模式不需要快进）
        
        Returns:
            (timestamp, radar_objs) 或 None
        """
        if len(self.buffer) > 0:
            return self.buffer[-1]
        return None

    def stop(self):
        """停止UDP接收"""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass


def create_radar_source(config) -> Optional[BaseRadarSource]:
    """
    工厂函数：根据配置创建对应的雷达数据源
    
    Args:
        config: SystemConfig 实例
    
    Returns:
        BaseRadarSource 的子类实例
    """
    if config.mode == "PROD":
        logger.info(f"🔧 创建实时雷达数据源 (PROD模式) - 端口: {config.radar_port}")
        return RealtimeRadarSource(port=config.radar_port)
    else:
        logger.info(f"🔧 创建文件雷达数据源 (TEST模式) - 文件: {config.jsonl_file}")
        return FileRadarSource(file_path=config.jsonl_file)
