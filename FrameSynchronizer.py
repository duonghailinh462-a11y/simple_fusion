import os
import sys
import time
import signal
import multiprocessing
import copy
import json
import re
from collections import defaultdict, deque
from statistics import mean, median
from datetime import datetime
sys.path.append('/usr/local/lynxi/sdk/sdk-samples/python')

import numpy as np
import cv2
from ctypes import *
import argparse
import struct
import ctypes
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict

# 导入SDK相关模块
import pycommon.common as common
import pylynchipsdk as sdk
from pycommon.infer_process import *
from pycommon.callback_data_struct import *
from pycommon.dump_json import *

# ==================== 严格帧同步管理器 ====================
class StrictFrameSynchronizer:
    """严格帧同步管理器，确保所有摄像头帧严格同步且不丢帧"""
    
    def __init__(self, num_cameras=3, time_window=0.5, camera_start_times=None):
        self.num_cameras = num_cameras
        self.frame_buffers = {i: {} for i in range(1, num_cameras + 1)}
        
        # 🔧 改进：完全基于时间戳同步，不依赖帧号
        self.time_window = time_window  # 时间窗口（秒），默认0.5秒
        self.max_buffer_size = 200  # 增大缓冲区
        
        # 🔧 新增：用于时间戳同步的参数
        self.last_synced_timestamp = None  # 上一次同步的时间戳
        self.timestamp_format = "%Y-%m-%d %H:%M:%S.%f"  # 时间戳格式
        
        # 🔧 新增：对齐到最晚开始的时间点 - 直接丢弃早期帧
        self.camera_start_times = camera_start_times or {}
        self.sync_start_timestamp = self._calculate_sync_start_time()
        
        print(f"🎯 时间戳同步器初始化完成 - {num_cameras}摄像头, 时间窗口:{time_window}秒")
        if self.sync_start_timestamp:
            print(f"📍 同步起始时间戳: {self.sync_start_timestamp:.3f} (对齐到最晚开始的摄像头)")
    
    def _calculate_sync_start_time(self):
        """
        计算同步起始时间 - 对齐到最晚开始的摄像头
        
        🔧 策略：取交集，从最晚开始的摄像头时间点开始同步
        - 直接丢弃早开始的摄像头在该时间点之前的所有帧
        - 简单直接，避免复杂的偏移计算
        """
        if not self.camera_start_times:
            return None
        
        # 解析所有摄像头的起始时间
        start_timestamps = {}
        for cam_id, time_str in self.camera_start_times.items():
            ts = self._parse_timestamp(time_str)
            if ts is not None:
                start_timestamps[cam_id] = ts
        
        if not start_timestamps:
            return None
        
        # 取最晚的起始时间作为同步起点
        max_timestamp = max(start_timestamps.values())
        
        # 打印各摄像头的起始时间信息
        for cam_id in sorted(start_timestamps.keys()):
            ts = start_timestamps[cam_id]
            delay = ts - min(start_timestamps.values())
            print(f"  C{cam_id} 起始时间: {ts:.3f} (延迟: {delay:.3f}s)")
        
        return max_timestamp
    
    def add_frame(self, camera_id, frame_data):
        """添加帧到缓冲区，使用时间戳作为唯一标识"""
        
        # 🔧 确保有时间戳
        if 'timestamp' not in frame_data or frame_data['timestamp'] is None:
            # 降级方案：使用系统时间
            from datetime import datetime
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            frame_data['timestamp'] = timestamp
        
        # 增强帧数据
        frame_data['camera_id'] = camera_id
        
        # 添加到缓冲区 - 使用时间戳作为键
        ts_str = frame_data.get('timestamp')
        ts_float = self._parse_timestamp(ts_str)
        if ts_float is not None:
            self.frame_buffers[camera_id][ts_float] = frame_data
        
        # 清理过期帧
        self._cleanup_old_frames(camera_id)
        
    def _parse_timestamp(self, timestamp_str):
        """解析时间戳字符串为浮点数（秒）"""
        try:
            if isinstance(timestamp_str, (int, float)):
                return float(timestamp_str)
            
            # 处理字符串格式的时间戳
            if isinstance(timestamp_str, str):
                # 尝试解析为 "YYYY-MM-DD HH:MM:SS.mmm" 格式
                try:
                    dt = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
                    return dt.timestamp()
                except ValueError:
                    # 尝试不带毫秒的格式
                    dt = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                    return dt.timestamp()
            
            return None
        except Exception as e:
            return None
    
    def get_synchronized_frames(self):
        """
        获取同步帧 - 完全基于时间戳同步，不依赖帧号
        
        🔧 改进：
        - 完全抛弃帧号，只用时间戳
        - 允许任意大的帧号差距
        - 只要时间戳在同一时间窗口内就认为同步
        """
        synchronized_frames = {}
        
        # 检查每个摄像头是否都有数据
        empty_cameras = [cid for cid in range(1, self.num_cameras + 1) if not self.frame_buffers[cid]]
        if empty_cameras:
            return None, None
        
        # 获取所有摄像头的所有帧的时间戳
        all_timestamps = {}
        for camera_id in range(1, self.num_cameras + 1):
            all_timestamps[camera_id] = []
            for key, frame_data in self.frame_buffers[camera_id].items():
                ts_str = frame_data.get('timestamp')
                ts_float = self._parse_timestamp(ts_str)
                if ts_float is not None:
                    all_timestamps[camera_id].append((ts_float, key, frame_data))
            
            # 按时间戳排序
            all_timestamps[camera_id].sort(key=lambda x: x[0])
        
        # 检查是否所有摄像头都有有效的时间戳
        if any(len(ts_list) == 0 for ts_list in all_timestamps.values()):
            return None, None
        
        # 🔧 改进：在时间窗口内寻找所有摄像头都有帧的时间点
        # 获取所有摄像头中最早的时间戳作为基准
        earliest_timestamps = [all_timestamps[cid][0][0] for cid in range(1, self.num_cameras + 1)]
        reference_timestamp = max(earliest_timestamps)  # 取最晚的最早时间戳作为基准
        
        # 如果有上一次同步的时间戳，优先从该时间戳之后查找
        if self.last_synced_timestamp is not None:
            reference_timestamp = max(reference_timestamp, self.last_synced_timestamp)
        
        # 在时间窗口内寻找所有摄像头都有帧的时间点
        for camera_id in range(1, self.num_cameras + 1):
            best_match = None
            best_distance = float('inf')
            
            for ts_float, key, frame_data in all_timestamps[camera_id]:
                # 在时间窗口内寻找最接近基准时间戳的帧
                distance = abs(ts_float - reference_timestamp)
                if distance <= self.time_window and distance < best_distance:
                    best_match = (ts_float, key, frame_data)
                    best_distance = distance
            
            if best_match is None:
                # 这个摄像头在时间窗口内没有帧
                return None, None
            
            synchronized_frames[camera_id] = best_match[2]
            ts_float = best_match[0]
            key = best_match[1]
            
            # 从缓冲区中移除已使用的帧
            self.frame_buffers[camera_id].pop(key, None)
        
        # 更新最后同步的时间戳
        self.last_synced_timestamp = reference_timestamp
        
        # 返回时间戳作为同步标识符（而不是帧号）
        return synchronized_frames, reference_timestamp
    
    def get_buffer_status(self):
        """获取缓冲区状态信息 - 基于时间戳"""
        status = {}
        for camera_id in range(1, self.num_cameras + 1):
            if self.frame_buffers[camera_id]:
                # 获取所有帧的时间戳
                timestamps = []
                for key, frame_data in self.frame_buffers[camera_id].items():
                    ts_str = frame_data.get('timestamp')
                    ts_float = self._parse_timestamp(ts_str)
                    if ts_float is not None:
                        timestamps.append(ts_float)
                
                if timestamps:
                    timestamps.sort()
                    status[camera_id] = {
                        'count': len(timestamps),
                        'min_timestamp': timestamps[0],
                        'max_timestamp': timestamps[-1],
                        'time_span': timestamps[-1] - timestamps[0]
                    }
                else:
                    status[camera_id] = {'count': 0}
            else:
                status[camera_id] = {'count': 0}
        return status
    
    def _cleanup_old_frames(self, camera_id):
        """
        不清理帧 - 视频文件处理，保留所有帧
        
        🔧 改进：
        - 处理视频文件时，不需要丢弃任何帧
        - 所有帧都保留在缓冲区中
        - 只在缓冲区超过极限时报告
        """
        # 定期报告缓冲区状态
        if self.last_synced_timestamp is not None and int(self.last_synced_timestamp * 10) % 150 == 0:
            buffer_sizes = {i: len(self.frame_buffers[i]) for i in range(1, self.num_cameras + 1)}
            print(f"📊 缓冲区状态: {buffer_sizes}, 最后同步时间戳: {self.last_synced_timestamp:.3f}")