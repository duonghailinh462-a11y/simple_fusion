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

# ==================== 帧号同步管理器 ====================
class StrictFrameSynchronizer:
    """帧号同步管理器，基于帧号同步确保所有摄像头帧严格同步"""
    
    def __init__(self, num_cameras=3, fps=25, start_timestamp=None):
        self.num_cameras = num_cameras
        self.frame_buffers = {i: {} for i in range(1, num_cameras + 1)}
        
        # 🔧 改进：基于帧号同步，而不是时间戳
        self.fps = fps  # 帧率（fps）
        self.start_timestamp = start_timestamp  # 起始时间戳（秒级Unix时间戳）
        self.max_buffer_size = 500  # 增大缓冲区以容纳帧号差异
        
        # 🔧 新增：用于帧号同步的参数
        self.last_synced_frame_id = None  # 上一次同步的帧号
        
        print(f"🎯 帧号同步器初始化完成 - {num_cameras}摄像头, FPS:{fps}")
        if self.start_timestamp:
            print(f"📍 起始时间戳: {self.start_timestamp:.3f}")
    
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
        """添加帧到缓冲区，使用帧号作为唯一标识"""
        
        # 🔧 确保有帧号
        if 'frame_id' not in frame_data or frame_data['frame_id'] is None:
            print(f"⚠️  Camera{camera_id} 帧数据缺少frame_id字段")
            return
        
        # 增强帧数据
        frame_data['camera_id'] = camera_id
        
        # 添加到缓冲区 - 使用帧号作为键
        frame_id = frame_data.get('frame_id')
        self.frame_buffers[camera_id][frame_id] = frame_data
        
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
        获取同步帧 - 基于帧号同步
        
        🔧 改进：
        - 基于帧号同步，三路帧号相同时认为同步
        - 时间戳由 start_timestamp + frame_id/fps 计算
        - 简单高效，不需要时间窗口容差
        """
        synchronized_frames = {}
        
        # 检查每个摄像头是否都有数据
        empty_cameras = [cid for cid in range(1, self.num_cameras + 1) if not self.frame_buffers[cid]]
        if empty_cameras:
            return None, None
        
        # 获取所有摄像头的所有帧号
        all_frame_ids = {}
        for camera_id in range(1, self.num_cameras + 1):
            all_frame_ids[camera_id] = sorted(self.frame_buffers[camera_id].keys())
        
        # 检查是否所有摄像头都有帧
        if any(len(fn_list) == 0 for fn_list in all_frame_ids.values()):
            return None, None
        
        # 🔧 改进：寻找三路都有的帧号
        # 获取所有摄像头中最小的帧号作为基准
        min_frame_ids = [all_frame_ids[cid][0] for cid in range(1, self.num_cameras + 1)]
        reference_frame_id = max(min_frame_ids)  # 取最大的最小帧号作为基准
        
        # 如果有上一次同步的帧号，优先从该帧号之后查找
        if self.last_synced_frame_id is not None:
            reference_frame_id = max(reference_frame_id, self.last_synced_frame_id + 1)
        
        # 检查所有摄像头是否都有该帧号
        for camera_id in range(1, self.num_cameras + 1):
            if reference_frame_id not in self.frame_buffers[camera_id]:
                # 这个摄像头没有该帧号，返回None
                return None, None
            
            synchronized_frames[camera_id] = self.frame_buffers[camera_id][reference_frame_id]
            
            # 从缓冲区中移除已使用的帧（以及之前的帧）
            frames_to_remove = [fn for fn in self.frame_buffers[camera_id].keys() if fn <= reference_frame_id]
            for fn in frames_to_remove:
                self.frame_buffers[camera_id].pop(fn, None)
        
        # 更新最后同步的帧号
        self.last_synced_frame_id = reference_frame_id
        
        # 计算同步时间戳：start_timestamp + frame_id/fps
        if self.start_timestamp is not None and self.fps > 0:
            sync_timestamp = self.start_timestamp + (reference_frame_id / self.fps)
        else:
            sync_timestamp = reference_frame_id  # 降级方案：直接用帧号
        
        # 返回同步帧和时间戳
        return synchronized_frames, sync_timestamp
    
    def get_buffer_status(self):
        """获取缓冲区状态信息 - 基于帧号"""
        status = {}
        for camera_id in range(1, self.num_cameras + 1):
            if self.frame_buffers[camera_id]:
                # 获取所有帧号
                frame_ids = sorted(self.frame_buffers[camera_id].keys())
                
                if frame_ids:
                    status[camera_id] = {
                        'count': len(frame_ids),
                        'min_frame_id': frame_ids[0],
                        'max_frame_id': frame_ids[-1],
                        'frame_span': frame_ids[-1] - frame_ids[0]
                    }
                else:
                    status[camera_id] = {'count': 0}
            else:
                status[camera_id] = {'count': 0}
        return status
    
    def _cleanup_old_frames(self, camera_id):
        """
        清理过期帧 - 视频文件处理
        
        🔧 改进：
        - 基于帧号清理
        - 保留最近的帧，避免缓冲区过大
        - 定期报告缓冲区状态
        """
        # 如果缓冲区超过最大大小，清理最旧的帧
        if len(self.frame_buffers[camera_id]) > self.max_buffer_size:
            frame_ids = sorted(self.frame_buffers[camera_id].keys())
            # 保留最后300帧，删除更旧的帧
            frames_to_remove = frame_ids[:-300]
            for fn in frames_to_remove:
                self.frame_buffers[camera_id].pop(fn, None)
        
        # 定期报告缓冲区状态
        if self.last_synced_frame_id is not None and self.last_synced_frame_id % 150 == 0:
            buffer_sizes = {i: len(self.frame_buffers[i]) for i in range(1, self.num_cameras + 1)}
            print(f"📊 缓冲区状态: {buffer_sizes}, 最后同步帧号: {self.last_synced_frame_id}")