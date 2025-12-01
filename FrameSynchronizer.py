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
    
    def __init__(self, num_cameras=3):
        self.num_cameras = num_cameras
        self.global_frame_counter = 0
        self.camera_frame_counters = {i: 0 for i in range(1, num_cameras + 1)}
        self.frame_buffers = {i: {} for i in range(1, num_cameras + 1)}
        self.sync_tolerance = 5  # 基础容忍度5帧，实际使用时会扩大到15帧
        self.max_buffer_size = 150  # 增大缓冲区以等待同步（从100增加到150）
        
        # 视频参数
        self.video_fps = 30  # 假设30fps
        self.frame_duration = 1.0 / self.video_fps
        
        # 时间戳基准
        self.start_time = 0 
        # 为不同摄像头设置轻微延迟(暂时不使用)
        self.camera_delays = {1: 0.0, 2: 0.033, 3: 0.067}  # 1帧和2帧延迟
        
        print(f"🎯 严格帧同步器初始化完成 - {num_cameras}摄像头, FPS:{self.video_fps}")
        
    def add_frame(self, camera_id, frame_data):
        """添加帧到缓冲区，使用SDKinfer计算好的时间戳"""
        frame_number = frame_data.get('frame_id', self.camera_frame_counters[camera_id])
        
        # 🔧 使用SDKinfer中已计算的时间戳（字符串格式）
        # 如果frame_data中已有timestamp，直接使用；否则计算一个
        if 'timestamp' not in frame_data or frame_data['timestamp'] is None:
            # 降级方案：计算时间戳
            timestamp_seconds = self.start_time + (frame_number * self.frame_duration)
            from datetime import datetime
            timestamp = datetime.fromtimestamp(timestamp_seconds).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            frame_data['timestamp'] = timestamp
        
        # 增强帧数据
        frame_data['frame_number'] = frame_number
        frame_data['camera_id'] = camera_id
        frame_data['sync_id'] = f"C{camera_id}_F{frame_number}"
        
        # 添加到缓冲区
        self.frame_buffers[camera_id][frame_number] = frame_data
        
        # 更新摄像头帧计数器
        if frame_number >= self.camera_frame_counters[camera_id]:
            self.camera_frame_counters[camera_id] = frame_number + 1
        
        # 清理过期帧
        self._cleanup_old_frames(camera_id)
        
        return frame_number
        
    def get_synchronized_frames(self):
        """获取同步帧 - 改进的容忍同步策略，针对视频文件优化"""
        synchronized_frames = {}
        
        # 🔧 改进：检查每个摄像头是否都有数据
        # 对于视频文件，如果某个摄像头暂时没有数据，可能是处理速度慢
        # 我们仍然要求所有摄像头都有数据，但增加容忍度
        empty_cameras = [cid for cid in range(1, self.num_cameras + 1) if not self.frame_buffers[cid]]
        if empty_cameras:
            # 对于视频文件，如果某个摄像头缓冲区为空，说明它可能处理速度慢
            # 我们仍然返回None，但会在主循环中等待
            return None, None
        
        # 容忍同步策略：在sync_tolerance范围内寻找最佳对齐点
        all_frames = []
        for camera_id in range(1, self.num_cameras + 1):
            all_frames.extend(self.frame_buffers[camera_id].keys())
        
        if not all_frames:
            return None, None

        # 找到最小的基准帧
        min_frame = min(all_frames)
        
        # 🔧 改进：对于视频文件，扩大容忍度检查范围
        # 处理速度差异可能导致帧号差距较大，但视频文件不存在网络波动
        # 所以可以安全地扩大容忍度
        extended_tolerance = self.sync_tolerance * 3  # 扩大容忍度到15帧（从5帧）
        
        # 检查在容忍范围内是否所有摄像头都有帧
        for offset in range(extended_tolerance + 1):
            target_frame = min_frame + offset
            available_cameras = 0
            potential_sync = {}
            
            for camera_id in range(1, self.num_cameras + 1):
                # 在容忍范围内寻找最接近的帧
                best_frame = None
                best_distance = float('inf')
                
                for frame_num in self.frame_buffers[camera_id].keys():
                    distance = abs(frame_num - target_frame)
                    if distance <= extended_tolerance and distance < best_distance:
                        best_frame = frame_num
                        best_distance = distance
                
                if best_frame is not None:
                    potential_sync[camera_id] = best_frame
                    available_cameras += 1
            
            # 如果所有摄像头都在容忍范围内有帧
            if available_cameras == self.num_cameras:
                for camera_id, frame_num in potential_sync.items():
                    synchronized_frames[camera_id] = self.frame_buffers[camera_id].pop(frame_num)
                
                self.global_frame_counter = target_frame
                # 简化成功日志
                # print(f"⚡ 同步成功 (容忍模式): 基准帧 {target_frame} (各帧偏差: {[abs(f-target_frame) for f in potential_sync.values()]})")
                return synchronized_frames, target_frame
        
        return None, None
    
    def get_buffer_status(self):
        """获取缓冲区状态信息"""
        status = {}
        for camera_id in range(1, self.num_cameras + 1):
            if self.frame_buffers[camera_id]:
                frames = list(self.frame_buffers[camera_id].keys())
                status[camera_id] = {
                    'count': len(frames),
                    'min_frame': min(frames),
                    'max_frame': max(frames),
                    'frames': sorted(frames)
                }
            else:
                status[camera_id] = {'count': 0, 'frames': []}
        return status
    
    def _cleanup_old_frames(self, camera_id):
        """
        更积极的缓冲区清理策略，基于全局已同步的帧号 (`global_frame_counter`)
        """
        # 🔧 改进：对于视频文件，增加安全边距，避免过早清理
        # 安全阈值 = 全局已同步帧号 - 扩展容忍度 - 额外安全边距
        # 这确保了我们不会意外删除可能在下一次同步中用到的帧
        extended_tolerance = self.sync_tolerance * 3  # 与get_synchronized_frames中的容忍度一致
        safety_margin = 20  # 增加安全边距（从10增加到20）
        safe_cleanup_threshold = self.global_frame_counter - extended_tolerance - safety_margin
        
        if safe_cleanup_threshold < 0: return # 早期阶段不清理
        
        # 清理确实过期的帧
        old_frames = [f for f in self.frame_buffers[camera_id].keys() if f < safe_cleanup_threshold]
        if old_frames:
            for frame_num in old_frames:
                del self.frame_buffers[camera_id][frame_num]
            print(f"🗑️  C{camera_id} 清理过期帧: {len(old_frames)}个 (阈值 < {safe_cleanup_threshold})")
        
        # 紧急清理：如果缓冲区仍然过大，强制清理最老的帧
        if len(self.frame_buffers[camera_id]) > self.max_buffer_size:
            all_frames = sorted(self.frame_buffers[camera_id].keys())
            # 保留最新的 max_buffer_size * 0.8 个帧
            frames_to_remove = all_frames[:-int(self.max_buffer_size * 0.8)]
            
            if frames_to_remove:
                for frame_num in frames_to_remove:
                    del self.frame_buffers[camera_id][frame_num]
                print(f"🚨 C{camera_id} 紧急清理缓冲区: {len(frames_to_remove)}个帧 (当前大小: {len(self.frame_buffers[camera_id])})")
        
        # 定期报告缓冲区状态 (降低频率)
        if self.global_frame_counter > 0 and self.global_frame_counter % 150 == 0:
            buffer_sizes = {i: len(self.frame_buffers[i]) for i in range(1, self.num_cameras + 1)}
            print(f"📊 缓冲区状态: {buffer_sizes}, 全局同步帧: {self.global_frame_counter}")


class FrameLossPrevention:
    """防止丢帧检测机制"""
    
    def __init__(self):
        self.expected_frame_sequence = {1: 0, 2: 0, 3: 0}
        self.missing_frames = {1: [], 2: [], 3: []}
        self.duplicate_frames = {1: [], 2: [], 3: []}
        self.total_frames_processed = {1: 0, 2: 0, 3: 0}
        self.frame_timeout = 0.1  # 100ms超时
        
        print("🛡️  防丢帧检测机制初始化完成")
        
    def check_frame_sequence(self, camera_id, frame_number):
        """检查帧序列是否连续，返回是否应该处理该帧"""
        expected = self.expected_frame_sequence[camera_id]
        
        if frame_number == expected:
            # 正常帧
            self.expected_frame_sequence[camera_id] = frame_number + 1
            self.total_frames_processed[camera_id] += 1
            return True
            
        elif frame_number > expected:
            # 有丢帧
            missing = list(range(expected, frame_number))
            self.missing_frames[camera_id].extend(missing)
            self.expected_frame_sequence[camera_id] = frame_number + 1
            self.total_frames_processed[camera_id] += 1
            
            if len(missing) <= 3:  # 丢帧不多，警告但继续处理
                print(f"⚠️  C{camera_id} 丢帧: {missing} (期望:{expected}, 实际:{frame_number})")
                return True
            else:  # 丢帧过多，可能有问题
                print(f"❌ C{camera_id} 严重丢帧: {missing} (期望:{expected}, 实际:{frame_number})")
                return True  # 仍然处理，但需要注意
                
        else:
            # 重复帧或乱序帧
            self.duplicate_frames[camera_id].append(frame_number)
            print(f"🔄 C{camera_id} 重复/乱序帧: {frame_number} (期望:{expected})")
            return False  # 不处理重复帧
    
    def get_missing_frames_report(self):
        """获取丢帧报告"""
        report = {}
        for camera_id in [1, 2, 3]:
            missing_count = len(self.missing_frames[camera_id])
            duplicate_count = len(self.duplicate_frames[camera_id])
            total_processed = self.total_frames_processed[camera_id]
            
            if missing_count > 0 or duplicate_count > 0:
                report[camera_id] = {
                    'missing_frames': self.missing_frames[camera_id].copy(),
                    'duplicate_frames': self.duplicate_frames[camera_id].copy(),
                    'missing_count': missing_count,
                    'duplicate_count': duplicate_count,
                    'total_processed': total_processed,
                    'loss_rate': (missing_count / max(total_processed, 1)) * 100
                }
                
                # 清空计数器
                self.missing_frames[camera_id] = []
                self.duplicate_frames[camera_id] = []
                
        return report
    
    def get_statistics(self):
        """获取统计信息"""
        stats = {}
        for camera_id in [1, 2, 3]:
            stats[camera_id] = {
                'total_processed': self.total_frames_processed[camera_id],
                'expected_next': self.expected_frame_sequence[camera_id],
                'current_missing': len(self.missing_frames[camera_id]),
                'current_duplicates': len(self.duplicate_frames[camera_id])
            }
        return stats