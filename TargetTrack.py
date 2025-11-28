import os
import sys
import time
import multiprocessing
import copy
import json
from collections import defaultdict, deque
from statistics import mean, median
sys.path.append('/usr/local/lynxi/sdk/sdk-samples/python')

import numpy as np
import cv2
from ctypes import *
import ctypes
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict
from Basic import Config, DetectionUtils
from ByteTrack.optimized_byte_tracker import OptimizedBYTETracker as BYTETracker


# --- 目标跟踪缓冲区和融合系统类  ---

class TargetBuffer:
    def __init__(self, time_window: int = Config.TIME_WINDOW):
        self.buffer = deque(maxlen=500)
        self.time_window = time_window
        self.frame_counter = 0
        self.active_targets = {}

    def add_target(self, local_id: int, center_point: Tuple[float, float], 
                  class_name: str, confidence: float):
        target_info = {
            'local_id': local_id,
            'center_point': center_point,
            'class_name': class_name,
            'confidence': confidence,
            'timestamp': self.frame_counter
        }
        self.active_targets[local_id] = target_info
        self.buffer.append(target_info)
        self._cleanup_old_targets()

    def _cleanup_old_targets(self):
        cutoff_time = self.frame_counter - self.time_window
        self.buffer = deque([t for t in self.buffer if t['timestamp'] > cutoff_time], 
                          maxlen=self.buffer.maxlen)
        
        self.active_targets = {k: v for k, v in self.active_targets.items() 
                             if v['timestamp'] > cutoff_time}

    def find_matching_targets(self, class_name: str, 
                            tolerance_frames: int = Config.TOLERANCE_FRAMES) -> List[dict]:
        cutoff_time = self.frame_counter - tolerance_frames
        matches = [target for target in self.buffer 
                  if (target['timestamp'] >= cutoff_time and 
                      DetectionUtils.is_class_compatible(target['class_name'], class_name))]
        return sorted(matches, key=lambda x: x['timestamp'], reverse=True)

    def next_frame(self):
        self.frame_counter += 1

# --- 新增：数据结构 ---
@dataclass
class GlobalTarget:
    """全局目标数据类"""
    global_id: int
    camera_id: int
    local_id: int
    class_name: str
    bev_trajectory: List[Tuple[float, float]]
    pixel_trajectory: List[Tuple[int, int]]
    last_seen_frame: int
    is_active: bool
    fusion_alpha: float = 0.2
    is_in_fusion_zone: bool = False
    confidence_history: List[float] = None
    fusion_entry_frame: int = -1
    
    def __post_init__(self):
        if self.confidence_history is None:
            self.confidence_history = []

@dataclass
class LocalTarget:
    """本地目标数据类"""
    local_id: int
    camera_id: int
    class_name: str
    current_bev_pos: Tuple[float, float]
    current_pixel_pos: Tuple[int, int]
    confidence: float
    is_in_fusion_area: bool
    matched_global_id: Optional[int] = None
    detection_box: List[int] = None
    fusion_entry_frame: int = -1
    
    def __post_init__(self):
        if self.detection_box is None:
            self.detection_box = []

@dataclass
class FusionEntry:
    """融合缓冲区条目 - 用于按对队列匹配"""
    seq_id: int  # 在 (from_cam, to_cam) 对中的序列号
    from_cam: int  # 源摄像头ID
    local_id: int  # 本地track ID
    first_seen_frame: int  # 首次出现帧号
    entry_pos: Tuple[float, float]  # BEV位置
    class_name: str  # 类别名称
    confidence: float  # 置信度
    eligible_targets: Set[int]  # 可能的目标摄像头集合 {1}, {3}, 或 {1,3}
    reserved_for: Optional[int] = None  # 为哪个摄像头保留（None表示未保留）
    reservation_ts: Optional[int] = None  # Reservation时间戳（帧号）
    matched: bool = False  # 是否已匹配
    matched_global_id: Optional[int] = None  # 匹配到的全局ID
    created_ts: int = 0  # 创建时间戳（帧号）
    
    def __post_init__(self):
        if self.created_ts == 0:
            self.created_ts = self.first_seen_frame
    
    def get_entry_id(self) -> str:
        """生成唯一entry ID"""
        return f"C{self.from_cam}_L{self.local_id}_S{self.seq_id}"

class LocalTrackBuffer:
    """本地轨迹缓冲区 - 维护每个摄像头的track轨迹历史"""
    def __init__(self, max_history: int = 30):
        self.max_history = max_history
        self.tracks: Dict[int, Dict[int, List[Tuple[float, float]]]] = defaultdict(lambda: defaultdict(list))
        self.pixel_tracks: Dict[int, Dict[int, List[Tuple[int, int]]]] = defaultdict(lambda: defaultdict(list))
        self.assigned_global_ids: Dict[int, Dict[int, int]] = defaultdict(dict)
        self.track_classes: Dict[int, Dict[int, str]] = defaultdict(dict)
        # 🔧 新增：跟踪每个轨迹的最后更新时间（帧号）
        self.last_update_frame: Dict[int, Dict[int, int]] = defaultdict(dict)
    
    def __len__(self):
        """返回track总数"""
        total = 0
        for camera_id in self.tracks:
            total += len(self.tracks[camera_id])
        return total
    
    def update_track(self, camera_id: int, local_id: int, bev_pos: Tuple[float, float], 
                    pixel_pos: Tuple[int, int], class_name: str, current_frame: int = None):
        """更新本地轨迹"""
        self.tracks[camera_id][local_id].append(bev_pos)
        self.pixel_tracks[camera_id][local_id].append(pixel_pos)
        self.track_classes[camera_id][local_id] = class_name
        
        # 🔧 新增：记录轨迹最后更新时间（帧号）
        if current_frame is not None:
            self.last_update_frame[camera_id][local_id] = current_frame
        
        if len(self.tracks[camera_id][local_id]) > self.max_history:
            self.tracks[camera_id][local_id].pop(0)
            self.pixel_tracks[camera_id][local_id].pop(0)
    
    def get_track_history(self, camera_id: int, local_id: int) -> List[Tuple[float, float]]:
        """获取轨迹历史"""
        return self.tracks[camera_id].get(local_id, [])
    
    def get_pixel_track_history(self, camera_id: int, local_id: int) -> List[Tuple[int, int]]:
        """获取像素轨迹历史"""
        return self.pixel_tracks[camera_id].get(local_id, [])
    
    def has_global_id(self, camera_id: int, local_id: int) -> bool:
        """检查是否已分配global_id"""
        return local_id in self.assigned_global_ids[camera_id]
    
    def get_global_id(self, camera_id: int, local_id: int) -> Optional[int]:
        """获取已分配的global_id"""
        return self.assigned_global_ids[camera_id].get(local_id)
    
    def assign_global_id(self, camera_id: int, local_id: int, global_id: int):
        """记录local_id到global_id的映射"""
        self.assigned_global_ids[camera_id][local_id] = global_id
    
    def cleanup_track(self, camera_id: int, local_id: int):
        """清理过期的轨迹"""
        if local_id in self.tracks[camera_id]:
            del self.tracks[camera_id][local_id]
        if local_id in self.pixel_tracks[camera_id]:
            del self.pixel_tracks[camera_id][local_id]
        if local_id in self.assigned_global_ids[camera_id]:
            del self.assigned_global_ids[camera_id][local_id]
        if local_id in self.track_classes[camera_id]:
            del self.track_classes[camera_id][local_id]
        # 🔧 新增：清理更新时间记录
        if local_id in self.last_update_frame[camera_id]:
            del self.last_update_frame[camera_id][local_id]

def analyze_trajectory_for_global_assignment(pixel_track_history: List[Tuple[int, int]], 
                                            min_trajectory_length: int = 3,
                                            pixel_bottom_threshold: float = 700,
                                            pixel_top_threshold: float = 1080) -> bool:
    """
    分析轨迹是否值得分配global_id
    基于像素Y值判断是否在底部区域
    """
    if len(pixel_track_history) < min_trajectory_length:
        return False
    
    start_pos = pixel_track_history[0]
    start_y = start_pos[1]
    
    if pixel_bottom_threshold <= start_y <= pixel_top_threshold:
        return True
    
    return False
