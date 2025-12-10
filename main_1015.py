#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDK版多摄像头融合系统 - 重构版
职责分离: 
1. 子进程 (yolov5_SDK): 仅负责视频读取、SDK推理、结果入队列 
2. 主进程 (main): 负责跟踪(BYTETracker)、区域过滤、跨摄像头融合、帧同步 
"""
import threading
from queue import Queue
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
from ByteTrack.optimized_byte_tracker import OptimizedBYTETracker as BYTETracker

# 导入RTSP和MQTT相关模块 (新增)
try:
    from rtsp_reader import RTSPStreamReader
    from mqtt_publisher import MqttPublisher  
    from config_reader import ConfigReader
    RTSP_MQTT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  无法导入RTSP/MQTT模块: {e}, 将使用原有的本地视频模式")
    RTSP_MQTT_AVAILABLE = False
# 创建共享布尔值用于停止运行线程
cancel_flag = multiprocessing.Value('b', False)
# --- 异步JSON保存器类 (在 CrossCameraFusion 类之前添加) ---
class AsyncJsonSaver:
    """异步JSON保存器，使用后台线程避免阻塞主线程"""
    
    def __init__(self, num_workers=1):
        self.save_queue = Queue(maxsize=10)  # 限制队列大小防止内存溢出
        self.workers = []
        self.stop_event = threading.Event()
        
        # 启动后台工作线程
        for i in range(num_workers):
            worker = threading.Thread(target=self._worker_loop, daemon=True, name=f"JsonSaver-{i}")
            worker.start()
            self.workers.append(worker)
    
    def _worker_loop(self):
        """后台工作线程循环"""
        while not self.stop_event.is_set():
            try:
                # 从队列获取任务，超时防止死锁
                task = self.save_queue.get(timeout=0.1)
                if task is None:  # 停止信号
                    break
                
                output_file, data = task
                self._write_json_file(output_file, data)
                
            except:
                # 队列超时或其他错误，继续等待
                continue
    
    def _write_json_file(self, output_file: str, data: dict):
        """实际写入JSON文件的操作"""
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"❌ 异步JSON保存失败 ({output_file}): {e}")
    
    def save_async(self, output_file: str, data: dict):
        """非阻塞地提交保存任务"""
        try:
            # 不阻塞，如果队列满则丢弃最新的保存任务
            self.save_queue.put_nowait((output_file, data))
        except:
            # 队列满时静默处理
            pass
    
    def shutdown(self):
        """关闭异步保存器并等待所有任务完成"""
        # 等待队列清空
        while not self.save_queue.empty():
            time.sleep(0.01)
        
        # 发送停止信号
        self.stop_event.set()
        
        # 等待所有工作线程结束
        for worker in self.workers:
            worker.join(timeout=2.0)

# --- 配置类  ---
@dataclass
class Config:
    IMAGE_WIDTH: int = 1920
    IMAGE_HEIGHT: int = 1080
    FPS: int = 25
    TRACK_THRESH: float = 0.3
    MATCH_THRESH: float = 0.6
    MIN_FRAMES_THRESHOLD: int = 10
    TIME_WINDOW: int = 80
    TEMPORAL_WINDOW_MAX: int = 225
    BASE_SPATIAL_THRESHOLD: float = 400.0
    IOU_THRESHOLD: float = 0.5
    TOLERANCE_FRAMES: int = 60
    # 新增：像素Y值阈值（用于判断是否在底部区域）
    PIXEL_BOTTOM_THRESHOLD: int = 700  # Y值下限
    PIXEL_TOP_THRESHOLD: int = 1080    # Y值上限
    # 新增：融合时间窗口
    FUSION_TIME_WINDOW: int = 60
    VEHICLE_CLASSES = ['mini_truck','truck','bus','van','car','person','bike','electric_vehicle',
    'tricycle','engineer','ambulance','fireEngine','schoolBus','tanker','muckTruck',
    'concreteTruck','policeCar']
    EXCLUDE_CLASSES = ["person", "electric_vehicle", "bike", "tricycle"]
    SIMILAR_CLASSES = {
        'mini_truck': ['truck', 'van', 'car'],
        'truck': ['mini_truck', 'van', 'car'],
        'van': ['mini_truck', 'truck', 'car'],
        'car': ['van', 'mini_truck', 'truck'],
        'bus': ['truck', 'van', 'car'],
    }

# 矩阵和区域配置
CAMERA_MATRICES = {
    1: np.array([[-10.5149, 222.4408462, -9790.39534446],
                 [-9.19971325, 69.92947807, 19308.61520761],
                 [-0.00395588, 0.18180162, 1.0]], dtype=np.float64),
    2: np.array([[-1.27499084, -13.21167486, -554.98403431],
                 [-1.56665505, -18.83507418, 3226.90502457],
                 [-0.00079458, -0.019352, 1.0]], dtype=np.float64),
    3: np.array([[-0.00256905, -7.80437003, -464.08522659],
                 [-0.64679995, -4.01640045, -320.34097331],
                 [-0.00060155, -0.01122723, 1.0]], dtype=np.float64)
}

BEV_TO_GEO_MATRIX = np.array([
  [-0.025828551721,  0.026850072654, 113.582941425078],
  [-0.005351012238,  0.005561209192,  23.531419339129],
  [-0.000227406715,  0.000236388495,   1.0]
])
PUBLIC_AREA_BEV = np.array([[1075, 569], [1106, 604], [850, 761], [825, 727]], dtype=np.int32)

# YOLOv5 类别名称
NAMES = [
    'mini_truck','truck','bus','van','car','person','bike','electric_vehicle',
    'tricycle','engineer','ambulance','fireEngine','schoolBus','tanker','muckTruck',
    'concreteTruck','policeCar'
]

# --- 新增：数据结构 (从0915融合逻辑移植) ---
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

class LocalTrackBuffer:
    """本地轨迹缓冲区 - 维护每个摄像头的track轨迹历史"""
    def __init__(self, max_history: int = 30):
        self.max_history = max_history
        self.tracks: Dict[int, Dict[int, List[Tuple[float, float]]]] = defaultdict(lambda: defaultdict(list))
        self.pixel_tracks: Dict[int, Dict[int, List[Tuple[int, int]]]] = defaultdict(lambda: defaultdict(list))
        self.assigned_global_ids: Dict[int, Dict[int, int]] = defaultdict(dict)
        self.track_classes: Dict[int, Dict[int, str]] = defaultdict(dict)
    
    def __len__(self):
        """返回track总数"""
        total = 0
        for camera_id in self.tracks:
            total += len(self.tracks[camera_id])
        return total
    
    def update_track(self, camera_id: int, local_id: int, bev_pos: Tuple[float, float], 
                    pixel_pos: Tuple[int, int], class_name: str):
        """更新本地轨迹"""
        self.tracks[camera_id][local_id].append(bev_pos)
        self.pixel_tracks[camera_id][local_id].append(pixel_pos)
        self.track_classes[camera_id][local_id] = class_name
        
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

# --- 几何和检测工具类 ---
class GeometryUtils:
    @staticmethod
    def project_pixel_to_bev(H: np.ndarray, u: float, v: float) -> Optional[Tuple[float, float]]:
        """单个像素到BEV的转换（保留用于兼容性）"""
        p = np.array([u, v, 1.0])
        q = H @ p
        if abs(q[2]) < 1e-8: return None
        x, y = q[0] / q[2], q[1] / q[2]
        if 0 <= x < Config.IMAGE_WIDTH and 0 <= y < Config.IMAGE_HEIGHT: return (x, y)
        return None


    @staticmethod
    def bev_to_geo(x_bev: float, y_bev: float) -> Optional[Tuple[float, float]]:
        """单个BEV到地理坐标的转换（保留用于兼容性）"""
        try:
            p = np.array([x_bev, y_bev, 1.0])
            q = BEV_TO_GEO_MATRIX @ p
            q /= q[2]
            return q[0], q[1]
        except: return None


    @staticmethod
    def calculate_iou(box1: List[float], box2: List[float]) -> float:
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        x1_i, y1_i = max(x1_1, x1_2), max(y1_1, y1_2)
        x2_i, y2_i = min(x2_1, x2_2), min(y2_1, y2_2)
        if x2_i <= x1_i or y2_i <= y1_i: return 0.0
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        area1 = (x2_1 - x1_1) * (x2_1 - x1_1) 
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        return intersection / union if union > 0 else 0.0

    @staticmethod
    def is_in_public_area(bev_point: Tuple[float, float]) -> bool:
        return cv2.pointPolygonTest(PUBLIC_AREA_BEV, bev_point, False) >= 0

class DetectionUtils:
    @staticmethod
    def is_class_compatible(class1: str, class2: str) -> bool:
        if class1 == class2: return True
        return (class1 in Config.SIMILAR_CLASSES and 
                class2 in Config.SIMILAR_CLASSES[class1])

    @staticmethod
    def non_max_suppression(detections: List[dict], 
                          iou_threshold: float = Config.IOU_THRESHOLD) -> List[dict]:
        if not detections: return detections
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        keep = []
        for det in detections:
            should_keep = True
            for kept_det in keep:
                if DetectionUtils.is_class_compatible(det['class'], kept_det['class']):
                    iou = GeometryUtils.calculate_iou(det['box'], kept_det['box'])
                    if iou > iou_threshold:
                        should_keep = False; break
            if should_keep: keep.append(det)
        return keep

    @staticmethod
    def is_turn_left(trajectory: List[Tuple[int, int]]) -> bool:
        if len(trajectory) < 2: return False
        start = np.array(trajectory[0])
        # 简化左转判断：如果起始点在图像底部，则可能为左转
        return start[1] >= Config.IMAGE_HEIGHT * 0.7 

# --- 目标缓冲区类 ---
class TargetBuffer:
    """目标缓冲区，用于时间窗口内的目标匹配"""
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

# --- 平滑滤波器类 ---
class SmoothingFilter:
    """对目标的BEV坐标进行平滑处理"""
    def __init__(self, history_len: int = 10, alpha: float = 0.5):
        self.bev_history = defaultdict(lambda: deque(maxlen=history_len))
        self.alpha = alpha
        
    def _sliding_average(self, track_id: int, current_bev: Tuple[float, float]) -> Tuple[float, float]:
        """滑动平均平滑"""
        self.bev_history[track_id].append(current_bev)
        
        history = self.bev_history[track_id]
        if len(history) < 2:
            return current_bev
            
        avg_x = sum(p[0] for p in history) / len(history)
        avg_y = sum(p[1] for p in history) / len(history)
        
        return (avg_x, avg_y)

    def _exponential_smoothing(self, track_id: int, current_bev: Tuple[float, float]) -> Tuple[float, float]:
        """指数平滑"""
        history_deque = self.bev_history[track_id]
        
        if not history_deque:
            smoothed_bev = current_bev
        else:
            last_smoothed_bev = history_deque[-1]
            smoothed_x = self.alpha * current_bev[0] + (1 - self.alpha) * last_smoothed_bev[0]
            smoothed_y = self.alpha * current_bev[1] + (1 - self.alpha) * last_smoothed_bev[1]
            smoothed_bev = (smoothed_x, smoothed_y)

        history_deque.append(smoothed_bev)
        return smoothed_bev
    
    def apply_smoothing(self, track_id: int, current_bev: Tuple[float, float], method: str = 'exponential') -> Tuple[float, float]:
        """应用平滑算法"""
        if method == 'sliding':
            return self._sliding_average(track_id, current_bev)
        else:
            return self._exponential_smoothing(track_id, current_bev)

    def remove_track(self, track_id: int):
        """移除不再活跃的目标的历史记录"""
        self.bev_history.pop(track_id, None)

# --- 跨摄像头融合系统 (新融合版) ---
class CrossCameraFusion:
    """
    新的跨摄像头融合系统 - 基于0915融合逻辑
    核心思路：
    1. 基于像素Y值判断是否分配global_id
    2. 使用融合区进入时间同步进行匹配
    3. 通过local_to_global永久绑定防止跨帧重复绑定
    """
    
    def __init__(self):
        self.config = Config()
        
        # 全局目标管理
        self.global_id_counter = 1
        self.global_targets: Dict[int, GlobalTarget] = {}  # global_id -> GlobalTarget
        self.local_to_global: Dict[Tuple[int, int], int] = {}  # (camera_id, local_id) -> global_id
        
        # 新增：本地轨迹缓冲区
        self.local_track_buffer = LocalTrackBuffer(max_history=30)
        
        # 颜色管理
        self.colors: Dict[int, Tuple[int, int, int]] = {}
        
        # 帧计数
        self.frame_count = 0
        self.json_output_data = []
        
        # 确认目标管理
        self.confirmed_targets: Set[int] = set()
        self.target_frame_count: Dict[int, int] = defaultdict(int)
        
        # 初始化异步JSON保存器
        self.json_saver = AsyncJsonSaver(num_workers=1)
        
        print("✅ 新融合版CrossCameraFusion初始化完成 - 基于0915融合逻辑")
    
    def _assign_color(self, global_id: int) -> Tuple[int, int, int]:
        """为全局ID分配颜色"""
        if global_id not in self.colors:
            np.random.seed(global_id)
            color = tuple(int(np.random.randint(0, 255)) for _ in range(3))
            self.colors[global_id] = color
        return self.colors[global_id]

    def assign_new_global_id(self, camera_id: int, local_id: int) -> int:
        """分配新的全局ID"""
        global_id = self.global_id_counter
        self.global_id_counter += 1
        self._assign_color(global_id)
        return global_id

    def create_global_target(self, global_id: int, detection: dict, camera_id: int) -> GlobalTarget:
        """创建全局目标"""
        center_x = int((detection['box'][0] + detection['box'][2]) / 2)
        center_y = int(detection['box'][3])
        
        # BEV坐标转换
        H_matrix = CAMERA_MATRICES[camera_id]
        bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, center_x, center_y)
        if not bev_result:
            bev_result = (0.0, 0.0)
        
        # 检查是否在融合区域
        is_in_fusion_zone = GeometryUtils.is_in_public_area(bev_result)
        
        # 如果在融合区，记录进入时间
        fusion_entry_frame = self.frame_count if is_in_fusion_zone else -1
        
        return GlobalTarget(
            global_id=global_id,
            camera_id=camera_id,
            local_id=detection['track_id'],
            class_name=detection['class'],
            bev_trajectory=[bev_result],
            pixel_trajectory=[(center_x, center_y)],
            last_seen_frame=self.frame_count,
            is_active=True,
            fusion_alpha=0.2,
            is_in_fusion_zone=is_in_fusion_zone,
            confidence_history=[detection['confidence']],
            fusion_entry_frame=fusion_entry_frame
        )
    
    def create_local_target(self, detection: dict, camera_id: int) -> LocalTarget:
        """创建本地目标"""
        center_x = int((detection['box'][0] + detection['box'][2]) / 2)
        center_y = int(detection['box'][3])
            
        # BEV坐标转换
        H_matrix = CAMERA_MATRICES[camera_id]
        bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, center_x, center_y)
        if not bev_result:
            bev_result = (0.0, 0.0)
        
        # 检查是否在融合区域
        is_in_fusion_area = GeometryUtils.is_in_public_area(bev_result)
        
        # 如果在融合区，记录进入时间
        fusion_entry_frame = self.frame_count if is_in_fusion_area else -1
        
        return LocalTarget(
            local_id=detection['track_id'],
            camera_id=camera_id,
            class_name=detection['class'],
            current_bev_pos=bev_result,
            current_pixel_pos=(center_x, center_y),
            confidence=detection['confidence'],
            is_in_fusion_area=is_in_fusion_area,
            detection_box=detection['box'],
            fusion_entry_frame=fusion_entry_frame
        )
    
    def classify_targets(self, detections: List[dict], camera_id: int) -> Tuple[List[GlobalTarget], List[LocalTarget]]:
        """
        将检测结果分类为global_targets和local_targets
        基于轨迹分析判断是否应该升级为global_id
        """
        global_targets = []
        local_targets = []
        
        for detection in detections:
            if 'track_id' not in detection:
                continue
            
            track_id = detection['track_id']
            class_name = detection['class']
            confidence = detection['confidence']
            center_y = detection['box'][3]  # 底部y坐标
            
            # 获取 BEV 坐标
            H_matrix = CAMERA_MATRICES[camera_id]
            center_x = int((detection['box'][0] + detection['box'][2]) / 2)
            bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, center_x, center_y)
            if not bev_result:
                bev_result = (0.0, 0.0)
            
            # 更新本地轨迹缓冲区
            self.local_track_buffer.update_track(camera_id, track_id, bev_result, (center_x, int(center_y)), class_name)
            track_history = self.local_track_buffer.get_track_history(camera_id, track_id)
            pixel_track_history = self.local_track_buffer.get_pixel_track_history(camera_id, track_id)
            
            # 检查是否已分配 global_id
            if self.local_track_buffer.has_global_id(camera_id, track_id):
                # 已分配过，直接更新现有的全局目标
                global_id = self.local_track_buffer.get_global_id(camera_id, track_id)
                global_target = self.global_targets.get(global_id)
                
                if global_target:
                    # 更新全局目标的轨迹
                    global_target.bev_trajectory.append(bev_result)
                    global_target.pixel_trajectory.append((center_x, int(center_y)))
                    global_target.confidence_history.append(confidence)
                    global_target.last_seen_frame = self.frame_count
                    
                    # 更新融合区进入时间
                    if global_target.is_in_fusion_zone and global_target.fusion_entry_frame == -1:
                        global_target.fusion_entry_frame = self.frame_count
                    
                    # 更新融合区状态
                    current_bev = global_target.bev_trajectory[-1]
                    global_target.is_in_fusion_zone = GeometryUtils.is_in_public_area(current_bev)
                    
                    # 限制轨迹长度
                    max_length = 50
                    if len(global_target.bev_trajectory) > max_length:
                        global_target.bev_trajectory = global_target.bev_trajectory[-max_length:]
                        global_target.pixel_trajectory = global_target.pixel_trajectory[-max_length:]
                        global_target.confidence_history = global_target.confidence_history[-max_length:]
                    
                    global_targets.append(global_target)
                continue
            
            # 未分配过，基于轨迹分析判断是否应该分配 global_id
            if analyze_trajectory_for_global_assignment(pixel_track_history, 
                                                       min_trajectory_length=3,
                                                       pixel_bottom_threshold=self.config.PIXEL_BOTTOM_THRESHOLD,
                                                       pixel_top_threshold=self.config.PIXEL_TOP_THRESHOLD):
                # 满足条件，分配新的 global_id
                global_id = self.assign_new_global_id(camera_id, track_id)
                global_target = self.create_global_target(global_id, detection, camera_id)
                
                # 记录分配关系
                self.local_track_buffer.assign_global_id(camera_id, track_id, global_id)
                
                # 添加到全局目标字典
                self.global_targets[global_id] = global_target
                self.target_frame_count[global_id] = 1
                
                global_targets.append(global_target)
            else:
                # 不满足条件，作为本地目标
                local_target = self.create_local_target(detection, camera_id)
                local_targets.append(local_target)
        
        return global_targets, local_targets
    
    def _smoothly_merge_trajectory(self, global_target: GlobalTarget, 
                                  local_target: LocalTarget):
        """动态加权平滑轨迹融合"""
        # 计算加权位置
        current_bev = global_target.bev_trajectory[-1]
        local_bev = local_target.current_bev_pos
        
        # 使用动态融合权重
        alpha = global_target.fusion_alpha
        new_bev_x = ((1.0 - alpha) * current_bev[0] + alpha * local_bev[0])
        new_bev_y = ((1.0 - alpha) * current_bev[1] + alpha * local_bev[1])
        
        # 更新轨迹
        global_target.bev_trajectory.append((new_bev_x, new_bev_y))
        global_target.pixel_trajectory.append(local_target.current_pixel_pos)
        global_target.confidence_history.append(local_target.confidence)
        global_target.last_seen_frame = self.frame_count
        
        # 动态增加融合权重，实现平滑过渡
        global_target.fusion_alpha += 0.01
        if global_target.fusion_alpha > 1.0:
            global_target.fusion_alpha = 1.0
        
        # 更新融合区域状态
        global_target.is_in_fusion_zone = local_target.is_in_fusion_area
    
    def _perform_matching(self, local_targets_this_frame: List[LocalTarget], 
                     active_global_targets: List[GlobalTarget]):
        """
        核心匹配方法：基于融合区进入时间进行时间同步匹配
        使用永久绑定记录防止跨帧重复绑定
        """
        # 用于锁定本帧已匹配的 global_id，防止一对多
        locked_global_ids_this_frame = set()

        # 创建一个包含所有已被永久绑定的 global_id 的集合
        permanently_bound_global_ids = set(self.local_to_global.values())

        # 1. 预处理：刷新所有全局目标的融合区状态和进入时间
        for gt in active_global_targets:
            if gt.bev_trajectory:
                current_bev = gt.bev_trajectory[-1]
                is_now_in_zone = GeometryUtils.is_in_public_area(current_bev)
                gt.is_in_fusion_zone = is_now_in_zone
                if is_now_in_zone and gt.fusion_entry_frame == -1:
                    gt.fusion_entry_frame = self.frame_count

        time_window = self.config.FUSION_TIME_WINDOW

        # 2. 遍历所有本帧的 local_target 进行处理
        for local_target in local_targets_this_frame:
            lookup_key = (local_target.camera_id, local_target.local_id)

            # 2.1 检查此 local_target 是否已经绑定
            if lookup_key in self.local_to_global:
                bound_global_id = self.local_to_global[lookup_key]
                bound_global_target = self.global_targets.get(bound_global_id)

                if bound_global_target:
                    self._smoothly_merge_trajectory(bound_global_target, local_target)
                    local_target.matched_global_id = bound_global_id
                else:
                    del self.local_to_global[lookup_key]
                
                continue

            # 2.2 如果未绑定，执行首次匹配逻辑
            if not local_target.is_in_fusion_area:
                continue
                
            if local_target.fusion_entry_frame == -1:
                local_target.fusion_entry_frame = self.frame_count
            
            # 确定候选池 - 基于摄像头配对关系（0915方式）
            candidate_globals = []
            if local_target.camera_id == 1:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
            elif local_target.camera_id == 2:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id in [1, 3]]
            elif local_target.camera_id == 3:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
            
            # 筛选时，同时检查帧内锁和永久绑定状态
            fusion_candidates = [
                gt for gt in candidate_globals 
                if (gt.is_in_fusion_zone and 
                    gt.global_id not in locked_global_ids_this_frame and
                    gt.global_id not in permanently_bound_global_ids)
            ]
            
            if not fusion_candidates:
                continue
            
            best_match = None
            best_time_diff = float('inf')
            
            for candidate in fusion_candidates:
                if not DetectionUtils.is_class_compatible(local_target.class_name, candidate.class_name):
                    continue
                if candidate.fusion_entry_frame == -1:
                    continue
                time_diff = abs(local_target.fusion_entry_frame - candidate.fusion_entry_frame)
                if time_diff <= time_window and time_diff < best_time_diff:
                    best_time_diff = time_diff
                    best_match = candidate
            
            if best_match:
                local_target.matched_global_id = best_match.global_id
                self.local_to_global[lookup_key] = best_match.global_id
                locked_global_ids_this_frame.add(best_match.global_id)
                self._smoothly_merge_trajectory(best_match, local_target)
    
    def update_global_state(self, all_global_targets: List[GlobalTarget], all_local_targets: List[LocalTarget]):
        """更新全局状态"""
        # 处理直接的全局目标
        for global_target in all_global_targets:
            self.target_frame_count[global_target.global_id] += 1
            
            # 更新融合区域状态
            if global_target.bev_trajectory:
                current_bev = global_target.bev_trajectory[-1]
                global_target.is_in_fusion_zone = GeometryUtils.is_in_public_area(current_bev)
            
            # 确认目标
            if (self.target_frame_count[global_target.global_id] >= self.config.MIN_FRAMES_THRESHOLD and 
                global_target.global_id not in self.confirmed_targets):
                self.confirmed_targets.add(global_target.global_id)
    
    def process_detections(self, detections: List[dict], camera_id: int, perf_monitor=None) -> Tuple[List[GlobalTarget], List[LocalTarget]]:
        """处理单个摄像头的检测结果"""
        if perf_monitor:
            perf_monitor.start_timer('process_detections')
        
        # 分类目标
        global_targets, local_targets = self.classify_targets(detections, camera_id)
        
        # 将全局目标添加到全局目标字典
        for global_target in global_targets:
            if global_target.global_id not in self.global_targets:
                self.global_targets[global_target.global_id] = global_target
                self.target_frame_count[global_target.global_id] = 1
        
        if perf_monitor:
            duration = perf_monitor.end_timer('process_detections')
            perf_monitor.record_fusion_stats('process_detections', duration, {
                'detection_count': len(detections),
                'global_target_count': len(global_targets),
                'local_target_count': len(local_targets)
            })
        
        return global_targets, local_targets
    
    def generate_json_data_new(self, all_global_targets: List[GlobalTarget], 
                              all_local_targets: List[LocalTarget]) -> dict:
        """生成新的JSON数据"""
        current_time_ms = int(self.frame_count * 1000 / self.config.FPS)
        participants = []
        
        # 处理全局目标
        for global_target in all_global_targets:
            if not self.is_confirmed_target(global_target.global_id):
                continue
            
            if not global_target.bev_trajectory:
                continue
            
            current_bev = global_target.bev_trajectory[-1]
            geo_result = GeometryUtils.bev_to_geo(current_bev[0], current_bev[1])
            if not geo_result:
                continue
            
            lng, lat = geo_result
            participants.append({
                "pid": global_target.global_id,
                "type": global_target.class_name,
                "plate": f"GID{global_target.global_id}",
                "heading": 0,
                "lon": lng,
                "lat": lat
            })
        
        # 处理已匹配的本地目标
        for local_target in all_local_targets:
            if not local_target.matched_global_id:
                continue
                
            if not self.is_confirmed_target(local_target.matched_global_id):
                continue
                
            geo_result = GeometryUtils.bev_to_geo(local_target.current_bev_pos[0], local_target.current_bev_pos[1])
            if not geo_result:
                continue
                
            lng, lat = geo_result
            participants.append({
                "pid": local_target.matched_global_id,
                "type": local_target.class_name,
                "plate": f"GID{local_target.matched_global_id}",
                "heading": 0,
                "lon": lat,
                "lat": lng
            })
        
        return {
            "reportTime": current_time_ms,
            "participant": participants
        }

    def is_confirmed_target(self, global_id: int) -> bool:
        """检查目标是否已确认"""
        return global_id in self.confirmed_targets

    def cleanup_inactive_targets(self):
        """清理不活跃目标"""
        if self.frame_count % 20 != 0:
            return
        
        inactive_threshold = 100
        current_time = self.frame_count
        
        # 清理不活跃的全局目标
        inactive_global_ids = []
        for global_id, global_target in self.global_targets.items():
            if current_time - global_target.last_seen_frame > inactive_threshold:
                inactive_global_ids.append(global_id)
        
        for global_id in inactive_global_ids:
            self.global_targets.pop(global_id, None)
            self.colors.pop(global_id, None)
            self.target_frame_count.pop(global_id, None)
            self.confirmed_targets.discard(global_id)
            
            # 清理映射关系
            keys_to_remove = [k for k, v in self.local_to_global.items() if v == global_id]
            for key in keys_to_remove:
                del self.local_to_global[key]

    def next_frame(self):
        """进入下一帧"""
        self.frame_count += 1
        self.cleanup_inactive_targets()

    def save_json_data(self, output_file: str):
        """保存JSON数据到文件"""
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(self.json_output_data, f, ensure_ascii=False, indent=2)
            print(f"✅ JSON数据已保存: {output_file}, 共{len(self.json_output_data)}帧")
        except Exception as e:
            print(f"❌ 保存JSON文件出错: {e}")
    
    def save_json_data_realtime(self, output_file: str, current_frame: int):
        """异步实时保存JSON数据"""
        try:
            realtime_data = {
                "frame_number": current_frame,
                "timestamp": time.time(),
                "total_frames": len(self.json_output_data),
                "global_targets": len(self.global_targets),
                "confirmed_targets": len(self.confirmed_targets),
                "data": self.json_output_data
            }
            
            # 使用异步保存器非阻塞地保存JSON
            self.json_saver.save_async(output_file, realtime_data)
            
            if current_frame % 10 == 0:
                print(f"💾 异步JSON提交: Frame {current_frame}, Global={len(self.global_targets)}, Confirmed={len(self.confirmed_targets)}")
                
        except Exception as e:
            print(f"❌ 异步JSON提交失败: {e}")

# --- 性能监视器类 ---
class PerformanceMonitor:
    """性能监视器，用于定位系统瓶颈"""
    
    def __init__(self):
        self.timers = {}
        self.counters = {}
        self.queue_stats = {}
        self.fusion_stats = {}
        self.last_report_time = time.time()
        self.report_interval = 10.0
        
        self.counters = {
            'frames_processed': 0,
            'frames_synchronized': 0,
            'detections_processed': 0,
            'fusion_operations': 0,
            'bev_conversions': 0,
            'tracker_updates': 0,
            'queue_operations': 0,
            'mqtt_sends': 0,
            'mqtt_failures': 0
        }
        
        print("📊 性能监视器初始化完成")
    
    def start_timer(self, name: str):
        self.timers[name] = time.time()
    
    def end_timer(self, name: str) -> float:
        if name not in self.timers:
            return 0.0
        elapsed = (time.time() - self.timers[name]) * 1000
        del self.timers[name]
        return elapsed
    
    def add_counter(self, name: str, value: int = 1):
        if name in self.counters:
            self.counters[name] += value
        else:
            self.counters[name] = value
    
    def record_queue_stats(self, camera_id: int, queue_size: int, operation: str):
        if camera_id not in self.queue_stats:
            self.queue_stats[camera_id] = {
                'max_size': 0,
                'avg_size': 0,
                'operations': 0,
                'total_size': 0
            }
        
        stats = self.queue_stats[camera_id]
        stats['max_size'] = max(stats['max_size'], queue_size)
        stats['operations'] += 1
        stats['total_size'] += queue_size
        stats['avg_size'] = stats['total_size'] / stats['operations']
    
    def record_fusion_stats(self, operation: str, duration_ms: float, details: dict = None):
        if operation not in self.fusion_stats:
            self.fusion_stats[operation] = {
                'count': 0,
                'total_time': 0.0,
                'max_time': 0.0,
                'min_time': float('inf'),
                'avg_time': 0.0
            }
        
        stats = self.fusion_stats[operation]
        stats['count'] += 1
        stats['total_time'] += duration_ms
        stats['max_time'] = max(stats['max_time'], duration_ms)
        stats['min_time'] = min(stats['min_time'], duration_ms)
        stats['avg_time'] = stats['total_time'] / stats['count']
    
    def get_performance_report(self) -> str:
        current_time = time.time()
        elapsed = current_time - self.last_report_time
        
        if elapsed < self.report_interval:
            return ""
        
        self.last_report_time = current_time
        
        report = []
        report.append("\n" + "="*60)
        report.append("📊 性能监控报告")
        report.append("="*60)
        
        fps = self.counters['frames_processed'] / elapsed if elapsed > 0 else 0
        sync_fps = self.counters['frames_synchronized'] / elapsed if elapsed > 0 else 0
        report.append(f"🚀 处理速度:")
        report.append(f"  总帧处理速度: {fps:.2f} FPS")
        report.append(f"  同步帧处理速度: {sync_fps:.2f} FPS")
        
        if self.fusion_stats:
            report.append(f"\n🔄 融合算法性能:")
            for operation, stats in self.fusion_stats.items():
                report.append(f"  {operation}: 平均{stats['avg_time']:.2f}ms")
        
        report.append("="*60)
        
        return "\n".join(report)
    
    def reset_counters(self):
        for key in self.counters:
            self.counters[key] = 0
        self.queue_stats.clear()
        self.fusion_stats.clear()

# --- SDK 推理类 (生产者，精简版) ---
class yolov5_SDK(infer_process):
    """精简版的 SDK 推理进程"""
    def __init__(self, attr, result_queue):
        super().__init__(attr)
        self.class_num = self.model_desc.outputTensorAttrArray[0].dims[3] - 5
        self.anchor_size = self.model_desc.outputTensorAttrArray[0].dims[1]
        
        self.result_queue = result_queue
        self.frame_count = 0
        
        self.boxes_info, ret = sdk.lyn_malloc(ctypes.sizeof(Box))
        if ret != 0:
            raise RuntimeError(f"Camera{self.attr.chan_id + 1}: 内存分配失败: {ret}")

    def update_class_name(self, class_name_path: str) -> None:
        try:
            with open(class_name_path, 'r') as file:
                file_content = file.read()
                pattern = re.compile(r"^(x7|normal):([^,]+(,[^,]+)*)$", re.IGNORECASE)
                if not pattern.match(file_content):
                    print(f'"{file_content}" is not right!')
                    os._exit(-1)
        except FileNotFoundError:
            print(f"File at path '{class_name_path}' not found.")
            return None
        except IOError as e:
            print(f"Error reading file at path '{class_name_path}': {e}")
            return None
        
        ary = np.fromfile(class_name_path)
        ptr = sdk.lyn_numpy_to_ptr(ary)
        device_ptr, ret = sdk.lyn_malloc(ary.nbytes)
        sdk.lyn_memcpy(device_ptr, ptr, ary.nbytes, sdk.ClientToServer)
        class_name_arg = struct.pack("Pi", pythonapi.PyCapsule_GetPointer(device_ptr, None), ary.nbytes)
        sdk.lyn_plugin_run_async(self.ipe_stream, self.plugin, "lynClassNameUpdata", class_name_arg, len(class_name_arg))
        sdk.lyn_synchronize_stream(self.ipe_stream)
        sdk.lyn_free(device_ptr)
        print(f"✅ Camera{self.attr.chan_id + 1} 成功更新类别名称")

    def process_box_data_callback(self, params):
        try:
            boxes_info = params[0]
            frame_count = params[1]
            
            dst_img_size = ctypes.sizeof(Box)
            host_buf_arr = np.ones(dst_img_size, dtype=np.uint8)
            host_buf = sdk.lyn_numpy_to_ptr(host_buf_arr)
            ret = sdk.lyn_memcpy(
                host_buf, boxes_info, dst_img_size, 
                sdk.lyn_memcpy_dir_t.ServerToClient
            )
            if ret != 0:
                print(f"❌ Camera{self.attr.chan_id + 1} memcpy失败: {ret}", flush=True)
                return 0
            
            pythonapi.PyCapsule_GetPointer.restype = c_void_p
            pythonapi.PyCapsule_GetPointer.argtypes = [py_object, c_char_p]
            host_buf_c = pythonapi.PyCapsule_GetPointer(host_buf, None)
            box_data = ctypes.cast(host_buf_c, ctypes.POINTER(Box)).contents
            
            frame_result = self.extract_detection_info_from_box(box_data, frame_count)
            
            if frame_result:
                self.result_queue.put(copy.deepcopy(frame_result))
                if frame_count % 30 == 0:
                    print(f"C{self.attr.chan_id + 1} | F{frame_count}: PUSH {frame_result['boxes_num']} boxes", flush=True)
            else:
                self.result_queue.put({
                    'frame_id': frame_count, 'camera_id': self.attr.chan_id + 1,
                    'boxes_num': 0, 'detections': []
                })
            
            return 0
        except Exception as e:
            print(f"❌ Camera{self.attr.chan_id + 1} callback错误: {e}", flush=True)
            import traceback
            traceback.print_exc()
            return 0

    def extract_detection_info_from_box(self, box_data, frame_count):
        if not box_data: return None
            
        frame_result = {
            'frame_id': frame_count,
            'camera_id': self.attr.chan_id + 1,
            'boxes_num': box_data.boxesnum,
            'detections': []
        }
        
        for i in range(box_data.boxesnum):
            try:
                box = box_data.boxes[i]
            except Exception as e:
                print(f"❌ Camera{self.attr.chan_id + 1} 访问boxes[{i}]错误: {e}", flush=True)
                continue
                
            try:
                if hasattr(box, 'label') and box.label is not None:
                    if isinstance(box.label, str):
                        label_str = box.label
                        try:
                            class_id = int(box.label)
                        except ValueError:
                            class_id = hash(box.label) % 1000
                    else:
                        class_id = int(box.label)
                        label_str = NAMES[class_id] if class_id < len(NAMES) else f"class_{class_id}"
                else:
                    label_str = "unknown"
                    class_id = 0
            except (ValueError, UnicodeError, TypeError) as e:
                label_str = "unknown"
                class_id = 0
            
            detection = {
                'box': [float(box.xmin), float(box.ymin), float(box.xmax), float(box.ymax)],
                'confidence': float(box.score),
                'class': label_str, 
            }
            frame_result['detections'].append(detection)
        
        return frame_result

    def plugin_process(self, apu_output_data, cb_data):
        if self.frame_count == 0 or self.frame_count % 30 == 0:
            print(f"🎯 Camera{self.attr.chan_id + 1} plugin_process 被调用，帧{self.frame_count}", flush=True)
        
        try:
            ret = sdk.lyn_record_event(self.apu_stream, self.apu_event)
            if ret != 0: 
                print(f"❌ Camera{self.attr.chan_id + 1} lyn_record_event 失败: {ret}", flush=True)
                return
            ret = sdk.lyn_stream_wait_event(self.plugin_stream, self.apu_event)
            if ret != 0: 
                print(f"❌ Camera{self.attr.chan_id + 1} lyn_stream_wait_event 失败: {ret}", flush=True)
                return
            
            pythonapi.PyCapsule_GetPointer.restype = c_void_p
            pythonapi.PyCapsule_GetPointer.argtypes = [py_object, c_char_p]
            apu_data_ptr = pythonapi.PyCapsule_GetPointer(apu_output_data, None)
            boxes_info_ptr = pythonapi.PyCapsule_GetPointer(self.boxes_info, None)

            post_para = struct.pack(
                '6IH2f?2P',
                self.codec_para.width, self.codec_para.height, self.model_width,
                self.model_height, self.class_num, 500, self.anchor_size,
                0.25, 0.45, True, apu_data_ptr, boxes_info_ptr,
            )
            ret = sdk.lyn_plugin_run_async(
                self.plugin_stream, self.plugin, "lynPostProcess", post_para, len(post_para)
            )
            if ret != 0:
                print(f"❌ Camera{self.attr.chan_id + 1} lyn_plugin_run_async 失败: {ret}", flush=True)
                return
            
            ret = sdk.lyn_stream_add_callback(
                self.plugin_stream,
                self.process_box_data_callback,
                [self.boxes_info, self.frame_count],
            )
            if ret != 0:
                print(f"❌ Camera{self.attr.chan_id + 1} lyn_stream_add_callback 失败: {ret}", flush=True)
                return
            
            self.frame_count += 1

            ret = sdk.lyn_stream_add_async_callback(
                self.plugin_stream, free_to_pool_callback, [self.apu_output_mem_pool, apu_output_data]
            )
            
        except Exception as e:
            print(f"❌ Camera{self.attr.chan_id + 1}: plugin_process 出错: {e}", flush=True)
            import traceback
            traceback.print_exc()

    def run(self, cancel_flag):
        super().run(cancel_flag)

# --- 辅助函数 ---
def cancel_process(signum, frame):
    global cancel_flag
    cancel_flag.value = True
    print("🛑 收到停止信号，正在退出...")

def create_sdk_worker_process(camera_id: int, video_path: str, result_queue: multiprocessing.Queue):
    """创建并运行一个独立的 SDK 推理子进程"""
    try:
        print(f"🔧 Camera{camera_id} 开始初始化SDK...", flush=True)
        
        attr = infer_process_attr()
        attr.url = video_path
        attr.device_id = 0
        attr.chan_id = camera_id - 1
        attr.plugin_path = "/usr/local/lynxi/sdk/sdk-samples/plugin/obj/libYolov5Plugin.so"
        attr.model_path = "/root/yolov5-7.0_lyngor1.17.0/best_yolov5s_onnx/Net_0/"
        attr.show_type = 2
        attr.output_path = ""
        
        worker = yolov5_SDK(attr, result_queue) 
        
        class_name_path = "/usr/local/lynxi/sdk/sdk-samples/data/class.txt"
        if os.path.exists(class_name_path):
            worker.update_class_name(class_name_path)
        
        print(f"🚀 Camera{camera_id} 开始运行SDK推理...", flush=True)
        worker.run(cancel_flag)
        
        while not cancel_flag.value:
            time.sleep(1)
        
        print(f"🛑 Camera{camera_id} 收到停止信号，正在关闭...", flush=True)
        worker.close()
    except Exception as e:
        print(f"❌ Camera{camera_id} SDK进程失败: {e}", flush=True)
        import traceback
        traceback.print_exc()
        os._exit(1)

def filter_by_detect_areas(detections: List[dict], areas: List[np.ndarray]) -> List[dict]:
    """根据检测区域过滤检测结果"""
    filtered_detections = []
    for detection in detections:
        x1, y1, x2, y2 = detection['box']
        center_x, center_y = int((x1 + x2) / 2), int(y2) 
        in_detect_area = any(cv2.pointPolygonTest(area, (center_x, center_y), False) >= 0 
                           for area in areas)
        if in_detect_area:
            filtered_detections.append(detection)
    return filtered_detections

def batch_prepare_tracker_input(nms_detections: List[dict]) -> np.ndarray:
    """批量准备跟踪器输入"""
    if not nms_detections:
        return np.empty((0, 6), dtype=np.float32)
    
    boxes_scores = np.array([[d['box'][0], d['box'][1], d['box'][2], d['box'][3], d['confidence']] for d in nms_detections])
    labels = np.array([NAMES.index(d['class']) if d['class'] in NAMES else 0 for d in nms_detections])
    
    tracker_input_array = np.column_stack([boxes_scores, labels]).astype(np.float32)
    return tracker_input_array

def batch_convert_track_results(tracked_objects: List, result: dict, camera_id: int, current_frame: int, 
                               original_detections: List[dict] = None) -> List[dict]:
    """批量转换跟踪结果"""
    tracked_detections = []
    
    for track in tracked_objects:
        tlwh = track.tlwh
        tlbr = [tlwh[0], tlwh[1], tlwh[0] + tlwh[2], tlwh[1] + tlwh[3]]
        
        class_name = 'vehicle'
        if original_detections:
            best_iou = 0
            for orig_det in original_detections:
                iou = GeometryUtils.calculate_iou(tlbr, orig_det['box'])
                if iou > best_iou and iou > 0.3:
                    best_iou = iou
                    class_name = orig_det['class']
        
        detection = {
            'box': tlbr,
            'confidence': track.score,
            'class': class_name,
            'track_id': track.track_id,
            'local_id': track.track_id,
            'center_point': [(tlbr[0] + tlbr[2]) / 2, (tlbr[1] + tlbr[3]) / 2],
            'timestamp': result.get('timestamp', time.time()),
            'frame_number': result.get('frame_number', current_frame),
            'camera_id': camera_id,
            'sync_id': result.get('sync_id', f"C{camera_id}_F{current_frame}")
        }
        tracked_detections.append(detection)
    
    return tracked_detections

# --- 新增：速度监控类 ---
class SpeedMonitor:
    """实时速度监控 - 追踪每个摄像头的帧到达速度和处理速度"""
    
    def __init__(self, window_size=300):
        self.window_size = window_size
        
        # 帧到达时间记录（用于计算实际FPS）
        self.frame_arrival_times = {1: deque(maxlen=window_size), 
                                   2: deque(maxlen=window_size), 
                                   3: deque(maxlen=window_size)}
        self.last_frame_id = {1: -1, 2: -1, 3: -1}
        
        # 处理环节统计
        self.detection_times = {1: deque(maxlen=100), 
                               2: deque(maxlen=100), 
                               3: deque(maxlen=100)}
        self.tracking_times = {1: deque(maxlen=100), 
                              2: deque(maxlen=100), 
                              3: deque(maxlen=100)}
        self.fusion_times = deque(maxlen=100)
        
        # 缓冲区监控
        self.buffer_snapshots = {1: deque(maxlen=100), 
                                2: deque(maxlen=100), 
                                3: deque(maxlen=100)}
        
        # 报告时间
        self.last_report_time = time.time()
        self.report_interval = 10.0
        
        print("📊 速度监控器初始化完成")
    
    def record_frame_arrival(self, camera_id, frame_id):
        """记录帧到达事件"""
        now = time.time()
        self.frame_arrival_times[camera_id].append((frame_id, now))
        self.last_frame_id[camera_id] = frame_id
    
    def record_detection_time(self, camera_id, duration_ms):
        """记录检测耗时"""
        self.detection_times[camera_id].append(duration_ms)
    
    def record_tracking_time(self, camera_id, duration_ms):
        """记录跟踪耗时"""
        self.tracking_times[camera_id].append(duration_ms)
    
    def record_fusion_time(self, duration_ms):
        """记录融合耗时"""
        self.fusion_times.append(duration_ms)
    
    def record_buffer_snapshot(self, camera_id, buffer_size):
        """记录缓冲区大小"""
        self.buffer_snapshots[camera_id].append((time.time(), buffer_size))
    
    def get_real_fps(self, camera_id):
        """计算实际FPS（最近100帧）"""
        times = self.frame_arrival_times[camera_id]
        if len(times) < 2:
            return 0
        
        frame_count = times[-1][0] - times[0][0]
        time_diff = times[-1][1] - times[0][1]
        
        if time_diff > 0:
            return frame_count / time_diff
        return 0
    
    def get_buffer_growth_rate(self, camera_id):
        """计算缓冲区增长速率（帧/秒）"""
        snapshots = self.buffer_snapshots[camera_id]
        if len(snapshots) < 2:
            return 0
        
        size_diff = snapshots[-1][1] - snapshots[0][1]
        time_diff = snapshots[-1][0] - snapshots[0][0]
        
        if time_diff > 0:
            return size_diff / time_diff
        return 0
    
    def get_speed_statistics(self):
        """生成详细的速度统计报告"""
        report = {}
        report['timestamp'] = time.time()
        report['cameras'] = {}
        report['processing_stages'] = {}
        
        # 1. 摄像头速度统计
        fps_values = []
        for cam_id in [1, 2, 3]:
            real_fps = self.get_real_fps(cam_id)
            buffer_growth = self.get_buffer_growth_rate(cam_id)
            fps_values.append(real_fps)
            
            report['cameras'][f'C{cam_id}'] = {
                'real_fps': round(real_fps, 2),
                'buffer_growth_rate': round(buffer_growth, 2),
                'frames_arrived': self.last_frame_id[cam_id]
            }
        
        # 2. 速度比例
        if fps_values and min(fps_values) > 0:
            max_fps = max(fps_values)
            min_fps = min(fps_values)
            speed_ratio = max_fps / min_fps
            report['speed_ratio'] = round(speed_ratio, 2)
            report['speed_imbalance'] = "严重" if speed_ratio > 1.5 else ("中等" if speed_ratio > 1.2 else "正常")
        
        # 3. 检测阶段统计
        for cam_id in [1, 2, 3]:
            if self.detection_times[cam_id]:
                det_times = list(self.detection_times[cam_id])
                report['processing_stages'][f'detection_C{cam_id}'] = {
                    'avg_ms': round(mean(det_times), 2),
                    'max_ms': round(max(det_times), 2),
                    'min_ms': round(min(det_times), 2),
                    'count': len(det_times)
                }
        
        # 4. 跟踪阶段统计
        for cam_id in [1, 2, 3]:
            if self.tracking_times[cam_id]:
                track_times = list(self.tracking_times[cam_id])
                report['processing_stages'][f'tracking_C{cam_id}'] = {
                    'avg_ms': round(mean(track_times), 2),
                    'max_ms': round(max(track_times), 2),
                    'min_ms': round(min(track_times), 2),
                    'count': len(track_times)
                }
        
        # 5. 融合阶段统计
        if self.fusion_times:
            fusion_times_list = list(self.fusion_times)
            report['processing_stages']['fusion_all'] = {
                'avg_ms': round(mean(fusion_times_list), 2),
                'max_ms': round(max(fusion_times_list), 2),
                'min_ms': round(min(fusion_times_list), 2),
                'count': len(fusion_times_list)
            }
        
        return report
    
    def print_speed_report(self):
        """打印格式化的速度报告"""
        stats = self.get_speed_statistics()
        
        current_time = time.time()
        elapsed = current_time - self.last_report_time
        
        if elapsed < self.report_interval:
            return ""
        
        self.last_report_time = current_time
        
        report_lines = []
        report_lines.append("\n" + "="*70)
        report_lines.append("🚀 实时速度监控报告")
        report_lines.append("="*70)
        
        # 摄像头速度
        report_lines.append("\n📹 摄像头帧到达速度:")
        for cam_name, cam_data in stats['cameras'].items():
            growth_indicator = "📈" if cam_data['buffer_growth_rate'] > 2 else ("⚠️" if cam_data['buffer_growth_rate'] > 0.5 else "✅")
            report_lines.append(f"  {cam_name}: {cam_data['real_fps']:.2f} FPS | 缓冲增速: {cam_data['buffer_growth_rate']:.2f}帧/s {growth_indicator}")
        
        # 速度失衡警告
        if 'speed_ratio' in stats:
            ratio = stats['speed_ratio']
            imbalance = stats['speed_imbalance']
            emoji = "🔴" if imbalance == "严重" else ("🟡" if imbalance == "中等" else "🟢")
            report_lines.append(f"\n⚙️  速度失衡度: {ratio:.2f}x [{imbalance}] {emoji}")
        
        # 检测阶段耗时
        detection_entries = {k: v for k, v in stats['processing_stages'].items() if 'detection' in k}
        if detection_entries:
            report_lines.append("\n🔍 检测阶段耗时:")
            for stage_name, stage_data in detection_entries.items():
                cam_name = stage_name.replace('detection_', '')
                report_lines.append(f"  {cam_name}: {stage_data['avg_ms']:.2f}ms (min:{stage_data['min_ms']:.2f}, max:{stage_data['max_ms']:.2f})")
        
        # 跟踪阶段耗时
        tracking_entries = {k: v for k, v in stats['processing_stages'].items() if 'tracking' in k}
        if tracking_entries:
            report_lines.append("\n👁️  跟踪阶段耗时:")
            for stage_name, stage_data in tracking_entries.items():
                cam_name = stage_name.replace('tracking_', '')
                report_lines.append(f"  {cam_name}: {stage_data['avg_ms']:.2f}ms (min:{stage_data['min_ms']:.2f}, max:{stage_data['max_ms']:.2f})")
        
        # 融合阶段耗时
        if 'fusion_all' in stats['processing_stages']:
            fusion_data = stats['processing_stages']['fusion_all']
            report_lines.append("\n🔗 融合阶段耗时:")
            report_lines.append(f"  平均: {fusion_data['avg_ms']:.2f}ms (min:{fusion_data['min_ms']:.2f}, max:{fusion_data['max_ms']:.2f})")
        
        # 整体耗时计算
        total_detection = 0
        total_tracking = 0
        count = 0
        for cam_id in [1, 2, 3]:
            if self.detection_times[cam_id]:
                total_detection += sum(self.detection_times[cam_id])
                total_tracking += sum(self.tracking_times[cam_id])
                count += len(self.detection_times[cam_id])
        
        if count > 0:
            report_lines.append("\n📊 总体耗时构成:")
            report_lines.append(f"  检测: {total_detection/count:.2f}ms/帧")
            report_lines.append(f"  跟踪: {total_tracking/count:.2f}ms/帧")
            if self.fusion_times:
                report_lines.append(f"  融合: {mean(self.fusion_times):.2f}ms/帧")
            report_lines.append(f"  合计: {(total_detection + total_tracking)/count + (mean(self.fusion_times) if self.fusion_times else 0):.2f}ms/帧")
        
        # 建议
        report_lines.append("\n💡 优化建议:")
        if 'speed_ratio' in stats and stats['speed_ratio'] > 1.5:
            report_lines.append("  ⚠️  摄像头处理速度差异大，建议检查SDK配置或模型选择")
        
        detection_max = max([v['max_ms'] for v in detection_entries.values()]) if detection_entries else 0
        if detection_max > 50:
            report_lines.append("  ⚠️  检测耗时过长，考虑降低模型精度或缩小输入分辨率")
        
        tracking_max = max([v['max_ms'] for v in tracking_entries.values()]) if tracking_entries else 0
        if tracking_max > 20:
            report_lines.append("  ⚠️  跟踪耗时过长，考虑优化ByteTracker参数")
        
        if self.fusion_times and mean(self.fusion_times) > 30:
            report_lines.append("  ⚠️  融合耗时过长，考虑优化融合算法")
        
        report_lines.append("="*70)
        
        return "\n".join(report_lines)

# ==================== 帧汇聚器 (Frame Assembler) ====================
class FrameAssembler:
    """
    第二阶段：帧汇聚器
    职责：从三个检测生产者的队列读取检测结果，当某一帧号的三路数据齐全时，
    打包输出到融合队列供第三阶段（跟踪与融合）消费。
    
    核心逻辑：
    - 内部维护 frame_buffer = {frame_id: {camera_id: detection_result}}
    - 从三个队列中读取检测结果，按 frame_id 存储
    - 检查完整性：len(frame_buffer[frame_id]) == 3？
    - 若齐全，打包到融合队列；若超时，放弃该帧
    """
    
    def __init__(self, detection_queues: Dict[int, multiprocessing.Queue],
                 fusion_queue: multiprocessing.Queue, 
                 num_cameras: int = 3,
                 fps: int = 25,
                 timeout_frames: int = 10):
        """
        Args:
            detection_queues: {camera_id -> Queue}，来自三个检测生产者的队列
            fusion_queue: 输出融合队列，发送齐全的三路数据
            num_cameras: 摄像头数量
            fps: 视频帧率，用于计算超时
            timeout_frames: 超时时间（以帧数计），如果等待超过 timeout_frames，放弃
        """
        self.detection_queues = detection_queues
        self.fusion_queue = fusion_queue
        self.num_cameras = num_cameras
        self.fps = fps
        self.timeout_frames = timeout_frames
        self.timeout_ms = (timeout_frames / fps) * 1000  # 转换为毫秒
        
        # 帧缓冲区：frame_id -> {camera_id: detection_result}
        self.frame_buffer = {}
        
        # 记录每个frame的到达时间，用于超时检测
        self.frame_timestamps = {}  # frame_id -> arrival_time
        
        # 统计信息
        self.stats = {
            'frames_assembled': 0,      # 完整的三路帧组
            'frames_timeout': 0,        # 超时丢弃的帧
            'detections_received': 0,   # 接收的检测结果总数
            'max_buffer_size': 0        # 缓冲区最大尺寸
        }
        
        # 缓冲区大小限制
        self.max_buffer_size = 300
        
        print(f"✅ 帧汇聚器初始化完成 - {num_cameras}摄像头, FPS:{fps}, 超时:{timeout_frames}帧({self.timeout_ms:.1f}ms)")
    
    def add_detection(self, camera_id: int, detection_result: dict):
        """
        接收一条检测结果（来自某个生产者）
        
        Args:
            camera_id: 摄像头ID (1/2/3)
            detection_result: 包含 frame_id 和 detections 的结果字典
        """
        frame_id = detection_result.get('frame_id')
        if frame_id is None:
            print(f"⚠️  警告: C{camera_id} 检测结果缺少 frame_id，已丢弃")
            return
        
        # 初始化该帧的缓冲区（如果不存在）
        if frame_id not in self.frame_buffer:
            self.frame_buffer[frame_id] = {}
            self.frame_timestamps[frame_id] = time.time()
        
        # 存储检测结果
        self.frame_buffer[frame_id][camera_id] = detection_result
        self.stats['detections_received'] += 1
        
        # 更新缓冲区统计
        self.stats['max_buffer_size'] = max(self.stats['max_buffer_size'], len(self.frame_buffer))
    
    def try_assemble(self) -> Optional[dict]:
        """
        尝试组装一个完整的三路帧。返回齐全的帧，或 None。
        
        Returns:
            如果某个帧号的三路数据齐全，返回打包的字典；否则返回 None
            打包格式：{frame_id: int, cameras: {1: result, 2: result, 3: result}}
        """
        current_time = time.time()
        
        # 遍历缓冲区，寻找齐全或超时的帧
        frame_ids_to_process = sorted(self.frame_buffer.keys())
        
        for frame_id in frame_ids_to_process:
            frame_data = self.frame_buffer[frame_id]
            arrival_time = self.frame_timestamps[frame_id]
            elapsed_ms = (current_time - arrival_time) * 1000
            
            # 检查是否齐全
            if len(frame_data) == self.num_cameras:
                # 完整的三路帧，直接返回
                assembled_frame = {
                    'frame_id': frame_id,
                    'cameras': frame_data,
                    'assembled_time': current_time
                }
                
                # 从缓冲区移除
                del self.frame_buffer[frame_id]
                del self.frame_timestamps[frame_id]
                
                self.stats['frames_assembled'] += 1
                return assembled_frame
            
            # 检查是否超时
            if elapsed_ms > self.timeout_ms:
                # 超时了，放弃这一帧
                print(f"⏱️  Frame {frame_id} 超时 ({elapsed_ms:.1f}ms > {self.timeout_ms:.1f}ms), 已有 {len(frame_data)}/3 摄像头数据，放弃")
                
                # 从缓冲区移除
                del self.frame_buffer[frame_id]
                del self.frame_timestamps[frame_id]
                
                self.stats['frames_timeout'] += 1
                # 不返回，继续查找下一个可能齐全的帧
                continue
        
        # 没有齐全或超时的帧
        return None
    
    def run_loop(self, stop_event: threading.Event):
        """
        在独立线程中运行的主循环。持续监听三个检测队列，并尝试组装帧。
        """
        print("🚀 FrameAssembler 线程启动")
        
        while not stop_event.is_set():
            # 从三个队列中读取所有可用的检测结果
            for camera_id in [1, 2, 3]:
                try:
                    detection_result = self.detection_queues[camera_id].get_nowait()
                    self.add_detection(camera_id, detection_result)
                except:
                    # 队列为空或其他错误，继续
                    pass
            
            # 尝试组装齐全的帧
            assembled_frame = self.try_assemble()
            while assembled_frame:
                # 发送到融合队列
                try:
                    self.fusion_queue.put(assembled_frame, timeout=0.1)
                except:
                    # 融合队列满，等待一下
                    time.sleep(0.001)
                    continue
                
                # 尝试继续组装
                assembled_frame = self.try_assemble()
            
            # 短暂休眠，避免 CPU 占用过高
            time.sleep(0.001)
        
        print("🛑 FrameAssembler 线程已退出")
    
    def get_statistics(self) -> dict:
        """获取统计信息"""
        return {
            'frames_assembled': self.stats['frames_assembled'],
            'frames_timeout': self.stats['frames_timeout'],
            'detections_received': self.stats['detections_received'],
            'buffer_size': len(self.frame_buffer),
            'max_buffer_size': self.stats['max_buffer_size']
        }

# --- 主程序 ---
if __name__ == "__main__":
    
    # 1. 配置
    print("🚀 程序开始启动 - 重构版融合系统")
    
    # 尝试从配置文件读取RTSP URLs，失败则使用本地视频文件
    video_paths = {}
    if RTSP_MQTT_AVAILABLE:
        try:
            config_reader = ConfigReader()
            enabled_cameras = config_reader.get_enabled_cameras()
            
            if enabled_cameras and len(enabled_cameras) >= 3:
                for i, camera in enumerate(enabled_cameras[:3], 1):
                    video_paths[i] = camera['rtsp_url']
                    print(f"📷 摄像头 {i}: {camera['name']} - {camera['rtsp_url']}")
                print("✅ 使用RTSP流作为输入")
            else:
                print("⚠️  配置文件中摄像头数量不足，回退到本地视频文件")
                raise Exception("配置不足")
        except Exception as e:
            print(f"⚠️  读取RTSP配置失败: {e}, 使用本地视频文件")
            RTSP_MQTT_AVAILABLE = False
    
    # 回退到本地视频文件
    if not RTSP_MQTT_AVAILABLE or not video_paths:
        print("❌ RTSP拉流失败，需要配置本地视频路径")
        # 这里需要用户手动配置视频路径
        video_paths = {
            1: "/path/to/video1.mp4",
            2: "/path/to/video2.mp4",
            3: "/path/to/video3.mp4"
        }

    # 检测区域（像素坐标）
    detect_areas = {
        1: [np.array([[0, 1080], [1920, 1080], [1920, 592], [1108, 139], [726, 125], [0, 797]], dtype=np.int32),
            np.array([[64, 647], [355, 474], [621, 224], [494, 228]], dtype=np.int32)],
        2: [np.array([[0, 792], [716, 181], [1238, 175], [1920, 628], [1920, 1080]], dtype=np.int32)],
        3: [np.array([[0, 1080], [1185, 1080], [988, 257], [0, 276]], dtype=np.int32)]
    }
        
    signal.signal(signal.SIGINT, cancel_process)
    
    # 2. 初始化核心组件
    fusion_system = CrossCameraFusion()
    queues = {i: multiprocessing.Queue(maxsize=10) for i in [1, 2, 3]}
    perf_monitor = PerformanceMonitor()
    
    # 新增：初始化速度监控
    speed_monitor = SpeedMonitor(window_size=300)
    
    # 初始化MQTT发布器
    mqtt_publisher = None
    if RTSP_MQTT_AVAILABLE:
        try:
            mqtt_publisher = MqttPublisher("config/mqtt_config.ini")
            mqtt_publisher.connect()
            print("✅ MQTT发布器已连接")
        except Exception as e:
            print(f"⚠️  MQTT连接失败: {e}, 将使用JSON文件保存")
            mqtt_publisher = None
    
    # 初始化ByteTracker
    class TrackerArgs:
        def __init__(self):
            self.track_thresh = 0.3
            self.track_buffer = 15
            self.match_thresh = 0.8
            self.mot20 = False  # 添加mot20属性，用于区分MOT20数据集
    
    tracker_args = TrackerArgs()
    trackers = {i: BYTETracker(tracker_args, frame_rate=Config.FPS) for i in [1, 2, 3]}
    print("✅ ByteTracker初始化完成") 

    # 3. 创建并启动 SDK 推理进程
    processes = []
    print("🚀 启动SDK版多摄像头融合系统")
    for camera_id in [1, 2, 3]:
        process = multiprocessing.Process(
            target=create_sdk_worker_process,
            args=(camera_id, video_paths[camera_id], queues[camera_id]),
            daemon=True
        )
        processes.append(process)
        process.start()
        print(f"🔄 启动Camera{camera_id} SDK推理进程...")
        
    print("✅ 所有SDK推理进程已启动")

    # 3.5. 预热阶段
    print("\n⏱️  进入预热阶段，等待所有摄像头推送第一帧数据...")
    PREHEAT_TIMEOUT = 30
    start_time = time.time()
    ready_cameras = {i: False for i in [1, 2, 3]}
    
    while time.time() - start_time < PREHEAT_TIMEOUT:
        all_ready = True
        for cam_id in [1, 2, 3]:
            if not ready_cameras[cam_id] and not queues[cam_id].empty():
                ready_cameras[cam_id] = True
                print(f"✅ 摄像头 C{cam_id} 已就绪！")
            elif not ready_cameras[cam_id]:
                all_ready = False
        
        if all(ready_cameras.values()):
            print("🎉 所有摄像头均已就绪，预热完成！")
            break
        
        time.sleep(0.5)
    else:
        print("❌ 预热超时！")

    # 4. 主循环：三阶段生产者-消费者架构
    max_frames = 500
    
    # 创建融合队列（用于阶段二和阶段三之间的通信）
    fusion_queue = multiprocessing.Queue(maxsize=20)
    
    # 创建帧汇聚器（阶段二：Frame Assembler）
    frame_assembler = FrameAssembler(
        detection_queues=queues,
        fusion_queue=fusion_queue,
        num_cameras=3,
        fps=Config.FPS,
        timeout_frames=10
    )
    
    # 启动 FrameAssembler 线程
    assembler_stop_event = threading.Event()
    assembler_thread = threading.Thread(
        target=frame_assembler.run_loop,
        args=(assembler_stop_event,),
        daemon=True,
        name="FrameAssembler"
    )
    assembler_thread.start()
    print("✅ FrameAssembler 线程已启动")
    
    # 新增：详细计时统计
    detailed_timing_stats = {
        'frame_sync': [],      # 从融合队列获取帧的时间
        'detection': [],       # 检测耗时
        'tracking': [],        # 跟踪耗时
        'fusion': [],          # 融合耗时
        'json_save': [],       # JSON保存耗时
        'total': []            # 总耗时
    }
    
    print("\n--- 融合主循环启动 (三阶段架构) ---")

    try:
        current_frame = 0
        last_assembled_frame_id = -1
        
        while not cancel_flag.value and current_frame < max_frames:
            # ============ 主循环总耗时计时开始 ============
            loop_start_time = time.time()
            
            # A. 从融合队列获取完整的三路数据
            # ============ 帧同步计时 ============
            frame_sync_start = time.time()
            
            try:
                assembled_frame = fusion_queue.get(timeout=0.5)
            except:
                # 融合队列超时，继续等待
                time.sleep(0.001)
                continue
            
            frame_sync_time = (time.time() - frame_sync_start) * 1000
            detailed_timing_stats['frame_sync'].append(frame_sync_time)
            
            # 解包组装好的帧
            frame_id = assembled_frame['frame_id']
            cameras_data = assembled_frame['cameras']  # {1: result, 2: result, 3: result}
            
            current_frame = frame_id
            perf_monitor.add_counter('frames_synchronized')
            perf_monitor.add_counter('frames_processed')
            
            # B. 为每个摄像头进行检测、跟踪、融合处理
            all_global_targets_this_frame = []
            all_local_targets_this_frame = []
            
            # ============ 检测和跟踪计时统计 ============
            detection_times = []
            tracking_times = []
            
            perf_monitor.start_timer('frame_processing')
            
            for camera_id in [1, 2, 3]:
                result = cameras_data[camera_id]
                perf_monitor.start_timer(f'camera_{camera_id}_processing')
                
                # 记录帧到达
                speed_monitor.record_frame_arrival(camera_id, frame_id)
                
                # 1. 类别过滤和区域过滤
                raw_detections = [d for d in result['detections'] 
                                if d['class'] in Config.VEHICLE_CLASSES and d['class'] not in Config.EXCLUDE_CLASSES]
                perf_monitor.add_counter('detections_processed', len(raw_detections))

                filtered_detections = filter_by_detect_areas(raw_detections, detect_areas[camera_id])

                # 2. NMS
                perf_monitor.start_timer(f'detection_{camera_id}')
                det_for_nms = [{'box': d['box'], 'confidence': d['confidence'], 'class': d['class']} for d in filtered_detections]
                nms_detections = DetectionUtils.non_max_suppression(det_for_nms)
                detection_time = perf_monitor.end_timer(f'detection_{camera_id}')
                
                # 记录检测耗时
                speed_monitor.record_detection_time(camera_id, detection_time)
                detection_times.append(detection_time)
                
                # 3. 跟踪器输入
                tracker_input_tensor = batch_prepare_tracker_input(nms_detections)
                
                # 4. 局部跟踪
                perf_monitor.start_timer(f'tracker_update_{camera_id}')
                
                # ByteTracker的update方法需要3个参数：output_results, img_info, img_size
                img_info = [Config.IMAGE_HEIGHT, Config.IMAGE_WIDTH]  # [height, width]
                img_size = (Config.IMAGE_HEIGHT, Config.IMAGE_WIDTH)  # (height, width)
                tracked_objects = trackers[camera_id].update(tracker_input_tensor, img_info, img_size)

                tracker_time = perf_monitor.end_timer(f'tracker_update_{camera_id}')
                perf_monitor.add_counter('tracker_updates')
                
                # 记录跟踪耗时
                speed_monitor.record_tracking_time(camera_id, tracker_time)
                tracking_times.append(tracker_time)

                # 5. 跟踪结果转换
                tracked_detections = batch_convert_track_results(tracked_objects, result, camera_id, current_frame, nms_detections)
                
                # 6. 分类处理
                global_targets, local_targets = fusion_system.process_detections(tracked_detections, camera_id, perf_monitor)
                
                all_global_targets_this_frame.extend(global_targets)
                all_local_targets_this_frame.extend(local_targets)
                
                camera_processing_time = perf_monitor.end_timer(f'camera_{camera_id}_processing')
                perf_monitor.record_fusion_stats(f'camera_{camera_id}_processing', camera_processing_time, {
                    'raw_detections': len(raw_detections),
                    'tracked_detections': len(tracked_detections),
                    'global_targets': len(global_targets),
                    'local_targets': len(local_targets)
                })
            
            frame_processing_time = perf_monitor.end_timer('frame_processing')
            detailed_timing_stats['detection'].append(sum(detection_times))
            detailed_timing_stats['tracking'].append(sum(tracking_times))

            # C. 匹配融合（跨摄像头）
            perf_monitor.start_timer('matching_processing')
            active_global_targets = list(fusion_system.global_targets.values())
            fusion_system._perform_matching(all_local_targets_this_frame, active_global_targets)
            
            # 更新全局状态
            fusion_system.update_global_state(all_global_targets_this_frame, all_local_targets_this_frame)
            matching_time = perf_monitor.end_timer('matching_processing')
            
            # 记录融合耗时
            speed_monitor.record_fusion_time(matching_time)
            detailed_timing_stats['fusion'].append(matching_time)
            
            # 收集所有目标用于JSON输出
            all_frame_detections = all_global_targets_this_frame + all_local_targets_this_frame

            # D. 生成JSON数据并尝试发送MQTT
            # ============ JSON保存计时 ============
            json_save_start = time.time()
            
            perf_monitor.start_timer('json_mqtt_processing')
            json_data = fusion_system.generate_json_data_new(all_global_targets_this_frame, all_local_targets_this_frame)
            
            mqtt_sent = False
            if mqtt_publisher:
                try:
                    participants = json_data.get('participant', [])
                    mqtt_sent = mqtt_publisher.publish_rsm(participants)
                    if mqtt_sent:
                        perf_monitor.add_counter('mqtt_sends')
                    else:
                        perf_monitor.add_counter('mqtt_failures')
                except Exception as e:
                    print(f"❌ MQTT发送异常: {e}")
                    perf_monitor.add_counter('mqtt_failures')
                    mqtt_sent = False
            
            if not mqtt_sent:
                fusion_system.json_output_data.append(json_data)
            
            # 实时写入JSON文件
            try:
                fusion_system.save_json_data_realtime("output_fusion_refactored.json", current_frame)
            except Exception as e:
                print(f"❌ 实时JSON保存失败: {e}")
            
            json_mqtt_time = perf_monitor.end_timer('json_mqtt_processing')
            detailed_timing_stats['json_save'].append(json_mqtt_time)
            
            # 打印处理信息
            print(f"✅ 融合帧 {current_frame} | 目标数: {len(all_frame_detections)} | MQTT: {'成功' if mqtt_sent else '失败/未配置'}")
            
            fusion_system.next_frame()
            
            # 定期输出性能报告
            perf_report = perf_monitor.get_performance_report()
            if perf_report:
                print(perf_report)
            
            # 定期输出速度监控报告
            speed_report = speed_monitor.print_speed_report()
            if speed_report:
                print(speed_report)

            # E. 定期报告汇聚器统计
            if current_frame > 0 and current_frame % 300 == 0:
                assembler_stats = frame_assembler.get_statistics()
                print(f"\n📊 ----- FrameAssembler 统计 (截至Frame {current_frame}) -----")
                print(f"  已组装帧: {assembler_stats['frames_assembled']}")
                print(f"  超时帧: {assembler_stats['frames_timeout']}")
                print(f"  接收检测数: {assembler_stats['detections_received']}")
                print(f"  当前缓冲区大小: {assembler_stats['buffer_size']}")
                print(f"  最大缓冲区大小: {assembler_stats['max_buffer_size']}")
                print("-" * 50)
        
        print("\n🎯 所有处理完成 (或达到最大帧数)")
        
        # 5. 保存融合结果
        fusion_system.save_json_data("output_fusion_refactored.json")
        
        # 6. 输出最终统计
        print("\n" + "="*60)
        print("📊 最终统计报告:")
        
        assembler_stats = frame_assembler.get_statistics()
        synchronized_frames_count = assembler_stats['frames_assembled']

        print(f"📈 处理概况:")
        print(f"  成功组装帧组: {synchronized_frames_count} 组")
        print(f"  超时丢弃帧: {assembler_stats['frames_timeout']} 个")
        print(f"  接收检测总数: {assembler_stats['detections_received']}")
        print(f"  Global车辆池大小: {len(fusion_system.global_targets)}")
        print(f"  Local车辆池大小: {len(fusion_system.local_track_buffer)}")
        print(f"  确认目标数: {len(fusion_system.confirmed_targets)}")

        # 输出详细计时统计
        print(f"\n📊 详细计时统计:")
        for stage, times in detailed_timing_stats.items():
            if times:
                print(f"  {stage}: 平均{sum(times)/len(times):.2f}ms, 最大{max(times):.2f}ms, 最小{min(times):.2f}ms, 次数{len(times)}")

        # 输出详细处理时间分解报告
        print("\n" + "="*70)
        print("📊 详细处理时间分解报告")
        print("="*70)
        
        if any(detailed_timing_stats.values()):
            print("\n⏱️  各阶段平均耗时:")
            total_accounted = 0
            for stage_name, times in detailed_timing_stats.items():
                if times:
                    avg_time = sum(times) / len(times)
                    max_time = max(times)
                    min_time = min(times)
                    total_accounted += avg_time
                    print(f"  {stage_name:15} | 平均: {avg_time:7.2f}ms | 最大: {max_time:7.2f}ms | 最小: {min_time:7.2f}ms | 样本: {len(times):3d}")
            
            print(f"\n  {'各阶段合计':15} | 总耗时: {total_accounted:.2f}ms")
            print(f"  {'处理速度':15} | {1000.0 / total_accounted:.2f} FPS")
            
            print("\n💡 优化建议:")
            if synchronized_frames_count < 100:
                print(f"  ⚠️  同步帧数较少 ({synchronized_frames_count})，可能摄像头间同步问题")
            if assembler_stats['frames_timeout'] > synchronized_frames_count * 0.1:
                print(f"  ⚠️  超时帧比例过高 ({assembler_stats['frames_timeout']}/{synchronized_frames_count})")
        
        print("="*70)

    except Exception as e:
        print(f"❌ 主程序执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 7. 清理资源
        print("\n🧹 开始清理资源...")
        
        # 停止 FrameAssembler 线程
        assembler_stop_event.set()
        assembler_thread.join(timeout=5.0)
        
        # 停止 SDK 推理进程
        cancel_flag.value = True
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(timeout=2.0)
        
        # 关闭异步JSON保存器
        if 'fusion_system' in locals():
            fusion_system.json_saver.shutdown()

        if mqtt_publisher:
            try:
                mqtt_publisher.disconnect()
                print("✅ MQTT连接已断开")
            except:
                pass
                
        print("🧹 资源清理完成")