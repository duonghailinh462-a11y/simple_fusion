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


# --- 性能监视器类 ---
import logging

logger = logging.getLogger(__name__)

class PerformanceMonitor:
    """简化的性能监视器 - 只保留基本计数和FPS统计"""
    
    def __init__(self):
        self.counters = defaultdict(int)
        self.timers = {}
        self.start_time = time.time()
        self.last_report_time = time.time()
        self.report_interval = 10.0
        
        logger.info("性能监视器初始化完成")
    
    def start_timer(self, name: str):
        """开始计时"""
        self.timers[name] = time.time()
    
    def end_timer(self, name: str) -> float:
        """结束计时并返回耗时（毫秒）"""
        if name not in self.timers:
            return 0.0
        elapsed = (time.time() - self.timers[name]) * 1000
        del self.timers[name]
        return elapsed
    
    def add_counter(self, name: str, value: int = 1):
        """增加计数器"""
        self.counters[name] += value
    
    def record_queue_stats(self, camera_id: int, queue_size: int, operation: str):
        """记录队列状态（简化版）"""
        pass  # 如果需要可以添加简单的日志
    
    def record_fusion_stats(self, operation: str, duration_ms: float, details: dict = None):
        """记录融合统计（简化版）"""
        pass  # 如果需要可以添加简单的日志
    
    def get_performance_report(self) -> str:
        """生成简化的性能报告"""
        current_time = time.time()
        elapsed = current_time - self.last_report_time
        
        if elapsed < self.report_interval:
            return ""
        
        total_elapsed = current_time - self.start_time
        self.last_report_time = current_time
        
        # 简单的FPS计算
        fps = self.counters.get('frames_processed', 0) / total_elapsed if total_elapsed > 0 else 0
        sync_fps = self.counters.get('frames_synchronized', 0) / total_elapsed if total_elapsed > 0 else 0
        
        report = []
        report.append(f"\n{'='*60}")
        report.append(f"📊 性能报告 (运行时间: {total_elapsed:.1f}s)")
        report.append(f"{'='*60}")
        report.append(f"处理速度: {fps:.2f} FPS (同步: {sync_fps:.2f} FPS)")
        report.append(f"总帧数: {self.counters.get('frames_processed', 0)}")
        report.append(f"同步帧数: {self.counters.get('frames_synchronized', 0)}")
        report.append(f"检测数: {self.counters.get('detections_processed', 0)}")
        report.append(f"BEV转换: {self.counters.get('bev_conversions', 0)}")
        report.append(f"MQTT发送: {self.counters.get('mqtt_sends', 0)} (失败: {self.counters.get('mqtt_failures', 0)})")
        report.append(f"{'='*60}")
        
        return "\n".join(report)
    
    def reset_counters(self):
        """重置计数器"""
        self.counters.clear()
        self.start_time = time.time()
        self.last_report_time = time.time()
# --- 配置类 (按职责拆分) ---

@dataclass
class ImageConfig:
    """图像和视频相关配置"""
    WIDTH: int = 1280
    HEIGHT: int = 720
    FPS: int = 25

@dataclass
class TrackingConfig:
    """目标跟踪相关配置"""
    TRACK_THRESH: float = 0.3
    MATCH_THRESH: float = 0.6
    MIN_FRAMES_THRESHOLD: int = 10
    IOU_THRESHOLD: float = 0.5
    TOLERANCE_FRAMES: int = 60

@dataclass
class VehicleConfig:
    """车辆类别相关配置"""
    VEHICLE_CLASSES: List[str] = None
    EXCLUDE_CLASSES: List[str] = None
    SIMILAR_CLASSES: Dict[str, List[str]] = None
    
    def __post_init__(self):
        if self.VEHICLE_CLASSES is None:
            self.VEHICLE_CLASSES = [
                'mini_truck', 'truck', 'bus', 'van', 'car', 'ambulance',
                'fireEngine', 'schoolBus', 'tanker', 'muckTruck',
                'concreteTruck', 'policeCar'
            ]
        if self.EXCLUDE_CLASSES is None:
            self.EXCLUDE_CLASSES = ["person", "electric_vehicle", "bike", "tricycle", "engineer"]
        if self.SIMILAR_CLASSES is None:
            self.SIMILAR_CLASSES = {
                'mini_truck': ['truck', 'van', 'car'],
                'truck': ['mini_truck', 'van', 'car'],
                'van': ['mini_truck', 'truck', 'car'],
                'car': ['van', 'mini_truck', 'truck'],
                'bus': ['truck', 'van', 'car'],
            }


@dataclass
class FusionConfig:
    """跨摄像头融合相关配置"""
    TIME_WINDOW: int = 80
    TEMPORAL_WINDOW_MAX: int = 225
    BASE_SPATIAL_THRESHOLD: float = 400.0
    
    # 按对队列融合配置
    ENABLE_SEQ_MATCHING: bool = True
    RESERVATION_TTL_FRAMES: int = 60
    MAX_RETENTION_FRAMES: int = 200
    TIME_WINDOW_STRICT: int = 30
    TIME_WINDOW_FLEXIBLE: int = 60
    
    # 🔧 融合时间窗口
    FUSION_TIME_WINDOW: int = 60
    
    # C2 出口区域 (BEV坐标)
    C2_EXIT_REGION_C3: np.ndarray = None
    C2_EXIT_REGION_C1: np.ndarray = None
    
    # 雷视融合区域 (像素坐标) - 用于标记可进行雷视融合的区域
    RADAR_VISION_FUSION_AREAS: Dict[int, np.ndarray] = None
    
    
    def __post_init__(self):
        if self.C2_EXIT_REGION_C3 is None:
            self.C2_EXIT_REGION_C3 = np.array(
                [[1022, 654], [1092, 605], [1082, 589], [1011, 642]], dtype=np.int32
            )
        if self.C2_EXIT_REGION_C1 is None:
            self.C2_EXIT_REGION_C1 = np.array(
                [[1022, 654], [1011, 642], [871, 735], [884, 757]], dtype=np.int32
            )
        if self.RADAR_VISION_FUSION_AREAS is None:
            self.RADAR_VISION_FUSION_AREAS = {
                1: np.array([[110, 429], [0, 536], [0, 720], [1280, 720], [1280, 458]], dtype=np.int32),
                2: np.array([[0, 720], [1280, 720], [1280, 418], [109, 432]], dtype=np.int32),
                3: np.array([[328, 472], [186, 720], [1033, 720], [985, 468]], dtype=np.int32),
            }

@dataclass
class TimestampConfig:
    """时间戳相关配置"""
    CAMERA_START_DATETIMES: Dict[int, str] = None
    
    def __post_init__(self):
        if self.CAMERA_START_DATETIMES is None:
            self.CAMERA_START_DATETIMES = {
                1: "2025-11-21 11:59:12.135",
                2: "2025-11-21 11:59:12.150",
                3: "2025-11-21 11:59:12.143",
            }

class _Config:
    """统一的配置管理类 - 使用组合模式"""
    def __init__(self):
        self.image = ImageConfig()
        self.tracking = TrackingConfig()
        self.vehicle = VehicleConfig()
        self.fusion = FusionConfig()
        self.timestamp = TimestampConfig()
    
    # 为了向后兼容，提供属性访问
    @property
    def IMAGE_WIDTH(self): return self.image.WIDTH
    @property
    def IMAGE_HEIGHT(self): return self.image.HEIGHT
    @property
    def FPS(self): return self.image.FPS
    @property
    def TRACK_THRESH(self): return self.tracking.TRACK_THRESH
    @property
    def MATCH_THRESH(self): return self.tracking.MATCH_THRESH
    @property
    def MIN_FRAMES_THRESHOLD(self): return self.tracking.MIN_FRAMES_THRESHOLD
    @property
    def IOU_THRESHOLD(self): return self.tracking.IOU_THRESHOLD
    @property
    def TOLERANCE_FRAMES(self): return self.tracking.TOLERANCE_FRAMES
    @property
    def VEHICLE_CLASSES(self): return self.vehicle.VEHICLE_CLASSES
    @property
    def EXCLUDE_CLASSES(self): return self.vehicle.EXCLUDE_CLASSES
    @property
    def SIMILAR_CLASSES(self): return self.vehicle.SIMILAR_CLASSES
    @property
    def TIME_WINDOW(self): return self.fusion.TIME_WINDOW
    @property
    def TEMPORAL_WINDOW_MAX(self): return self.fusion.TEMPORAL_WINDOW_MAX
    @property
    def BASE_SPATIAL_THRESHOLD(self): return self.fusion.BASE_SPATIAL_THRESHOLD
    @property
    def ENABLE_SEQ_MATCHING(self): return self.fusion.ENABLE_SEQ_MATCHING
    @property
    def RESERVATION_TTL_FRAMES(self): return self.fusion.RESERVATION_TTL_FRAMES
    @property
    def MAX_RETENTION_FRAMES(self): return self.fusion.MAX_RETENTION_FRAMES
    @property
    def TIME_WINDOW_STRICT(self): return self.fusion.TIME_WINDOW_STRICT
    @property
    def TIME_WINDOW_FLEXIBLE(self): return self.fusion.TIME_WINDOW_FLEXIBLE
    @property
    def FUSION_TIME_WINDOW(self): return self.fusion.FUSION_TIME_WINDOW
    @property
    def PIXEL_BOTTOM_THRESHOLD(self): return self.fusion.PIXEL_BOTTOM_THRESHOLD
    @property
    def PIXEL_TOP_THRESHOLD(self): return self.fusion.PIXEL_TOP_THRESHOLD
    @property
    def C2_EXIT_REGION_C3(self): return self.fusion.C2_EXIT_REGION_C3
    @property
    def C2_EXIT_REGION_C1(self): return self.fusion.C2_EXIT_REGION_C1
    @property
    def RADAR_VISION_FUSION_AREAS(self): return self.fusion.RADAR_VISION_FUSION_AREAS
    @property
    def CAMERA_START_DATETIMES(self): return self.timestamp.CAMERA_START_DATETIMES

# 矩阵和区域配置
CAMERA_MATRICES = {
    1: np.array([
        [3.57185777, -95.12052479, 4179.24844873],
        [3.46221359, -30.18092945, -4782.56623337],
        [0.00086667, -0.07779328, 1.00000000]
    ], dtype=np.float64),
    2: np.array([
        [-3.14205205, -15.41287574, -466.38259912],
        [-3.41382642, -24.02931190, 3191.87948399],
        [-0.00259235, -0.02359469, 1.00000000]
    ], dtype=np.float64),
    3: np.array([
        [2.30699835, -25.77644591, -1583.82133879],
        [-0.42448874, -13.71274357, -988.28445704],
        [0.00112695, -0.03632265, 1.00000000]
    ], dtype=np.float64),
}

# 🔧 新增：BEV到世界米制坐标的变换矩阵
BEV_TO_WORLD_METER_MATRIX = np.array([
    [1.32977514e-01, -1.04276598e-04, -1.50540001e+02],
    [1.45689395e-03, -1.33712569e-01, 7.61259809e+01],
    [-1.97872483e-06, -2.12579392e-05, 1.00000000e+00]
], dtype=np.float64)

# 🔧 新增：BEV到地理坐标的变换矩阵（使用BEV_TO_WORLD_METER_MATRIX作为基础）
BEV_TO_GEO_MATRIX = BEV_TO_WORLD_METER_MATRIX

# 🔧 新增：地理坐标原点 (参考点)
# 这是 calculate_geo.py 中使用的第一个点坐标
GEO_ORIGIN_LON = 113.584439426
GEO_ORIGIN_LAT = 23.530769118

# 🔧 新增：地球相关常数用于经纬度转换
import math
EARTH_RADIUS = 6378137.0  # 地球半径 (米)
METERS_PER_DEG_LAT = (math.pi / 180.0) * EARTH_RADIUS
METERS_PER_DEG_LON = (math.pi / 180.0) * EARTH_RADIUS * math.cos(math.radians(GEO_ORIGIN_LAT))

PUBLIC_AREA_BEV = np.array([[1075, 606], [1066, 575], [850, 747], [877, 760]], dtype=np.int32)

# YOLOv5 类别名称
NAMES = [
    'mini_truck','truck','bus','van','car','person','bike','electric_vehicle',
    'tricycle','engineer','ambulance','fireEngine','schoolBus','tanker','muckTruck',
    'concreteTruck','policeCar'
]

# --- 几何和检测工具类 ---
class GeometryUtils:
    @staticmethod
    def project_pixel_to_bev(H: np.ndarray, u: float, v: float) -> Optional[Tuple[float, float]]:
        """单个像素到BEV的转换"""
        p = np.array([u, v, 1.0])
        q = H @ p
        if abs(q[2]) < 1e-8: return None
        x, y = q[0] / q[2], q[1] / q[2]
        return (x, y)


    @staticmethod
    def bev_to_geo(x_bev: float, y_bev: float) -> Optional[Tuple[float, float]]:
        """BEV像素 -> 世界米制坐标 -> 地理坐标 (经纬度)"""
        try:
            # 步骤1: BEV像素 -> 世界米制坐标
            p = np.array([x_bev, y_bev, 1.0])
            q = BEV_TO_GEO_MATRIX @ p
            q /= q[2]
            x_meters = q[0]  # 相对于原点的X偏移 (米)
            y_meters = q[1]  # 相对于原点的Y偏移 (米)
            
            # 步骤2: 世界米制坐标 -> 地理坐标 (经纬度)
            # (米 / 每度米数) + 原点度数
            lon = (x_meters / METERS_PER_DEG_LON) + GEO_ORIGIN_LON
            lat = (y_meters / METERS_PER_DEG_LAT) + GEO_ORIGIN_LAT
            
            return lon, lat
        except: 
            return None


    @staticmethod
    def calculate_iou(box1: List[float], box2: List[float]) -> float:
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        x1_i, y1_i = max(x1_1, x1_2), max(y1_1, y1_2)
        x2_i, y2_i = min(x2_1, x2_2), min(y2_1, y2_2)
        if x2_i <= x1_i or y2_i <= y1_i: return 0.0
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        return intersection / union if union > 0 else 0.0

    @staticmethod
    def is_in_public_area(bev_point: Tuple[float, float]) -> bool:
        return cv2.pointPolygonTest(PUBLIC_AREA_BEV, bev_point, False) >= 0
    
    @staticmethod
    def is_in_radar_vision_fusion_area(pixel_point: Tuple[int, int], camera_id: int) -> bool:
        """检查像素点是否在该摄像头的雷视融合区域内"""
        # 获取Config实例的融合区域配置
        fusion_areas = Config.RADAR_VISION_FUSION_AREAS
        if not fusion_areas or camera_id not in fusion_areas:
            return False
        fusion_area = fusion_areas[camera_id]
        return cv2.pointPolygonTest(fusion_area, pixel_point, False) >= 0

class DetectionUtils:
    @staticmethod
    def is_class_compatible(class1: str, class2: str) -> bool:
        if class1 == class2: return True
        return (class1 in Config.SIMILAR_CLASSES and 
                class2 in Config.SIMILAR_CLASSES[class1])

    @staticmethod
    def non_max_suppression(detections: List[dict], 
                          iou_threshold: float = None) -> List[dict]:
        if iou_threshold is None:
            iou_threshold = Config.IOU_THRESHOLD
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
        
# --- 平滑滤波器类 ---
class SmoothingFilter:
    """
    对目标的BEV坐标进行平滑处理。
    """
    def __init__(self, history_len: int = 10, alpha: float = 0.5):
        # 存储每个目标 (track_id) 的BEV坐标历史
        self.bev_history = defaultdict(lambda: deque(maxlen=history_len))
        # 指数平滑的权重因子 (0 < alpha < 1)，数值越小越平滑
        self.alpha = alpha
        
    def _sliding_average(self, track_id: int, current_bev: Tuple[float, float]) -> Tuple[float, float]:
        """滑动平均平滑 (Moving Average)"""
        self.bev_history[track_id].append(current_bev)
        
        history = self.bev_history[track_id]
        if len(history) < 2:
            return current_bev
            
        # 计算历史点的平均值
        avg_x = sum(p[0] for p in history) / len(history)
        avg_y = sum(p[1] for p in history) / len(history)
        
        return (avg_x, avg_y)

    def _exponential_smoothing(self, track_id: int, current_bev: Tuple[float, float]) -> Tuple[float, float]:
        """指数平滑 (Exponential Smoothing)"""
        history_deque = self.bev_history[track_id]
        
        if not history_deque:
            # 第一次测量，直接使用当前值作为平滑值
            smoothed_bev = current_bev
        else:
            last_smoothed_bev = history_deque[-1]
            
            # P'_t = alpha * P_t + (1 - alpha) * P'_{t-1}
            smoothed_x = self.alpha * current_bev[0] + (1 - self.alpha) * last_smoothed_bev[0]
            smoothed_y = self.alpha * current_bev[1] + (1 - self.alpha) * last_smoothed_bev[1]
            smoothed_bev = (smoothed_x, smoothed_y)

        history_deque.append(smoothed_bev)
        return smoothed_bev
    
    def apply_smoothing(self, track_id: int, current_bev: Tuple[float, float], method: str = 'exponential') -> Tuple[float, float]:
        """应用平滑算法并返回平滑后的BEV坐标"""
        if method == 'sliding':
            return self._sliding_average(track_id, current_bev)
        else: # 默认为指数平滑，通常效果更好
            return self._exponential_smoothing(track_id, current_bev)

    def remove_track(self, track_id: int):
        """移除不再活跃的目标的历史记录"""
        self.bev_history.pop(track_id, None)


# ============================================================================
# 🔧 时间戳提供器使用示例
# ============================================================================
# 
# 在你的主程序中，初始化时间戳提供器之前，先设置摄像头的起始时间：
#
# 示例代码：
# --------
# from ffmpeg_timestamp_sync import FFmpegTimeStampProvider
#
# # 方法1：批量设置所有摄像头的起始时间
# FFmpegTimeStampProvider.set_all_camera_start_datetimes(Config.CAMERA_START_DATETIMES)
#
# # 方法2：单个设置（可选）
# # FFmpegTimeStampProvider.set_camera_start_datetime(1, "2025-11-21 11:18:09.304")
# # FFmpegTimeStampProvider.set_camera_start_datetime(2, "2025-11-21 11:18:09.500")
# # FFmpegTimeStampProvider.set_camera_start_datetime(3, "2025-11-21 11:18:10.000")
#
# # 然后创建时间戳提供器实例
# ts_provider_1 = FFmpegTimeStampProvider("path/to/video1.mp4", camera_id=1, fps=25)
# ts_provider_2 = FFmpegTimeStampProvider("path/to/video2.mp4", camera_id=2, fps=25)
# ts_provider_3 = FFmpegTimeStampProvider("path/to/video3.mp4", camera_id=3, fps=25)
#
# # 获取某一帧的时间戳
# frame_id = 100
# timestamp_1 = ts_provider_1.get_timestamp(frame_id)  # 返回绝对时间字符串
# # 时间戳 = start_datetime + frame_id / fps
# # 例如：2025-11-21 11:18:09.304 + 100/25秒 = 2025-11-21 11:18:13.304
#
# ============================================================================

# 创建全局Config实例，以便直接通过Config.FPS等方式访问
Config = _Config()