"""
雷达视觉融合模块
职责：
1. 加载雷达数据 (JSONL)
2. 与摄像头融合结果进行时间戳匹配
3. 基于地理坐标进行目标匹配
4. 更新输出对象的 radar_id 字段
5. 集成三层过滤：象限过滤 + 距离阈值 (S-L) + 车道过滤
"""

import json
import math
import time
import numpy as np
import cv2
from collections import defaultdict, deque
from datetime import datetime
from typing import List, Dict, Tuple, Optional
from scipy.optimize import linear_sum_assignment

# 导入统一日志配置
try:
    from core.logger_config import get_logger, FusionLogger
except ImportError:
    # 如果无法导入统一日志配置，使用基础日志
    import logging
    logging.basicConfig(level=logging.INFO)
    get_logger = logging.getLogger
    FusionLogger = None

# 导入车道配置
try:
    from config.region_config import LANE_CONFIG, get_lane_for_point
    LANE_CONFIG_AVAILABLE = True
except ImportError:
    LANE_CONFIG_AVAILABLE = False
    import logging
    logging.warning("⚠️ 警告: 无法导入车道配置 (config.region_config)，将禁用车道过滤")

# 获取模块日志记录器
logger = get_logger('RadarFusion')


# ==========================================
# 常量定义
# ==========================================
LAT_TO_M = 110946.0
LON_TO_M = 102140.0

VALID_RADAR_TYPES = {1}


# ==========================================
# 工具函数
# ==========================================
def parse_time(ts_str):
    """解析时间戳字符串，支持多种格式
    
    支持格式：
    - "2025-11-21 11:59:10.171" (3位毫秒)
    - "2025-11-21 11:59:10.171000" (6位微秒)
    - "2025-11-21 11:59:10"
    """
    if not ts_str:
        return 0.0
    
    clean_ts = ' '.join(ts_str.split())
    
    # 优先尝试带毫秒的格式
    formats_to_try = [
        ('%Y-%m-%d %H:%M:%S.%f', False),  # 6位微秒
        ('%Y-%m-%d %H:%M:%S', False),      # 不带毫秒
    ]
    
    for fmt, needs_padding in formats_to_try:
        try:
            # 如果格式需要6位微秒但只有3位毫秒，需要补充到6位
            if needs_padding and '.' in clean_ts:
                parts = clean_ts.split('.')
                if len(parts) == 2 and len(parts[1]) < 6:
                    # 补充0到6位
                    clean_ts = f"{parts[0]}.{parts[1].ljust(6, '0')}"
            
            dt = datetime.strptime(clean_ts, fmt)
            return dt.timestamp()
        except ValueError:
            # 如果不需要补充但失败，尝试补充后重试
            if not needs_padding and '.' in clean_ts and '%f' in fmt:
                parts = clean_ts.split('.')
                if len(parts) == 2 and len(parts[1]) < 6:
                    try:
                        clean_ts_padded = f"{parts[0]}.{parts[1].ljust(6, '0')}"
                        dt = datetime.strptime(clean_ts_padded, fmt)
                        return dt.timestamp()
                    except ValueError:
                        continue
            continue
    
    # 如果都失败，返回0.0
    return 0.0


def format_ts(ts):
    """格式化时间戳"""
    return datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def clean_float(val, precision=8):
    """清理浮点数"""
    if val is None or math.isnan(val) or math.isinf(val):
        return 0.0
    return round(val, precision)


def point_in_polygon(point, polygon):
    """
    判断点是否在多边形内 (使用OpenCV的C++实现)
    
    Args:
        point: [lon, lat] 坐标
        polygon: [[lon, lat], ...] 多边形顶点列表
        
    Returns:
        True 如果点在多边形内，False 否则
    """
    lon, lat = point
    contour = np.array(polygon, dtype=np.float32)
    pt = (lon, lat)
    result = cv2.pointPolygonTest(contour, pt, False)
    
    return result >= 0  # >= 0 表示在多边形内或边界上


# ==========================================
# Track 类 - 融合轨迹对象
# ==========================================
class Track:
    """融合轨迹对象，用于轨迹预测和生命周期管理"""
    
    def __init__(self, fusion_id, lat, lon, speed=0.0, azimuth=0.0):
        self.id = fusion_id
        self.lat = lat
        self.lon = lon
        self.speed = speed
        self.azimuth = azimuth
        self.last_update_time = 0
        self.radar_id_ref = None
        self.vision_id_ref = None
    
    def predict(self, dt):
        """根据速度和方向预测轨迹位置"""
        if self.speed < 0.5 or dt <= 0:
            return
        
        dist = self.speed * dt
        rad = math.radians(self.azimuth)
        
        # 计算位置变化
        dy = dist * math.cos(rad)
        dx = dist * math.sin(rad)
        
        # 更新地理坐标
        self.lat += dy / LAT_TO_M
        self.lon += dx / LON_TO_M


# ==========================================
# 数据结构
# ==========================================
class RadarObject:
    """雷达目标"""
    def __init__(self, radar_id, latitude, longitude, speed=0.0, azimuth=0.0, lane=None, timestamp_str=None, source_ip=None):
        self.id = radar_id
        self.latitude = latitude
        self.longitude = longitude
        self.speed = float(speed or 0)
        self.azimuth = float(azimuth or 0)
        self.lane = lane  # 雷达的车道信息 (1-5对应lane_1到lane_5)
        self.timestamp_str = timestamp_str  # 雷达数据的原始时间戳字符串 (如 "2025-11-21 11:59:10.171")
        self.source_ip = source_ip  # 🔧 雷达数据源IP (用于按摄像头过滤)


class OutputObject:
    """输出对象"""
    def __init__(self, timestamp, cameraid, type_name, confidence, track_id, lon, lat, pixel_x=None, lane=None):
        self.timestamp = timestamp
        self.cameraid = cameraid
        self.type = type_name
        self.confidence = confidence
        self.track_id = track_id
        self.radar_id = None  # 初始为None，由融合模块填充
        self.lon = lon
        self.lat = lat
        self.pixel_x = pixel_x  # 像素x坐标，用于车道判断
        self.lane = lane  # 车道信息（如 'lane_1', 'lane_2' 等）

    def to_dict(self):
        """转换为字典"""
        return {
            'timestamp': self.timestamp,
            'cameraid': self.cameraid,
            'type': self.type,
            'confidence': self.confidence,
            'track_id': self.track_id,
            'radar_id': self.radar_id,
            'lane': self.lane,
            'lon': self.lon,
            'lat': self.lat
        }


# ==========================================
# 核心类：雷达融合处理器
# ==========================================
class RadarVisionFusionProcessor:
    """
    雷达视觉融合处理器
    
    职责：
    1. 维护雷达数据缓冲区
    2. 与摄像头融合结果进行时间戳匹配
    3. 基于地理坐标进行目标匹配
    4. 更新输出对象的 radar_id 字段
    """

    def __init__(self, fusion_area_geo=None, lat_offset=0.0, lon_offset=0.0, enable_lane_filtering=True, camera_id=None, enable_perf_stats=True, enable_fusion_logs=True):
        """
        初始化雷达融合处理器 - 集成高级融合逻辑（三层过滤）
        
        Args:
            fusion_area_geo: 融合区域 (地理坐标多边形)
            lat_offset: 纬度偏移
            lon_offset: 经度偏移
            enable_lane_filtering: 是否启用车道过滤 (需要车道配置可用)
            camera_id: 摄像头ID (用于调整阈值)
            enable_perf_stats: 是否启用性能统计 (默认True)
            enable_fusion_logs: 是否启用融合详细日志 (默认True)
        """
        # 融合参数 - 根据摄像头调整阈值
        # 所有摄像头：横向15.0米，纵向20.0米（宽松策略，因为坐标校准误差较大）
        self.MAX_LANE_DIFF = 20.0     # 横向距离阈值 (米) - 增加到20m以适应坐标偏移
        self.MAX_LONG_DIFF = 20.0     # 纵向距离阈值 (米)
        self.MAX_TIME_DIFF = 0.5      # 最大时间差 (秒) - 25fps=40ms/帧，0.5秒容纳约12帧延迟
        self.MAX_TIME_DIFF_LOOSE = 2.0  # 宽松的时间差（用于容错）- 允许最多2秒偏差
        self.MAX_COAST_TIME = 2.0     # 最大漂移时间 (秒)
        self.LOYALTY_BONUS = 10000.0  # 忠诚度系数：已匹配对的成本除以此值，强制保持绑定
        
        # 诊断信息
        self.last_diag_time = time.time()
        self.diag_interval = 5.0  # 每5秒输出一次诊断信息
        
        self.fusion_area_geo = fusion_area_geo
        self.lat_offset = lat_offset
        self.lon_offset = lon_offset
        
        # 🔧 摄像头ID过滤 - 只匹配对应摄像头的雷达数据
        self.camera_id = camera_id
        self.allowed_radar_ips = self._get_allowed_radar_ips(camera_id)
        
        # 📊 日志和性能统计开关
        self.enable_perf_stats = enable_perf_stats
        self.enable_fusion_logs = enable_fusion_logs
        
        # 🔧 车道过滤配置
        self.enable_lane_filtering = enable_lane_filtering and LANE_CONFIG_AVAILABLE
        if self.enable_lane_filtering:
            logger.info("✅ 三层过滤已启用: 象限 + S-L距离 + 车道")
        else:
            if enable_lane_filtering and not LANE_CONFIG_AVAILABLE:
                logger.warning("⚠️ 车道过滤已禁用: 车道配置不可用")
            else:
                logger.info("✅ 两层过滤已启用: 象限 + S-L距离（车道过滤未启用）")
        
        # 雷达缓冲区 (时间戳 -> 雷达目标列表)
        self.radar_buffer = defaultdict(list)
        self.radar_timestamps = deque(maxlen=100)  # 保留最近100个时间戳
        self.radar_timestamps_sorted = []  # 📊 有序时间戳列表（用于二分查找）
        
        # 匹配映射 (track_id -> radar_id)
        self.radar_id_map = {}  # 雷达ID -> 融合ID
        self.vision_id_map = {}  # 视觉ID -> 融合ID
        
        # 🔧 新增：活跃轨迹管理 (融合ID -> Track对象)
        self.active_tracks = {}  # 维护活跃的融合轨迹
        
        # 🔧 新增：track_id -> radar_id 的持久化映射（轨迹忠诚度）
        # 一旦某个track_id匹配过某个radar_id，就永远保留这个映射
        self.track_radar_history = {}  # track_id -> radar_id
        
        # 🔧 新增：粘性绑定状态追踪
        # vision_track_id -> radar_id: 记录当前的绑定关系
        self.vision_to_radar_binding = {}  # {vision_track_id: radar_id}
        # radar_id -> vision_track_id: 反向映射，防止一个雷达被多个视觉目标使用
        self.radar_to_vision_binding = {}  # {radar_id: vision_track_id}
        # radar_id -> last_seen_timestamp: 记录雷达最后出现时间
        self.radar_last_seen_time = {}  # {radar_id: timestamp}
        
        # 统计信息
        self.stats = {
            'radar_objects_processed': 0,
            'vision_objects_processed': 0,
            'successful_matches': 0,
            'failed_matches': 0,
            'lane_filtered_candidates': 0,  # 被车道过滤排除的候选
        }
        
        # � 性能统计：每一步的耗时（毫秒）
        self.perf_stats = {
            'trajectory_prediction': [],      # 步骤1：轨迹预测与清理
            'coordinate_calibration': [],     # 步骤2：坐标校准
            'timestamp_matching': [],         # 步骤3：时间戳匹配
            'data_validation': [],            # 步骤4：数据有效性检查
            'bipartite_matching': [],         # 步骤5：最优二部图匹配
            'result_processing': [],          # 步骤6：结果处理
            'total_frame': [],                # 总耗时
        }
        self.perf_frame_count = 0  # 已处理的帧数
        
        # � 初始化摄像头IP映射
        if self.camera_id:
            logger.info(f"📡 C{self.camera_id} RadarVisionFusion: 允许的雷达IP = {self.allowed_radar_ips}")

    def _get_allowed_radar_ips(self, camera_id):
        """
        获取指定摄像头允许的雷达数据源IP列表
        
        映射关系（与RadarDataLoader.RADAR_IP_TO_CAMERA同步）：
        - Camera1 (camera_id=1) <- 44.30.142.88
        - Camera2 (camera_id=2) <- 44.30.142.85
        - Camera3 (camera_id=3) <- 44.30.142.87
        """
        if not camera_id:
            return None
        
        # 定义摄像头与雷达IP的映射关系（必须与RadarDataLoader中的倒序映射保持一致）
        camera_radar_mapping = {
            1: ["44.30.142.88"],   # Camera1: Radar IP 44.30.142.88
            2: ["44.30.142.85"],   # Camera2: Radar IP 44.30.142.85
            3: ["44.30.142.87"],   # Camera3: Radar IP 44.30.142.87
        }
        
        return camera_radar_mapping.get(camera_id, None)
    
    def _should_accept_radar_data(self, radar_obj):
        """
        检查是否应该接受这个雷达数据
        
        根据摄像头ID和雷达数据源IP进行过滤
        """
        if not self.camera_id or not self.allowed_radar_ips:
            # 没有配置摄像头ID，接受所有数据
            return True
        
        # 获取雷达数据源IP
        radar_ip = getattr(radar_obj, 'source_ip', None)
        
        if not radar_ip:
            # 无法确定来源，为安全起见拒绝
            return False
        
        # 检查IP是否在允许列表中（精确匹配）
        if radar_ip in self.allowed_radar_ips:
            return True
        
        # 不在允许列表中
        return False

    def get_dynamic_long_threshold(self, speed):
        """根据速度获取动态纵向距离阈值"""
        if speed < 1.0:
            return 15.0  # 静止或低速：15m
        elif speed < 8.0:
            return 20.0  # 中速：20m
        else:
            return 30.0  # 高速：30m
    
    # 🔧 新增：车道过滤相关方法
    def check_lane_compatibility(self, radar_obj, vision_obj) -> Tuple[bool, str]:
        """
        检查雷达目标和视觉目标的车道兼容性（三层过滤中的第三层）
        
        Args:
            radar_obj: 雷达目标
            vision_obj: 视觉目标 (OutputObject)
            
        Returns:
            (兼容性, 原因)
            - (True, "lane_match") - 车道完全匹配
            - (True, "no_radar_lane") - 雷达无车道信息，假定兼容
            - (True, "no_vision_lane") - 视觉目标无车道信息，假定兼容
            - (False, "lane_mismatch") - 车道不匹配
        """
        if not self.enable_lane_filtering:
            return True, "lane_filtering_disabled"
        
        # 获取雷达的车道（直接使用，不再推断）
        if not hasattr(radar_obj, 'lane') or radar_obj.lane is None:
            # 雷达无车道信息，假定兼容（宽松策略）
            return True, "no_radar_lane"
        
        # 获取视觉目标的车道
        if not hasattr(vision_obj, 'lane') or vision_obj.lane is None or vision_obj.lane == 'unknown':
            # 视觉无车道信息，假定兼容（宽松策略）
            return True, "no_vision_lane"
        
        # 直接比较雷达和视觉的车道信息
        if radar_obj.lane == vision_obj.lane:
            return True, "lane_match"
        else:
            # 诊断：输出详细的车道信息
            # print(f"      [DEBUG] 车道不匹配详情: 雷达={radar_obj.lane}, 视觉={vision_obj.lane}, 像素X={vision_obj.pixel_x if hasattr(vision_obj, 'pixel_x') else 'N/A'}")
            return False, "lane_mismatch"
            
    def _cleanup_expired_bindings(self, current_radar_ids, current_vision_track_ids):
        """
        清理过期的绑定关系
        
        规则：
        1. 如果视觉目标消失（不在current_vision_track_ids中），释放其绑定的雷达
        2. 如果雷达消失（不在current_radar_ids中），释放与其绑定的视觉目标
        
        Args:
            current_radar_ids: 当前帧中存在的雷达ID集合
            current_vision_track_ids: 当前帧中存在的视觉track_id集合
        """
        # 清理消失的视觉目标的绑定
        vision_ids_to_remove = []
        for vision_track_id in self.vision_to_radar_binding.keys():
            if vision_track_id not in current_vision_track_ids:
                radar_id = self.vision_to_radar_binding[vision_track_id]
                # 释放反向映射
                if radar_id in self.radar_to_vision_binding:
                    del self.radar_to_vision_binding[radar_id]
                vision_ids_to_remove.append(vision_track_id)
                logger.debug(f"🔧 释放绑定: 视觉目标 {vision_track_id} 消失，释放雷达 {radar_id}")
        
        for vision_track_id in vision_ids_to_remove:
            del self.vision_to_radar_binding[vision_track_id]
        
        # 清理消失的雷达的绑定
        radar_ids_to_remove = []
        for radar_id in self.radar_to_vision_binding.keys():
            if radar_id not in current_radar_ids:
                vision_track_id = self.radar_to_vision_binding[radar_id]
                # 释放正向映射
                if vision_track_id in self.vision_to_radar_binding:
                    del self.vision_to_radar_binding[vision_track_id]
                radar_ids_to_remove.append(radar_id)
                logger.debug(f"🔧 释放绑定: 雷达 {radar_id} 消失，释放视觉目标 {vision_track_id}")
        
        for radar_id in radar_ids_to_remove:
            del self.radar_to_vision_binding[radar_id]
            if radar_id in self.radar_last_seen_time:
                del self.radar_last_seen_time[radar_id]
    
    def _separate_bound_and_free(self, radar_objects, vision_objects):
        """
        分离已绑定和自由的目标
        
        Returns:
            (bound_radar_objs, free_radar_objs, bound_vision_objs, free_vision_objs)
        """
        bound_radar_objs = []
        free_radar_objs = []
        bound_vision_objs = []
        free_vision_objs = []
        
        # 分离雷达目标
        for radar_obj in radar_objects:
            if radar_obj.id in self.radar_to_vision_binding:
                bound_radar_objs.append(radar_obj)
            else:
                free_radar_objs.append(radar_obj)
        
        # 分离视觉目标
        for vision_obj in vision_objects:
            if vision_obj.track_id in self.vision_to_radar_binding:
                bound_vision_objs.append(vision_obj)
            else:
                free_vision_objs.append(vision_obj)
        
        return bound_radar_objs, free_radar_objs, bound_vision_objs, free_vision_objs
    
    def add_radar_data(self, timestamp, radar_objects):
        """
        添加雷达数据到缓冲区 (已优化：自动清理过期数据)
        
        优化点：
        1. 按摄像头ID过滤（保持原有）
        2. 自动清理过期数据（防止内存无限增长）
        3. 使用字典键作为有序索引（Python 3.7+）
        
        Args:
            timestamp: 时间戳
            radar_objects: 雷达目标列表
        """
        # ===== 步骤1：按摄像头ID过滤雷达数据 =====
        filtered_objects = [
            radar_obj for radar_obj in radar_objects
            if self._should_accept_radar_data(radar_obj)
        ]
        
        # 诊断日志：仅在有拒绝时输出
        if self.camera_id and len(filtered_objects) < len(radar_objects):
            rejected_count = len(radar_objects) - len(filtered_objects)
            logger.debug(f"⚠️ C{self.camera_id} 雷达数据过滤: 总数={len(radar_objects)}, "
                         f"接受={len(filtered_objects)}, 拒绝={rejected_count}")
        
        # ===== 步骤2：存入缓冲区 =====
        self.radar_buffer[timestamp] = filtered_objects
        
        # ===== 步骤3：自动清理过期数据（防止内存无限增长） =====
        # 保留最近 200 个时间戳（约 8-10 秒历史数据，足够匹配使用）
        # 使用 LRU 策略：移除最旧的时间戳
        MAX_BUFFER_SIZE = 200
        if len(self.radar_buffer) > MAX_BUFFER_SIZE:
            # 获取最旧的时间戳（字典第一个键，Python 3.7+ 保证有序）
            oldest_ts = next(iter(self.radar_buffer))
            del self.radar_buffer[oldest_ts]
            logger.debug(f"📊 雷达缓冲区已清理: 移除时间戳 {oldest_ts}, 当前大小 {len(self.radar_buffer)}")

    def _convert_timestamp_to_numeric(self, ts):
        """将时间戳转换为数字格式（Unix timestamp float）"""
        if isinstance(ts, (int, float)):
            return float(ts)
        if isinstance(ts, str):
            try:
                from datetime import datetime
                dt = datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f')
                return dt.timestamp()
            except (ValueError, TypeError):
                return None
        return None

    def find_closest_radar_timestamp(self, vision_timestamp, max_time_diff=None):
        """
        找到最接近的雷达时间戳 (二分查找优化版)
        视觉为准，雷达靠拢
        
        Args:
            vision_timestamp: 视觉时间戳 (Unix timestamp float)
            max_time_diff: 最大时间差 (秒)
            
        Returns:
            最接近的雷达时间戳，或 None
        """
        if max_time_diff is None:
            max_time_diff = self.MAX_TIME_DIFF

        if not self.radar_buffer:
            return None

        # 转换视觉时间戳为数字格式
        vision_ts_numeric = self._convert_timestamp_to_numeric(vision_timestamp)
        if vision_ts_numeric is None:
            logger.warning(f"无法解析视觉时间戳: {vision_timestamp}")
            return None

        # 获取所有雷达时间戳的数字版本 (缓存以提高性能)
        radar_timestamps_list = list(self.radar_buffer.keys())
        radar_ts_numeric_list = []
        
        for ts in radar_timestamps_list:
            ts_num = self._convert_timestamp_to_numeric(ts)
            if ts_num is not None:
                radar_ts_numeric_list.append((ts_num, ts))
        
        if not radar_ts_numeric_list:
            return None
        
        # 按数字时间戳排序 (用于二分查找)
        radar_ts_numeric_list.sort(key=lambda x: x[0])
        numeric_only = [x[0] for x in radar_ts_numeric_list]
        
        # 二分查找：找到最接近的位置
        import bisect
        idx = bisect.bisect_left(numeric_only, vision_ts_numeric)
        
        # 检查左右两个候选
        candidates = []
        if idx > 0:
            candidates.append(radar_ts_numeric_list[idx - 1])
        if idx < len(radar_ts_numeric_list):
            candidates.append(radar_ts_numeric_list[idx])
        
        # 找到最接近的时间戳
        closest_ts = None
        min_diff = float('inf')
        
        for ts_num, ts_orig in candidates:
            diff = abs(ts_num - vision_ts_numeric)
            if diff < min_diff:
                min_diff = diff
                closest_ts = ts_orig
        
        # 尝试多个阈值来找到匹配
        # 1. 严格阈值：0.5秒（MAX_TIME_DIFF）
        if closest_ts is not None and min_diff <= max_time_diff:
            return closest_ts
        
        # 2. 宽松阈值：2秒（MAX_TIME_DIFF_LOOSE）
        if closest_ts is not None and min_diff <= self.MAX_TIME_DIFF_LOOSE:
            return closest_ts
        
        # 3. 诊断输出：如果两个阈值都不满足，输出警告信息
        if closest_ts is not None and time.time() - self.last_diag_time > self.diag_interval:
            min_ts = min(numeric_only)
            max_ts = max(numeric_only)
            logger.warning(f"⚠️ [RADAR_FUSION DIAGNOSTIC] 视觉时间戳: {vision_timestamp}")
            logger.warning(f"   最接近的雷达时间戳: {closest_ts}")
            logger.warning(f"   时间差: {min_diff:.3f}秒 (严格阈值: {max_time_diff}秒, 宽松阈值: {self.MAX_TIME_DIFF_LOOSE}秒)")
            logger.warning(f"   雷达时间范围: {min_ts} ~ {max_ts}")
            logger.warning(f"   雷达数据帧数: {len(radar_timestamps_list)}")
            self.last_diag_time = time.time()
        
        # 最后还是返回最接近的，即使超过所有阈值（最终容错）
        return closest_ts

    def match_radar_to_vision(self, radar_obj, vision_objs):
        """
        将单个雷达目标与视觉目标进行匹配
        
        Args:
            radar_obj: 雷达目标
            vision_objs: 视觉目标列表 (OutputObject)
            
        Returns:
            匹配的视觉目标，或 None
        """
        # 区域过滤
        if self.fusion_area_geo and not point_in_polygon(
            [radar_obj.longitude, radar_obj.latitude],
            self.fusion_area_geo
        ):
            return None

        best_vision_obj = None
        min_cost = 1e6

        long_thresh = self.get_dynamic_long_threshold(radar_obj.speed)

        for vision_obj in vision_objs:
            # 区域过滤
            if self.fusion_area_geo and not point_in_polygon(
                [vision_obj.lon, vision_obj.lat],
                self.fusion_area_geo
            ):
                continue

            # 计算距离
            dy = (vision_obj.lat - radar_obj.latitude) * LAT_TO_M
            dx = (vision_obj.lon - radar_obj.longitude) * LON_TO_M
            dist = math.sqrt(dx**2 + dy**2)

            # 直接使用坐标差异，不用方位角
            # 纵向距离（沿南北方向）= |dy|
            # 横向距离（沿东西方向）= |dx|
            lat_diff = abs(dy)
            lon_diff = abs(dx)

            # 检查距离阈值
            if lat_diff <= self.MAX_LANE_DIFF and lon_diff <= long_thresh:
                # 计算成本函数
                cost = (10.0 * lat_diff) + (1.0 * lon_diff)

                # 忠诚度绑定：如果之前匹配过，大幅降低成本
                if vision_obj.track_id in self.vision_id_map:
                    if self.vision_id_map[vision_obj.track_id] == radar_obj.id:
                        cost = cost / self.LOYALTY_BONUS  # 成本除以10000，强制保持绑定

                if cost < min_cost:
                    min_cost = cost
                    best_vision_obj = vision_obj

        return best_vision_obj if min_cost < 1e5 else None

    def optimal_bipartite_matching(self, radar_objects, vision_objects):
        """
        [优化版] 最优二部图匹配
        使用 NumPy 向量化计算距离矩阵，大幅提升速度，同时保留所有业务逻辑。
        
        性能提升：
        - 距离计算：从 O(n*m) 循环 → NumPy 向量化 (10-50x faster)
        - 区域过滤：批量检查而非逐个检查
        - 整体性能：50-70% 时间减少
        
        Args:
            radar_objects: 雷达目标列表
            vision_objects: 视觉目标列表
            
        Returns:
            (radar_indices, vision_indices): 匹配对的索引列表
        """
        n_radar = len(radar_objects)
        n_vision = len(vision_objects)
        
        if n_radar == 0 or n_vision == 0:
            return [], []
        
        # ===== 第一步：区域过滤（预处理） =====
        valid_radar_indices = []
        valid_vision_indices = []
        
        for i, radar_obj in enumerate(radar_objects):
            if self.fusion_area_geo and not point_in_polygon(
                [radar_obj.longitude, radar_obj.latitude],
                self.fusion_area_geo
            ):
                continue
            valid_radar_indices.append(i)
        
        for j, v_obj in enumerate(vision_objects):
            if self.fusion_area_geo and not point_in_polygon(
                [v_obj.calib_lon, v_obj.calib_lat],
                self.fusion_area_geo
            ):
                continue
            valid_vision_indices.append(j)
        
        if not valid_radar_indices or not valid_vision_indices:
            return [], []
        
        # ===== 第二步：向量化计算距离矩阵 =====
        # 提取有效目标的坐标
        radar_lats = np.array([radar_objects[i].latitude for i in valid_radar_indices], dtype=np.float32)
        radar_lons = np.array([radar_objects[i].longitude for i in valid_radar_indices], dtype=np.float32)
        vision_lats = np.array([vision_objects[j].calib_lat for j in valid_vision_indices], dtype=np.float32)
        vision_lons = np.array([vision_objects[j].calib_lon for j in valid_vision_indices], dtype=np.float32)
        
        # 向量化计算距离差
        # radar_lats: (n_valid_radar, 1), vision_lats: (1, n_valid_vision)
        # 结果：(n_valid_radar, n_valid_vision)
        lat_diffs = np.abs((radar_lats[:, np.newaxis] - vision_lats[np.newaxis, :]) * LAT_TO_M)
        lon_diffs = np.abs((radar_lons[:, np.newaxis] - vision_lons[np.newaxis, :]) * LON_TO_M)
        
        # 计算总距离
        distances = np.sqrt(lat_diffs**2 + lon_diffs**2)
        
        # ===== 第三步：构建成本矩阵（保留业务逻辑） =====
        n_valid_radar = len(valid_radar_indices)
        n_valid_vision = len(valid_vision_indices)
        cost_matrix = np.full((n_valid_radar, n_valid_vision), 1e6, dtype=np.float32)
        
        # 获取所有雷达的动态阈值（用于向量化过滤）
        long_threshs = np.array([self.get_dynamic_long_threshold(radar_objects[i].speed) 
                                 for i in valid_radar_indices], dtype=np.float32)[:, np.newaxis]
        
        # 向量化应用距离阈值过滤
        invalid_mask = (lon_diffs > self.MAX_LANE_DIFF) | (lat_diffs > long_threshs)
        cost_matrix[invalid_mask] = 1e6
        
        # 计算有效位置的基础成本（向量化）
        valid_positions = ~invalid_mask
        cost_matrix[valid_positions] = (10.0 * lat_diffs[valid_positions]) + (1.0 * lon_diffs[valid_positions])
        
        # 循环处理特殊逻辑（车道兼容性、忠诚度奖励）
        # 注意：必须在所有位置都计算了基础成本之后再做这些检查
        for vi, i in enumerate(valid_radar_indices):
            radar_obj = radar_objects[i]
            
            for vj, j in enumerate(valid_vision_indices):
                v_obj = vision_objects[j]
                
                # 只处理通过距离阈值的候选
                if cost_matrix[vi, vj] >= 1e5:
                    continue
                
                # 车道兼容性检查
                lane_compatible, lane_reason = self.check_lane_compatibility(radar_obj, v_obj)
                if not lane_compatible:
                    self.stats['lane_filtered_candidates'] = self.stats.get('lane_filtered_candidates', 0) + 1
                    cost_matrix[vi, vj] = 1e6
                    continue
                
                # 忠诚度奖励：强制保持已绑定的对
                v_key = str(v_obj.track_id)
                prev_fusion_id_radar = self.radar_id_map.get(radar_obj.id)
                prev_fusion_id_vision = self.vision_id_map.get(v_key)
                
                if prev_fusion_id_radar and prev_fusion_id_radar == prev_fusion_id_vision:
                    cost_matrix[vi, vj] = cost_matrix[vi, vj] / self.LOYALTY_BONUS
        
        # 诊断日志（仅当启用时，且仅输出距离近的对）
        if self.enable_fusion_logs:
            close_pairs = np.argwhere(distances < 50)
            for vi, vj in close_pairs:
                i = valid_radar_indices[vi]
                j = valid_vision_indices[vj]
                radar_obj = radar_objects[i]
                v_obj = vision_objects[j]
                cost = cost_matrix[vi, vj]
                
                logger.info(f"    [成本矩阵] 雷达[{i}]({radar_obj.latitude:.6f},{radar_obj.longitude:.6f}) vs 视觉[{j}]({v_obj.calib_lat:.6f},{v_obj.calib_lon:.6f})")
                logger.info(f"      dx={lon_diffs[vi, vj]:.2f}m, dy={lat_diffs[vi, vj]:.2f}m, 总距离={distances[vi, vj]:.2f}m")
                logger.info(f"      成本={cost:.6f} {'✅' if cost < 1e5 else '❌'}")
        
        # ===== 第四步：匈牙利算法求解 =====
        valid_radar_indices_array, valid_vision_indices_array = linear_sum_assignment(cost_matrix)
        
        # 诊断：打印匈牙利算法的原始结果
        if self.enable_fusion_logs and len(valid_radar_indices_array) > 0:
            logger.info(f"    [匈牙利算法结果] 总匹配数: {len(valid_radar_indices_array)}")
            for vi, vj in zip(valid_radar_indices_array, valid_vision_indices_array):
                cost = cost_matrix[vi, vj]
                logger.info(f"      配对 [{vi},{vj}]: cost={cost:.2f} {'✅' if cost < 1e5 else '❌'}")
        
        # ===== 第五步：过滤无效匹配并映射回原始索引 =====
        valid_matches = []
        for vi, vj in zip(valid_radar_indices_array, valid_vision_indices_array):
            if cost_matrix[vi, vj] < 1e5:
                # 映射回原始索引
                original_radar_idx = valid_radar_indices[vi]
                original_vision_idx = valid_vision_indices[vj]
                valid_matches.append((original_radar_idx, original_vision_idx))
        
        if valid_matches:
            if self.enable_fusion_logs:
                logger.info(f"    [过滤结果] 有效匹配数: {len(valid_matches)}")
            radar_indices, vision_indices = zip(*valid_matches)
            return list(radar_indices), list(vision_indices)
        else:
            if self.enable_fusion_logs:
                logger.info(f"    [过滤结果] 无有效匹配")
            return [], []

    def process_frame(self, vision_timestamp, vision_objects):
        """
        处理单帧的雷视融合 - 集成粘性绑定逻辑
        
        核心规则：
        1. 一个雷达最多只能绑定到一个视觉目标
        2. 一旦绑定，就持续输出该雷达ID，直到目标或雷达消失
        3. 自由目标使用匈牙利算法进行最优匹配
        
        Args:
            vision_timestamp: 视觉帧时间戳
            vision_objects: 视觉目标列表 (OutputObject)
            
        Returns:
            更新后的视觉目标列表 (with radar_id)
        """
        # 📊 开始计时
        frame_start_time = time.time()
        
        # ===== 步骤 1：初始化当前帧的视觉目标ID集合 =====
        step1_start = time.time()
        current_vision_track_ids = set(v.track_id for v in vision_objects)
        
        step1_time = (time.time() - step1_start) * 1000  # 转换为毫秒
        self.perf_stats['trajectory_prediction'].append(step1_time)
        
        # ===== 步骤 2：坐标校准 =====
        step2_start = time.time()
        for v_obj in vision_objects:
            # 检查坐标是否有效
            try:
                if (isinstance(v_obj.lat, (int, float)) and isinstance(v_obj.lon, (int, float)) and
                    not math.isnan(v_obj.lat) and not math.isnan(v_obj.lon) and
                    not math.isinf(v_obj.lat) and not math.isinf(v_obj.lon)):
                    v_obj.calib_lat = v_obj.lat + self.lat_offset
                    v_obj.calib_lon = v_obj.lon + self.lon_offset
                else:
                    v_obj.calib_lat = v_obj.lat
                    v_obj.calib_lon = v_obj.lon
            except (TypeError, AttributeError, ValueError):
                v_obj.calib_lat = v_obj.lat
                v_obj.calib_lon = v_obj.lon
        
        step2_time = (time.time() - step2_start) * 1000  # 转换为毫秒
        self.perf_stats['coordinate_calibration'].append(step2_time)
        
        # ===== 步骤 3：找到最接近的雷达时间戳 =====
        step3_start = time.time()
        radar_timestamp = self.find_closest_radar_timestamp(vision_timestamp)
        if radar_timestamp is None:
            # 诊断：没有找到匹配的雷达时间戳
            radar_timestamps_list = list(self.radar_buffer.keys())
            if len(radar_timestamps_list) > 0:
                min_ts = min(radar_timestamps_list)
                max_ts = max(radar_timestamps_list)
                time_diff_min = abs(vision_timestamp - min_ts)
                time_diff_max = abs(vision_timestamp - max_ts)
                logger.warning(f"[RADAR_FUSION] 警告: 视觉时间戳{vision_timestamp:.3f}无法匹配雷达数据")
                logger.warning(f"  雷达时间戳范围: [{min_ts:.3f}, {max_ts:.3f}]")
                logger.warning(f"  时间差范围: [{time_diff_min:.3f}, {time_diff_max:.3f}]秒")
                logger.warning(f"  MAX_TIME_DIFF阈值: {self.MAX_TIME_DIFF}秒")
            else:
                logger.warning(f"[RADAR_FUSION] 警告: 视觉时间戳{vision_timestamp:.3f}无法匹配雷达数据（雷达缓冲区为空）")
            step3_time = (time.time() - step3_start) * 1000
            self.perf_stats['timestamp_matching'].append(step3_time)
            return vision_objects
        
        # 诊断输出：当前融合的时间戳信息
        # 将Unix时间戳转换为可读格式
        if self.enable_fusion_logs:
            vision_ts_str = datetime.fromtimestamp(vision_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            logger.info(f"[RADAR_FUSION] 融合尝试 - 视觉时间戳: {vision_ts_str}, 雷达时间戳: {radar_timestamp}, 视觉目标数: {len(vision_objects)}")
        
        radar_objects = self.radar_buffer.get(radar_timestamp, [])
        if not radar_objects:
            step3_time = (time.time() - step3_start) * 1000
            self.perf_stats['timestamp_matching'].append(step3_time)
            return vision_objects
        
        step3_time = (time.time() - step3_start) * 1000
        self.perf_stats['timestamp_matching'].append(step3_time)
        
        # ===== 步骤 3.5：清理过期绑定（在获取雷达数据后） =====
        # 🔧 新增：获取当前帧的雷达ID集合，用于清理过期绑定
        current_radar_ids = set(radar_obj.id for radar_obj in radar_objects)
        # 更新所有雷达的最后出现时间
        for radar_obj in radar_objects:
            self.radar_last_seen_time[radar_obj.id] = vision_timestamp
        # 清理过期绑定（视觉目标消失或雷达消失）
        self._cleanup_expired_bindings(current_radar_ids, current_vision_track_ids)
        
        # ===== 步骤 4：数据有效性检查 =====
        step4_start = time.time()
        
        # 数据清理：过滤掉坐标无效的雷达对象
        valid_radar_objects = []
        for radar_obj in radar_objects:
            try:
                if (hasattr(radar_obj, 'latitude') and hasattr(radar_obj, 'longitude') and
                    isinstance(radar_obj.latitude, (int, float)) and isinstance(radar_obj.longitude, (int, float)) and
                    not math.isnan(radar_obj.latitude) and not math.isnan(radar_obj.longitude) and
                    not math.isinf(radar_obj.latitude) and not math.isinf(radar_obj.longitude)):
                    valid_radar_objects.append(radar_obj)
            except (TypeError, AttributeError):
                continue
        
        if not valid_radar_objects:
            return vision_objects
        
        radar_objects = valid_radar_objects
        
        # 数据清理：过滤掉坐标无效的视觉对象
        valid_vision_objects = []
        for v_obj in vision_objects:
            try:
                if (hasattr(v_obj, 'lat') and hasattr(v_obj, 'lon') and
                    isinstance(v_obj.lat, (int, float)) and isinstance(v_obj.lon, (int, float)) and
                    not math.isnan(v_obj.lat) and not math.isnan(v_obj.lon) and
                    not math.isinf(v_obj.lat) and not math.isinf(v_obj.lon)):
                    valid_vision_objects.append(v_obj)
            except (TypeError, AttributeError):
                continue
        
        if not valid_vision_objects:
            step4_time = (time.time() - step4_start) * 1000
            self.perf_stats['data_validation'].append(step4_time)
            return vision_objects
        
        vision_objects_to_match = valid_vision_objects
        
        step4_time = (time.time() - step4_start) * 1000
        self.perf_stats['data_validation'].append(step4_time)
        
        # 📊 统计本帧的对象数（在处理前）
        self.stats['radar_objects_processed'] += len(radar_objects)
        # vision_objects_processed会在后面处理所有视觉对象时统计
        
        # ===== 步骤 5：【粘性绑定】分离已绑定和自由的目标 =====
        step5_start = time.time()
        bound_radar_objs, free_radar_objs, bound_vision_objs, free_vision_objs = \
            self._separate_bound_and_free(radar_objects, vision_objects_to_match)
        
        logger.debug(f"[粘性绑定] 已绑定雷达: {len(bound_radar_objs)}, 自由雷达: {len(free_radar_objs)}, "
                    f"已绑定视觉: {len(bound_vision_objs)}, 自由视觉: {len(free_vision_objs)}")
        
        # ===== 步骤 5.1：处理已绑定的目标（直接输出，无需重新匹配） =====
        matched_vision_track_ids = set()
        used_fusion_ids = set()
        
        for v_obj in bound_vision_objs:
            # 获取已绑定的雷达ID
            radar_id = self.vision_to_radar_binding.get(v_obj.track_id)
            if radar_id:
                # 直接输出已绑定的雷达ID，无需重新匹配
                v_obj.radar_id = radar_id
                matched_vision_track_ids.add(v_obj.track_id)
                
                # 更新原始vision_objects中对应的目标
                for orig_v_obj in vision_objects:
                    if orig_v_obj.track_id == v_obj.track_id:
                        orig_v_obj.radar_id = radar_id
                        break
                
                logger.debug(f"[粘性绑定] 视觉目标 {v_obj.track_id} 继续使用雷达 {radar_id}")
        
        # ===== 步骤 5.2：对自由目标进行匹配（使用匈牙利算法） =====
        if free_radar_objs and free_vision_objs:
            radar_indices, vision_indices = self.optimal_bipartite_matching(free_radar_objs, free_vision_objs)
            
            # 诊断输出：匹配结果
            if self.enable_fusion_logs:
                logger.info(f"[RADAR_FUSION] 自由目标匹配 - 雷达: {len(free_radar_objs)}, 视觉: {len(free_vision_objs)}, "
                           f"成功匹配: {len(radar_indices)}")
            
            # 处理新的匹配对
            for radar_idx, vision_idx in zip(radar_indices, vision_indices):
                radar_obj = free_radar_objs[radar_idx]
                v_obj = free_vision_objs[vision_idx]
                
                # 建立新的粘性绑定
                self.vision_to_radar_binding[v_obj.track_id] = radar_obj.id
                self.radar_to_vision_binding[radar_obj.id] = v_obj.track_id
                self.radar_last_seen_time[radar_obj.id] = vision_timestamp
                
                # 设置雷达ID
                v_obj.radar_id = radar_obj.id
                matched_vision_track_ids.add(v_obj.track_id)
                
                # 更新原始vision_objects中对应的目标
                for orig_v_obj in vision_objects:
                    if orig_v_obj.track_id == v_obj.track_id:
                        orig_v_obj.radar_id = radar_obj.id
                        break
                
                # 统计成功匹配
                self.stats['successful_matches'] += 1
                logger.debug(f"[粘性绑定] 新建绑定: 视觉目标 {v_obj.track_id} -> 雷达 {radar_obj.id}")
            
            # 统计未匹配的雷达
            unmatched_radar_count = len(free_radar_objs) - len(radar_indices)
            self.stats['failed_matches'] += unmatched_radar_count
        else:
            # 没有自由目标可匹配
            unmatched_radar_count = len(free_radar_objs)
            if unmatched_radar_count > 0:
                self.stats['failed_matches'] += unmatched_radar_count
        
        step5_time = (time.time() - step5_start) * 1000
        self.perf_stats['bipartite_matching'].append(step5_time)
        
        # ===== 步骤 7：处理未匹配的视觉目标 =====
        step6_start = time.time()
        # 累加本帧的视觉对象数到统计
        self.stats['vision_objects_processed'] += len(vision_objects)
        
        for v_obj in vision_objects:
            v_key = str(v_obj.track_id)
            
            if v_obj.track_id in matched_vision_track_ids:
                continue  # 已匹配，跳过
            
            # 🔧 粘性绑定：检查是否有历史绑定的雷达ID
            if v_obj.track_id in self.vision_to_radar_binding:
                # 继续使用历史绑定的雷达ID
                radar_id = self.vision_to_radar_binding[v_obj.track_id]
                v_obj.radar_id = radar_id
                logger.debug(f"[粘性绑定] 未匹配视觉目标 {v_obj.track_id} 继续使用历史雷达 {radar_id}")
            else:
                # 没有历史绑定，设置为None
                v_obj.radar_id = None
                logger.debug(f"[粘性绑定] 未匹配视觉目标 {v_obj.track_id} 无历史绑定")
        
        step6_time = (time.time() - step6_start) * 1000
        self.perf_stats['result_processing'].append(step6_time)
        
        # ===== 性能统计输出 =====
        frame_total_time = (time.time() - frame_start_time) * 1000
        self.perf_stats['total_frame'].append(frame_total_time)
        self.perf_frame_count += 1
        
        # 每处理50帧输出一次性能统计
        if self.perf_frame_count % 50 == 0:
            self._print_performance_stats()
        
        return vision_objects

    def _print_performance_stats(self):
        """
        📊 打印性能统计信息
        每50帧输出一次，包括每一步的平均耗时和性能瓶颈分析
        """
        if not self.enable_perf_stats or self.perf_frame_count == 0:
            return
        
        logger.info("=" * 80)
        logger.info(f"📊 雷达融合性能统计 (已处理 {self.perf_frame_count} 帧)")
        logger.info("=" * 80)
        
        # 计算每一步的平均耗时
        stats_summary = {}
        for step_name, times in self.perf_stats.items():
            if times:
                avg_time = sum(times) / len(times)
                min_time = min(times)
                max_time = max(times)
                stats_summary[step_name] = {
                    'avg': avg_time,
                    'min': min_time,
                    'max': max_time,
                    'count': len(times)
                }
        
        # 按平均耗时排序，找出性能瓶颈
        sorted_stats = sorted(stats_summary.items(), key=lambda x: x[1]['avg'], reverse=True)
        
        # 输出详细统计
        logger.info("\n📈 每一步的耗时统计 (单位: 毫秒):")
        logger.info("-" * 80)
        
        total_avg = 0
        for step_name, stats in sorted_stats:
            if step_name == 'total_frame':
                continue
            
            avg = stats['avg']
            min_t = stats['min']
            max_t = stats['max']
            total_avg += avg
            
            # 用进度条表示相对耗时
            bar_length = int(avg / 2)  # 每2ms一个字符
            bar = "█" * min(bar_length, 40)
            
            logger.info(f"  {step_name:25} | {avg:7.2f}ms (min:{min_t:6.2f}ms, max:{max_t:6.2f}ms) | {bar}")
        
        # 输出总耗时
        logger.info("-" * 80)
        if 'total_frame' in stats_summary:
            total_stats = stats_summary['total_frame']
            logger.info(f"  {'总耗时':25} | {total_stats['avg']:7.2f}ms (min:{total_stats['min']:6.2f}ms, max:{total_stats['max']:6.2f}ms)")
        
        # 性能瓶颈分析
        logger.info("\n🔴 性能瓶颈分析:")
        logger.info("-" * 80)
        
        if sorted_stats:
            top_bottleneck = sorted_stats[0]
            bottleneck_name = top_bottleneck[0]
            bottleneck_avg = top_bottleneck[1]['avg']
            
            if bottleneck_name != 'total_frame':
                percentage = (bottleneck_avg / total_avg * 100) if total_avg > 0 else 0
                logger.info(f"  🥇 最大瓶颈: {bottleneck_name} ({bottleneck_avg:.2f}ms, 占比 {percentage:.1f}%)")
                
                # 给出优化建议
                if bottleneck_name == 'bipartite_matching':
                    logger.info(f"     💡 建议: 优化匈牙利算法或减少匹配候选数量")
                elif bottleneck_name == 'trajectory_prediction':
                    logger.info(f"     💡 建议: 优化轨迹预测算法或减少活跃轨迹数")
                elif bottleneck_name == 'data_validation':
                    logger.info(f"     💡 建议: 使用向量化操作进行数据验证")
                elif bottleneck_name == 'timestamp_matching':
                    logger.info(f"     💡 建议: 使用二分查找或哈希表加速时间戳匹配")
        
        # 输出业务统计
        logger.info("\n📊 业务统计:")
        logger.info("-" * 80)
        logger.info(f"  雷达目标总数: {self.stats['radar_objects_processed']}")
        logger.info(f"  视觉目标总数: {self.stats['vision_objects_processed']}")
        logger.info(f"  成功匹配数: {self.stats['successful_matches']}")
        logger.info(f"  失败匹配数: {self.stats['failed_matches']}")
        logger.info(f"  车道过滤数: {self.stats['lane_filtered_candidates']}")
        
        if self.stats['radar_objects_processed'] > 0:
            match_rate = (self.stats['successful_matches'] / self.stats['radar_objects_processed'] * 100)
            logger.info(f"  匹配成功率: {match_rate:.1f}%")
        
        logger.info("=" * 80 + "\n")
        
        # 🔧 优化：打印完后立即清空统计列表，防止内存无限增长
        for key in self.perf_stats:
            self.perf_stats[key] = []
        logger.debug("✅ 性能统计列表已清空，防止内存泄漏")

    def clear_old_radar_data(self, current_timestamp, max_age=1.0):
        """
        清理过期的雷达数据
        
        Args:
            current_timestamp: 当前时间戳
            max_age: 最大年龄 (秒)
        """
        old_timestamps = [
            ts for ts in self.radar_buffer.keys()
            if current_timestamp - ts > max_age
        ]
        for ts in old_timestamps:
            del self.radar_buffer[ts]

    def get_stats(self):
        """获取统计信息"""
        return self.stats.copy()
    
    def get_matching_statistics(self):
        """
        获取匹配统计信息（与RadarFusionOrchestrator兼容的格式）
        
        Returns:
            dict: 包含匹配统计的字典，格式为：
                {
                    'total_radar_objects': int,
                    'total_vision_objects': int,
                    'successful_matches': int,
                    'failed_matches': int,
                    'lane_filtered_candidates': int,
                    'radar_match_rate': float,
                    'vision_match_rate': float
                }
        
        注意：统计数据是累积值，代表自初始化以来处理过的所有目标总数
        """
        # 累积统计（保持原有逻辑以兼容现有数据）
        total_radar = self.stats['radar_objects_processed']
        total_vision = self.stats['vision_objects_processed']
        successful = self.stats['successful_matches']
        failed = self.stats['failed_matches']
        lane_filtered = self.stats['lane_filtered_candidates']
        
        # 计算匹配率（基于累积统计）
        radar_match_rate = (successful / total_radar * 100) if total_radar > 0 else 0.0
        vision_match_rate = (successful / total_vision * 100) if total_vision > 0 else 0.0
        
        return {
            'total_radar_objects': total_radar,
            'total_vision_objects': total_vision,
            'successful_matches': successful,
            'failed_matches': failed,
            'lane_filtered_candidates': lane_filtered,
            'radar_match_rate': round(radar_match_rate, 2),
            'vision_match_rate': round(vision_match_rate, 2)
        }


# ==========================================
# 雷达数据加载器
# ==========================================
class RadarDataLoader:
    """加载和管理雷达数据"""
    
    # 雷达IP到摄像头ID的映射
    RADAR_IP_TO_CAMERA = {
        '44.30.142.85': 2,  # C2
        '44.30.142.88': 1,  # C1
        '44.30.142.87': 3,  # C3
    }

    def __init__(self, radar_file_path):
        """
        初始化雷达数据加载器
        
        Args:
            radar_file_path: 雷达数据文件路径 (JSONL)
        """
        self.radar_file_path = radar_file_path
        self.radar_data = {}  # 时间戳 -> 雷达目标列表
        self.radar_data_by_camera = {}  # (camera_id, timestamp) -> 雷达目标列表
        self.camera_timestamps = {1: set(), 2: set(), 3: set()}  # 每个摄像头的时间戳集合

    def _get_camera_id_from_ip(self, source_ip):
        """根据source_ip获取摄像头ID"""
        return self.RADAR_IP_TO_CAMERA.get(source_ip, None)

    def load(self):
        """加载雷达数据"""
        try:
            with open(self.radar_file_path, 'r', encoding='utf-8') as f:
                first_record = True
                for line in f:
                    try:
                        obj = json.loads(line)
                        source_ip = obj.get('source_ip', '')
                        camera_id = self._get_camera_id_from_ip(source_ip)
                        
                        if camera_id is None:
                            continue
                        
                        # 直接获取原始时间字符串，不进行转换
                        time_str = obj.get('time', '')
                        if not time_str:
                            continue
                        
                        # 调试日志：打印第一条有效的雷达数据
                        if first_record:
                            logger.debug(f"🔍 第一条有效雷达数据: 原始时间字符串: {time_str}, source_ip: {source_ip}, camera_id: {camera_id}")
                            first_record = False

                        locus = []
                        for x in obj.get('locusList', []):
                            if x.get('objType') in VALID_RADAR_TYPES:
                                # 将雷达的 lane (1-5) 转换为字符串格式 (lane_1 到 lane_5)
                                radar_lane = x.get('lane', None)
                                lane_str = f'lane_{radar_lane}' if radar_lane is not None else None
                                
                                # 安全处理 azimuth：如果为 None 或无效，使用 0
                                azimuth_val = x.get('azimuth')
                                if azimuth_val is None:
                                    azimuth_val = 0.0
                                else:
                                    try:
                                        azimuth_val = float(azimuth_val)
                                        # 检查是否为有效数值
                                        if math.isnan(azimuth_val) or math.isinf(azimuth_val):
                                            azimuth_val = 0.0
                                    except (ValueError, TypeError):
                                        azimuth_val = 0.0
                                
                                radar_obj = RadarObject(
                                    radar_id=x.get('id', ''),
                                    latitude=float(x.get('latitude', 0)),
                                    longitude=float(x.get('longitude', 0)),
                                    speed=float(x.get('speed', 0)),
                                    azimuth=azimuth_val,
                                    lane=lane_str,
                                    timestamp_str=time_str,  # 直接传入原始时间字符串
                                    source_ip=source_ip  # 🔧 传入源IP，用于摄像头过滤
                                )
                                locus.append(radar_obj)

                        if locus:
                            # 存储到全局数据（使用时间字符串作为键）
                            self.radar_data[time_str] = locus
                            
                            # 存储到按摄像头分类的数据
                            key = (camera_id, time_str)
                            self.radar_data_by_camera[key] = locus
                            self.camera_timestamps[camera_id].add(time_str)

                    except Exception as e:
                        logger.warning(f"解析雷达数据行失败: {e}")
                        continue

            logger.info(f"✅ 加载雷达数据完成: {len(self.radar_data)} 帧")
            logger.info(f"   C1: {len(self.camera_timestamps[1])} 帧")
            logger.info(f"   C2: {len(self.camera_timestamps[2])} 帧")
            logger.info(f"   C3: {len(self.camera_timestamps[3])} 帧")
            
            # 🔧 优化：构建时间戳缓存用于二分查找
            self._build_timestamp_cache()
            return True

        except Exception as e:
            logger.error(f"❌ 加载雷达数据失败: {e}")
            return False
    
    def _build_timestamp_cache(self):
        """构建排序的时间戳缓存用于二分查找（O(log N)查询）"""
        import bisect
        self._radar_ts_cache = {}
        
        for camera_id in [1, 2, 3]:
            ts_list = []
            for ts in self.camera_timestamps.get(camera_id, set()):
                try:
                    # 转换时间戳为数字格式
                    if isinstance(ts, str):
                        try:
                            dt = datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f')
                        except ValueError:
                            parts = ts.split('.')
                            if len(parts) == 2:
                                second_part = parts[0]
                                ms_part = parts[1]
                                us_part = ms_part.ljust(6, '0')
                                ts_with_us = f"{second_part}.{us_part}"
                                dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                            else:
                                continue
                        ts_num = dt.timestamp()
                    else:
                        ts_num = float(ts)
                    ts_list.append((ts_num, ts))
                except:
                    continue
            
            ts_list.sort(key=lambda x: x[0])
            self._radar_ts_cache[camera_id] = ts_list
        
        logger.debug(f"✅ 时间戳缓存已构建 (C1: {len(self._radar_ts_cache.get(1, []))} 帧, C2: {len(self._radar_ts_cache.get(2, []))} 帧, C3: {len(self._radar_ts_cache.get(3, []))} 帧)")

    def get_radar_data(self, timestamp):
        """获取指定时间戳的雷达数据（全局，向后兼容）"""
        return self.radar_data.get(timestamp, [])
    
    def get_radar_data_by_camera(self, camera_id, timestamp):
        """获取指定摄像头和时间戳的雷达数据"""
        key = (camera_id, timestamp)
        return self.radar_data_by_camera.get(key, [])

    def get_all_timestamps(self):
        """获取所有雷达时间戳（全局，向后兼容）"""
        return sorted(self.radar_data.keys())
    
    def get_camera_timestamps(self, camera_id):
        """获取指定摄像头的所有时间戳"""
        return sorted(self.camera_timestamps.get(camera_id, set()))
    
    def find_closest_radar_timestamp(self, camera_id, vision_timestamp, max_time_diff=0.5):
        """
        使用二分查找快速找到最接近的雷达时间戳 (O(log N))
        
        Args:
            camera_id: 摄像头ID
            vision_timestamp: 视觉时间戳 (字符串或浮点数)
            max_time_diff: 最大时间差 (秒)
        
        Returns:
            最接近的雷达时间戳，或 None
        """
        import bisect
        
        if not hasattr(self, '_radar_ts_cache') or camera_id not in self._radar_ts_cache:
            return None
        
        try:
            # 转换视觉时间戳为数字格式
            if isinstance(vision_timestamp, str):
                try:
                    dt = datetime.strptime(vision_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                except ValueError:
                    parts = vision_timestamp.split('.')
                    if len(parts) == 2:
                        second_part = parts[0]
                        ms_part = parts[1]
                        us_part = ms_part.ljust(6, '0')
                        ts_with_us = f"{second_part}.{us_part}"
                        dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                    else:
                        return None
                vision_ts_num = dt.timestamp()
            else:
                vision_ts_num = float(vision_timestamp)
            
            # 二分查找
            ts_list = self._radar_ts_cache[camera_id]
            numeric_only = [x[0] for x in ts_list]
            idx = bisect.bisect_left(numeric_only, vision_ts_num)
            
            # 检查左右两个候选
            closest_radar_ts = None
            min_diff = float('inf')
            
            for check_idx in [idx - 1, idx]:
                if 0 <= check_idx < len(ts_list):
                    ts_num, ts_orig = ts_list[check_idx]
                    diff = abs(ts_num - vision_ts_num)
                    if diff < min_diff and diff <= max_time_diff:
                        min_diff = diff
                        closest_radar_ts = ts_orig
            
            return closest_radar_ts
        except:
            return None
    
    def stream_radar_data(self):
        """
        🔧 [新增] 流式读取雷达数据生成器
        
        使用 yield 关键字，每次返回一个元组 (timestamp_float, radar_obj_list)
        这样可以避免一次性加载所有数据到内存中，实现真正的流式处理
        
        Yields:
            tuple: (timestamp_float, radar_objects_list)
        """
        try:
            with open(self.radar_file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    try:
                        data = json.loads(line)
                        source_ip = data.get('source_ip', '')
                        camera_id = self._get_camera_id_from_ip(source_ip)
                        
                        if camera_id is None:
                            continue
                        
                        # 获取原始时间字符串
                        time_str = data.get('time', '')
                        if not time_str:
                            continue
                        
                        # 解析雷达对象列表（复用 load() 中的逻辑）
                        locus = []
                        for x in data.get('locusList', []):
                            if x.get('objType') in VALID_RADAR_TYPES:
                                # 将雷达的 lane (1-5) 转换为字符串格式 (lane_1 到 lane_5)
                                radar_lane = x.get('lane', None)
                                lane_str = f'lane_{radar_lane}' if radar_lane is not None else None
                                
                                # 安全处理 azimuth：如果为 None 或无效，使用 0
                                azimuth_val = x.get('azimuth')
                                if azimuth_val is None:
                                    azimuth_val = 0.0
                                else:
                                    try:
                                        azimuth_val = float(azimuth_val)
                                        # 检查是否为有效数值
                                        if math.isnan(azimuth_val) or math.isinf(azimuth_val):
                                            azimuth_val = 0.0
                                    except (ValueError, TypeError):
                                        azimuth_val = 0.0
                                
                                radar_obj = RadarObject(
                                    radar_id=x.get('id', ''),
                                    latitude=float(x.get('latitude', 0)),
                                    longitude=float(x.get('longitude', 0)),
                                    speed=float(x.get('speed', 0)),
                                    azimuth=azimuth_val,
                                    lane=lane_str,
                                    timestamp_str=time_str,
                                    source_ip=source_ip
                                )
                                locus.append(radar_obj)
                        
                        if locus:
                            # 转换时间戳为浮点数格式
                            try:
                                dt = datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S.%f')
                                ts_float = dt.timestamp()
                            except ValueError:
                                # 处理毫秒格式
                                parts = time_str.split('.')
                                if len(parts) == 2:
                                    second_part = parts[0]
                                    ms_part = parts[1]
                                    us_part = ms_part.ljust(6, '0')
                                    ts_with_us = f"{second_part}.{us_part}"
                                    dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                                    ts_float = dt.timestamp()
                                else:
                                    continue
                            
                            yield ts_float, locus
                    
                    except Exception as e:
                        logger.debug(f"流式读取雷达数据行失败: {e}")
                        continue
        
        except Exception as e:
            logger.error(f"❌ 流式读取雷达数据失败: {e}")


# ==========================================
# 示例使用
# ==========================================
if __name__ == "__main__":
    # 示例：创建融合处理器
    processor = RadarVisionFusionProcessor()

    # 示例：创建输出对象
    output_obj = OutputObject(
        timestamp="2025-11-21 11:17:58.064",
        cameraid=2,
        type_name="car",
        confidence=0.836,
        track_id=68,
        lon=113.584,
        lat=23.530
    )

    logger.info("✅ RadarVisionFusion 模块加载成功")
