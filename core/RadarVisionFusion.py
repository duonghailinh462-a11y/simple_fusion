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
from collections import defaultdict, deque
from datetime import datetime
from typing import List, Dict, Tuple, Optional
from scipy.optimize import linear_sum_assignment

# 导入车道配置
try:
    from config.region_config import LANE_CONFIG, get_lane_for_point
    LANE_CONFIG_AVAILABLE = True
except ImportError:
    LANE_CONFIG_AVAILABLE = False
    print("⚠️ 警告: 无法导入车道配置 (config.region_config)，将禁用车道过滤")


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
    """判断点是否在多边形内"""
    lon, lat = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if lat > min(p1y, p2y):
            if lat <= max(p1y, p2y):
                if lon <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (lat - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or lon <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


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

    def __init__(self, fusion_area_geo=None, lat_offset=0.0, lon_offset=0.0, enable_lane_filtering=True, camera_id=None):
        """
        初始化雷达融合处理器 - 集成高级融合逻辑（三层过滤）
        
        Args:
            fusion_area_geo: 融合区域 (地理坐标多边形)
            lat_offset: 纬度偏移
            lon_offset: 经度偏移
            enable_lane_filtering: 是否启用车道过滤 (需要车道配置可用)
            camera_id: 摄像头ID (用于调整阈值)
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
        
        # 🔧 车道过滤配置
        self.enable_lane_filtering = enable_lane_filtering and LANE_CONFIG_AVAILABLE
        if self.enable_lane_filtering:
            print("✅ 三层过滤已启用: 象限 + S-L距离 + 车道")
        else:
            if enable_lane_filtering and not LANE_CONFIG_AVAILABLE:
                print("⚠️ 车道过滤已禁用: 车道配置不可用")
            else:
                print("✅ 两层过滤已启用: 象限 + S-L距离（车道过滤未启用）")
        
        # 雷达缓冲区 (时间戳 -> 雷达目标列表)
        self.radar_buffer = defaultdict(list)
        self.radar_timestamps = deque(maxlen=100)  # 保留最近100个时间戳
        
        # 匹配映射 (track_id -> radar_id)
        self.radar_id_map = {}  # 雷达ID -> 融合ID
        self.vision_id_map = {}  # 视觉ID -> 融合ID
        
        # 🔧 新增：活跃轨迹管理 (融合ID -> Track对象)
        self.active_tracks = {}  # 维护活跃的融合轨迹
        
        # 🔧 新增：track_id -> radar_id 的持久化映射（轨迹忠诚度）
        # 一旦某个track_id匹配过某个radar_id，就永远保留这个映射
        self.track_radar_history = {}  # track_id -> radar_id
        
        # 统计信息
        self.stats = {
            'radar_objects_processed': 0,
            'vision_objects_processed': 0,
            'successful_matches': 0,
            'failed_matches': 0,
            'lane_filtered_candidates': 0,  # 被车道过滤排除的候选
        }
        
        # 🔧 初始化摄像头IP映射
        if self.camera_id:
            import sys
            sys.stderr.write(f"📡 C{self.camera_id} RadarVisionFusion: 允许的雷达IP = {self.allowed_radar_ips}\n")
            sys.stderr.flush()

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
            
    def add_radar_data(self, timestamp, radar_objects):
        """
        添加雷达数据到缓冲区
        
        Args:
            timestamp: 时间戳
            radar_objects: 雷达目标列表
        """
        # 🔧 按摄像头ID过滤雷达数据
        filtered_objects = []
        rejected_objects = []
        
        for radar_obj in radar_objects:
            if self._should_accept_radar_data(radar_obj):
                filtered_objects.append(radar_obj)
            else:
                rejected_objects.append(radar_obj)
        
        # 诊断日志：仅在有拒绝时输出
        if self.camera_id and rejected_objects:
            print(f"⚠️ C{self.camera_id} 雷达数据过滤: 总数={len(radar_objects)}, "
                  f"接受={len(filtered_objects)}, 拒绝={len(rejected_objects)}")
        
        self.radar_buffer[timestamp] = filtered_objects
        self.radar_timestamps.append(timestamp)

    def find_closest_radar_timestamp(self, vision_timestamp, max_time_diff=None):
        """
        找到最接近的雷达时间戳
        视觉为准，雷达靠拢
        
        Args:
            vision_timestamp: 视觉时间戳 (Unix timestamp float)
            max_time_diff: 最大时间差 (秒)
            
        Returns:
            最接近的雷达时间戳，或 None
        """
        if max_time_diff is None:
            max_time_diff = self.MAX_TIME_DIFF

        # 🔧 修复：直接从 radar_buffer 的键中查找，而不是从受限的 deque 中查找
        # 这样可以访问所有已加载的雷达时间戳，而不会因为 deque 的 maxlen 限制而丢失早期数据
        radar_timestamps_list = list(self.radar_buffer.keys())
        
        if not radar_timestamps_list:
            return None

        closest_ts = None
        min_diff = float('inf')

        # 🔧 修复：正确处理时间戳格式转换
        # 将所有时间戳转换为秒级 Unix 时间戳用于比较
        from datetime import datetime
        
        # 首先转换 vision_timestamp 为秒级 Unix 时间戳
        if isinstance(vision_timestamp, str):
            # 假设格式为 "2025-11-21 11:59:10.171"
            try:
                vision_dt = datetime.strptime(vision_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                vision_ts_numeric = vision_dt.timestamp()
            except (ValueError, TypeError):
                # 降级：如果解析失败，尝试其他格式或使用当前时间
                logger.warning(f"无法解析视觉时间戳: {vision_timestamp}")
                return None
        else:
            # 假设已经是 Unix 时间戳
            vision_ts_numeric = float(vision_timestamp)
        
        # 然后处理雷达时间戳列表
        for radar_ts in radar_timestamps_list:
            if isinstance(radar_ts, str):
                # 字符串时间戳：转换为 Unix 时间戳
                # 格式: "2025-11-21 11:59:10.171"
                try:
                    radar_dt = datetime.strptime(radar_ts, '%Y-%m-%d %H:%M:%S.%f')
                    radar_ts_numeric = radar_dt.timestamp()
                except (ValueError, TypeError):
                    logger.warning(f"无法解析雷达时间戳: {radar_ts}")
                    continue
            else:
                # 数字时间戳：直接使用
                radar_ts_numeric = float(radar_ts)
            
            # 计算时间差（秒级）
            diff = abs(radar_ts_numeric - vision_ts_numeric)
            if diff < min_diff:
                min_diff = diff
                closest_ts = radar_ts

        # 尝试多个阈值来找到匹配
        # 1. 严格阈值：0.5秒（MAX_TIME_DIFF）
        if closest_ts is not None and min_diff <= max_time_diff:
            return closest_ts
        
        # 2. 宽松阈值：2秒（MAX_TIME_DIFF_LOOSE）
        if closest_ts is not None and min_diff <= self.MAX_TIME_DIFF_LOOSE:
            return closest_ts
        
        # 3. 诊断输出：如果两个阈值都不满足，输出警告信息
        if closest_ts is not None and time.time() - self.last_diag_time > self.diag_interval:
            min_ts = min(radar_timestamps_list)
            max_ts = max(radar_timestamps_list)
            print(f"\n⚠️ [RADAR_FUSION DIAGNOSTIC]")
            print(f"   视觉时间戳: {vision_timestamp}")
            print(f"   最接近的雷达时间戳: {closest_ts}")
            print(f"   时间差: {min_diff:.3f}秒 (严格阈值: {max_time_diff}秒, 宽松阈值: {self.MAX_TIME_DIFF_LOOSE}秒)")
            print(f"   雷达时间范围: {min_ts} ~ {max_ts}")
            print(f"   雷达数据帧数: {len(radar_timestamps_list)}")
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
        🔄 最优二部图匹配（替代贪心算法）
        使用匈牙利算法找到全局最优的匹配方案
        
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
        
        # 构建成本矩阵 (n_radar × n_vision)
        # 如果无法匹配，成本设为极大值（1e6）
        cost_matrix = np.full((n_radar, n_vision), 1e6, dtype=np.float32)
        
        for i, radar_obj in enumerate(radar_objects):
            # 区域过滤
            if self.fusion_area_geo and not point_in_polygon(
                [radar_obj.longitude, radar_obj.latitude],
                self.fusion_area_geo
            ):
                continue
            
            long_thresh = self.get_dynamic_long_threshold(radar_obj.speed)
            
            for j, v_obj in enumerate(vision_objects):
                # 区域过滤
                if self.fusion_area_geo and not point_in_polygon(
                    [v_obj.calib_lon, v_obj.calib_lat],
                    self.fusion_area_geo
                ):
                    # 诊断：如果距离很近但被区域过滤拒绝
                    dy = (v_obj.calib_lat - radar_obj.latitude) * LAT_TO_M
                    dx = (v_obj.calib_lon - radar_obj.longitude) * LON_TO_M
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist < 50:
                        print(f"    [成本矩阵] 雷达[{i}]({radar_obj.latitude:.6f},{radar_obj.longitude:.6f}) vs 视觉[{j}]({v_obj.calib_lat:.6f},{v_obj.calib_lon:.6f})")
                        print(f"      dx={dx:.2f}m, dy={dy:.2f}m, 总距离={dist:.2f}m")
                        print(f"      ❌ 视觉目标不在融合区域内，跳过")
                    continue
                
                # 数据有效性检查（防止NaN/Inf）
                try:
                    if (not isinstance(v_obj.calib_lat, (int, float)) or 
                        not isinstance(v_obj.calib_lon, (int, float)) or
                        not isinstance(radar_obj.latitude, (int, float)) or
                        not isinstance(radar_obj.longitude, (int, float)) or
                        math.isnan(v_obj.calib_lat) or math.isnan(v_obj.calib_lon) or
                        math.isnan(radar_obj.latitude) or math.isnan(radar_obj.longitude) or
                        math.isinf(v_obj.calib_lat) or math.isinf(v_obj.calib_lon) or
                        math.isinf(radar_obj.latitude) or math.isinf(radar_obj.longitude)):
                        cost_matrix[i, j] = 1e6
                        continue
                except (TypeError, ValueError):
                    cost_matrix[i, j] = 1e6
                    continue
                
                # 计算距离成本
                dy = (v_obj.calib_lat - radar_obj.latitude) * LAT_TO_M
                dx = (v_obj.calib_lon - radar_obj.longitude) * LON_TO_M
                dist = math.sqrt(dx**2 + dy**2)
                
                # 直接使用坐标差异，不用方位角
                # 纵向距离（沿南北方向）= |dy|
                # 横向距离（沿东西方向）= |dx|
                lat_diff = abs(dy)  # 纵向距离
                lon_diff = abs(dx)  # 横向距离
                
                # 诊断：打印所有距离较近的目标对
                if dist < 50:
                    print(f"    [成本矩阵] 雷达[{i}]({radar_obj.latitude:.6f},{radar_obj.longitude:.6f}) vs 视觉[{j}]({v_obj.calib_lat:.6f},{v_obj.calib_lon:.6f})")
                    print(f"      dx={dx:.2f}m, dy={dy:.2f}m, 总距离={dist:.2f}m")
                    print(f"      lon_diff(横向)={lon_diff:.2f}m(阈值{self.MAX_LANE_DIFF}m), lat_diff(纵向)={lat_diff:.2f}m(阈值{long_thresh}m)")
                
                # 检查计算结果的有效性
                if math.isnan(lat_diff) or math.isnan(lon_diff) or math.isinf(lat_diff) or math.isinf(lon_diff):
                    cost_matrix[i, j] = 1e6
                    continue
                
                # 距离阈值检查（第二层过滤：S-L）
                # lon_diff(横向) <= MAX_LANE_DIFF, lat_diff(纵向) <= long_thresh
                if lon_diff > self.MAX_LANE_DIFF or lat_diff > long_thresh:
                    if dist < 50:
                        print(f"      ❌ 距离超过阈值，设为1e6")
                    cost_matrix[i, j] = 1e6
                    continue
                
                # 车道兼容性检查（第三层过滤：车道）
                lane_compatible, lane_reason = self.check_lane_compatibility(radar_obj, v_obj)
                if not lane_compatible:
                    self.stats['lane_filtered_candidates'] = self.stats.get('lane_filtered_candidates', 0) + 1
                    if dist < 50:
                        # 获取雷达和视觉的车道信息用于诊断输出
                        radar_lane = radar_obj.lane if hasattr(radar_obj, 'lane') else None
                        vision_lane = v_obj.lane if hasattr(v_obj, 'lane') else None
                        pixel_x = v_obj.pixel_x if hasattr(v_obj, 'pixel_x') else None
                        camera_id = v_obj.cameraid if hasattr(v_obj, 'cameraid') else 'N/A'
                        radar_ip = radar_obj.ip if hasattr(radar_obj, 'ip') else None
                        radar_device_name = RADAR_IP_TO_CAMERA.get(radar_ip, 'Unknown') if radar_ip else 'N/A'
                        print(f"      ❌ 车道不兼容: {lane_reason}，设为1e6")
                        print(f"         📹 摄像头: C{camera_id} | 🎯 雷达: {radar_device_name} ({radar_ip})")
                        print(f"         🛣️  雷达车道: {radar_lane} | 🛣️  视觉车道: {vision_lane} (像素X: {pixel_x})")
                    cost_matrix[i, j] = 1e6
                    continue
                else:
                    # 车道检查通过时的诊断日志
                    if dist < 50:
                        radar_lane = radar_obj.lane if hasattr(radar_obj, 'lane') else None
                        vision_lane = v_obj.lane if hasattr(v_obj, 'lane') else None
                        pixel_x = v_obj.pixel_x if hasattr(v_obj, 'pixel_x') else None
                        print(f"      ✅ 车道兼容: {lane_reason} | 雷达车道: {radar_lane}, 视觉车道: {vision_lane} (像素X: {pixel_x})")
                
                # 计算总成本
                cost = (10.0 * lat_diff) + (1.0 * lon_diff)
                
                # 检查总成本的有效性
                if math.isnan(cost) or math.isinf(cost):
                    if dist < 50:
                        print(f"      ❌ 成本计算无效: cost={cost}，设为1e6")
                    cost_matrix[i, j] = 1e6
                    continue
                
                # 忠诚度奖励：强制保持已绑定的对
                # 策略：如果曾经匹配过，大幅降低成本（但保持正数）
                v_key = str(v_obj.track_id)
                prev_fusion_id_radar = self.radar_id_map.get(radar_obj.id)
                prev_fusion_id_vision = self.vision_id_map.get(v_key)
                
                original_cost = cost
                if prev_fusion_id_radar and prev_fusion_id_radar == prev_fusion_id_vision:
                    # 忠诚度绑定：成本除以很大的系数，保持正数
                    cost = cost / self.LOYALTY_BONUS  # 如果LOYALTY_BONUS=10000，则成本变为原来的1/10000
                    if dist < 50:
                        print(f"      💰 忠诚度绑定（已匹配过）: {original_cost:.4f} -> {cost:.6f} (系数1/{self.LOYALTY_BONUS:.0f})")
                
                # 最终成本有效性检查
                if math.isnan(cost) or math.isinf(cost):
                    if dist < 50:
                        print(f"      ❌ 最终成本无效: {cost}，设为1e6")
                    cost_matrix[i, j] = 1e6
                else:
                    cost_matrix[i, j] = cost
                    if dist < 50:
                        print(f"      ✅ 成本矩阵设置: cost_matrix[{i},{j}] = {cost:.6f}")
        
        # 使用匈牙利算法求解
        radar_indices, vision_indices = linear_sum_assignment(cost_matrix)
        
        # 诊断：打印匈牙利算法的原始结果
        if len(radar_indices) > 0:
            print(f"    [匈牙利算法结果] 总匹配数: {len(radar_indices)}")
            for r_idx, v_idx in zip(radar_indices, vision_indices):
                cost = cost_matrix[r_idx, v_idx]
                print(f"      配对 [{r_idx},{v_idx}]: cost={cost:.2f} {'✅' if cost < 1e5 else '❌'}")
        
        # 过滤掉无效匹配（成本 >= 1e5）
        valid_matches = [
            (r_idx, v_idx) 
            for r_idx, v_idx in zip(radar_indices, vision_indices)
            if cost_matrix[r_idx, v_idx] < 1e5
        ]
        
        if valid_matches:
            print(f"    [过滤结果] 有效匹配数: {len(valid_matches)}")
            radar_indices, vision_indices = zip(*valid_matches)
            return list(radar_indices), list(vision_indices)
        else:
            print(f"    [过滤结果] 无有效匹配")
            return [], []

    def process_frame(self, vision_timestamp, vision_objects):
        """
        处理单帧的雷视融合 - 集成高级融合逻辑
        使用贪婪匹配、去重机制、轨迹预测
        
        Args:
            vision_timestamp: 视觉帧时间戳
            vision_objects: 视觉目标列表 (OutputObject)
            
        Returns:
            更新后的视觉目标列表 (with radar_id)
        """
        # ===== 步骤 1：轨迹预测与清理 =====
        dead_ids = []
        for fusion_id, track in self.active_tracks.items():
            dt = vision_timestamp - track.last_update_time
            if dt > 0:
                track.predict(dt)  # 预测轨迹位置
            if dt > self.MAX_COAST_TIME:
                dead_ids.append(fusion_id)
        
        # 清理过期轨迹
        for fusion_id in dead_ids:
            del self.active_tracks[fusion_id]
            self.vision_id_map = {k: v for k, v in self.vision_id_map.items() if v != fusion_id}
            self.radar_id_map = {k: v for k, v in self.radar_id_map.items() if v != fusion_id}
        
        # ===== 步骤 2：坐标校准 =====
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
        
        # ===== 步骤 3：找到最接近的雷达时间戳 =====
        radar_timestamp = self.find_closest_radar_timestamp(vision_timestamp)
        if radar_timestamp is None:
            # 诊断：没有找到匹配的雷达时间戳
            radar_timestamps_list = list(self.radar_buffer.keys())
            if len(radar_timestamps_list) > 0:
                min_ts = min(radar_timestamps_list)
                max_ts = max(radar_timestamps_list)
                time_diff_min = abs(vision_timestamp - min_ts)
                time_diff_max = abs(vision_timestamp - max_ts)
                print(f"[RADAR_FUSION] 警告: 视觉时间戳{vision_timestamp:.3f}无法匹配雷达数据")
                print(f"  雷达时间戳范围: [{min_ts:.3f}, {max_ts:.3f}]")
                print(f"  时间差范围: [{time_diff_min:.3f}, {time_diff_max:.3f}]秒")
                print(f"  MAX_TIME_DIFF阈值: {self.MAX_TIME_DIFF}秒")
            return vision_objects
        
        # 诊断输出：当前融合的时间戳信息
        # 将Unix时间戳转换为可读格式
        vision_ts_str = datetime.fromtimestamp(vision_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print(f"[RADAR_FUSION] 融合尝试 - 视觉时间戳: {vision_ts_str}, 雷达时间戳: {radar_timestamp}, 视觉目标数: {len(vision_objects)}")
        
        radar_objects = self.radar_buffer.get(radar_timestamp, [])
        if not radar_objects:
            return vision_objects
        
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
            return vision_objects
        
        vision_objects_to_match = valid_vision_objects
        
        # 📊 统计本帧的对象数（在处理前）
        self.stats['radar_objects_processed'] += len(radar_objects)
        # vision_objects_processed会在后面处理所有视觉对象时统计
        
        # ===== 步骤 4：初始化本帧的ID占用表（去重机制） =====
        used_fusion_ids = set()
        matched_vision_track_ids = set()
        
        # ===== 步骤 5：【改进】最优二部图匹配（替代贪心算法） =====
        # 使用匈牙利算法找到全局最优匹配，避免前期贪心造成后期缺配
        radar_indices, vision_indices = self.optimal_bipartite_matching(radar_objects, vision_objects_to_match)
        
        # 诊断输出：匹配结果
        print(f"[RADAR_FUSION] 匹配结果 - 雷达目标数: {len(radar_objects)}, 视觉目标数: {len(vision_objects_to_match)}, 成功匹配: {len(radar_indices)}")
        
        # 诊断：显示雷达和视觉目标的坐标
        if len(radar_objects) > 0 and len(vision_objects_to_match) > 0:
            print(f"  雷达目标示例: {radar_objects[0].latitude:.6f}, {radar_objects[0].longitude:.6f}")
            print(f"  视觉目标示例: {vision_objects_to_match[0].lat:.6f}, {vision_objects_to_match[0].lon:.6f}")
            # 计算距离
            dy = (vision_objects_to_match[0].lat - radar_objects[0].latitude) * 111000  # 米/度
            dx = (vision_objects_to_match[0].lon - radar_objects[0].longitude) * 111000 * 0.7  # 米/度（纬度修正）
            dist = (dx**2 + dy**2)**0.5
            print(f"  距离: {dist:.2f}米 (阈值: 横向{self.MAX_LANE_DIFF}米, 纵向{self.MAX_LONG_DIFF}米)")
        
        # 处理匹配对
        for radar_idx, vision_idx in zip(radar_indices, vision_indices):
            radar_obj = radar_objects[radar_idx]
            v_obj = vision_objects_to_match[vision_idx]
            v_key = str(v_obj.track_id)
            
            # 只统计成功匹配数（雷达和视觉对象数已在前面统计）
            self.stats['successful_matches'] += 1
            
            matched_vision_track_ids.add(v_obj.track_id)
            
            # 确定融合ID
            fusion_id = self.vision_id_map.get(v_key) or self.radar_id_map.get(radar_obj.id)
            if not fusion_id:
                fusion_id = f"r{radar_obj.id[-4:]}-v{v_key}"
            
            # 去重检查
            if fusion_id in used_fusion_ids:
                fusion_id = f"r{radar_obj.id[-4:]}-v{v_key}-{int(vision_timestamp*1000)%1000}"
            
            used_fusion_ids.add(fusion_id)
            
            # 更新映射
            self.vision_id_map[v_key] = fusion_id
            self.radar_id_map[radar_obj.id] = fusion_id
            
            # 创建或更新轨迹
            track = Track(fusion_id, v_obj.calib_lat, v_obj.calib_lon, 
                         radar_obj.speed, radar_obj.azimuth)
            track.last_update_time = vision_timestamp
            track.radar_id_ref = radar_obj.id
            track.vision_id_ref = v_key
            self.active_tracks[fusion_id] = track
            
            # 设置视觉目标的雷达ID（不更新坐标，保持视觉坐标）
            v_obj.radar_id = radar_obj.id
            
            # 🔧 新增：保存track_id到radar_id的历史映射（轨迹忠诚度）
            self.track_radar_history[v_obj.track_id] = radar_obj.id
            
            # 关键修复：同时更新原始vision_objects中对应的目标
            # 因为vision_objects_to_match只是valid_vision_objects的别名，
            # 但最后返回的是原始vision_objects，所以需要找到并更新原始对象
            for orig_v_obj in vision_objects:
                if orig_v_obj.track_id == v_obj.track_id:
                    orig_v_obj.radar_id = radar_obj.id
                    break
        
        # 统计本帧的匹配情况
        matched_radar_count = len(radar_indices)
        unmatched_radar_count = len(radar_objects) - matched_radar_count
        # 累加本帧未匹配的雷达对象数
        self.stats['failed_matches'] += unmatched_radar_count
        
        # ===== 步骤 6：处理未匹配的视觉目标 =====
        # 累加本帧的视觉对象数到统计
        self.stats['vision_objects_processed'] += len(vision_objects)
        
        for v_obj in vision_objects:
            v_key = str(v_obj.track_id)
            
            if v_obj.track_id in matched_vision_track_ids:
                continue  # 已匹配，跳过
            
            # 尝试获取已有的融合ID
            fusion_id = self.vision_id_map.get(v_key)
            
            # 去重检查：如果融合ID已被占用，清除
            if fusion_id and fusion_id in used_fusion_ids:
                fusion_id = None
            
            # 幽灵复活：尝试继承已有的融合ID
            if not fusion_id:
                min_dist = 5.0
                best_ghost = None
                
                for exist_fusion_id, track in self.active_tracks.items():
                    # 只能继承含雷达历史的轨迹
                    if "r" in exist_fusion_id and track.last_update_time < vision_timestamp and exist_fusion_id not in used_fusion_ids:
                        dist_m = math.sqrt(
                            ((track.lat - v_obj.calib_lat) * LAT_TO_M)**2 +
                            ((track.lon - v_obj.calib_lon) * LON_TO_M)**2
                        )
                        if dist_m < min_dist:
                            min_dist = dist_m
                            best_ghost = exist_fusion_id
                
                if best_ghost:
                    fusion_id = best_ghost
                    self.vision_id_map[v_key] = fusion_id
            
            # 最终确定融合ID
            if not fusion_id:
                fusion_id = f"v{v_key}"
            
            # 去重检查（双保险）
            if fusion_id in used_fusion_ids and "r" in fusion_id:
                fusion_id = f"v{v_key}"
            
            used_fusion_ids.add(fusion_id)
            
            # 🔧 改进：检查历史映射，保留曾经匹配过的radar_id（轨迹忠诚度）
            if v_obj.track_id in self.track_radar_history:
                v_obj.radar_id = self.track_radar_history[v_obj.track_id]
            else:
                v_obj.radar_id = None  # 从未匹配过的视觉目标没有雷达ID
        
        return vision_objects

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
                            print(f"🔍 第一条有效雷达数据:")
                            print(f"   原始时间字符串: {time_str}")
                            print(f"   source_ip: {source_ip}, camera_id: {camera_id}")
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
                        print(f"  警告: 解析雷达数据行失败: {e}")
                        continue

            print(f"✅ 加载雷达数据完成: {len(self.radar_data)} 帧")
            print(f"   C1: {len(self.camera_timestamps[1])} 帧")
            print(f"   C2: {len(self.camera_timestamps[2])} 帧")
            print(f"   C3: {len(self.camera_timestamps[3])} 帧")
            return True

        except Exception as e:
            print(f"❌ 加载雷达数据失败: {e}")
            return False

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

    print("✅ RadarVisionFusion 模块加载成功")
