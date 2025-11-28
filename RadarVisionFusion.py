"""
雷达视觉融合模块
职责：
1. 加载雷达数据 (JSONL)
2. 与摄像头融合结果进行时间戳匹配
3. 基于地理坐标进行目标匹配
4. 更新输出对象的 radar_id 字段
"""

import json
import math
import numpy as np
from collections import defaultdict, deque
from datetime import datetime
from typing import List, Dict, Tuple, Optional


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
    """解析时间戳字符串"""
    if not ts_str:
        return 0.0
    clean_ts = ' '.join(ts_str.split())
    try:
        return datetime.strptime(clean_ts, '%Y-%m-%d %H:%M:%S.%f').timestamp()
    except:
        try:
            return datetime.strptime(clean_ts, '%Y-%m-%d %H:%M:%S').timestamp()
        except:
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
# 数据结构
# ==========================================
class RadarObject:
    """雷达目标"""
    def __init__(self, radar_id, latitude, longitude, speed=0.0, azimuth=0.0):
        self.id = radar_id
        self.latitude = latitude
        self.longitude = longitude
        self.speed = float(speed or 0)
        self.azimuth = float(azimuth or 0)


class OutputObject:
    """输出对象"""
    def __init__(self, timestamp, cameraid, type_name, confidence, track_id, lon, lat):
        self.timestamp = timestamp
        self.cameraid = cameraid
        self.type = type_name
        self.confidence = confidence
        self.track_id = track_id
        self.radar_id = None  # 初始为None，由融合模块填充
        self.lon = lon
        self.lat = lat

    def to_dict(self):
        """转换为字典"""
        return {
            'timestamp': self.timestamp,
            'cameraid': self.cameraid,
            'type': self.type,
            'confidence': self.confidence,
            'track_id': self.track_id,
            'radar_id': self.radar_id,
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

    def __init__(self, fusion_area_geo=None, lat_offset=0.0, lon_offset=0.0):
        """
        初始化雷达融合处理器
        
        Args:
            fusion_area_geo: 融合区域 (地理坐标多边形)
            lat_offset: 纬度偏移
            lon_offset: 经度偏移
        """
        # 融合参数
        self.MAX_LANE_DIFF = 3.5      # 横向距离阈值 (米)
        self.MAX_LONG_DIFF = 20.0     # 纵向距离阈值 (米)
        self.MAX_TIME_DIFF = 0.2      # 最大时间差 (秒)
        self.LOYALTY_BONUS = 10000.0  # 忠诚度奖励
        
        self.fusion_area_geo = fusion_area_geo
        self.lat_offset = lat_offset
        self.lon_offset = lon_offset
        
        # 雷达缓冲区 (时间戳 -> 雷达目标列表)
        self.radar_buffer = defaultdict(list)
        self.radar_timestamps = deque(maxlen=100)  # 保留最近100个时间戳
        
        # 匹配映射 (track_id -> radar_id)
        self.radar_id_map = {}  # 当前帧的匹配关系
        self.vision_id_map = {}  # 视觉ID -> 雷达ID
        
        # 统计信息
        self.stats = {
            'radar_objects_processed': 0,
            'vision_objects_processed': 0,
            'successful_matches': 0,
            'failed_matches': 0,
        }

    def get_dynamic_long_threshold(self, speed):
        """根据速度获取动态纵向距离阈值"""
        if speed < 1.0:
            return 5.0
        elif speed < 8.0:
            return 10.0
        else:
            return 25.0

    def add_radar_data(self, timestamp, radar_objects):
        """
        添加雷达数据到缓冲区
        
        Args:
            timestamp: 时间戳
            radar_objects: 雷达目标列表
        """
        self.radar_buffer[timestamp] = radar_objects
        self.radar_timestamps.append(timestamp)

    def find_closest_radar_timestamp(self, vision_timestamp, max_time_diff=None):
        """
        找到最接近的雷达时间戳
        
        Args:
            vision_timestamp: 视觉时间戳
            max_time_diff: 最大时间差 (秒)
            
        Returns:
            最接近的雷达时间戳，或 None
        """
        if max_time_diff is None:
            max_time_diff = self.MAX_TIME_DIFF

        closest_ts = None
        min_diff = max_time_diff

        for radar_ts in self.radar_timestamps:
            diff = abs(radar_ts - vision_timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_ts = radar_ts

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

            # 计算方位角差异
            angle = math.degrees(math.atan2(dx, dy))
            if angle < 0:
                angle += 360
            delta_rad = math.radians((angle - radar_obj.azimuth + 180) % 360 - 180)

            # 计算横向和纵向距离
            lat_diff = abs(dist * math.sin(delta_rad))
            lon_diff = abs(dist * math.cos(delta_rad))

            # 检查距离阈值
            if lat_diff <= self.MAX_LANE_DIFF and lon_diff <= long_thresh:
                # 计算成本函数
                cost = (10.0 * lat_diff) + (1.0 * lon_diff)

                # 忠诚度奖励 (如果之前匹配过)
                if vision_obj.track_id in self.vision_id_map:
                    if self.vision_id_map[vision_obj.track_id] == radar_obj.id:
                        cost -= self.LOYALTY_BONUS

                if cost < min_cost:
                    min_cost = cost
                    best_vision_obj = vision_obj

        return best_vision_obj if min_cost < 1e5 else None

    def process_frame(self, vision_timestamp, vision_objects):
        """
        处理单帧的雷视融合
        
        Args:
            vision_timestamp: 视觉帧时间戳
            vision_objects: 视觉目标列表 (OutputObject)
            
        Returns:
            更新后的视觉目标列表 (with radar_id)
        """
        # 找到最接近的雷达时间戳
        radar_timestamp = self.find_closest_radar_timestamp(vision_timestamp)

        if radar_timestamp is None:
            # 没有雷达数据，直接返回
            return vision_objects

        radar_objects = self.radar_buffer.get(radar_timestamp, [])

        if not radar_objects:
            return vision_objects

        # 初始化本帧的ID占用表
        used_track_ids = set()

        # 雷达主动匹配视觉
        matched_vision_track_ids = set()

        for radar_obj in radar_objects:
            self.stats['radar_objects_processed'] += 1

            # 尝试匹配
            matched_vision_obj = self.match_radar_to_vision(radar_obj, vision_objects)

            if matched_vision_obj is not None:
                # 匹配成功
                self.stats['successful_matches'] += 1
                matched_vision_obj.radar_id = radar_obj.id
                matched_vision_track_ids.add(matched_vision_obj.track_id)
                used_track_ids.add(matched_vision_obj.track_id)

                # 更新映射关系
                self.vision_id_map[matched_vision_obj.track_id] = radar_obj.id
            else:
                # 匹配失败
                self.stats['failed_matches'] += 1

        # 处理未匹配的视觉目标
        for vision_obj in vision_objects:
            self.stats['vision_objects_processed'] += 1

            if vision_obj.track_id not in matched_vision_track_ids:
                # 尝试从历史映射中恢复
                if vision_obj.track_id in self.vision_id_map:
                    # 检查这个映射是否仍然有效
                    radar_id = self.vision_id_map[vision_obj.track_id]
                    # 可选：验证这个雷达ID是否仍在当前帧中
                    vision_obj.radar_id = radar_id
                else:
                    # 没有历史映射，保持为 None
                    vision_obj.radar_id = None

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


# ==========================================
# 雷达数据加载器
# ==========================================
class RadarDataLoader:
    """加载和管理雷达数据"""

    def __init__(self, radar_file_path):
        """
        初始化雷达数据加载器
        
        Args:
            radar_file_path: 雷达数据文件路径 (JSONL)
        """
        self.radar_file_path = radar_file_path
        self.radar_data = {}  # 时间戳 -> 雷达目标列表

    def load(self):
        """加载雷达数据"""
        try:
            with open(self.radar_file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    try:
                        obj = json.loads(line)
                        ts = parse_time(obj.get('time', ''))
                        if ts == 0:
                            continue

                        locus = []
                        for x in obj.get('locusList', []):
                            if x.get('objType') in VALID_RADAR_TYPES:
                                radar_obj = RadarObject(
                                    radar_id=x.get('id', ''),
                                    latitude=float(x.get('latitude', 0)),
                                    longitude=float(x.get('longitude', 0)),
                                    speed=float(x.get('speed', 0)),
                                    azimuth=float(x.get('azimuth', 0))
                                )
                                locus.append(radar_obj)

                        if locus:
                            self.radar_data[ts] = locus

                    except Exception as e:
                        print(f"  警告: 解析雷达数据行失败: {e}")
                        continue

            print(f"✅ 加载雷达数据完成: {len(self.radar_data)} 帧")
            return True

        except Exception as e:
            print(f"❌ 加载雷达数据失败: {e}")
            return False

    def get_radar_data(self, timestamp):
        """获取指定时间戳的雷达数据"""
        return self.radar_data.get(timestamp, [])

    def get_all_timestamps(self):
        """获取所有雷达时间戳"""
        return sorted(self.radar_data.keys())


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
