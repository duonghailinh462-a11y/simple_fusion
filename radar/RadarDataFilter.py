"""
雷达数据筛选和转发模块

核心逻辑：
1. 将融合区域像素坐标转换为经纬度范围
2. 筛选雷达数据：融合区内的完全不输出，融合区外的输出原始经纬度
3. 为输出的雷达数据添加source标记
"""

import numpy as np
import cv2
import logging
from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# 导入现有的工具和配置
from core.Basic import Config, GeometryUtils, CAMERA_MATRICES, GEO_ORIGIN_LON, GEO_ORIGIN_LAT, METERS_PER_DEG_LON, METERS_PER_DEG_LAT


@dataclass
class RadarGeoFusionArea:
    """融合区域地理坐标范围"""
    lon_min: float
    lon_max: float
    lat_min: float
    lat_max: float
    
    def contains(self, lon: float, lat: float) -> bool:
        """检查经纬度是否在区域内"""
        return (self.lon_min <= lon <= self.lon_max and 
                self.lat_min <= lat <= self.lat_max)


class PixelToGeoConverter:
    """像素坐标到地理坐标的转换器"""
    
    def __init__(self):
        """初始化转换器"""
        self.camera_matrices = CAMERA_MATRICES
    
    def pixel_to_geo(self, pixel_point: Tuple[int, int], camera_id: int) -> Optional[Tuple[float, float]]:
        """
        将像素坐标转换为经纬度
        
        Args:
            pixel_point: (x, y) 像素坐标
            camera_id: 摄像头ID
            
        Returns:
            (lon, lat) 或 None 如果转换失败
        """
        if camera_id not in self.camera_matrices:
            return None
        
        H_matrix = self.camera_matrices[camera_id]
        x, y = pixel_point
        
        # 像素 -> BEV
        bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, float(x), float(y))
        if not bev_result:
            return None
        
        # BEV -> 地理坐标
        geo_result = GeometryUtils.bev_to_geo(bev_result[0], bev_result[1])
        return geo_result
    
    def polygon_to_geo_bounds(self, pixel_polygon: np.ndarray, camera_id: int) -> Optional[Dict]:
        """
        将像素坐标多边形转换为地理坐标边界
        
        Args:
            pixel_polygon: 像素坐标多边形 [(x1,y1), (x2,y2), ...]
            camera_id: 摄像头ID
            
        Returns:
            包含 lon_min, lon_max, lat_min, lat_max 的字典，或 None 如果转换失败
        """
        geo_points = []
        
        for point in pixel_polygon:
            geo_point = self.pixel_to_geo((int(point[0]), int(point[1])), camera_id)
            if geo_point:
                geo_points.append(geo_point)
        
        if not geo_points:
            logger.warning(f"C{camera_id}: 无法转换多边形到地理坐标")
            return None
        
        lons = [p[0] for p in geo_points]
        lats = [p[1] for p in geo_points]
        
        return {
            'lon_min': min(lons),
            'lon_max': max(lons),
            'lat_min': min(lats),
            'lat_max': max(lats),
            'geo_points': geo_points  # 用于调试
        }


class RadarFusionAreaManager:
    """管理所有摄像头的融合区域（地理坐标版本）"""
    
    def __init__(self):
        """初始化融合区域管理器"""
        self.pixel_converter = PixelToGeoConverter()
        self.geo_fusion_areas: Dict[int, RadarGeoFusionArea] = {}
        self._init_geo_fusion_areas()
    
    def _init_geo_fusion_areas(self):
        """初始化地理坐标融合区域"""
        pixel_areas = Config.RADAR_VISION_FUSION_AREAS
        
        for camera_id, pixel_polygon in pixel_areas.items():
            bounds = self.pixel_converter.polygon_to_geo_bounds(pixel_polygon, camera_id)
            
            if bounds:
                geo_area = RadarGeoFusionArea(
                    lon_min=bounds['lon_min'],
                    lon_max=bounds['lon_max'],
                    lat_min=bounds['lat_min'],
                    lat_max=bounds['lat_max']
                )
                self.geo_fusion_areas[camera_id] = geo_area
                
                logger.info(
                    f"C{camera_id} 地理坐标融合区域: "
                    f"lon=[{bounds['lon_min']:.8f}, {bounds['lon_max']:.8f}], "
                    f"lat=[{bounds['lat_min']:.8f}, {bounds['lat_max']:.8f}]"
                )
                logger.debug(f"  地理坐标关键点: {bounds['geo_points']}")
            else:
                logger.error(f"C{camera_id}: 无法初始化地理坐标融合区域")
    
    def is_in_any_fusion_area(self, lon: float, lat: float) -> Tuple[bool, int]:
        """
        检查经纬度是否在任何摄像头的融合区域内
        
        Returns:
            (is_in_area, camera_id) - camera_id 是融合区域所属的摄像头，如果不在任何区域内则为 None
        """
        for camera_id, geo_area in self.geo_fusion_areas.items():
            if geo_area.contains(lon, lat):
                return True, camera_id
        
        return False, None
    
    def get_fusion_area_bounds(self, camera_id: int) -> Optional[RadarGeoFusionArea]:
        """获取指定摄像头的融合区域"""
        return self.geo_fusion_areas.get(camera_id)


class RadarDataFilter:
    """
    雷达数据筛选器
    
    核心逻辑：
    - 融合区域内的数据：完全不输出
    - 融合区域外的数据：输出原始经纬度 + source="radar"
    """
    
    def __init__(self):
        """初始化筛选器"""
        self.fusion_area_manager = RadarFusionAreaManager()
        self.stats = {
            'total_radar_objects': 0,
            'filtered_in_fusion_area': 0,
            'output_outside_fusion_area': 0
        }
    
    def filter_radar_frame(self, radar_frame: dict) -> List[dict]:
        """
        筛选单帧雷达数据
        
        Args:
            radar_frame: 原始雷达数据帧 (from radar_data_cleaned.jsonl)
                {
                    "source_ip": "...",
                    "deviceSn": "...",
                    "time": "2025-11-21 11:59:10.171",
                    "count": 9,
                    "locusList": [...]
                }
        
        Returns:
            输出对象列表 (可输出的雷达数据)
            [
                {
                    "source": "radar",
                    "radar_id": "00000060-5e1c-4083-bd10-e37f00073c12",
                    "type": "车型",
                    "lon": 113.58454508897765,
                    "lat": 23.53096181562125,
                    "timestamp": "2025-11-21 11:59:10.171",
                    ...其他原始信息
                }
            ]
        """
        output_objects = []
        frame_time = radar_frame.get('time', '')
        locusList = radar_frame.get('locusList', [])
        
        self.stats['total_radar_objects'] += len(locusList)
        
        for obj in locusList:
            radar_id = obj.get('id')
            lon = obj.get('longitude')
            lat = obj.get('latitude')
            
            # 检查是否在融合区域内
            is_in_fusion_area, camera_id = self.fusion_area_manager.is_in_any_fusion_area(lon, lat)
            
            if is_in_fusion_area:
                # 融合区域内的数据完全不输出
                self.stats['filtered_in_fusion_area'] += 1
                logger.debug(
                    f"过滤融合区内数据: radar_id={radar_id}, "
                    f"lon={lon:.8f}, lat={lat:.8f}, camera_id={camera_id}"
                )
            else:
                # 融合区域外的数据输出
                self.stats['output_outside_fusion_area'] += 1
                
                # 将车辆类型代码转换为字符串
                vehicle_type = self._get_vehicle_type_name(obj.get('objType', 0))
                
                output_obj = {
                    'source': 'radar',
                    'radar_id': radar_id,
                    'type': vehicle_type,
                    'confidence': obj.get('probability', 0) / 255.0,  # 概率值 0-255 转换为 0-1
                    'lon': lon,
                    'lat': lat,
                    'timestamp': frame_time,
                    'track_id': None,  # 雷达数据没有 track_id
                    # 保留原始信息用于调试
                    'posX': obj.get('posX'),
                    'posY': obj.get('posY'),
                    'speed': obj.get('speed', 0),
                    'azimuth': obj.get('azimuth'),
                }
                output_objects.append(output_obj)
        
        return output_objects
    
    @staticmethod
    def _get_vehicle_type_name(obj_type_code: int) -> str:
        """
        将雷达物体类型代码转换为字符串
        
        Args:
            obj_type_code: 物体类型代码 (1=truck, 2=van, 3=car, etc.)
        
        Returns:
            物体类型字符串
        """
        type_mapping = {
            1: 'truck',      # 卡车
            2: 'van',        # 面包车
            3: 'car',        # 小车
            4: 'bus',        # 公交
            5: 'motorcycle', # 摩托车
            6: 'bicycle',    # 自行车
            9: 'car',        # 小轿车
        }
        return type_mapping.get(obj_type_code, 'unknown')
    
    def filter_radar_data(self, radar_objects_batch: List[dict]) -> List[dict]:
        """
        批量筛选雷达数据
        
        Args:
            radar_objects_batch: 多帧雷达数据
        
        Returns:
            所有输出对象的列表
        """
        output_data = []
        
        for frame in radar_objects_batch:
            frame_outputs = self.filter_radar_frame(frame)
            output_data.extend(frame_outputs)
        
        return output_data
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return {
            **self.stats,
            'output_rate': (
                self.stats['output_outside_fusion_area'] / self.stats['total_radar_objects']
                if self.stats['total_radar_objects'] > 0 else 0
            )
        }
    
    def reset_stats(self):
        """重置统计信息"""
        for key in self.stats:
            self.stats[key] = 0


def load_and_filter_radar_data(jsonl_file_path: str) -> Tuple[List[dict], dict]:
    """
    从JSONL文件加载并筛选雷达数据
    
    Args:
        jsonl_file_path: JSONL文件路径
    
    Returns:
        (output_objects, stats)
    """
    import json
    
    radar_filter = RadarDataFilter()
    radar_frames = []
    
    # 读取JSONL文件
    try:
        with open(jsonl_file_path, 'r', encoding='utf-8') as f:
            for line in f:
                if line.strip():
                    radar_frames.append(json.loads(line))
        logger.info(f"已加载 {len(radar_frames)} 帧雷达数据")
    except Exception as e:
        logger.error(f"加载JSONL文件出错: {e}")
        return [], radar_filter.get_stats()
    
    # 筛选数据
    output_objects = radar_filter.filter_radar_data(radar_frames)
    stats = radar_filter.get_stats()
    
    logger.info(
        f"雷达数据筛选完成: 总数={stats['total_radar_objects']}, "
        f"过滤融合区={stats['filtered_in_fusion_area']}, "
        f"输出融合区外={stats['output_outside_fusion_area']}, "
        f"输出率={stats['output_rate']:.2%}"
    )
    
    return output_objects, stats

