"""
雷达数据筛选和转发模块
核心逻辑：
1. 根据经纬度区域筛选雷达数据：融合区内的送入RadarVisionFusion进行匹配，融合区外的直接输出原始经纬度
2. 为输出的雷达数据添加source标记，输出标记为radar
"""

import numpy as np
import cv2
import logging
from typing import Dict, Tuple, List, Optional

logger = logging.getLogger(__name__)

# 导入现有的工具和配置
from core.Basic import Config, GeometryUtils
from core.RadarVisionFusion import RadarObject
from core.StreamingDataLoader import RadarObject as StreamingRadarObject

# 导入统一日志配置
try:
    from core.logger_config import FusionLogger
except ImportError:
    FusionLogger = None

# ==========================================
# 融合区域定义（经纬度坐标）
# ==========================================
# 这些区域是RADAR_VISION_FUSION_AREAS转换为经纬度坐标后的结果
FUSION_AREAS_GEO = {
    1: np.array([
        [113.58442632948723, 23.53082277234001],
        [113.5844626813328, 23.530850485064967],
        [113.58448764143974, 23.530890894029334],
        [113.58440759630842, 23.530977684412633],
        [113.58431585041077, 23.53096077252552]
    ], dtype=np.float32),
    
    2: np.array([
        [113.5838948939388, 23.53039488038343],
        [113.58400574941096, 23.53030364063503],
        [113.58412601077292, 23.53034912220365],
        [113.58398192357281, 23.53049983740589]
    ], dtype=np.float32),
    
    3: np.array([
        [113.5840323268417, 23.53088644576019],
        [113.58398885976942, 23.530938299868428],
        [113.58392264474017, 23.53089831862778],
        [113.58395313106759, 23.5308325770396]
    ], dtype=np.float32)
}


# ==========================================
# 雷达数据过滤器
# ==========================================
class RadarDataFilter:
    """
    雷达数据地理区域过滤器
    
    职责：
    1. 判断雷达数据是否在融合区内
    2. 区内数据返回给RadarVisionFusion进行匹配
    3. 区外数据直接输出（添加source标记）
    """
    
    def __init__(self):
        """初始化过滤器"""
        self.fusion_areas = FUSION_AREAS_GEO
        logger.info(f"✓ RadarDataFilter初始化完成，加载了{len(self.fusion_areas)}个融合区域")
    
    def is_in_fusion_area(self, lon: float, lat: float) -> bool:
        """
        检查地理坐标(经纬度)是否在任意融合区内
        
        参数:
            lon: 经度
            lat: 纬度
        
        返回:
            True if 点在任意融合区内，False otherwise
        """
        point = np.array([lon, lat], dtype=np.float32)
        
        for camera_id, area in self.fusion_areas.items():
            # 使用cv2.pointPolygonTest判断点是否在多边形内
            # 返回值 >= 0 表示在区域内
            if cv2.pointPolygonTest(area, tuple(point), False) >= 0:
                return True
        
        return False
    
    def filter_radar_data(self, radar_data) -> Tuple[Optional[dict], Optional[dict]]:
        """
        过滤单条雷达数据
        
        参数:
            radar_data: 原始雷达数据，可以是dict或RadarObject对象
                        如果是dict，必须包含 'lon' 和 'lat' 字段
                        如果是RadarObject，使用 longitude 和 latitude 属性
        
        返回:
            (fusion_data, direct_output_data)
            - fusion_data: 需要送入RadarVisionFusion的数据（融合区内），如果不在融合区则为None
            - direct_output_data: 直接输出的数据（融合区外），如果在融合区则为None
        
        示例:
            >>> filter = RadarDataFilter()
            >>> fusion, output = filter.filter_radar_data({
            ...     'lon': 113.5845, 'lat': 23.5310, 'track_id': 'r001'
            ... })
            >>> if fusion:
            ...     fusion_system.process(fusion)
            >>> if output:
            ...     results.append(output)
        """
        try:
            # 支持dict和RadarObject两种格式
            if isinstance(radar_data, dict):
                lon = radar_data.get('lon')
                lat = radar_data.get('lat')
                data_dict = radar_data
            elif isinstance(radar_data, (RadarObject, StreamingRadarObject)):
                lon = radar_data.longitude
                lat = radar_data.latitude
                # 将RadarObject转换为字典格式用于输出
                # 直接使用原始时间字符串
                data_dict = {
                    'radar_id': radar_data.id,
                    'lon': lon,
                    'lat': lat,
                    'speed': radar_data.speed,
                    'azimuth': radar_data.azimuth,
                    'lane': radar_data.lane,
                    'timestamp': radar_data.timestamp_str  # 直接使用原始时间字符串
                }
            else:
                logger.warning(f"⚠️ 不支持的雷达数据格式: {type(radar_data)}")
                return None, None
            
            # 参数校验
            if lon is None or lat is None:
                logger.warning(f"⚠️ 雷达数据缺少经纬度字段: {radar_data}")
                return None, None
            
            # 判断是否在融合区
            if self.is_in_fusion_area(lon, lat):
                # 在融合区内 → 送入融合系统
                #logger.debug(f"📍 雷达点({lon:.6f}, {lat:.6f})在融合区内，送入融合系统")
                return data_dict, None
            else:
                # 区域外 → 直接输出
                output = data_dict.copy()
                output['source'] = 'radar'  # 添加source标记
                #logger.debug(f"📍 雷达点({lon:.6f}, {lat:.6f})在融合区外，直接输出")
                return None, output
                
        except Exception as e:
            logger.error(f"❌ 过滤雷达数据时出错: {e}", exc_info=True)
            return None, None
    
    def batch_filter_radar_data(self, radar_data_list: List[dict]) -> Tuple[List[dict], List[dict]]:
        """
        批量过滤雷达数据
        
        参数:
            radar_data_list: 雷达数据list
        
        返回:
            (fusion_data_list, direct_output_list)
        """
        import time as time_module
        start_time = time_module.time()
        
        fusion_data_list = []
        direct_output_list = []
        
        for radar_data in radar_data_list:
            fusion_data, output_data = self.filter_radar_data(radar_data)
            if fusion_data:
                fusion_data_list.append(fusion_data)
                # 条件日志：记录融合区内的数据（受ENABLE_RADAR_FILTER_LOG控制）
                if len(fusion_data_list) == 1 and FusionLogger and FusionLogger.ENABLE_RADAR_FILTER_LOG:  # 只记录第一条
                    logger.info(f"📍 第一条融合区内数据: timestamp={fusion_data.get('timestamp')}, radar_id={fusion_data.get('radar_id')}")
            if output_data:
                direct_output_list.append(output_data)
                # 条件日志：记录融合区外的数据（受ENABLE_RADAR_FILTER_LOG控制）
                if len(direct_output_list) == 1 and FusionLogger and FusionLogger.ENABLE_RADAR_FILTER_LOG:  # 只记录第一条
                    logger.info(f"📍 第一条融合区外数据: timestamp={output_data.get('timestamp')}, radar_id={output_data.get('radar_id')}")
        
        elapsed = (time_module.time() - start_time) * 1000
        # 条件日志：批量过滤完成信息（受ENABLE_RADAR_FILTER_LOG控制）
        if FusionLogger and FusionLogger.ENABLE_RADAR_FILTER_LOG:
        logger.info(f"📊 批量过滤完成: 总数={len(radar_data_list)}, "
                   f"融合区内={len(fusion_data_list)}, 融合区外={len(direct_output_list)}, 耗时={elapsed:.2f}ms")
        
        return fusion_data_list, direct_output_list


# ==========================================
# 使用示例
# ==========================================
"""
在主程序中的使用方式：

from radar.RadarDataFilter import RadarDataFilter
from core.RadarVisionFusion import RadarVisionFusion

# 初始化
radar_filter = RadarDataFilter()
fusion_system = RadarVisionFusion()

# 处理每一帧的雷达数据
for frame_data in frames:
    radar_detections = frame_data['radar_detections']  # 原始雷达数据列表
    
    # ===== 第一道关卡：地理区域过滤 =====
    fusion_radar_data, direct_output_radar = radar_filter.batch_filter_radar_data(
        radar_detections
    )
    
    # ===== 区外雷达数据：直接输出 =====
    # 这些数据已经加上了 'source': 'radar' 标记
    for radar_obj in direct_output_radar:
        output_results.append(radar_obj)
        logger.info(f"直接输出区外雷达: {radar_obj['track_id']}")
    
    # ===== 区内雷达数据：送入融合系统 =====
    # 融合系统会与视觉数据进行匹配，并输出带有radar_id的融合结果
    if fusion_radar_data:
        # 将过滤后的雷达数据加入融合系统的缓冲区
        for radar_obj in fusion_radar_data:
            fusion_system.add_radar_data(
                timestamp=frame_data['timestamp'],
                radar_obj=radar_obj
            )
        
        # 处理视觉数据（触发匹配）
        vision_objects = frame_data['vision_detections']
        fusion_results = fusion_system.process_frame(
            vision_timestamp=frame_data['timestamp'],
            vision_objects=vision_objects
        )
        
        # 融合结果已包含 'radar_id' 字段（表示关联的雷达ID）
        for fusion_obj in fusion_results:
            output_results.append(fusion_obj)
            if fusion_obj.get('radar_id'):
                logger.info(f"融合输出: {fusion_obj['track_id']} -> radar_id={fusion_obj['radar_id']}")

数据流总结：
┌─────────────────────────────┐
│   原始雷达数据               │
│ (lon, lat, track_id, ...)   │
└────────────────┬────────────┘
                 │
        ┌────────▼────────┐
        │ RadarDataFilter │
        │   (地理过滤)     │
        └────┬────────┬───┘
             │        │
   ┌─────────▼──┐  ┌─┴──────────────┐
   │ 融合区外   │  │  融合区内       │
   │ (直接输出) │  │ (送入融合系统)  │
   │ src=radar  │  │                │
   └────────────┘  └────────┬───────┘
                           │
                    ┌──────▼──────────┐
                    │RadarVisionFusion│
                    │  (三层过滤匹配)  │
                    │ 象限+S-L+车道   │
                    └──────┬──────────┘
                           │
                    ┌──────▼──────────────┐
                    │  融合输出           │
                    │ (带radar_id)        │
                    │ src=fusion          │
                    └─────────────────────┘
"""