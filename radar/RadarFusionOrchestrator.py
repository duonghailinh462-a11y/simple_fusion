# -*- coding: utf-8 -*-
"""
雷达融合协调器 - 统一处理雷达融合流程
职责：
1. 协调雷达数据过滤
2. 执行按摄像头的雷达融合
3. 管理雷达ID映射
4. 统计融合结果
"""

import time
import logging
from typing import List, Dict, Tuple, Optional
from datetime import datetime

from core.Basic import GeometryUtils
from core.RadarVisionFusion import OutputObject
from config.region_config import get_lane_for_point

logger = logging.getLogger(__name__)


class RadarFusionOrchestrator:
    """雷达融合协调器 - 统一处理雷达融合的完整流程"""
    
    def __init__(self, radar_data_loader, radar_filter, radar_fusion_processors):
        """
        初始化雷达融合协调器
        
        Args:
            radar_data_loader: 雷达数据加载器
            radar_filter: 雷达数据过滤器
            radar_fusion_processors: 按摄像头的融合处理器字典 {camera_id: processor}
        """
        self.radar_data_loader = radar_data_loader
        self.radar_filter = radar_filter
        self.radar_fusion_processors = radar_fusion_processors
    
    def process_radar_fusion(self, current_frame: int, current_frame_results: Dict,
                            all_global_targets: List, all_local_targets: List,
                            perf_monitor=None) -> Tuple[Dict, List]:
        """
        执行完整的雷达融合处理流程
        
        Args:
            current_frame: 当前帧号
            current_frame_results: 当前帧的视觉结果 {camera_id: result}
            all_global_targets: 所有全局目标
            all_local_targets: 所有本地目标
            perf_monitor: 性能监控器
        
        Returns:
            (radar_id_map, direct_radar_outputs)
            - radar_id_map: {track_id: radar_id} 映射
            - direct_radar_outputs: 直接输出的雷达数据列表
        """
        radar_id_map = {}
        direct_radar_outputs = []
        
        if not self.radar_fusion_processors:
            return radar_id_map, direct_radar_outputs
        
        # ===== 第一道关卡：地理区域过滤 =====
        if perf_monitor:
            perf_monitor.start_timer('radar_filtering')
        
        fusion_radar_data = []
        radar_timestamps_list = list(self.radar_data_loader.radar_data.keys()) if self.radar_data_loader else []
        
        if radar_timestamps_list and current_frame_results:
            # 获取视觉时间戳（以第一个摄像头为基准）
            vision_timestamp = current_frame_results.get(1, {}).get('timestamp')
            
            if vision_timestamp:
                # 找到最接近的雷达时间戳
                closest_radar_ts = self._find_closest_radar_timestamp(
                    vision_timestamp, radar_timestamps_list, current_frame
                )
                
                if closest_radar_ts and closest_radar_ts in self.radar_data_loader.radar_data:
                    all_radar_data = self.radar_data_loader.radar_data[closest_radar_ts]
                    
                    # 执行地理区域过滤
                    fusion_radar_data, direct_radar_outputs = self.radar_filter.batch_filter_radar_data(
                        all_radar_data
                    )
                    
                    if current_frame % 100 == 0:
                        logger.debug(f"Frame {current_frame}: 雷达过滤 总数={len(all_radar_data)}, "
                                   f"融合区内={len(fusion_radar_data)}, 融合区外={len(direct_radar_outputs)}")
        
        if perf_monitor:
            perf_monitor.end_timer('radar_filtering')
        
        # ===== 第二道关卡：按摄像头进行雷达融合 =====
        if perf_monitor:
            perf_monitor.start_timer('radar_fusion_processing')
        
        for camera_id in [1, 2, 3]:
            if camera_id not in self.radar_fusion_processors:
                continue
            
            # 收集该摄像头的所有目标（全局+本地已匹配）
            vision_objects = self._collect_vision_objects(
                camera_id, all_global_targets, all_local_targets
            )
            
            if not vision_objects:
                continue
            
            # 获取该摄像头的原始时间戳
            original_timestamp = self._get_camera_timestamp(camera_id, current_frame_results)
            
            # 诊断输出：显示当前处理的摄像头和时间戳
            print(f"[RADAR_FUSION_ORCHESTRATOR] Frame {current_frame} C{camera_id}: 原始时间戳={original_timestamp}, 视觉目标数={len(vision_objects)}")
            
            # 执行雷达融合
            updated_vision_objects = self.radar_fusion_processors[camera_id].process_frame(
                original_timestamp, vision_objects
            )
            
            # 构建雷达ID映射
            for vision_obj in updated_vision_objects:
                if vision_obj.radar_id is not None:
                    radar_id_map[vision_obj.track_id] = vision_obj.radar_id
                    
                    if current_frame % 100 == 0:
                        logger.debug(f"Frame {current_frame} C{camera_id}: 雷达ID映射 "
                                   f"track_id={vision_obj.track_id} -> radar_id={vision_obj.radar_id}")
            
            # 统计信息
            matched_count = sum(1 for v in updated_vision_objects if v.radar_id is not None)
            if current_frame % 100 == 0 and matched_count > 0:
                logger.info(f"Frame {current_frame} C{camera_id}: 雷达匹配 "
                           f"{matched_count}/{len(updated_vision_objects)} 个目标")
        
        if perf_monitor:
            perf_monitor.end_timer('radar_fusion_processing')
        
        return radar_id_map, direct_radar_outputs
    
    def _find_closest_radar_timestamp(self, vision_timestamp, radar_timestamps_list: List[str], 
                                     current_frame: int) -> Optional[str]:
        """找到最接近的雷达时间戳"""
        try:
            vision_ts_str = str(vision_timestamp)
            
            # 将时间戳字符串转换为可比较的数值
            closest_radar_ts = min(
                radar_timestamps_list,
                key=lambda ts: abs(
                    int(ts.replace('-', '').replace(':', '').replace(' ', '').replace('.', '')) -
                    int(vision_ts_str.replace('-', '').replace(':', '').replace(' ', '').replace('.', ''))
                )
            )
            
            # 调试日志
            if current_frame <= 3:
                time_diff = abs(
                    int(closest_radar_ts.replace('-', '').replace(':', '').replace(' ', '').replace('.', '')) -
                    int(vision_ts_str.replace('-', '').replace(':', '').replace(' ', '').replace('.', ''))
                )
                logger.info(f"Frame {current_frame}: 视觉时间戳={vision_ts_str}, "
                           f"最接近雷达时间戳={closest_radar_ts}, 差值={time_diff}")
            
            return closest_radar_ts
        except Exception as e:
            logger.warning(f"查找最接近的雷达时间戳失败: {e}")
            return None
    
    def _collect_vision_objects(self, camera_id: int, all_global_targets: List, 
                               all_local_targets: List) -> List[OutputObject]:
        """收集指定摄像头的所有视觉目标"""
        vision_objects = []
        
        # 处理全局目标
        for global_target in all_global_targets:
            if global_target.camera_id != camera_id:
                continue
            if not global_target.bev_trajectory:
                continue
            
            current_bev = global_target.bev_trajectory[-1]
            if current_bev[0] == 0.0 and current_bev[1] == 0.0:
                continue
            
            geo_result = GeometryUtils.bev_to_geo(current_bev[0], current_bev[1])
            if not geo_result:
                continue
            
            lng, lat = geo_result
            confidence = global_target.confidence_history[-1] if global_target.confidence_history else 0.0
            
            # 获取车道信息
            lane = None
            pixel_x, pixel_y = None, None
            if global_target.pixel_trajectory:
                pixel_x, pixel_y = global_target.pixel_trajectory[-1]
                lane = get_lane_for_point(camera_id, pixel_x, pixel_y)
            
            vision_obj = OutputObject(
                timestamp="",
                cameraid=global_target.camera_id,
                type_name=global_target.class_name,
                confidence=confidence,
                track_id=global_target.global_id,
                lon=lng,
                lat=lat,
                pixel_x=pixel_x,
                lane=lane
            )
            vision_objects.append(vision_obj)
        
        # 处理本地目标（已匹配的）
        for local_target in all_local_targets:
            if local_target.camera_id != camera_id:
                continue
            if not local_target.matched_global_id:
                continue
            
            if local_target.current_bev_pos[0] == 0.0 and local_target.current_bev_pos[1] == 0.0:
                continue
            
            geo_result = GeometryUtils.bev_to_geo(local_target.current_bev_pos[0], 
                                                 local_target.current_bev_pos[1])
            if not geo_result:
                continue
            
            lng, lat = geo_result
            
            # 检查是否已经添加过这个global_id
            if not any(v.track_id == local_target.matched_global_id for v in vision_objects):
                pixel_x, pixel_y = local_target.current_pixel_pos
                lane = get_lane_for_point(camera_id, pixel_x, pixel_y)
                
                vision_obj = OutputObject(
                    timestamp="",
                    cameraid=local_target.camera_id,
                    type_name=local_target.class_name,
                    confidence=local_target.confidence,
                    track_id=local_target.matched_global_id,
                    lon=lng,
                    lat=lat,
                    pixel_x=pixel_x,
                    lane=lane
                )
                vision_objects.append(vision_obj)
        
        return vision_objects
    
    def _get_camera_timestamp(self, camera_id: int, current_frame_results: Dict) -> float:
        """获取摄像头的原始时间戳（转换为浮点数）"""
        if camera_id not in current_frame_results:
            logger.warning(f"C{camera_id} 没有当前帧结果，使用当前时间作为时间戳")
            return time.time()
        
        result = current_frame_results[camera_id]
        original_timestamp = result.get('timestamp', time.time())
        
        # 如果是字符串，转换为浮点数
        if isinstance(original_timestamp, str):
            return self._convert_timestamp_string_to_float(original_timestamp)
        
        return original_timestamp
    
    @staticmethod
    def _convert_timestamp_string_to_float(timestamp_str: str) -> float:
        """将时间戳字符串转换为浮点数
        
        支持格式：
        - 'YYYY-MM-DD HH:MM:SS.mmm' (3位毫秒)
        - 'YYYY-MM-DD HH:MM:SS.mmmmmm' (6位微秒)
        """
        try:
            # 方法1：先尝试6位微秒格式
            try:
                dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
                return dt.timestamp()
            except ValueError:
                # 方法2：如果失败，说明可能是3位毫秒，需要补充到6位
                parts = timestamp_str.split('.')
                if len(parts) == 2:
                    second_part = parts[0]
                    ms_part = parts[1]
                    # 补充到6位微秒
                    us_part = ms_part.ljust(6, '0')
                    ts_with_us = f"{second_part}.{us_part}"
                    dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                    return dt.timestamp()
                else:
                    raise ValueError("时间戳格式错误")
        except Exception as e:
            logger.warning(f"时间戳转换失败: {timestamp_str}, 错误: {e}")
            return time.time()
    
    def get_overall_statistics(self):
        """
        获取所有摄像头的总体融合统计信息
        
        Returns:
            dict: 包含每个摄像头和总体的统计信息
        """
        overall_stats = {
            'by_camera': {},
            'total': {
                'total_radar_objects': 0,
                'total_vision_objects': 0,
                'successful_matches': 0,
                'failed_matches': 0,
                'lane_filtered_candidates': 0,
            }
        }
        
        # 收集每个摄像头的统计
        for camera_id in [1, 2, 3]:
            if camera_id in self.radar_fusion_processors:
                processor = self.radar_fusion_processors[camera_id]
                camera_stats = processor.get_matching_statistics()
                overall_stats['by_camera'][camera_id] = camera_stats
                
                # 累加到总体统计
                overall_stats['total']['total_radar_objects'] += camera_stats['total_radar_objects']
                overall_stats['total']['total_vision_objects'] += camera_stats['total_vision_objects']
                overall_stats['total']['successful_matches'] += camera_stats['successful_matches']
                overall_stats['total']['failed_matches'] += camera_stats['failed_matches']
                overall_stats['total']['lane_filtered_candidates'] += camera_stats['lane_filtered_candidates']
        
        # 计算总体匹配率
        total_radar = overall_stats['total']['total_radar_objects']
        total_vision = overall_stats['total']['total_vision_objects']
        total_successful = overall_stats['total']['successful_matches']
        
        overall_stats['total']['radar_match_rate'] = (
            round(total_successful / total_radar * 100, 2) if total_radar > 0 else 0.0
        )
        overall_stats['total']['vision_match_rate'] = (
            round(total_successful / total_vision * 100, 2) if total_vision > 0 else 0.0
        )
        
        return overall_stats
    
    def print_overall_statistics(self):
        """打印所有摄像头的总体融合统计信息"""
        stats = self.get_overall_statistics()
        
        print("\n" + "="*80)
        print("【雷达视觉融合 - 全摄像头统计】")
        print("="*80)
        
        # 按摄像头打印
        for camera_id in [1, 2, 3]:
            if camera_id in stats['by_camera']:
                cam_stats = stats['by_camera'][camera_id]
                print(f"\n【摄像头 C{camera_id}】")
                print(f"  雷达目标:        {cam_stats['total_radar_objects']:>5} 个")
                print(f"  视觉目标:        {cam_stats['total_vision_objects']:>5} 个")
                print(f"  成功匹配:        {cam_stats['successful_matches']:>5} 个")
                print(f"  失败匹配:        {cam_stats['failed_matches']:>5} 个")
                print(f"  车道过滤:        {cam_stats['lane_filtered_candidates']:>5} 个")
                print(f"  雷达匹配率:      {cam_stats['radar_match_rate']:>5.1f}%")
                print(f"  视觉匹配率:      {cam_stats['vision_match_rate']:>5.1f}%")
        
        # 打印总体统计
        total = stats['total']
        print("\n" + "-"*80)
        print("【总体统计】")
        print(f"  雷达目标总数:    {total['total_radar_objects']:>5} 个")
        print(f"  视觉目标总数:    {total['total_vision_objects']:>5} 个")
        print(f"  成功匹配总数:    {total['successful_matches']:>5} 个")
        print(f"  失败匹配总数:    {total['failed_matches']:>5} 个")
        print(f"  车道过滤总数:    {total['lane_filtered_candidates']:>5} 个")
        print(f"  总体雷达匹配率:  {total['radar_match_rate']:>5.1f}%")
        print(f"  总体视觉匹配率:  {total['vision_match_rate']:>5.1f}%")
        print("="*80 + "\n")
