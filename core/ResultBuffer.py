# -*- coding: utf-8 -*-
"""
三路结果缓冲和时间对齐模块

功能：
1. 为每路摄像头维护结果缓冲区（按时间戳）
2. 定期查找三路中时间最接近的结果组合
3. 进行三路匹配和融合
4. 确保所有结果都被输出，不丢弃任何数据

核心思想：
- 单路处理 → 结果缓冲 → 时间对齐 → 三路匹配 → 输出
"""

from typing import Dict, List, Tuple, Optional
from collections import deque
import logging
from vision.TargetTrack import LocalTarget, GlobalTarget

logger = logging.getLogger(__name__)


class CameraResultBuffer:
    """单个摄像头的结果缓冲区"""
    
    def __init__(self, camera_id: int, max_buffer_size: int = 100):
        self.camera_id = camera_id
        self.max_buffer_size = max_buffer_size
        # 使用字典存储，key为时间戳，value为结果
        self.buffer: Dict[float, Dict] = {}
        # 保持时间戳的有序性
        self.timestamps = deque(maxlen=max_buffer_size)
    
    def add_result(self, timestamp: float, global_targets: List[GlobalTarget], 
                   radar_ids: Dict[int, Optional[int]]):
        """添加单路处理结果（存储已融合的GlobalTarget）"""
        if timestamp in self.buffer:
            logger.warning(f"C{self.camera_id} 时间戳 {timestamp} 已存在，将被覆盖")
        
        self.buffer[timestamp] = {
            'timestamp': timestamp,
            'global_targets': global_targets,
            'radar_ids': radar_ids
        }
        self.timestamps.append(timestamp)
    
    def get_result(self, timestamp: float) -> Optional[Dict]:
        """获取指定时间戳的结果"""
        return self.buffer.get(timestamp)
    
    def remove_result(self, timestamp: float):
        """移除指定时间戳的结果"""
        if timestamp in self.buffer:
            del self.buffer[timestamp]
    
    def get_all_timestamps(self) -> List[float]:
        """获取所有时间戳（按时间顺序）"""
        return sorted(self.buffer.keys())
    
    def get_buffer_size(self) -> int:
        """获取缓冲区大小"""
        return len(self.buffer)
    
    def clear(self):
        """清空缓冲区"""
        self.buffer.clear()
        self.timestamps.clear()


class TripleResultMatcher:
    """三路结果匹配器 - 找到时间最接近的三路结果组合"""
    
    def __init__(self, time_threshold: float = 0.5):
        """
        Args:
            time_threshold: 时间阈值（秒），超过此阈值的结果不进行匹配
        """
        self.time_threshold = time_threshold
        self.buffers = {
            1: CameraResultBuffer(1),
            2: CameraResultBuffer(2),
            3: CameraResultBuffer(3)
        }
    
    def add_result(self, camera_id: int, timestamp: float, 
                   global_targets: List[GlobalTarget], 
                   radar_ids: Dict[int, Optional[int]]):
        """添加单路处理结果（存储已融合的GlobalTarget）"""
        self.buffers[camera_id].add_result(timestamp, global_targets, radar_ids)
    
    def find_closest_triple(self) -> Optional[Tuple[float, float, float, Dict, Dict, Dict]]:
        """
        找到三路中时间最接近的结果组合
        
        Returns:
            (ts1, ts2, ts3, result1, result2, result3) 或 None
            其中 ts_i 是摄像头i的时间戳，result_i 是对应的结果
        """
        timestamps_c1 = self.buffers[1].get_all_timestamps()
        timestamps_c2 = self.buffers[2].get_all_timestamps()
        timestamps_c3 = self.buffers[3].get_all_timestamps()
        
        if not timestamps_c1 or not timestamps_c2 or not timestamps_c3:
            return None
        
        # 取最早的时间戳作为参考点
        min_ts = min(timestamps_c1[0], timestamps_c2[0], timestamps_c3[0])
        
        # 在参考时间戳附近查找三路最接近的时间戳
        best_match = None
        best_time_diff = float('inf')
        
        for ts1 in timestamps_c1:
            # 跳过太早的时间戳
            if ts1 < min_ts:
                continue
            
            # 在时间窗口内查找C2和C3的最接近时间戳
            ts2_candidates = [ts for ts in timestamps_c2 
                            if abs(ts - ts1) <= self.time_threshold]
            ts3_candidates = [ts for ts in timestamps_c3 
                            if abs(ts - ts1) <= self.time_threshold]
            
            if not ts2_candidates or not ts3_candidates:
                continue
            
            # 选择最接近的C2和C3时间戳
            ts2 = min(ts2_candidates, key=lambda t: abs(t - ts1))
            ts3 = min(ts3_candidates, key=lambda t: abs(t - ts1))
            
            # 计算总时间差
            total_time_diff = abs(ts1 - ts2) + abs(ts1 - ts3) + abs(ts2 - ts3)
            
            if total_time_diff < best_time_diff:
                best_time_diff = total_time_diff
                best_match = (ts1, ts2, ts3)
        
        if best_match is None:
            return None
        
        ts1, ts2, ts3 = best_match
        result1 = self.buffers[1].get_result(ts1)
        result2 = self.buffers[2].get_result(ts2)
        result3 = self.buffers[3].get_result(ts3)
        
        return (ts1, ts2, ts3, result1, result2, result3)
    
    def remove_matched_results(self, ts1: float, ts2: float, ts3: float):
        """移除已匹配的结果"""
        self.buffers[1].remove_result(ts1)
        self.buffers[2].remove_result(ts2)
        self.buffers[3].remove_result(ts3)
    
    def get_buffer_status(self) -> Dict:
        """获取缓冲区状态"""
        return {
            'c1_size': self.buffers[1].get_buffer_size(),
            'c2_size': self.buffers[2].get_buffer_size(),
            'c3_size': self.buffers[3].get_buffer_size(),
            'c1_timestamps': self.buffers[1].get_all_timestamps()[:5],  # 只显示前5个
            'c2_timestamps': self.buffers[2].get_all_timestamps()[:5],
            'c3_timestamps': self.buffers[3].get_all_timestamps()[:5]
        }
    
    def clear(self):
        """清空所有缓冲区"""
        for buffer in self.buffers.values():
            buffer.clear()


class ResultOutputManager:
    """结果输出管理器 - 管理三路匹配和输出流程"""
    
    def __init__(self, fusion_system, mqtt_publisher=None, time_threshold: float = 0.5):
        """
        Args:
            fusion_system: CrossCameraFusion 实例
            mqtt_publisher: MQTT发布器实例
            time_threshold: 时间阈值（秒）
        """
        self.fusion_system = fusion_system
        self.mqtt_publisher = mqtt_publisher
        self.matcher = TripleResultMatcher(time_threshold)
        self.output_count = 0
        self.pending_radar_data = []  # 存储待输出的雷达数据
    
    def add_single_camera_result(self, camera_id: int, timestamp: float,
                                global_targets: List[GlobalTarget],
                                radar_ids: Dict[int, Optional[int]]):
        """添加单路处理结果到缓冲区（存储已融合的GlobalTarget）"""
        self.matcher.add_result(camera_id, timestamp, global_targets, radar_ids)
    
    def add_radar_data(self, radar_data_list):
        """
        添加雷达数据到待输出列表
        Args:
            radar_data_list: 直接输出的雷达数据列表（在融合区外的数据）
        """
        if radar_data_list:
            logger.debug(f"📡 添加 {len(radar_data_list)} 条雷达数据到待输出列表 (当前待输出总数: {len(self.pending_radar_data) + len(radar_data_list)})")
            self.pending_radar_data.extend(radar_data_list)
        else:
            logger.debug(f"📡 无雷达数据添加 (radar_data_list为空或为None)")
    
    def output_pending_radar_data(self) -> bool:
        """
        独立输出所有待处理的雷达数据（融合区外的数据）
        不依赖三路匹配，直接输出
        
        Returns:
            True 如果有雷达数据输出，False 如果没有待输出的雷达数据
        """
        if not self.pending_radar_data:
            logger.debug("📡 待输出雷达数据为空，跳过")
            return False
        
        logger.debug(f"📡 开始处理 {len(self.pending_radar_data)} 条待输出雷达数据")
        
        try:
            from datetime import datetime
            import math
            
            # 构建输出JSON
            output_data = {
                "reportTime": int(datetime.now().timestamp() * 1000),
                "participant": []
            }
            
            # 处理所有待输出的雷达数据
            for radar_data in self.pending_radar_data:
                try:
                    # 支持字典格式的数据（由RadarDataFilter返回）
                    if isinstance(radar_data, dict):
                        # 从字典中获取经纬度
                        lon = radar_data.get('lon')
                        lat = radar_data.get('lat')
                        
                        if lon is not None and lat is not None:
                            # 🔧 直接使用雷达数据中的原始时间戳字符串
                            timestamp_str = radar_data.get('timestamp')
                            
                            if not timestamp_str:
                                # 如果没有时间戳，使用当前时间（备选方案）
                                logger.warning(f"⚠️ 雷达字典数据缺少时间戳，使用当前时间")
                                logger.warning(f"   雷达数据键: {list(radar_data.keys())}")
                                timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                            
                            radar_id = radar_data.get('radar_id', '')
                            radar_id_last6 = radar_id[-6:] if len(radar_id) >= 6 else radar_id
                            try:
                                pid = int(radar_id_last6, 16) if radar_id_last6 else 0
                            except ValueError:
                                pid = 0
                            
                            radar_participant = {
                                "pid": pid,
                                "cameraid": 1,  
                                "type": "car",
                                "plate": radar_id_last6,
                                "heading": 0,
                                "lng": lon*1e7,
                                "lat": lat*1e7
                            }
                            output_data['participant'].append(radar_participant)
                    else:
                        # 支持对象格式的数据（兼容旧版本）
                        # 从雷达极坐标(距离、角度)转换为BEV坐标
                        if hasattr(radar_data, 'distance') and hasattr(radar_data, 'angle'):
                            x = radar_data.distance * math.cos(math.radians(radar_data.angle))
                            y = radar_data.distance * math.sin(math.radians(radar_data.angle))
                            
                            # 转换为地理坐标
                            from core.Basic import GeometryUtils
                            geo_result = GeometryUtils.bev_to_geo(x, y)
                            if geo_result:
                                lng, lat = geo_result
                                
                                # 🔧 使用雷达对象的原始时间戳字符串
                                timestamp_str = getattr(radar_data, 'timestamp_str', None)
                                
                                if not timestamp_str:
                                    # 如果没有时间戳，使用当前时间（备选方案）
                                    logger.warning(f"⚠️ 雷达对象缺少时间戳，使用当前时间")
                                    logger.warning(f"   雷达对象属性: {vars(radar_data)}")
                                    timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                                
                                radar_id_str = str(getattr(radar_data, 'id', ''))
                                radar_id_last6 = radar_id_str[-6:] if len(radar_id_str) >= 6 else radar_id_str
                                try:
                                    pid = int(radar_id_last6, 16) if radar_id_last6 else 0
                                except ValueError:
                                    pid = 0
                                
                                radar_participant = {
                                    "pid": pid,
                                    "cameraid": 1,  # 雷达数据源标记
                                    "plate": radar_id_last6,
                                    "type": "car",
                                    "heading": 0,
                                    "lng": lng*1e7,
                                    "lat": lat*1e7
                                }
                                output_data['participant'].append(radar_participant)
                except Exception as e:
                    logger.debug(f"处理单个雷达数据失败: {e}")
                    continue
            
            # 如果有有效的雷达数据，输出结果
            if output_data['participant']:
                self._output_result(output_data)
                self.output_count += 1
            
            # 清空已处理的雷达数据
            self.pending_radar_data.clear()
            
            return len(output_data['participant']) > 0
            
        except Exception as e:
            logger.error(f"输出雷达数据异常: {e}")
            import traceback
            traceback.print_exc()
            # 清空待输出数据，避免重复处理
            self.pending_radar_data.clear()
            return False
    
    def process_and_output(self) -> bool:
        """
        处理缓冲区中的结果，进行三路匹配和输出
        
        Returns:
            True 如果有结果输出，False 如果缓冲区为空
        """
        match_result = self.matcher.find_closest_triple()
        
        if match_result is None:
            return False
        
        ts1, ts2, ts3, result1, result2, result3 = match_result
        
        # 进行三路匹配和融合
        try:
            # 调用融合系统的三路匹配方法
            # （这里需要根据实际的融合系统API进行调整）
            json_data = self._perform_triple_matching(result1, result2, result3)
            
            # 输出结果
            self._output_result(json_data, ts1, ts2, ts3)
            
            # 移除已处理的结果
            self.matcher.remove_matched_results(ts1, ts2, ts3)
            
            self.output_count += 1
            return True
            
        except Exception as e:
            logger.error(f"三路匹配处理异常: {e}")
            # 即使出错也要移除结果，避免缓冲区堆积
            self.matcher.remove_matched_results(ts1, ts2, ts3)
            return False
    
    def _perform_triple_matching(self, result1: Dict, result2: Dict, 
                                result3: Dict) -> Dict:
        """
        执行三路结果合并（GlobalTarget已经在main.py中融合过了）
        
        将三路时间对齐后的GlobalTarget转换成可输出的JSON格式
        """
        from datetime import datetime
        from core.Basic import GeometryUtils
        
        global_targets_c1 = result1['global_targets']
        global_targets_c2 = result2['global_targets']
        global_targets_c3 = result3['global_targets']
        
        # 获取雷达ID映射
        radar_ids_c1 = result1['radar_ids']
        radar_ids_c2 = result2['radar_ids']
        radar_ids_c3 = result3['radar_ids']
        
        # 合并所有global_targets（按global_id去重）
        unique_global_targets = {}
        for gt in global_targets_c1 + global_targets_c2 + global_targets_c3:
            if gt.global_id not in unique_global_targets:
                unique_global_targets[gt.global_id] = gt
        
        # 合并雷达ID映射
        combined_radar_ids = {}
        combined_radar_ids.update(radar_ids_c1)
        combined_radar_ids.update(radar_ids_c2)
        combined_radar_ids.update(radar_ids_c3)
        
        # 🔧 修改：reportTime 应该是当前时间，而不是数据时间戳
        from datetime import datetime
        reportTime_ms = int(datetime.now().timestamp() * 1000)
        
        # 从 global_targets 生成 participant 对象
        participants = []
        try:
            for global_target in unique_global_targets.values():
                # 🔧 过滤：只输出起点在融合区域内的目标（should_output=True）
                if not getattr(global_target, 'should_output', True):
                    continue
                
                # 跳过没有轨迹或位置无效的目标
                if not global_target.bev_trajectory:
                    continue
                
                current_bev = global_target.bev_trajectory[-1]
                if current_bev[0] == 0.0 and current_bev[1] == 0.0:
                    continue
                
                # 获取该target的雷达ID（如果存在）
                radar_id = combined_radar_ids.get(global_target.global_id)
                
                # 将BEV坐标转换为地理坐标
                geo_result = GeometryUtils.bev_to_geo(current_bev[0], current_bev[1])
                
                if not geo_result:
                    # BEV转换失败，跳过此目标
                    continue
                
                lng, lat = geo_result
                
                # 获取置信度（使用最新的）
                confidence = global_target.confidence_history[-1] if global_target.confidence_history else 0.0
                
                # 合并track_id和radar_id：如果匹配上了就是trackid_radarid后六位，否则只用trackid
                #track_id = f"{global_target.global_id}_{radar_id[-6:]}" if radar_id else global_target.global_id
                
                # 构建participant对象
                participant = {
                    "pid": global_target.global_id,
                    "cameraid": 1,  # 视觉数据源标记
                    "type":"car",
                    "plate": radar_id[-6:] if radar_id else global_target.global_id,
                    "heading": 0,
                    "lng": lng*1e7,
                    "lat": lat*1e7
                }
                participants.append(participant)
        except Exception as e:
            logger.error(f"三路结果合并失败: {e}")
            import traceback
            traceback.print_exc()
            participants = []
        
        # 注：直接输出的雷达数据不再添加到三路融合结果中
        # 雷达数据由 output_pending_radar_data() 独立输出
        
        json_data = {
            'reportTime': reportTime_ms,
            'participant': participants
        }
        
        return json_data
    
    def _output_result(self, json_data: Dict, ts1: float = None, ts2: float = None, ts3: float = None):
        """
        输出结果到MQTT、融合系统和文件
        
        Args:
            json_data: 输出数据
            ts1, ts2, ts3: 三路摄像头时间戳（可选，用于三路匹配结果日志）
        """
        # 支持两种格式：'participant' 或 'participants'
        participants = json_data.get('participant', json_data.get('participants', []))
        
        # 尝试发送MQTT
        mqtt_sent = False
        if self.mqtt_publisher:
            try:
                mqtt_sent = self.mqtt_publisher.publish_rsm(participants)
            except Exception as e:
                logger.error(f"MQTT发送异常: {e}")
        
        # 保存到融合系统的输出列表（用于最终的JSON文件保存）
        if self.fusion_system:
            try:
                self.fusion_system.json_output_data.append(json_data)
            except Exception as e:
                logger.error(f"保存到融合系统输出列表失败: {e}")
        
        # 记录输出信息
        if ts1 is not None and ts2 is not None and ts3 is not None:
            # 三路匹配结果的日志
            logger.info(f"输出结果 #{self.output_count}: "
                       f"C1({ts1:.3f}) C2({ts2:.3f}) C3({ts3:.3f}) | "
                       f"参与者数: {len(participants)} | "
                       f"MQTT: {'成功' if mqtt_sent else '失败/未配置'}")
        else:
            # 雷达直接输出的日志
            event = json_data.get('event', 'direct_radar_output')
            logger.info(f"输出结果 #{self.output_count}: "
                       f"事件类型: {event} | "
                       f"参与者数: {len(participants)} | "
                       f"MQTT: {'成功' if mqtt_sent else '失败/未配置'}")
    
    def get_buffer_status(self) -> Dict:
        """获取缓冲区状态"""
        return self.matcher.get_buffer_status()
    
    def flush_all(self):
        """
        刷新所有缓冲区中的结果
        
        当程序结束时调用，确保所有结果都被输出
        """
        # 首先输出所有三路匹配的结果
        triple_match_count = 0
        while self.process_and_output():
            triple_match_count += 1
        
        # 然后输出所有剩余的雷达直接数据
        radar_count = 0
        while self.output_pending_radar_data():
            radar_count += 1
        
        logger.info(f"缓冲区刷新完成，共输出 {self.output_count} 组结果 "
                   f"(三路匹配: {triple_match_count}, 雷达直接: {radar_count})")
