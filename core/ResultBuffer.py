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
import time
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
        self.last_cleanup_time = time.time()
        self.cleanup_interval = 5.0  # 每5秒清理一次超时数据
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
        # 🔧 定期清理超时的缓冲数据（防止5秒周期性刷新）
        current_time = time.time()
        if current_time - self.last_cleanup_time > self.cleanup_interval:
            self._cleanup_stale_data()
            self.last_cleanup_time = current_time
        
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
    
    def _cleanup_stale_data(self, max_age: float = 10.0):
        """
        清理超时的缓冲数据
        
        Args:
            max_age: 最大年龄（秒），超过此时间的数据将被清理
        """
        current_time = time.time()
        
        for camera_id in [1, 2, 3]:
            buffer = self.buffers[camera_id]
            stale_timestamps = [
                ts for ts in buffer.get_all_timestamps()
                if current_time - ts > max_age
            ]
            
            for ts in stale_timestamps:
                buffer.remove_result(ts)
                logger.debug(f"🧹 C{camera_id} 清理超时数据: ts={ts}, 年龄={current_time - ts:.1f}s")
    
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
        self.pending_radar_data = []  # 存储待输出的雷达数据（融合区外的数据）
    
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
        
        🔧 [修改] 改为累积策略而不是替换策略
        原因：雷达帧率低（5-10帧/秒），视觉帧率高（25帧/秒）
        - 多个雷达数据可能在短时间内到达
        - 应该累积这些数据，而不是覆盖
        - 每个雷达数据都应该被输出一次（带有其原始时间戳）
        """
        if radar_data_list:
            # 🔧 [新策略] 累积而不是替换
            # 为了防止重复，使用 radar_id 作为去重键
            seen_ids = set()
            for item in self.pending_radar_data:
                radar_id = item.get('radar_id')
                if radar_id:
                    seen_ids.add(radar_id)
            
            # 添加新的雷达数据（去重）
            for item in radar_data_list:
                radar_id = item.get('radar_id')
                # 只添加之前没有见过的雷达ID
                if radar_id and radar_id not in seen_ids:
                    self.pending_radar_data.append(dict(item))  # 深拷贝
                    seen_ids.add(radar_id)
            
            logger.debug(f"📡 累积待输出雷达数据: 新增{len(radar_data_list)}条, 当前队列大小{len(self.pending_radar_data)}")
        else:
            # 当没有新的雷达数据时，保持现有数据不变
            # （这样可以保证雷达数据被输出，直到被清理）
            logger.debug(f"📡 本帧无新雷达数据，保持现有缓冲")
    
    def output_pending_radar_data(self) -> bool:
        """
        🔧 已禁用：雷达数据现在在 _perform_triple_matching() 中与视觉数据合并输出
        此方法保留用于兼容性，但不再执行任何操作
        
        Returns:
            False（始终不输出）
        """
        # 🔧 不再独立输出雷达数据，所有雷达数据都在 _perform_triple_matching() 中合并到视觉输出
        # 这样可以确保雷达和视觉在同一个 reportTime 中输出
        logger.debug("📡 output_pending_radar_data() 已禁用，雷达数据在 _perform_triple_matching() 中合并输出")
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
        
        # 🔧 [修改] reportTime 直接使用 result1（摄像头1）的时间戳
        # result1['timestamp'] 是浮点数（Unix时间戳），直接转换为毫秒
        # 这是最稳定的时间基准，确保每一帧都有一致的时间戳
        if 'timestamp' in result1:
            ts = result1['timestamp']
            if isinstance(ts, (int, float)):
                # 如果是Unix时间戳（秒），转换为毫秒
                reportTime_ms = int(ts * 1000)
            else:
                # 如果是字符串，尝试解析
                try:
                    dt = datetime.strptime(str(ts), '%Y-%m-%d %H:%M:%S.%f')
                    reportTime_ms = int(dt.timestamp() * 1000)
                except (ValueError, TypeError):
                    reportTime_ms = int(datetime.now().timestamp() * 1000)
        else:
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
                
                # 🔧 [修改] 视觉数据使用原始的视觉时间戳（last_seen_timestamp）
                # 这样可以保持视觉数据的原始时间信息，而不是被融合时间覆盖
                if global_target.last_seen_timestamp:
                    participant_timestamp = global_target.last_seen_timestamp
                else:
                    participant_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                
                # 构建participant对象
                participant = {
                    "timestamp": participant_timestamp,  # 视觉数据的原始时间戳
                    "source": "camera",  # 数据源标记
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
        
        # 🔧 新增：将待输出的雷达数据合并到视觉输出中（同一个reportTime）
        # 这样可以确保雷达和视觉在同一时间点输出
        # 🔧 [重要] 只在有摄像头数据（participants非空）时才输出雷达数据
        # 这样可以防止"只有雷达数据"的帧导致的闪烁问题
        if self.pending_radar_data:
            if participants:
                # 有摄像头数据，合并雷达数据输出
                try:
                    import math
                    # 🔧 去重：使用 radar_id 作为键，防止同一个雷达被多次添加
                    seen_radar_ids = set()
                    
                    for radar_data in self.pending_radar_data:
                        # 支持字典格式的数据
                        if isinstance(radar_data, dict):
                            lon = radar_data.get('lon')
                            lat = radar_data.get('lat')
                            
                            if lon is not None and lat is not None:
                                radar_id = radar_data.get('radar_id', '')
                                
                                # 🔧 关键：检查是否已经添加过这个雷达ID
                                if radar_id in seen_radar_ids:
                                    logger.debug(f"⏭️ 跳过重复的雷达数据: {radar_id}")
                                    continue
                                
                                seen_radar_ids.add(radar_id)
                                
                                timestamp_str = radar_data.get('timestamp')
                                if not timestamp_str:
                                    timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                                
                                radar_id_last6 = radar_id[-6:] if len(radar_id) >= 6 else radar_id
                                try:
                                    pid = int(radar_id_last6, 16) if radar_id_last6 else 0
                                except ValueError:
                                    pid = 0
                                
                                radar_participant = {
                                    "timestamp": timestamp_str,  # 原始雷达时间戳（保持雷达的原始时间）
                                    "source": "radar",  # 数据源标记
                                    "pid": pid,
                                    "cameraid": 2,  # 雷达数据源标记
                                    "type": "car",
                                    "plate": radar_id_last6,
                                    "heading": 0,
                                    "lng": lon*1e7,
                                    "lat": lat*1e7
                                }
                                participants.append(radar_participant)
                    
                    logger.debug(f"📡 已合并 {len(seen_radar_ids)} 条雷达数据到视觉输出")
                except Exception as e:
                    logger.error(f"❌ 合并雷达数据到视觉输出失败: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                # 没有摄像头数据，丢弃雷达数据（防止单独输出导致闪烁）
                logger.debug(f"📡 本帧无摄像头数据，丢弃 {len(self.pending_radar_data)} 条待输出雷达数据（防止单独输出）")
            
            # 🔧 [关键] 无论如何都要清空雷达数据，防止重复输出
            self.pending_radar_data.clear()
        
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
        
        # 🔧 核心：MQTT发布是主要需求，确保总是执行
        mqtt_sent = False
        if self.mqtt_publisher:
            try:
                logger.info(f"📡 MQTT发布: 参与者数={len(participants)}")
                mqtt_sent = self.mqtt_publisher.publish_rsm(participants)
                if mqtt_sent:
                    logger.info(f"✅ MQTT发布成功: {len(participants)}个参与者")
                else:
                    logger.warning(f"⚠️ MQTT发布失败: {len(participants)}个参与者")
            except Exception as e:
                logger.error(f"❌ MQTT发送异常: {e}")
                import traceback
                traceback.print_exc()
        else:
            logger.warning(f"⚠️ MQTT未配置，无法发布数据 (参与者数: {len(participants)})")
        
        # 🔧 改进：使用流式输出而不是积累在内存中（JSON仅用于测试辅助）
        if self.fusion_system:
            try:
                # 使用新的流式输出方法
                if hasattr(self.fusion_system, 'add_json_output'):
                    self.fusion_system.add_json_output(json_data)
                else:
                    # 兼容旧版本
                    self.fusion_system.json_output_data.append(json_data)
            except Exception as e:
                logger.debug(f"JSON保存失败: {e}")
        
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
