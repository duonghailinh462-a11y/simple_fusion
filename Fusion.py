import os
import sys
import time
import multiprocessing
import copy
import json
import logging
from collections import defaultdict, deque
from statistics import mean, median
from datetime import datetime
sys.path.append('/usr/local/lynxi/sdk/sdk-samples/python')

import numpy as np
import cv2
from ctypes import *
import ctypes
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict

logger = logging.getLogger(__name__)

# 导入SDK相关模块
import pycommon.common as common
import pylynchipsdk as sdk
from pycommon.infer_process import *
from pycommon.callback_data_struct import *
from pycommon.dump_json import *
from ByteTrack.optimized_byte_tracker import OptimizedBYTETracker as BYTETracker

from Basic import Config, DetectionUtils, GeometryUtils, PerformanceMonitor, CAMERA_MATRICES
from TargetTrack import GlobalTarget, LocalTarget, LocalTrackBuffer, analyze_trajectory_for_global_assignment, FusionEntry

# 导入新的融合组件
from FusionComponents import (
    TargetManager,
    MatchingEngine,
    TrajectoryMerger,
    C2BufferEntry
)

class NumpyJSONEncoder(json.JSONEncoder):
    """自定义JSON编码器，处理NumPy类型"""
    def default(self, obj):
        if isinstance(obj, (np.integer, np.int_, np.intc, np.intp, np.int8,
                            np.int16, np.int32, np.int64, np.uint8, np.uint16,
                            np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.floating, np.float_, np.float16, np.float32, np.float64)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.bool_):
            return bool(obj)
        return super(NumpyJSONEncoder, self).default(obj)

class CrossCameraFusion:
    """
    跨摄像头融合协调器 (重构版 - 遵循单一职责原则)
    
    职责：
    - 协调目标管理器、匹配引擎和轨迹融合器
    - 管理全局目标字典和映射关系
    - 处理帧级别的融合逻辑
    
    核心逻辑：
    1. C1 <-> C2: 基于时间窗口的匹配
    2. C3 -> C2: FIFO 匹配（基于像素方向判断）
    """
    
    def __init__(self):
        # 使用全局Config实例
        pass
        
        # 使用新的组件
        self.target_manager = TargetManager()
        self.matching_engine = MatchingEngine()
        self.trajectory_merger = TrajectoryMerger()
        
        # 全局目标管理
        self.global_targets: Dict[int, GlobalTarget] = {}
        self.local_to_global: Dict[Tuple[int, int], int] = {}
        
        # 本地轨迹缓冲区
        self.local_track_buffer = LocalTrackBuffer(max_history=30)
        
        # 日志缓冲区
        self.log_buffer: List[dict] = []
        self.log_buffer_max_size = 100
        
        # 帧计数
        self.frame_count = 0
        self.json_output_data = []
        
        logger.info("CrossCameraFusion初始化完成 (重构版)")
    
    def assign_new_global_id(self, camera_id: int, local_id: int) -> int:
        """分配新的全局ID (委托给 TargetManager)"""
        return self.target_manager.assign_new_global_id(camera_id, local_id)

    def create_global_target(self, global_id: int, detection: dict, camera_id: int, timestamp: str = None, perf_monitor=None) -> GlobalTarget:
        """创建全局目标 (委托给 TargetManager)"""
        return self.target_manager.create_global_target(
            global_id, detection, camera_id, self.frame_count, timestamp, perf_monitor
        )
    
    def create_local_target(self, detection: dict, camera_id: int, perf_monitor=None) -> LocalTarget:
        """创建本地目标 (委托给 TargetManager)"""
        return self.target_manager.create_local_target(
            detection, camera_id, self.frame_count, perf_monitor
        )
    
    def classify_targets(self, detections: List[dict], camera_id: int, timestamp: str = None, perf_monitor=None) -> Tuple[List[GlobalTarget], List[LocalTarget]]:
        """
        [已重构 - 像素方向修正版]
        - C1/C3: 正常分配 GlobalID [cite: 235-250]。
        - C2: 只创建 LocalTarget 。
        - C2: 检查 LocalTarget *在融合区* [cite: 197-198] 且 *像素轨迹向上* (Y值变小) 时, Push 到 c2_buffer_from_c3 队列。
        """
        if perf_monitor:
            perf_monitor.start_timer('classify_targets')
        
        global_targets = []
        local_targets = []
        
        # 轨迹方向判断的参数
        MIN_TRAJ_LEN_FOR_DIRECTION = 10 # 需要10帧轨迹才能判断方向
        PIXEL_Y_DIRECTION_THRESHOLD = 50 # Y 轴像素变化超过50才算有效移动 (Y值变小是C3, Y值变大是C1)
        
        for detection in detections:
            if 'track_id' not in detection:
                continue
            
            track_id = detection['track_id']
            class_name = detection['class']
            
            H_matrix = CAMERA_MATRICES[camera_id]
            center_x = int((detection['box'][0] + detection['box'][2]) / 2)
            center_y = int(detection['box'][3])
            center_x = max(0, min(center_x, Config.IMAGE_WIDTH - 1))
            center_y = max(0, min(center_y, Config.IMAGE_HEIGHT - 1))
            
            bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, center_x, center_y)
            if not bev_result:
                bev_result = (0.0, 0.0)
            
            if perf_monitor:
                perf_monitor.add_counter('bev_conversions')
            
            self.local_track_buffer.update_track(camera_id, track_id, bev_result, (center_x, int(center_y)), class_name, current_frame=self.frame_count)
            
            # 检查是否已分配 global_id
            if self.local_track_buffer.has_global_id(camera_id, track_id): # [cite: 204-205]
                # ... (GlobalTarget 更新逻辑, 不变) [cite: 206-234] ...
                global_id = self.local_track_buffer.get_global_id(camera_id, track_id)
                global_target = self.global_targets.get(global_id)
                if global_target:
                    if bev_result[0] != 0.0 or bev_result[1] != 0.0:
                        global_target.bev_trajectory.append(bev_result)
                        global_target.pixel_trajectory.append((center_x, int(center_y)))
                        global_target.confidence_history.append(detection['confidence'])
                        global_target.last_seen_frame = self.frame_count
                        if timestamp:
                            global_target.last_seen_timestamp = timestamp
                    
                    current_bev = global_target.bev_trajectory[-1]
                    global_target.is_in_fusion_zone = GeometryUtils.is_in_public_area(current_bev)
                    if global_target.is_in_fusion_zone and global_target.fusion_entry_frame == -1:
                         global_target.fusion_entry_frame = self.frame_count
                    
                    max_trajectory_length = 50
                    if len(global_target.bev_trajectory) > max_trajectory_length:
                        global_target.bev_trajectory = global_target.bev_trajectory[-max_trajectory_length:]
                        global_target.pixel_trajectory = global_target.pixel_trajectory[-max_trajectory_length:]
                        global_target.confidence_history = global_target.confidence_history[-max_trajectory_length:]
                    
                    global_targets.append(global_target)
                continue
            
            # ⬇️ ⬇️ ⬇️ [重构] GlobalID 分配逻辑 ⬇️ ⬇️ ⬇️
            pixel_track_history = self.local_track_buffer.get_pixel_track_history(camera_id, track_id) # [cite: 201-203]

            should_assign_gid = (
                camera_id != 2 and 
                analyze_trajectory_for_global_assignment(pixel_track_history, camera_id, # [cite: 235-236]
                                                       min_trajectory_length=3)
            )

            if should_assign_gid:
                # C1 或 C3 满足条件, 分配 GlobalID
                global_id = self.assign_new_global_id(camera_id, track_id)
                global_target = self.create_global_target(global_id, detection, camera_id, timestamp, perf_monitor)
                self.local_track_buffer.assign_global_id(camera_id, track_id, global_id) 
                self.global_targets[global_id] = global_target
                global_targets.append(global_target)
            
            else:
                # C1/C3 不满足条件, 或 (重要!) 目标来自 C2, 始终创建 LocalTarget
                local_target = self.create_local_target(detection, camera_id, perf_monitor)
                local_targets.append(local_target)
                
                # ⬇️ ⬇️ ⬇️ [新] C3->C2 "Push" 逻辑 (基于像素方向) ⬇️ ⬇️ ⬇️
                if camera_id == 2:
                    # 如果这个 C2 目标已经被判断过方向, 则跳过
                    if track_id in self.matching_engine.c2_targets_processed_direction:
                        continue
                    
                    # 关键: 必须在融合区内才进行方向判断
                    if not local_target.is_in_fusion_area:
                        logger.debug(f"C2 LID:{track_id} 不在融合区, 跳过方向判断")
                        continue

                    # 检查轨迹长度是否足够判断方向
                    if len(pixel_track_history) >= MIN_TRAJ_LEN_FOR_DIRECTION:
                        start_y = pixel_track_history[0][1]
                        current_y = pixel_track_history[-1][1]
                        delta_y = current_y - start_y
                        
                        logger.debug(f"C2 LID:{track_id} 轨迹长度:{len(pixel_track_history)}, Delta_Y:{delta_y:.1f}px")

                        # Y值变小 (轨迹向上) -> 来自 C3
                        if delta_y < -PIXEL_Y_DIRECTION_THRESHOLD:
                            # 判定来自 C3
                            # 使用 MatchingEngine 添加到缓冲区
                            self.matching_engine.add_c2_to_buffer(local_target, self.frame_count)
                        
                        # Y值变大 (轨迹向下) -> 来自 C1
                        elif delta_y > PIXEL_Y_DIRECTION_THRESHOLD:
                            self.matching_engine.c2_targets_processed_direction.add(track_id)
                            logger.debug(f"C2 LID:{track_id} 判定来自 C1 (轨迹向下)")
                        
                        else:
                            logger.debug(f"C2 LID:{track_id} 轨迹Y轴变化不明显")
                    else:
                        logger.debug(f"C2 LID:{track_id} 轨迹太短 ({len(pixel_track_history)}帧)")
                # ⬆️ ⬆️ ⬆️ 结束 ⬆️ ⬆️ ⬆️
        
        if perf_monitor:
            perf_monitor.end_timer('classify_targets')
        return global_targets, local_targets
    
    # ... (移除了 C2->C3 的旧函数) ...

    def _smoothly_merge_trajectory(self, global_target: GlobalTarget, 
                                  local_target: LocalTarget):
        """平滑融合轨迹 (委托给 TrajectoryMerger)"""
        self.trajectory_merger.merge_trajectory(global_target, local_target)
        global_target.last_seen_frame = self.frame_count
        global_target.is_in_fusion_zone = local_target.is_in_fusion_area
    
    def _match_C3_gid_to_C2_lid_fifo(self, c3_global_target: GlobalTarget, 
                                       locked_global_ids: Set[int]) -> bool:
        """
        C3->C2 FIFO 匹配 (委托给 MatchingEngine)
        """
        c2_local_id = self.matching_engine.match_c3_to_c2_fifo(
            c3_global_target, self.frame_count
        )
        
        if c2_local_id is not None:
            # 匹配成功，绑定
            lookup_key = (2, c2_local_id)
            self.local_to_global[lookup_key] = c3_global_target.global_id
            locked_global_ids.add(c3_global_target.global_id)
            return True
        
        return False

    def _match_fallback_time_window(self, local_target: LocalTarget, active_global_targets: List[GlobalTarget],
                                   locked_global_ids: Set[int], permanently_bound_global_ids: Set[int]):
        """
        [C1 <-> C2 专用]
        使用旧的时间窗口匹配逻辑
        """
        # 确定候选池 - C1 匹配 C2, C2 匹配 C1
        candidate_globals = []
        if local_target.camera_id == 1:
            candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
        elif local_target.camera_id == 2:
            candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 1]
        
        fusion_candidates = [
            gt for gt in candidate_globals 
            if (gt.is_in_fusion_zone and 
                gt.global_id not in locked_global_ids and
                gt.global_id not in permanently_bound_global_ids)
        ]
        
        if not fusion_candidates:
            return
        
        # 使用 MatchingEngine 进行匹配
        best_match = self.matching_engine.match_time_window(
            local_target, fusion_candidates, time_window
        )
        
        if best_match:
            # 检查是否已被锁定
            if best_match.global_id not in locked_global_ids and \
               best_match.global_id not in permanently_bound_global_ids:
                local_target.matched_global_id = best_match.global_id
                lookup_key = (local_target.camera_id, local_target.local_id)
                self.local_to_global[lookup_key] = best_match.global_id
                locked_global_ids.add(best_match.global_id)
                self._smoothly_merge_trajectory(best_match, local_target)
    
    def _perform_matching(self, local_targets_this_frame: List[LocalTarget], 
                     active_global_targets: List[GlobalTarget], perf_monitor=None):
        """
        [已重构]
        核心匹配方法
        1. C3->C2 (新): 遍历 GlobalTarget, C3 GID 触发 FIFO 匹配。
        2. C1<->C2 (旧): 遍历 LocalTarget, C1/C2 LID 触发时间窗口匹配。
        """
        if perf_monitor:
            perf_monitor.start_timer('perform_matching')
        
        locked_global_ids_this_frame = set()
        permanently_bound_global_ids = set(self.local_to_global.values())

        # --- 1. C3 -> C2 (FIFO 逻辑) ---
        # 遍历所有活跃的 GlobalTarget, 寻找 C3 触发器
        for gt in active_global_targets:
            # 必须是 C3, 必须在融合区, 且尚未被绑定
            if (gt.camera_id == 3 and 
                gt.is_in_fusion_zone and 
                gt.global_id not in permanently_bound_global_ids and
                gt.global_id not in locked_global_ids_this_frame):
                
                # C3 GID 触发 FIFO 匹配
                self._match_C3_gid_to_C2_lid_fifo(gt, locked_global_ids_this_frame)

        # --- 2. C1 <-> C2 (时间窗口逻辑) ---
        # 遍历所有 LocalTarget, 寻找 C1 或 C2 触发器
        for lt in local_targets_this_frame:
            lookup_key = (lt.camera_id, lt.local_id)
            
            # 检查是否已绑定 (C3->C2 在上一步中可能已绑定, 或 C1<->C2 之前已绑定)
            if lookup_key in self.local_to_global:
                bound_global_id = self.local_to_global[lookup_key]
                bound_global_target = self.global_targets.get(bound_global_id)
                if bound_global_target:
                    # 自动轨迹更新
                    self._smoothly_merge_trajectory(bound_global_target, lt)
                    lt.matched_global_id = bound_global_id
                else:
                    del self.local_to_global[lookup_key]
                continue

            # 只处理 C1 和 C2 的 LocalTarget
            # C3 的 LocalTarget 不需要匹配 (它们是 GID 的起点)
            if lt.camera_id == 1 or lt.camera_id == 2:
                # 检查这个 C2 目标是否是 "来自C3" 且在缓冲区中等待
                if lt.camera_id == 2 and lt.local_id in self.matching_engine.c2_targets_processed_direction:
                    # 检查它是否在 C3 缓冲区
                    if self.matching_engine.is_c2_in_buffer(lt.local_id):
                        logger.debug(f"C2 LID:{lt.local_id} 在C3缓冲区中, 跳过C1<->C2匹配")
                        continue 

                # 目标来自 C1, 或者来自 C2 (且判断为 C1 方向)
                if not lt.is_in_fusion_area:
                    continue
                
                self._match_fallback_time_window(lt, active_global_targets, 
                                               locked_global_ids_this_frame, 
                                               permanently_bound_global_ids)
        
        if perf_monitor:
            perf_monitor.end_timer('perform_matching')
    
    def _perform_matching_legacy(self, *args, **kwargs):
        """(占位符) 旧的函数, 逻辑已合并到 _perform_matching"""
        pass
    
    def update_global_state(self, all_global_targets: List[GlobalTarget], all_local_targets: List[LocalTarget]):
        """更新全局状态（基于时间戳）"""
        # ... (不变) [cite: 879-894]
        for global_target in all_global_targets:
            if global_target.bev_trajectory:
                current_bev = global_target.bev_trajectory[-1]
                global_target.is_in_fusion_zone = GeometryUtils.is_in_public_area(current_bev)
            
            # 使用时间戳判断是否达到确认阈值
            if global_target.global_id not in self.target_manager.confirmed_targets:
                if global_target.first_seen_timestamp and global_target.last_seen_timestamp:
                    try:
                        # 安全解析时间戳（支持3位毫秒或6位微秒）
                        def safe_parse_timestamp(ts_str):
                            if not ts_str:
                                return None
                            try:
                                # 尝试6位微秒格式
                                return datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S.%f')
                            except ValueError:
                                # 尝试补充到6位微秒
                                if '.' in ts_str:
                                    parts = ts_str.split('.')
                                    if len(parts) == 2 and len(parts[1]) < 6:
                                        ts_padded = f"{parts[0]}.{parts[1].ljust(6, '0')}"
                                        return datetime.strptime(ts_padded, '%Y-%m-%d %H:%M:%S.%f')
                                raise ValueError(f"无法解析时间戳: {ts_str}")
                        
                        first_time = safe_parse_timestamp(global_target.first_seen_timestamp)
                        last_time = safe_parse_timestamp(global_target.last_seen_timestamp)
                        if first_time and last_time:
                            time_diff = (last_time - first_time).total_seconds()
                            # MIN_FRAMES_THRESHOLD 帧转换为时间（假设30fps）
                            min_time_threshold = Config.MIN_FRAMES_THRESHOLD / 30.0  # 秒
                        if time_diff >= min_time_threshold:
                            self.target_manager.confirmed_targets.add(global_target.global_id)
                    except (ValueError, AttributeError) as e:
                        logger.warning(f"时间戳解析失败 (GID:{global_target.global_id}): {e}")
    
    def process_detections(self, detections: List[dict], camera_id: int, timestamp: str = None, perf_monitor=None) -> Tuple[List[GlobalTarget], List[LocalTarget]]:
        """处理单个摄像头的检测结果"""
        # ... (不变) [cite: 896-918]
        if perf_monitor:
            perf_monitor.start_timer('process_detections')
        
        global_targets, local_targets = self.classify_targets(detections, camera_id, timestamp, perf_monitor)
        
        for global_target in global_targets:
            if global_target.global_id not in self.global_targets:
                self.global_targets[global_target.global_id] = global_target
        
        if perf_monitor:
            duration = perf_monitor.end_timer('process_detections')
            perf_monitor.record_fusion_stats('process_detections', duration, {
                'detection_count': len(detections),
                'global_target_count': len(global_targets),
                'local_target_count': len(local_targets)
            })
        
        return global_targets, local_targets
    
    def generate_json_data(self, all_global_targets: List[GlobalTarget], 
                              all_local_targets: List[LocalTarget],
                              radar_id_map: Dict[int, str] = None,
                              frame_timestamp: float = None) -> dict:
        """
        生成JSON数据
        
        Args:
            all_global_targets: 全局目标列表
            all_local_targets: 本地目标列表
            radar_id_map: track_id -> radar_id 的映射表 (可选)
            frame_timestamp: 视频帧的时间戳 (Unix timestamp，可选)
        """
        if radar_id_map is None:
            radar_id_map = {}
        
        # reportTime 始终使用当前时间（生成报告的时间）
        current_time_ms = int(time.time() * 1000)
        
        # timestamp 字段使用帧时间戳（如果提供），否则使用当前时间
        if frame_timestamp is not None:
            try:
                if isinstance(frame_timestamp, str):
                    # 字符串格式的时间戳（"YYYY-MM-DD HH:MM:SS.mmm"），直接解析
                    try:
                        # 先尝试标准格式（6位微秒）
                        dt = datetime.strptime(frame_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                    except ValueError:
                        # 如果失败，可能是3位毫秒格式，需要补充到6位
                        parts = frame_timestamp.split('.')
                        if len(parts) == 2:
                            second_part = parts[0]
                            ms_part = parts[1]
                            us_part = ms_part.ljust(6, '0')
                            ts_with_us = f"{second_part}.{us_part}"
                            dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                        else:
                            raise ValueError("时间戳格式错误")
                    current_timestamp = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                elif isinstance(frame_timestamp, (int, float)):
                    # Unix 时间戳（float/int），转换为 datetime
                    current_timestamp = datetime.fromtimestamp(frame_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                else:
                    raise ValueError(f"不支持的时间戳类型: {type(frame_timestamp)}")
            except (ValueError, TypeError, OSError) as e:
                # 如果转换失败，使用当前时间作为后备
                current_timestamp = datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        else:
            # 如果没有提供帧时间戳，使用当前时间
            current_timestamp = datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        participants = []
        
        # 处理全局目标
        for global_target in all_global_targets:
            # 使用时间戳判断是否应该输出（至少出现一定时间或已确认）
            should_output = global_target.global_id in self.target_manager.confirmed_targets
            if not should_output and global_target.first_seen_timestamp and global_target.last_seen_timestamp:
                try:
                    first_time = datetime.strptime(global_target.first_seen_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                    last_time = datetime.strptime(global_target.last_seen_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                    time_diff = (last_time - first_time).total_seconds()
                    # 至少出现 2 帧的时间（假设30fps）
                    min_time_for_output = 2 / 30.0  # 秒
                    should_output = time_diff >= min_time_for_output
                except (ValueError, AttributeError):
                    should_output = False
            
            if not should_output:
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
            
            # 获取雷达ID（如果匹配上了）
            radar_id = radar_id_map.get(global_target.global_id, None)
            
            # 新格式输出 - 只输出有视觉检测（经纬度）的结果
            # 经纬度坐标以视觉为准（以经纬度计算之后的结果为准）
            # 雷达的唯一作用就是匹配上之后把ID填进来，不输出雷达的经纬度
            participant = {
                "timestamp": current_timestamp,
                "cameraid": global_target.camera_id,
                "type": global_target.class_name,
                "confidence": global_target.confidence_history[-1] if global_target.confidence_history else 0.0,
                "track_id": global_target.global_id,
                "radar_id": radar_id,  # 匹配上了就输出radar_id，否则为null
                "lon": lng,
                "lat": lat
            }
            participants.append(participant)
        
        # 处理本地目标
        for local_target in all_local_targets:
            if not local_target.matched_global_id:
                continue
            
            if local_target.matched_global_id not in self.target_manager.confirmed_targets:
                continue
            
            if local_target.current_bev_pos[0] == 0.0 and local_target.current_bev_pos[1] == 0.0:
                continue
            
            geo_result = GeometryUtils.bev_to_geo(local_target.current_bev_pos[0], local_target.current_bev_pos[1])
            if not geo_result:
                continue
            
            lng, lat = geo_result
            
            # 检查是否已经添加过这个 global_id
            if not any(p['track_id'] == local_target.matched_global_id for p in participants):
                # 获取雷达ID（如果匹配上了）
                radar_id = radar_id_map.get(local_target.matched_global_id, None)
                
                # 新格式输出 - 只输出有视觉检测（经纬度）的结果
                # 经纬度坐标以视觉为准（以经纬度计算之后的结果为准）
                # 雷达的唯一作用就是匹配上之后把ID填进来，不输出雷达的经纬度
                participant = {
                    "timestamp": current_timestamp,
                    "cameraid": local_target.camera_id,
                    "type": local_target.class_name,
                    "confidence": local_target.confidence,
                    "track_id": local_target.matched_global_id,
                    "radar_id": radar_id,  # 匹配上了就输出radar_id，否则为null
                    "lon": lng,
                    "lat": lat
                }
                participants.append(participant)
        
        # 诊断：如果participants为空，记录日志
        if len(participants) == 0:
            import logging
            logger = logging.getLogger(__name__)
            logger.debug(f"空帧: reportTime={current_time_ms}, global_targets={len(all_global_targets)}, local_targets={len(all_local_targets)}, radar_id_map_size={len(radar_id_map)}")
        
        return {
            "reportTime": current_time_ms,
            "participant": participants
        }

    def is_confirmed_target(self, global_id: int) -> bool:
        """检查目标是否已确认"""
        # ... [cite: 987-989]
        return global_id in self.target_manager.confirmed_targets

    def cleanup_inactive_targets(self):
        """[已修改] 清理不活跃目标, 同时清理 C2 缓冲区"""
        if self.frame_count % 20 != 0:
            return
        
        inactive_threshold = 100
        current_time = self.frame_count
        
        inactive_global_ids = []
        for global_id, global_target in self.global_targets.items():
            if current_time - global_target.last_seen_frame > inactive_threshold:
                inactive_global_ids.append(global_id)
        
        inactive_local_ids_c2 = set()

        for global_id in inactive_global_ids:
            self.global_targets.pop(global_id, None)
            self.target_manager.colors.pop(global_id, None)
            self.target_manager.confirmed_targets.discard(global_id)
            
            keys_to_remove = [k for k, v in self.local_to_global.items() if v == global_id]
            for key in keys_to_remove:
                if key[0] == 2: # 如果是 C2 的 local_id
                    inactive_local_ids_c2.add(key[1])
                del self.local_to_global[key]

        # ⬇️ ⬇️ ⬇️ [新] 清理 C2 缓冲区 ⬇️ ⬇️ ⬇️
        # 1. 清理已超时的条目
        c2_buffer_timeout = Config.MAX_RETENTION_FRAMES
        active_c2_entries = [
            entry for entry in self.matching_engine.c2_buffer_from_c3
            if (current_time - entry.first_seen_frame) <= c2_buffer_timeout
        ]
        
        # 2. 清理 C2 本地跟踪器中已消失的条目 (很重要)
        self.local_track_buffer.cleanup_inactive_tracks(current_time)
        active_c2_local_ids = self.local_track_buffer.get_active_local_ids(camera_id=2)
        
        final_c2_buffer = []
        for entry in active_c2_entries:
            if entry.local_id in active_c2_local_ids:
                final_c2_buffer.append(entry)
            else:
                # C2 跟踪器跟丢了, 这个条目也失效了
                self.matching_engine.metrics['fifo_pop_stale'] += 1
                self.matching_engine.c2_targets_processed_direction.discard(entry.local_id)

        # 3. 清理已处理方向的集合 (C2 LID 已消失)
        active_processed_set = {
            lid for lid in self.matching_engine.c2_targets_processed_direction 
            if lid in active_c2_local_ids
        }
        self.matching_engine.c2_targets_processed_direction = active_processed_set

        if len(final_c2_buffer) < len(self.matching_engine.c2_buffer_from_c3):
            removed_count = len(self.matching_engine.c2_buffer_from_c3) - len(final_c2_buffer)
            logger.debug(f"清理C3->C2缓冲区: 移除{removed_count}个过期条目")
        self.matching_engine.c2_buffer_from_c3 = deque(final_c2_buffer)
        # ⬆️ ⬆️ ⬆️ 结束 ⬆️ ⬆️ ⬆️

    def _flush_logs(self):
        """刷新日志缓冲区到JSON输出数据"""
        if self.log_buffer:
            self.json_output_data.extend(self.log_buffer)
            self.log_buffer.clear()
    
    def store_single_camera_result(self, camera_id: int, timestamp: float, local_targets: List[LocalTarget], radar_ids: Dict[int, int]):
        """
        存储单路处理结果，用于后期三路匹配
        
        Args:
            camera_id: 摄像头ID (1, 2, 3)
            timestamp: 原始时间戳
            local_targets: 该摄像头的本地目标列表
            radar_ids: 该摄像头的radar_id映射 {track_id: radar_id}
        """
        if not hasattr(self, 'camera_results'):
            self.camera_results = {1: [], 2: [], 3: []}
        
        self.camera_results[camera_id].append({
            'timestamp': timestamp,
            'local_targets': local_targets,
            'radar_ids': radar_ids
        })
    
    def can_match_targets(self, target1: LocalTarget, target2: LocalTarget, spatial_threshold: float = 5.0) -> bool:
        """
        判断两个目标是否可以匹配
        
        匹配条件：
        1. 类别相同
        2. 地理位置相近
        3. 大小相近
        
        Args:
            target1: 第一个目标
            target2: 第二个目标
            spatial_threshold: 空间距离阈值（米）
        
        Returns:
            是否可以匹配
        """
        # 1. 类别必须相同
        if target1.class_name != target2.class_name:
            return False
        
        # 2. 地理位置相近
        if target1.current_bev_pos and target2.current_bev_pos:
            dist = np.sqrt(
                (target1.current_bev_pos[0] - target2.current_bev_pos[0])**2 +
                (target1.current_bev_pos[1] - target2.current_bev_pos[1])**2
            )
            if dist > spatial_threshold:
                return False
        
        # 3. 大小相近（允许0.7-1.3倍的变化）
        if target1.confidence > 0 and target2.confidence > 0:
            size_ratio = target1.confidence / target2.confidence
            if size_ratio < 0.7 or size_ratio > 1.3:
                return False
        
        return True
    
    def match_cross_camera_targets(self, time_window: float = 0.5) -> Tuple[List[Dict], List[LocalTarget]]:
        """
        三路视频匹配 - 识别同一目标在三个摄像头中的表现
        
        Args:
            time_window: 时间窗口大小（秒）
        
        Returns:
            (global_targets, unmatched_local_targets)
            - global_targets: 跨摄像头匹配的全局目标列表
            - unmatched_local_targets: 未匹配的单路目标列表
        """
        if not hasattr(self, 'camera_results'):
            return [], []
        
        global_targets = []
        matched_local_ids = set()  # 记录已匹配的本地目标
        
        # 获取所有时间戳，并确保转换为浮点数
        all_timestamps = set()
        for camera_id in [1, 2, 3]:
            for result in self.camera_results[camera_id]:
                ts = result['timestamp']
                # 确保时间戳是浮点数
                if isinstance(ts, str):
                    try:
                        ts = float(ts)
                    except (ValueError, TypeError):
                        continue
                all_timestamps.add(ts)
        
        all_timestamps = sorted(list(all_timestamps))
        
        # 遍历所有时间戳进行匹配
        for ts_ref in all_timestamps:
            # 在时间窗口内查找所有摄像头的目标
            candidates = {1: [], 2: [], 3: []}
            
            for camera_id in [1, 2, 3]:
                for result in self.camera_results[camera_id]:
                    ts = result['timestamp']
                    # 确保时间戳是浮点数
                    if isinstance(ts, str):
                        try:
                            ts = float(ts)
                        except (ValueError, TypeError):
                            continue
                    
                    if abs(ts - ts_ref) <= time_window:
                        candidates[camera_id].extend(result['local_targets'])
            
            # 进行三路匹配
            for target_c1 in candidates[1]:
                for target_c2 in candidates[2]:
                    if not self.can_match_targets(target_c1, target_c2):
                        continue
                    
                    for target_c3 in candidates[3]:
                        if not self.can_match_targets(target_c1, target_c3):
                            continue
                        if not self.can_match_targets(target_c2, target_c3):
                            continue
                        
                        # 三路都匹配上了
                        global_id = self.assign_new_global_id(1, target_c1.local_id)
                        
                        global_target = {
                            'global_id': global_id,
                            'camera_ids': [1, 2, 3],
                            'local_ids': {1: target_c1.local_id, 2: target_c2.local_id, 3: target_c3.local_id},
                            'class_name': target_c1.class_name,
                            'confidence': (target_c1.confidence + target_c2.confidence + target_c3.confidence) / 3,
                            'bev_positions': {
                                1: target_c1.current_bev_pos,
                                2: target_c2.current_bev_pos,
                                3: target_c3.current_bev_pos
                            },
                            'timestamp': ts_ref
                        }
                        
                        global_targets.append(global_target)
                        matched_local_ids.add((1, target_c1.local_id))
                        matched_local_ids.add((2, target_c2.local_id))
                        matched_local_ids.add((3, target_c3.local_id))
        
        # 收集未匹配的本地目标
        unmatched_local_targets = []
        for camera_id in [1, 2, 3]:
            for result in self.camera_results[camera_id]:
                for local_target in result['local_targets']:
                    if (camera_id, local_target.local_id) not in matched_local_ids:
                        unmatched_local_targets.append(local_target)
        
        return global_targets, unmatched_local_targets
    
    def next_frame(self):
        """进入下一帧"""
        # ... (不变) [cite: 1016-1022]
        self.frame_count += 1
        self.cleanup_inactive_targets()
        
        if self.frame_count % 100 == 0:
            self._flush_logs()

    def save_json_data(self, output_file: str):
        """保存JSON数据到文件"""
        # ... (不变) [cite: 1024-1033]
        try:
            self._flush_logs()
            
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(self.json_output_data, f, ensure_ascii=False, indent=2, cls=NumpyJSONEncoder)
            logger.info(f"JSON数据已保存: {output_file}, 共{len(self.json_output_data)}帧")
        except Exception as e:
            logger.error(f"保存JSON文件出错: {e}")