#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
融合组件模块 - 拆分自 CrossCameraFusion
遵循单一职责原则，将融合逻辑分解为独立的组件
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Set, Optional
from collections import defaultdict, deque
from dataclasses import dataclass

from Basic import Config, GeometryUtils, CAMERA_MATRICES
from TargetTrack import GlobalTarget, LocalTarget, LocalTrackBuffer

logger = logging.getLogger(__name__)


# ==================== 目标管理器 ====================

class TargetManager:
    """
    目标管理器 - 负责目标的创建、ID分配和生命周期管理
    
    职责：
    - 分配全局ID
    - 创建 GlobalTarget 和 LocalTarget
    - 管理颜色分配
    - 管理确认目标
    """
    
    def __init__(self):
        # 使用全局Config实例
        pass
        
        # ID管理
        self.global_id_counter = 1
        self.colors: Dict[int, Tuple[int, int, int]] = {}
        
        # 确认目标管理
        self.confirmed_targets: Set[int] = set()
        
        logger.info("TargetManager 初始化完成")
    
    def assign_new_global_id(self, camera_id: int, local_id: int) -> int:
        """分配新的全局ID"""
        global_id = self.global_id_counter
        self.global_id_counter += 1
        self._assign_color(global_id)
        return global_id
    
    def _assign_color(self, global_id: int) -> Tuple[int, int, int]:
        """为全局ID分配颜色"""
        if global_id not in self.colors:
            np.random.seed(global_id)
            color = tuple(int(np.random.randint(0, 255)) for _ in range(3))
            self.colors[global_id] = color
        return self.colors[global_id]
    
    def get_color(self, global_id: int) -> Tuple[int, int, int]:
        """获取全局ID的颜色"""
        return self.colors.get(global_id, (255, 255, 255))
    
    def create_global_target(self, global_id: int, detection: dict, 
                           camera_id: int, frame_count: int, 
                           timestamp: str = None, perf_monitor=None) -> GlobalTarget:
        """创建全局目标"""
        center_x = int((detection['box'][0] + detection['box'][2]) / 2)
        center_y = int(detection['box'][3])
        
        center_x = max(0, min(center_x, Config.IMAGE_WIDTH - 1))
        center_y = max(0, min(center_y, Config.IMAGE_HEIGHT - 1))
        
        H_matrix = CAMERA_MATRICES[camera_id]
        bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, center_x, center_y)
        if not bev_result:
            bev_result = (0.0, 0.0)
        
        if perf_monitor:
            perf_monitor.add_counter('bev_conversions')
        
        is_in_fusion_zone = GeometryUtils.is_in_public_area(bev_result)
        fusion_entry_frame = frame_count if is_in_fusion_zone else -1
        
        return GlobalTarget(
            global_id=global_id,
            camera_id=camera_id,
            local_id=detection['track_id'],
            class_name=detection['class'],
            bev_trajectory=[bev_result],
            pixel_trajectory=[(center_x, center_y)],
            last_seen_frame=frame_count,
            is_active=True,
            fusion_alpha=0.2,
            is_in_fusion_zone=is_in_fusion_zone,
            confidence_history=[detection['confidence']],
            fusion_entry_frame=fusion_entry_frame,
            first_seen_timestamp=timestamp,
            last_seen_timestamp=timestamp
        )
    
    def create_local_target(self, detection: dict, camera_id: int, 
                          frame_count: int, perf_monitor=None) -> LocalTarget:
        """创建本地目标"""
        center_x = int((detection['box'][0] + detection['box'][2]) / 2)
        center_y = int(detection['box'][3])
        
        center_x = max(0, min(center_x, Config.IMAGE_WIDTH - 1))
        center_y = max(0, min(center_y, Config.IMAGE_HEIGHT - 1))
            
        H_matrix = CAMERA_MATRICES[camera_id]
        bev_result = GeometryUtils.project_pixel_to_bev(H_matrix, center_x, center_y)
        if not bev_result:
            bev_result = (0.0, 0.0)
        
        if perf_monitor:
            perf_monitor.add_counter('bev_conversions')
        
        # 使用detection中的in_fusion_area标记（如果存在），否则从像素坐标判断
        if 'in_fusion_area' in detection:
            is_in_fusion_area = detection['in_fusion_area']
        else:
            # 备选方案：从像素坐标判断
            is_in_fusion_area = GeometryUtils.is_in_radar_vision_fusion_area((center_x, center_y), camera_id)
        
        fusion_entry_frame = frame_count if is_in_fusion_area else -1
        
        return LocalTarget(
            local_id=detection['track_id'],
            camera_id=camera_id,
            class_name=detection['class'],
            current_bev_pos=bev_result,
            current_pixel_pos=(center_x, center_y),
            confidence=detection['confidence'],
            is_in_fusion_area=is_in_fusion_area,
            detection_box=detection['box'],
            fusion_entry_frame=fusion_entry_frame
        )
    
    def update_target_confirmation(self, global_id: int, global_target: GlobalTarget) -> bool:
        """更新目标确认状态，返回是否为确认目标（基于时间戳）"""
        if global_id not in self.confirmed_targets:
            # 使用时间戳判断是否达到阈值
            if global_target.first_seen_timestamp and global_target.last_seen_timestamp:
                from datetime import datetime
                try:
                    first_time = datetime.strptime(global_target.first_seen_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                    last_time = datetime.strptime(global_target.last_seen_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                    time_diff = (last_time - first_time).total_seconds()
                    # MIN_FRAMES_THRESHOLD 帧转换为时间（假设30fps）
                    min_time_threshold = Config.MIN_FRAMES_THRESHOLD / 30.0  # 秒
                    if time_diff >= min_time_threshold:
                        self.confirmed_targets.add(global_id)
                        return True
                except (ValueError, AttributeError) as e:
                    logger.warning(f"时间戳解析失败: {e}, 使用帧计数作为备选方案")
                    # 备选方案：如果时间戳解析失败，使用帧计数
                    # 这里需要从 global_target 获取帧数信息，暂时跳过确认
                    pass
        return global_id in self.confirmed_targets


# ==================== 匹配引擎 ====================

@dataclass
class C2BufferEntry:
    """C2缓冲区条目"""
    local_id: int
    class_name: str
    first_seen_frame: int
    bev_pos: Tuple[float, float]


class MatchingEngine:
    """
    匹配引擎 - 负责不同摄像头间的目标匹配
    
    职责：
    - C3 -> C2 FIFO 匹配
    - C1 <-> C2 时间窗口匹配
    - 管理 C2 缓冲区
    """
    
    def __init__(self):
        # 使用全局Config实例
        pass
        
        # C3 -> C2 融合数据结构
        self.c2_buffer_from_c3: deque[C2BufferEntry] = deque()
        self.c2_targets_processed_direction: Set[int] = set()
        
        # 统计
        self.metrics = {
            'fallback_matches_c1_c2': 0,
            'fifo_match_c3_c2': 0,
            'fifo_pop_stale': 0,
            'fifo_pop_mismatch': 0
        }
        
        logger.info("MatchingEngine 初始化完成")
    
    def add_c2_to_buffer(self, local_target: LocalTarget, frame_count: int):
        """将 C2 目标添加到缓冲区（用于 C3 匹配）"""
        if local_target.local_id in self.c2_targets_processed_direction:
            return
        
        c2_entry = C2BufferEntry(
            local_id=local_target.local_id,
            class_name=local_target.class_name,
            first_seen_frame=frame_count,
            bev_pos=local_target.current_bev_pos
        )
        self.c2_buffer_from_c3.append(c2_entry)
        self.c2_targets_processed_direction.add(local_target.local_id)
        
        logger.info(f"C2 PUSH: LID:{local_target.local_id} -> C3缓冲区, 队列长度:{len(self.c2_buffer_from_c3)}")
    
    def match_c3_to_c2_fifo(self, c3_global_target: GlobalTarget, 
                           frame_count: int) -> Optional[int]:
        """
        C3 -> C2 FIFO 匹配
        返回匹配的 C2 local_id，如果没有匹配返回 None
        """
        if not self.c2_buffer_from_c3:
            logger.debug(f"C3 GID:{c3_global_target.global_id} 匹配失败: C2缓冲区为空")
            return None
        
        # 检查队头
        c2_entry_at_head = self.c2_buffer_from_c3[0]
        
        # 检查是否过期
        if (frame_count - c2_entry_at_head.first_seen_frame) > Config.MAX_RETENTION_FRAMES:
            logger.debug(f"C2 LID:{c2_entry_at_head.local_id} 已过期, POP丢弃")
            self.metrics['fifo_pop_stale'] += 1
            self.c2_buffer_from_c3.popleft()
            self.c2_targets_processed_direction.discard(c2_entry_at_head.local_id)
            return None
        
        # 检查类别兼容性
        from Basic import DetectionUtils
        if not DetectionUtils.is_class_compatible(c3_global_target.class_name, c2_entry_at_head.class_name):
            logger.debug(f"C3 GID:{c3_global_target.global_id} 与 C2 LID:{c2_entry_at_head.local_id} 类别不匹配")
            self.metrics['fifo_pop_mismatch'] += 1
            self.c2_buffer_from_c3.popleft()
            self.c2_targets_processed_direction.discard(c2_entry_at_head.local_id)
            return None
        
        # 匹配成功
        c2_entry_to_match = self.c2_buffer_from_c3.popleft()
        self.metrics['fifo_match_c3_c2'] += 1
        
        logger.info(f"C3->C2 匹配成功: C3 GID:{c3_global_target.global_id} <-> C2 LID:{c2_entry_to_match.local_id}")
        
        return c2_entry_to_match.local_id
    
    def match_time_window(self, local_target: LocalTarget, 
                         active_global_targets: List[GlobalTarget],
                         time_window: int) -> Optional[GlobalTarget]:
        """
        基于时间窗口的匹配（用于 C1 <-> C2）
        返回最佳匹配的 GlobalTarget，如果没有匹配返回 None
        """
        best_match = None
        best_time_diff = float('inf')
        
        for candidate in active_global_targets:
            # 跳过相同摄像头
            if candidate.camera_id == local_target.camera_id:
                continue
            
            # 检查融合入口帧
            if candidate.fusion_entry_frame == -1 or local_target.fusion_entry_frame == -1:
                continue
            
            time_diff = abs(local_target.fusion_entry_frame - candidate.fusion_entry_frame)
            if time_diff <= time_window and time_diff < best_time_diff:
                best_time_diff = time_diff
                best_match = candidate
        
        if best_match:
            logger.debug(f"时间窗口匹配: LID:{local_target.local_id}(C{local_target.camera_id}) <-> GID:{best_match.global_id}")
            self.metrics['fallback_matches_c1_c2'] += 1
        
        return best_match
    
    def is_c2_in_buffer(self, local_id: int) -> bool:
        """检查 C2 目标是否在缓冲区中"""
        for entry in self.c2_buffer_from_c3:
            if entry.local_id == local_id:
                return True
        return False
    
    def get_metrics(self) -> dict:
        """获取匹配统计"""
        return self.metrics.copy()


# ==================== 轨迹融合器 ====================

class TrajectoryMerger:
    """
    轨迹融合器 - 负责平滑地合并轨迹
    
    职责：
    - 平滑融合 BEV 轨迹
    - 平滑融合像素轨迹
    - 更新置信度历史
    """
    
    @staticmethod
    def merge_trajectory(global_target: GlobalTarget, local_target: LocalTarget):
        """平滑地合并轨迹 - 使用动态加权融合"""
        alpha = global_target.fusion_alpha
        
        # 融合 BEV 轨迹
        if global_target.bev_trajectory:
            last_bev = global_target.bev_trajectory[-1]
            new_bev_x = alpha * local_target.current_bev_pos[0] + (1 - alpha) * last_bev[0]
            new_bev_y = alpha * local_target.current_bev_pos[1] + (1 - alpha) * last_bev[1]
            global_target.bev_trajectory.append((new_bev_x, new_bev_y))
        else:
            global_target.bev_trajectory.append(local_target.current_bev_pos)
        
        # 融合像素轨迹
        if global_target.pixel_trajectory:
            last_pixel = global_target.pixel_trajectory[-1]
            new_pixel_x = int(alpha * local_target.current_pixel_pos[0] + (1 - alpha) * last_pixel[0])
            new_pixel_y = int(alpha * local_target.current_pixel_pos[1] + (1 - alpha) * last_pixel[1])
            global_target.pixel_trajectory.append((new_pixel_x, new_pixel_y))
        else:
            global_target.pixel_trajectory.append(local_target.current_pixel_pos)
        
        # 更新置信度历史
        global_target.confidence_history.append(local_target.confidence)
        
        # 【新增】动态增加融合权重，实现平滑过渡
        # 从 alpha=0 逐步增长到 1.0，使得融合逐步从原始轨迹过渡到新轨迹
        global_target.fusion_alpha += 0.01
        if global_target.fusion_alpha > 1.0:
            global_target.fusion_alpha = 1.0
        
        # 限制轨迹长度
        max_trajectory_length = 100
        if len(global_target.bev_trajectory) > max_trajectory_length:
            global_target.bev_trajectory = global_target.bev_trajectory[-max_trajectory_length:]
        if len(global_target.pixel_trajectory) > max_trajectory_length:
            global_target.pixel_trajectory = global_target.pixel_trajectory[-max_trajectory_length:]
        if len(global_target.confidence_history) > max_trajectory_length:
            global_target.confidence_history = global_target.confidence_history[-max_trajectory_length:]
