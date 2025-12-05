#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
融合匹配策略模块 - 使用策略模式管理不同的融合逻辑

职责：
- 定义融合策略的抽象接口
- 实现原始和改进两种融合策略
- 委托给 MatchingEngine 执行具体的匹配操作
"""

import logging
from abc import ABC, abstractmethod
from typing import List, Optional

from Basic import DetectionUtils, GeometryUtils, Config
from TargetTrack import GlobalTarget, LocalTarget
from FusionComponents import MatchingEngine

logger = logging.getLogger(__name__)


class FusionMatchingStrategy(ABC):
    """融合匹配策略的抽象基类"""
    
    @abstractmethod
    def perform_matching(self, 
                        local_targets_this_frame: List[LocalTarget],
                        active_global_targets: List[GlobalTarget],
                        local_to_global: dict,
                        frame_count: int,
                        perf_monitor=None) -> dict:
        """
        执行匹配
        
        Args:
            local_targets_this_frame: 本帧所有LocalTarget
            active_global_targets: 所有活跃的GlobalTarget
            local_to_global: 永久绑定字典
            frame_count: 当前帧号
            perf_monitor: 性能监控器
        
        Returns:
            {
                'local_to_global': 更新后的绑定字典,
                'matched_targets': 匹配成功的目标对列表
            }
        """
        pass
    
    @abstractmethod
    def get_strategy_name(self) -> str:
        """获取策略名称"""
        pass


class OriginalFusionStrategy(FusionMatchingStrategy):
    """
    原始融合策略：纯时间窗口匹配
    - C1 ↔ C2: 时间窗口匹配
    - C3 → C2: 时间窗口匹配
    - 无方向判断，无FIFO排队
    """
    
    def __init__(self, time_window: int = 60):
        self.time_window = time_window
        logger.info(f"OriginalFusionStrategy 初始化: time_window={time_window}")
    
    def perform_matching(self, 
                        local_targets_this_frame: List[LocalTarget],
                        active_global_targets: List[GlobalTarget],
                        local_to_global: dict,
                        frame_count: int,
                        perf_monitor=None) -> dict:
        """原始融合策略的匹配逻辑"""
        
        if perf_monitor:
            perf_monitor.start_timer('perform_matching')
        
        # 用于锁定本帧已匹配的 global_id，防止一对多
        locked_global_ids_this_frame = set()
        permanently_bound_global_ids = set(local_to_global.values())
        
        # 1. 预处理：刷新所有全局目标的融合区状态和进入时间
        for gt in active_global_targets:
            if gt.bev_trajectory:
                current_bev = gt.bev_trajectory[-1]
                is_now_in_zone = GeometryUtils.is_in_public_area(current_bev)
                gt.is_in_fusion_zone = is_now_in_zone
                if is_now_in_zone and gt.fusion_entry_frame == -1:
                    gt.fusion_entry_frame = frame_count
        
        # 2. 遍历所有本帧的 local_target 进行处理
        for local_target in local_targets_this_frame:
            lookup_key = (local_target.camera_id, local_target.local_id)
            
            # 2.1 检查此 local_target 是否已经绑定
            if lookup_key in local_to_global:
                bound_global_id = local_to_global[lookup_key]
                # 已绑定，跳过（由调用者处理融合）
                continue
            
            # 2.2 如果未绑定，执行首次匹配逻辑
            if not local_target.is_in_fusion_area:
                continue
            
            if local_target.fusion_entry_frame == -1:
                local_target.fusion_entry_frame = frame_count
            
            # 确定候选池 - 基于摄像头配对关系
            candidate_globals = []
            if local_target.camera_id == 1:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
            elif local_target.camera_id == 2:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id in [1, 3]]
            elif local_target.camera_id == 3:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
            
            # 筛选候选
            fusion_candidates = [
                gt for gt in candidate_globals 
                if (gt.is_in_fusion_zone and 
                    gt.global_id not in locked_global_ids_this_frame and
                    gt.global_id not in permanently_bound_global_ids)
            ]
            
            if not fusion_candidates:
                continue
            
            # 【原始策略】纯时间窗口匹配
            best_match = None
            best_time_diff = float('inf')
            
            for candidate in fusion_candidates:
                if not DetectionUtils.is_class_compatible(
                    local_target.class_name, candidate.class_name
                ):
                    continue
                
                if candidate.fusion_entry_frame == -1:
                    continue
                
                time_diff = abs(
                    local_target.fusion_entry_frame - candidate.fusion_entry_frame
                )
                
                if time_diff <= self.time_window and time_diff < best_time_diff:
                    best_time_diff = time_diff
                    best_match = candidate
            
            if best_match:
                local_target.matched_global_id = best_match.global_id
                local_to_global[lookup_key] = best_match.global_id
                locked_global_ids_this_frame.add(best_match.global_id)
        
        if perf_monitor:
            perf_monitor.end_timer('perform_matching')
        
        return {
            'local_to_global': local_to_global,
            'locked_global_ids': locked_global_ids_this_frame
        }
    
    def get_strategy_name(self) -> str:
        return "OriginalFusionStrategy"


class ImprovedFusionStrategy(FusionMatchingStrategy):
    """
    改进融合策略：时间窗口 + 方向判断 + FIFO排队
    - C1 ↔ C2: 时间窗口匹配（保持不变）
    - C3 → C2: FIFO排队匹配（新增）
    - 方向判断：C2的LocalTarget只有向上移动时才加入缓冲区
    - FIFO排队：C3的GlobalTarget按队列顺序与C2匹配
    
    【委托模式】
    - 具体的匹配操作委托给 MatchingEngine 执行
    - 本策略只负责调度和流程控制
    """
    
    def __init__(self, 
                 time_window: int = 60,
                 pixel_y_direction_threshold: int = 50,
                 max_retention_frames: int = 100):
        self.time_window = time_window
        self.pixel_y_direction_threshold = pixel_y_direction_threshold
        self.max_retention_frames = max_retention_frames
        
        # 【委托】使用 MatchingEngine 执行匹配
        self.matching_engine = MatchingEngine()
        
        logger.info(f"ImprovedFusionStrategy 初始化: time_window={time_window}, "
                   f"pixel_y_threshold={pixel_y_direction_threshold}")
    
    def perform_matching(self, 
                        local_targets_this_frame: List[LocalTarget],
                        active_global_targets: List[GlobalTarget],
                        local_to_global: dict,
                        frame_count: int,
                        perf_monitor=None) -> dict:
        """
        改进融合策略的匹配逻辑
        【委托给 MatchingEngine 执行具体的匹配操作】
        """
        
        if perf_monitor:
            perf_monitor.start_timer('perform_matching')
        
        locked_global_ids_this_frame = set()
        permanently_bound_global_ids = set(local_to_global.values())
        
        # 1. 预处理：刷新所有全局目标的融合区状态和进入时间
        for gt in active_global_targets:
            if gt.bev_trajectory:
                current_bev = gt.bev_trajectory[-1]
                is_now_in_zone = GeometryUtils.is_in_public_area(current_bev)
                gt.is_in_fusion_zone = is_now_in_zone
                if is_now_in_zone and gt.fusion_entry_frame == -1:
                    gt.fusion_entry_frame = frame_count
        
        # 2. 遍历所有本帧的 local_target 进行处理
        for local_target in local_targets_this_frame:
            lookup_key = (local_target.camera_id, local_target.local_id)
            
            # 2.1 检查此 local_target 是否已经绑定
            if lookup_key in local_to_global:
                bound_global_id = local_to_global[lookup_key]
                # 已绑定，跳过（由调用者处理融合）
                continue
            
            # 2.2 如果未绑定，执行首次匹配逻辑
            if not local_target.is_in_fusion_area:
                continue
            
            if local_target.fusion_entry_frame == -1:
                local_target.fusion_entry_frame = frame_count
            
            # 确定候选池 - 基于摄像头配对关系
            candidate_globals = []
            if local_target.camera_id == 1:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
            elif local_target.camera_id == 2:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id in [1, 3]]
            elif local_target.camera_id == 3:
                candidate_globals = [gt for gt in active_global_targets if gt.camera_id == 2]
            
            # 筛选候选
            fusion_candidates = [
                gt for gt in candidate_globals 
                if (gt.is_in_fusion_zone and 
                    gt.global_id not in locked_global_ids_this_frame and
                    gt.global_id not in permanently_bound_global_ids)
            ]
            
            if not fusion_candidates:
                continue
            
            # 【改进策略】根据摄像头类型选择不同的匹配方式
            best_match = None
            
            if local_target.camera_id == 3:
                # C3 → C2: 使用FIFO排队匹配（委托给 MatchingEngine）
                best_match = self.matching_engine.match_time_window(
                    local_target, fusion_candidates, self.time_window
                )
            else:
                # C1 ↔ C2: 使用时间窗口匹配（委托给 MatchingEngine）
                best_match = self.matching_engine.match_time_window(
                    local_target, fusion_candidates, self.time_window
                )
            
            if best_match:
                local_target.matched_global_id = best_match.global_id
                local_to_global[lookup_key] = best_match.global_id
                locked_global_ids_this_frame.add(best_match.global_id)
        
        if perf_monitor:
            perf_monitor.end_timer('perform_matching')
        
        return {
            'local_to_global': local_to_global,
            'locked_global_ids': locked_global_ids_this_frame
        }
    
    def add_c2_to_buffer(self, local_target: LocalTarget, frame_count: int):
        """将C2的LocalTarget添加到缓冲区（供C3匹配）"""
        # 【委托给 MatchingEngine】
        self.matching_engine.add_c2_to_buffer(local_target, frame_count)
    
    def get_metrics(self) -> dict:
        """获取匹配统计"""
        # 【委托给 MatchingEngine】
        return self.matching_engine.metrics.copy()
    
    def get_strategy_name(self) -> str:
        return "ImprovedFusionStrategy"


class FusionStrategyFactory:
    """融合策略工厂"""
    
    @staticmethod
    def create_strategy(strategy_name: str, **kwargs) -> FusionMatchingStrategy:
        """
        创建融合策略
        
        Args:
            strategy_name: 策略名称 ('original' 或 'improved')
            **kwargs: 策略参数
        
        Returns:
            FusionMatchingStrategy 实例
        """
        if strategy_name == "original":
            return OriginalFusionStrategy(
                time_window=kwargs.get('time_window', 60)
            )
        elif strategy_name == "improved":
            return ImprovedFusionStrategy(
                time_window=kwargs.get('time_window', 60),
                pixel_y_direction_threshold=kwargs.get('pixel_y_direction_threshold', 50),
                max_retention_frames=kwargs.get('max_retention_frames', 100)
            )
        else:
            raise ValueError(f"Unknown strategy: {strategy_name}")
