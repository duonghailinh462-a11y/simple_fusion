"""融合调试器模块 - 记录匹配和融合过程"""

import logging
from typing import List, Optional


logger = logging.getLogger(__name__)


class FusionDebugger:
    """融合过程调试器 - 记录匹配决策和融合详情（日志统一到fusion_system.log）"""
    
    def __init__(self, log_file: str = None):
        """初始化调试器
        
        Args:
            log_file: 已弃用（保留用于兼容性）
        """
        self.log_file = log_file
        self.frame_log = []
        self.matching_log = []
        
        # 不再创建独立的文件处理器，改用统一的logger
    
    def log_frame_start(self, frame_count: int):
        """记录帧开始"""
        self.frame_log.append({'frame': frame_count, 'type': 'start'})
    
    def log_matching_attempt(self, local_target, candidates: list):
        """记录匹配尝试
        
        Args:
            local_target: 本地目标
            candidates: 候选全局目标列表
        """
        entry = {
            'camera_id': local_target.camera_id,
            'local_id': local_target.local_id,
            'candidates_count': len(candidates)
        }
        self.matching_log.append(entry)
    
    def log_matching_decision(self, local_target, best_match, time_diff, time_window):
        """记录匹配决策
        
        Args:
            local_target: 本地目标
            best_match: 最佳匹配的全局目标 (None 表示无匹配)
            time_diff: 时间差 (None 表示无匹配)
            time_window: 时间窗口
        """
        if best_match:
            entry = {
                'local': f"C{local_target.camera_id}_L{local_target.local_id}",
                'global': f"C{best_match.camera_id}_G{best_match.global_id}",
                'time_diff': time_diff,
                'time_window': time_window,
                'result': 'matched'
            }
        else:
            entry = {
                'local': f"C{local_target.camera_id}_L{local_target.local_id}",
                'global': None,
                'time_diff': time_diff,
                'time_window': time_window,
                'result': 'no_match'
            }
        self.matching_log.append(entry)
    
    def log_frame_summary(self, global_count: int, local_count: int, matches_count: int):
        """记录帧总结
        
        Args:
            global_count: 活跃全局目标数
            local_count: 本帧本地目标数
            matches_count: 本帧匹配数
        """
        self.frame_log.append({
            'type': 'summary',
            'global_targets': global_count,
            'local_targets': local_count,
            'matches': matches_count
        })
    
    def save(self):
        """保存日志到文件"""
        try:
            logger.info(f"融合调试日志已保存: {self.log_file}")
        except Exception as e:
            logger.error(f"保存调试日志出错: {e}")
    
    def log_trajectory_merge(self, global_target, local_target, weights: dict, merged_bev: tuple):
        """记录轨迹融合
        
        Args:
            global_target: 全局目标
            local_target: 本地目标
            weights: 权重信息 {'global': w_g, 'local': w_l, 'reason': str}
            merged_bev: 融合后的BEV坐标
        """
        entry = {
            'type': 'trajectory_merge',
            'global': f"C{global_target.camera_id}_G{global_target.global_id}",
            'local': f"C{local_target.camera_id}_L{local_target.local_id}",
            'weights': weights,
            'merged_bev': merged_bev
        }
        self.matching_log.append(entry)
    
    def clear(self):
        """清空日志"""
        self.frame_log.clear()
        self.matching_log.clear()

