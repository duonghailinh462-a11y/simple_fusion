# -*- coding: utf-8 -*-
"""
雷达数据管理器 - 支持两种模式
1. JSONL模式：从本地文件加载历史数据
2. 实时模式：从TCP服务器实时接收数据
"""

import json
import logging
import multiprocessing
from typing import List, Dict, Optional, Tuple
from collections import deque
import time

from radar.RadarDataFilter import RadarDataFilter

logger = logging.getLogger(__name__)


class RadarDataManager:
    """
    统一的雷达数据管理器
    支持两种数据源模式的自动切换
    """
    
    def __init__(self, mode: str = "jsonl", jsonl_file: str = None, 
                 radar_queue: Optional[multiprocessing.Queue] = None):
        """
        初始化雷达数据管理器
        
        Args:
            mode: "jsonl" 或 "realtime"
            jsonl_file: JSONL文件路径（mode="jsonl"时必需）
            radar_queue: 实时数据队列（mode="realtime"时必需）
        """
        self.mode = mode
        self.jsonl_file = jsonl_file
        self.radar_queue = radar_queue
        
        # 初始化雷达过滤器
        self.radar_filter = RadarDataFilter()
        
        # 统计信息
        self.stats = {
            'frames_loaded': 0,
            'objects_filtered': 0,
            'objects_output': 0
        }
        
        # JSONL模式的缓存
        self.jsonl_frames = []
        self.jsonl_index = 0
        
        # 实时模式的缓冲
        self.realtime_buffer = deque(maxlen=100)
        self.last_processed_time = None
        
        # 验证模式
        if self.mode == "jsonl":
            if not jsonl_file:
                raise ValueError("jsonl模式需要指定jsonl_file参数")
            self._load_jsonl_data()
            logger.info(f"✅ 雷达数据管理器初始化 (JSONL模式) - 文件: {jsonl_file}, 帧数: {len(self.jsonl_frames)}")
        
        elif self.mode == "realtime":
            if not radar_queue:
                raise ValueError("realtime模式需要指定radar_queue参数")
            logger.info(f"✅ 雷达数据管理器初始化 (实时模式) - 监听数据队列")
        
        else:
            raise ValueError(f"不支持的模式: {mode}。必须是 'jsonl' 或 'realtime'")
    
    def _load_jsonl_data(self):
        """从JSONL文件加载所有雷达帧"""
        try:
            with open(self.jsonl_file, 'r', encoding='utf-8') as f:
                for line in f:
                    if line.strip():
                        self.jsonl_frames.append(json.loads(line))
            logger.info(f"已加载 {len(self.jsonl_frames)} 帧雷达数据")
        except Exception as e:
            logger.error(f"加载JSONL文件出错: {e}")
            self.jsonl_frames = []
    
    def get_frame(self) -> Optional[dict]:
        """
        获取下一帧雷达数据
        
        Returns:
            雷达数据帧，或None如果没有数据
        """
        if self.mode == "jsonl":
            return self._get_jsonl_frame()
        else:
            return self._get_realtime_frame()
    
    def _get_jsonl_frame(self) -> Optional[dict]:
        """JSONL模式：按顺序返回帧"""
        if self.jsonl_index < len(self.jsonl_frames):
            frame = self.jsonl_frames[self.jsonl_index]
            self.jsonl_index += 1
            self.stats['frames_loaded'] += 1
            return frame
        return None
    
    def _get_realtime_frame(self) -> Optional[dict]:
        """实时模式：从队列获取最新帧"""
        try:
            # 非阻塞地尝试从队列获取数据
            frame = self.radar_queue.get_nowait()
            self.realtime_buffer.append(frame)
            self.stats['frames_loaded'] += 1
            return frame
        except:
            return None
    
    def process_all_available_frames(self) -> List[dict]:
        """
        处理所有可用的雷达帧
        
        Returns:
            已过滤的雷达对象列表
        """
        all_output_objects = []
        
        if self.mode == "jsonl":
            # JSONL模式：一次性处理所有剩余帧
            while self.jsonl_index < len(self.jsonl_frames):
                frame = self.get_frame()
                if frame:
                    output_objects = self.radar_filter.filter_radar_frame(frame)
                    all_output_objects.extend(output_objects)
        else:
            # 实时模式：处理缓冲中的所有帧
            while True:
                frame = self.get_frame()
                if not frame:
                    break
                output_objects = self.radar_filter.filter_radar_frame(frame)
                all_output_objects.extend(output_objects)
        
        self.stats['objects_output'] += len(all_output_objects)
        return all_output_objects
    
    def process_batch_frames(self, batch_size: int = 10) -> List[dict]:
        """
        批量处理指定数量的雷达帧
        
        Args:
            batch_size: 单次处理的帧数
        
        Returns:
            已过滤的雷达对象列表
        """
        all_output_objects = []
        
        for _ in range(batch_size):
            frame = self.get_frame()
            if not frame:
                break
            
            output_objects = self.radar_filter.filter_radar_frame(frame)
            all_output_objects.extend(output_objects)
        
        self.stats['objects_output'] += len(all_output_objects)
        return all_output_objects
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        stats = self.radar_filter.get_stats()
        stats.update({
            'manager_frames_loaded': self.stats['frames_loaded'],
            'manager_objects_output': self.stats['objects_output'],
            'mode': self.mode
        })
        
        if self.mode == "jsonl":
            stats['jsonl_progress'] = f"{self.jsonl_index}/{len(self.jsonl_frames)}"
        else:
            stats['realtime_buffer_size'] = len(self.realtime_buffer)
        
        return stats
    
    def reset_stats(self):
        """重置统计信息"""
        self.radar_filter.reset_stats()
        for key in self.stats:
            self.stats[key] = 0
    
    def reset_jsonl_index(self):
        """重置JSONL索引（用于循环读取）"""
        if self.mode == "jsonl":
            self.jsonl_index = 0
            logger.info("JSONL索引已重置")

