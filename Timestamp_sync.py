#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFmpeg NTP时间戳同步模块 (已更新为混合策略)
负责：
1. 通过FFmpeg提取RTSP流的pts_time（毫秒级精度）
2. 为SDK推理进程提供准确的时间戳映射
3. 基于时间戳的多摄像头帧同步 (混合策略)
"""

import subprocess
import sys
import time
import re
import threading
import logging
from collections import deque
from typing import Dict, Optional, Tuple
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)

# 解析FFmpeg showinfo日志行的正则表达式
# 示例: [Parsed_showinfo_0 ...] n:  123 pts: 456789 pts_time:123.456
SHOWINFO_RE = re.compile(r"n:\s*(\d+).*?pts:\s*(-?\d+).*?pts_time:([0-9.]+)")


class FFmpegTimeStampProvider:
    """
    视频时间戳提供器 - 为每个摄像头计算基于帧号的绝对时间戳
    
    工作原理：
    - 每个摄像头有一个手动指定的起始时间（datetime格式，如 "2025-11-21 11:18:09.304"）
    - 时间戳 = start_datetime + frame_id / fps 的时间差
    - 所有摄像头使用 25fps
    - 返回格式：绝对时间字符串 "YYYY-MM-DD HH:MM:SS.mmm"
    """
    
    # 类变量：存储每个摄像头的起始时间（datetime对象）
    _camera_start_datetimes: Dict[int, datetime] = {}
    
    def __init__(self, video_path: str, camera_id: int, fps: float = 25.0):
        """
        初始化时间戳提供器
        
        Args:
            video_path: 视频文件路径
            camera_id: 摄像头ID (1, 2, 3)
            fps: 视频帧率 (默认25fps)
        """
        self.camera_id = camera_id
        self.video_path = video_path
        self.fps = fps
        
        # 获取该摄像头的起始时间（datetime对象）
        self.start_datetime = self._camera_start_datetimes.get(camera_id)
        
        # 帧号计数
        self.frame_count = 0
        self.lock = threading.Lock()
        
        logger.info(f"Camera{camera_id} 时间戳提供器初始化: {video_path}, FPS:{fps}")
        if not self.start_datetime:
            logger.warning(f"Camera{self.camera_id} 未设置起始时间")
        else:
            logger.info(f"Camera{camera_id} 起始时间: {self.start_datetime.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    
    @classmethod
    def set_camera_start_datetime(cls, camera_id: int, start_datetime_str: str):
        """
        设置指定摄像头的起始时间（字符串格式）
        
        Args:
            camera_id: 摄像头ID (1, 2, 3)
            start_datetime_str: 起始时间字符串，格式 "2025-11-21 11:18:09.304" 或 "2025-11-21 11:18:09"
        """
        try:
            # 尝试解析带毫秒的格式
            if '.' in start_datetime_str:
                dt = datetime.strptime(start_datetime_str, "%Y-%m-%d %H:%M:%S.%f")
            else:
                dt = datetime.strptime(start_datetime_str, "%Y-%m-%d %H:%M:%S")
            cls._camera_start_datetimes[camera_id] = dt
            logger.info(f"Camera{camera_id} 起始时间设置: {dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        except ValueError as e:
            logger.error(f"Camera{camera_id} 时间格式错误: {e}")
    
    @classmethod
    def set_all_camera_start_datetimes(cls, timestamps: Dict[int, str]):
        """
        批量设置所有摄像头的起始时间
        
        Args:
            timestamps: {camera_id: "2025-11-21 11:18:09.304", ...}
        """
        for camera_id, ts_str in timestamps.items():
            cls.set_camera_start_datetime(camera_id, ts_str)
    
    def get_timestamp(self, frame_id: int) -> Optional[str]:
        """
        获取指定帧号对应的绝对时间戳（字符串格式）
        
        计算公式：timestamp = start_datetime + timedelta(seconds=frame_id/fps)
        
        Returns:
            时间字符串，格式 "2025-11-21 11:18:09.304"，如果未设置起始时间则返回 None
        """
        with self.lock:
            if self.start_datetime is None:
                return None
            
            # 计算时间差（秒）
            time_offset_seconds = frame_id / self.fps
            
            # 计算目标时间
            target_datetime = self.start_datetime + timedelta(seconds=time_offset_seconds)
            
            # 格式化为字符串，保留毫秒
            return target_datetime.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    
    def increment_frame(self):
        """增加帧计数"""
        with self.lock:
            self.frame_count += 1
    
    def get_frame_count(self) -> int:
        """获取当前帧计数"""
        with self.lock:
            return self.frame_count
    
    def close(self):
        """关闭时间戳提供器"""
        logger.info(f"Camera{self.camera_id} 时间戳提供器关闭")

# ---------------------------------------------------------------------------
# ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 核心修改区域 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
# ---------------------------------------------------------------------------

class FFmpegTimestampFrameSynchronizer:
    """
    绝对时间戳帧同步管理器 (已更新为混合策略)
    
    工作原理：
    1. Warmup阶段：等待所有摄像头都有数据，找到"最晚的起跑者"，丢弃所有早于该时间的帧。
    2. 持续同步阶段：
       - A (已同步): 如果三帧时间戳在容忍度内，打包送出。
       - B (不同步): 如果超出容忍度，丢弃时间戳最早的那一帧，等待下一轮。
    
    时间戳格式：绝对时间字符串 "2025-11-21 11:18:09.304"
    """
    
    def __init__(self, num_cameras: int = 3, timestamp_tolerance_ms: int = 2000):
        """
        初始化时间戳帧同步器
        
        Args:
            num_cameras: 摄像头数量
            timestamp_tolerance_ms: 启动容忍度（毫秒），用于Warmup。
                                    (来自main.py, 设为2000ms是合理的)
        """
        self.num_cameras = num_cameras
        
        # 启动容忍度：用于Warmup，允许摄像头启动时间有较大差异（秒）
        self.STARTUP_TOLERANCE_SEC = timestamp_tolerance_ms / 1000.0
        
        # 持续同步容忍度：用于后续1:1:1同步，必须非常严格（秒）
        # 40ms 约等于 25fps下的 1 帧
        self.SYNC_TOLERANCE_SEC = 0.04
        
        # 混合策略状态
        self.warmup_complete = False
        
        # 缓冲区：{ camera_id -> { timestamp_str -> [frame_data, ...] } }
        # 使用时间戳字符串作为 key
        self.frame_buffers: Dict[int, Dict[str, list]] = {
            i: {} for i in range(1, num_cameras + 1)
        }
        
        # 各摄像头已同步的最新时间戳
        self.last_synchronized_timestamp = None
        
        # 统计信息
        self.sync_count = 0
        self.frame_count = 0
        
        logger.info(f"帧同步器初始化: {num_cameras}摄像头, 启动容忍:{self.STARTUP_TOLERANCE_SEC:.3f}s, 同步容忍:{self.SYNC_TOLERANCE_SEC:.3f}s")
    
    def _peek_earliest_timestamp(self, camera_id: int) -> Optional[str]:
        """(辅助函数) 窥视缓冲中最早的时间戳，不取出"""
        if not self.frame_buffers[camera_id]:
            return None
        try:
            # 时间戳字符串可以直接比较（因为格式是 YYYY-MM-DD HH:MM:SS.mmm）
            return min(self.frame_buffers[camera_id].keys())
        except ValueError:
            return None
    
    def _timestamp_to_datetime(self, timestamp_str: str) -> datetime:
        """将时间戳字符串转换为 datetime 对象"""
        if timestamp_str is None:
            raise ValueError("timestamp_str cannot be None")
        if not isinstance(timestamp_str, str):
            raise TypeError(f"timestamp_str must be a string, got {type(timestamp_str)}: {timestamp_str}")
        if '.' in timestamp_str:
            return datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
        else:
            return datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
    
    def _datetime_to_timestamp(self, dt: datetime) -> str:
        """将 datetime 对象转换为时间戳字符串"""
        return dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    
    def _get_time_difference_sec(self, ts1: str, ts2: str) -> float:
        """计算两个时间戳之间的差值（秒）"""
        if ts1 is None or ts2 is None:
            raise ValueError(f"Cannot calculate time difference: ts1={ts1}, ts2={ts2}")
        if not isinstance(ts1, str) or not isinstance(ts2, str):
            raise TypeError(f"Timestamps must be strings: ts1={type(ts1)}, ts2={type(ts2)}")
        dt1 = self._timestamp_to_datetime(ts1)
        dt2 = self._timestamp_to_datetime(ts2)
        return abs((dt2 - dt1).total_seconds())
            
    def _pop_earliest_frame(self, camera_id: int) -> Optional[dict]:
        """(辅助函数) 弹出缓冲中最早的一帧"""
        if not self.frame_buffers[camera_id]:
            return None
        
        try:
            # 1. 找到最早的时间戳
            ts_min = min(self.frame_buffers[camera_id].keys())
            
            # 2. 从该时间戳的列表中弹出一个帧
            frame = self.frame_buffers[camera_id][ts_min].pop(0)
            
            # 3. 如果列表空了，删除这个时间戳键
            if not self.frame_buffers[camera_id][ts_min]:
                del self.frame_buffers[camera_id][ts_min]
                
            return frame
        except Exception as e:
            logger.error(f"C{camera_id} _pop_earliest_frame 异常: {e}")
            return None

    def add_frame(self, camera_id: int, frame_data: dict):
        """
        添加帧到缓冲区，使用绝对时间戳字符串作为key
        
        Args:
            camera_id: 摄像头ID
            frame_data: 帧数据字典，应包含 'timestamp' 字段（绝对时间字符串）
        """
        # 获取时间戳（应该是绝对时间字符串）
        if 'timestamp' in frame_data and frame_data['timestamp'] is not None:
            timestamp_str = frame_data['timestamp']
        else:
            # 降级方案：使用当前系统时间
            now = datetime.now()
            timestamp_str = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            logger.warning(f"C{camera_id} 未提供时间戳，使用系统时间")
        
        # 增强帧数据
        frame_data['timestamp'] = timestamp_str
        frame_data['camera_id'] = camera_id
        frame_data['sync_id'] = f"C{camera_id}_T{timestamp_str}"
        
        # 添加到缓冲区
        if timestamp_str not in self.frame_buffers[camera_id]:
            self.frame_buffers[camera_id][timestamp_str] = []
        self.frame_buffers[camera_id][timestamp_str].append(frame_data)
        
        self.frame_count += 1
        
        # 清理过期帧
        self._cleanup_old_frames(camera_id)
    
    def get_synchronized_frames(self) -> Tuple[Optional[dict], Optional[int]]:
        """
        获取同步的帧组 - (核心修改：实现混合策略)
        
        返回：
            (synchronized_frames_dict, sync_frame_number) 或 (None, None)
        """
        
        # --- 0. 检查所有缓冲区是否都有数据 ---
        earliest_timestamps = {}
        for cid in range(1, self.num_cameras + 1):
            ts = self._peek_earliest_timestamp(cid)
            if ts is None:
                # 至少一个缓冲区是空的，等待
                return None, None
            if not isinstance(ts, str):
                logger.error(f"Camera {cid} timestamp is not a string: {type(ts)} = {ts}")
                return None, None
            earliest_timestamps[cid] = ts

        # --- 1. 阶段一: Warmup (只执行一次) ---
        if not self.warmup_complete:
            # 找到"最晚的起跑者" (T_start = max of the min timestamps)
            T_start = max(earliest_timestamps.values())
            
            # (可选检查) 检查启动差异是否过大
            T_min_global = min(earliest_timestamps.values())
            startup_diff_sec = self._get_time_difference_sec(T_min_global, T_start)
            if startup_diff_sec > self.STARTUP_TOLERANCE_SEC:
                logger.warning(f"摄像头启动时间差过大: {startup_diff_sec:.3f}s > {self.STARTUP_TOLERANCE_SEC:.3f}s")

            logger.info(f"Warmup: 对齐起跑线 T_start={T_start}")
            
            # 丢弃所有早于 T_start 的帧
            frames_dropped_count = 0
            for cid in range(1, self.num_cameras + 1):
                # 检查这个流是否需要丢帧
                while True:
                    peek_ts = self._peek_earliest_timestamp(cid)
                    if peek_ts is not None and peek_ts < T_start:
                        # 丢弃
                        self._pop_earliest_frame(cid)
                        frames_dropped_count += 1
                    else:
                        break # 此摄像头已对齐
            
            if frames_dropped_count > 0:
                logger.info(f"Warmup: 丢弃 {frames_dropped_count} 帧")
            
            logger.info("Warmup 完成，进入持续同步")
            self.warmup_complete = True
            
            # Warmup完成后，重新检查缓冲区状态
            earliest_timestamps = {}
            for cid in range(1, self.num_cameras + 1):
                ts = self._peek_earliest_timestamp(cid)
                if ts is None:
                    return None, None # 等待
                earliest_timestamps[cid] = ts
        
        # --- 2. 阶段二: 持续同步 (混合策略) ---
        
        # (此时 earliest_timestamps 变量已是Warmup后或上一轮同步后的最新状态)
        T_min = min(earliest_timestamps.values())
        T_max = max(earliest_timestamps.values())
        
        # 计算时间差（秒）
        time_diff_sec = self._get_time_difference_sec(T_min, T_max)
        
        # 【情况 A：已同步】
        if time_diff_sec <= self.SYNC_TOLERANCE_SEC:
            # 完美同步！
            synchronized_frames = {}
            for cid in range(1, self.num_cameras + 1):
                synchronized_frames[cid] = self._pop_earliest_frame(cid)
            
            self.sync_count += 1
            self.last_synchronized_timestamp = T_max # 使用最晚的作为同步点
            
            # 每100帧打印一次
            if self.sync_count % 100 == 0:
                 logger.debug(f"同步成功(第{self.sync_count}组): T_max={T_max}, 帧间差:{time_diff_sec:.3f}s")
                 
            # 返回帧组 和 帧组序号 (main.py用它当current_frame)
            return synchronized_frames, self.sync_count 

        # 【情况 B：不同步（保险丝）】
        else:
            # 丢弃"过早"的帧，即 T_min 对应的帧
            # 找到是哪个摄像头
            cid_to_drop = -1
            for cid, ts in earliest_timestamps.items():
                if ts == T_min:
                    cid_to_drop = cid
                    break
            
            if cid_to_drop != -1:
                # 丢弃这帧
                frame_dropped = self._pop_earliest_frame(cid_to_drop)
                if self.sync_count % 10 == 0:
                    logger.debug(f"不同步: 差{time_diff_sec:.3f}s, 丢弃C{cid_to_drop}帧")
            
            # 这一轮不产出数据，返回None，主循环会立刻回来重新检查
            return None, None
    
    def get_buffer_status(self) -> dict:
        """获取缓冲区状态信息 (与你原版一致)"""
        status = {}
        for camera_id in range(1, self.num_cameras + 1):
            total_frames = sum(len(frames) for frames in self.frame_buffers[camera_id].values())
            if self.frame_buffers[camera_id]:
                timestamps = sorted(self.frame_buffers[camera_id].keys())
                status[camera_id] = {
                    'count': total_frames,
                    'min_timestamp': timestamps[0],
                    'max_timestamp': timestamps[-1],
                    'timestamp_range_ms': timestamps[-1] - timestamps[0] if timestamps else 0,
                    'timestamps_count': len(timestamps)
                }
            else:
                status[camera_id] = {'count': 0}
        return status
    
    def _cleanup_old_frames(self, camera_id: int):
        """
        清理过期的帧，防止内存无限增长
        """
        
        # 紧急清理：如果缓冲区帧数过多
        max_buffer_frames = 1000
        total_frames = sum(len(frames) for frames in self.frame_buffers[camera_id].values())
        
        if total_frames > max_buffer_frames:
            all_timestamps = sorted(self.frame_buffers[camera_id].keys())
            
            frames_to_remove_count = total_frames - max_buffer_frames
            removed_count = 0
            
            # 从最早的时间戳开始删
            for ts in all_timestamps:
                if ts not in self.frame_buffers[camera_id]:
                    continue
                
                frames_at_ts = self.frame_buffers[camera_id][ts]
                remove_n = min(len(frames_at_ts), frames_to_remove_count - removed_count)
                
                if remove_n == len(frames_at_ts):
                    del self.frame_buffers[camera_id][ts]
                else:
                    self.frame_buffers[camera_id][ts] = frames_at_ts[remove_n:]
                
                removed_count += remove_n
                if removed_count >= frames_to_remove_count:
                    break
                    
            if removed_count > 0:
                logger.warning(f"C{camera_id} 缓冲区清理: {removed_count}帧 (超{max_buffer_frames}帧)")
    
    def next_frame(self):
        """进入下一帧处理周期"""
        pass

