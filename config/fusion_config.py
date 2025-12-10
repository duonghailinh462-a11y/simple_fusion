# -*- coding: utf-8 -*-
"""
融合系统配置 - 雷达数据源配置
"""

from dataclasses import dataclass
from typing import Literal

@dataclass
class RadarConfig:
    """雷达配置"""
    
    # 雷达数据源模式: "jsonl" 或 "realtime"
    mode: Literal["jsonl", "realtime"] = "jsonl"
    
    # JSONL模式下的文件路径
    jsonl_file: str = "../radar_data_cleaned.jsonl"
    
    # 实时模式下的处理频率（帧/秒）
    realtime_process_fps: int = 10
    
    # 雷达数据MQTT主题前缀（可选）
    mqtt_topic_prefix: str = "radar"
    
    def __str__(self):
        return f"RadarConfig(mode={self.mode})"


# 默认配置
DEFAULT_RADAR_CONFIG = RadarConfig(
    mode="jsonl",
    jsonl_file="data/radar_data_cleaned.jsonl"
)

