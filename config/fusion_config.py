# -*- coding: utf-8 -*-
"""
融合系统配置 - 运行模式和数据源配置
支持 TEST 模式（本地文件）和 PROD 模式（实时网络）
"""

from dataclasses import dataclass
from typing import Literal

@dataclass
class SystemConfig:
    """系统全局配置"""
    
    # 运行模式:
    # "TEST" = 本地视频文件 + JSONL雷达数据 + 模拟时间戳 + 写文件输出
    # "PROD" = RTSP流 + UDP Proto雷达数据 + 系统时间戳 + MQTT输出
    mode: Literal["TEST", "PROD"] = "PROD"
    
    # 生产环境雷达配置
    radar_port: int = 12400  # UDP端口，用于接收Protobuf格式的雷达数据
    
    # 测试环境雷达配置
    jsonl_file: str = "/root/yolov5-7.0_lyngor1.17.0/project-simple-video/videos/radar_data.jsonl"
    
    def __str__(self):
        return f"SystemConfig(mode={self.mode})"


@dataclass
class RadarConfig:
    """雷达配置（向后兼容）"""
    
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


# 全局配置实例 - 这是唯一的配置入口
CURRENT_CONFIG = SystemConfig(
    mode="PROD"  # 去现场前改为 "PROD" 即可
)

# 默认配置（向后兼容）
DEFAULT_RADAR_CONFIG = RadarConfig(
    mode="jsonl",
    jsonl_file="data/radar_data_cleaned.jsonl"
)

