#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDK版多摄像头融合系统 - 重构版
职责分离: 
1. 子进程 (yolov5_SDK): 仅负责视频读取、SDK推理、结果入队列 (使用 SDKinfer_ffmpeg.py)
2. 主进程 (main): 负责跟踪(BYTETracker)、区域过滤、跨摄像头融合、帧同步 

🔧 优化 (2025-12-16)：
   ✅ 雷达数据加载从批量预加载改为流式加载
   ✅ 缓冲区大小从 1000+ 帧降低到 5-10 帧
   ✅ 时间戳匹配耗时从 72ms 降低到 <1ms
   ✅ 立即启动，无需等待数据预加载完成
   
   核心改动：
   - RadarDataLoader.load() (批量) → StreamingRadarLoader (流式)
   - 在主循环中按需加载雷达数据，自动清理过期数据
   - 详见第 200-235 行和 478-520 行
"""

import os
import sys
import time
import multiprocessing
import copy
import json
import signal
import logging
import queue
from collections import defaultdict, deque
from statistics import mean, median
sys.path.append('/usr/local/lynxi/sdk/sdk-samples/python')

# 导入统一的日志配置
from core.logger_config import FusionLogger, get_logger

# 初始化日志系统（必须在导入其他模块之前）
FusionLogger.setup()
logger = get_logger(__name__)

import numpy as np
import cv2
from ctypes import *
import ctypes
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict

# 导入SDK相关模块
import pycommon.common as common
import pylynchipsdk as sdk
from pycommon.infer_process import *
from pycommon.callback_data_struct import *
from pycommon.dump_json import *
from ByteTrack.optimized_byte_tracker import OptimizedBYTETracker as BYTETracker

# 导入RTSP和MQTT相关模块 (新增)
try:
    from vision.rtsp_reader import RTSPStreamReader
    from core.mqtt_publisher import MqttPublisher  
    from core.config_reader import ConfigReader
    RTSP_MQTT_AVAILABLE = True
except ImportError as e:
    logger.warning(f"无法导入RTSP/MQTT模块: {e}, 将使用本地视频模式")
    RTSP_MQTT_AVAILABLE = False

# 🔧 移除FFmpeg相关导入，改用直接计算时间戳
# from Timestamp_sync import FFmpegTimeStampProvider, FFmpegTimestampFrameSynchronizer
from core.Basic import (Config, DetectionUtils, GeometryUtils, PerformanceMonitor,
                        filter_by_detect_areas, batch_prepare_tracker_input, 
                        batch_convert_track_results)
from vision.TargetTrack import TargetBuffer
from core.Fusion import CrossCameraFusion
from core.RadarVisionFusion import RadarVisionFusionProcessor, RadarDataLoader, OutputObject
from core.StreamingDataLoader import StreamingRadarLoader, parse_time  # 新增：流式雷达数据加载器
from radar.RadarDataFilter import RadarDataFilter  # 新增：雷达地理过滤
from radar.RadarFusionOrchestrator import RadarFusionOrchestrator  # 新增：雷达融合协调器
from vision.CameraManager import CameraManager
from core.ResultBuffer import ResultOutputManager
from config.region_config import get_lane_for_point

# 创建共享布尔值用于停止运行线程
cancel_flag = multiprocessing.Value('b', False)

# --- 信号处理函数 ---

def cancel_process(signum, frame):
    """取消处理信号"""
    global cancel_flag
    cancel_flag.value = True
    logger.info("收到停止信号，正在退出...")

# 🔧 修改：使用初始视频时间 + frame_id/fps 计算时间戳
def create_sdk_worker_process(camera_id: int, video_path: str, result_queue: multiprocessing.Queue):
    """创建并运行一个独立的 SDK 推理子进程 (生产者)"""
    
    # 确保子进程能找到 SDKinfer 模块
    from vision.SDKinfer import yolov5_SDK, infer_process_attr
    from core.Basic import Config
    
    try:
        logger.info(f"Camera{camera_id} 子进程启动")
        logger.info(f"Camera{camera_id} 视频源: {video_path[:80] if len(video_path) > 80 else video_path}")
        
        attr = infer_process_attr()
        attr.url = video_path
        attr.device_id = 0
        attr.chan_id = camera_id - 1
        attr.plugin_path = "/usr/local/lynxi/sdk/sdk-samples/plugin/obj/libYolov5Plugin.so"
        attr.model_path = "/root/yolov5-7.0_lyngor1.17.0/best_yolov5s_onnx/Net_0/"
        attr.show_type = 2
        attr.output_path = ""  # 🔧 参考 main_1015.py：设置输出路径
        # 🔧 修复：初始化 video_frame 为队列对象，避免 'int' object has no attribute 'queue' 错误
        attr.video_frame = queue.Queue(10)
        
        logger.info(f"Camera{camera_id} 初始化yolov5_SDK")
        logger.info(f"Camera{camera_id} 如果出现 'av.open' 错误，请检查RTSP URL/网络/视频文件")
        
        # 🔧 从配置中获取初始时间和fps
        start_datetime_str = Config.CAMERA_START_DATETIMES.get(camera_id)
        fps = Config.FPS
        
        # 注意：SDK初始化可能会在这里失败，如果RTSP连接不可用
        worker = yolov5_SDK(attr, result_queue, start_datetime_str=start_datetime_str, fps=fps) 
        logger.info(f"Camera{camera_id} yolov5_SDK初始化成功")
        
        # 🔧 参考 main_1015.py：更新类别名称到插件中
        class_name_path = "/usr/local/lynxi/sdk/sdk-samples/data/class.txt"
        if os.path.exists(class_name_path):
            worker.update_class_name(class_name_path)
            logger.info(f"Camera{camera_id} 类别名称已更新")
        
        logger.info(f"Camera{camera_id} 开始运行SDK推理...")
        worker.run(cancel_flag)
        logger.info(f"Camera{camera_id} 子进程正常退出")
        
    except Exception as e:
        error_msg = str(e)
        logger.error(f"Camera{camera_id} SDK进程失败: {error_msg}")
        import traceback
        traceback.print_exc()
        # 确保进程能退出
        os._exit(1)
        
# --- 主程序 (消费者，融合与同步) ---

if __name__ == "__main__":
    
    # 1. 配置
    logger.info("="*60)
    logger.info("程序开始启动...")
    logger.info("="*60)
    
    # 尝试从配置文件读取RTSP URLs，失败则使用本地视频文件
    video_paths = {}
    if RTSP_MQTT_AVAILABLE:
        try:
            config_reader = ConfigReader()
            enabled_cameras = config_reader.get_enabled_cameras()
            
            if enabled_cameras and len(enabled_cameras) >= 3:
                for i, camera in enumerate(enabled_cameras[:3], 1):
                    video_paths[i] = camera['rtsp_url']
                    logger.info(f"摄像头{i}: {camera['name']} - {camera['rtsp_url']}")
                logger.info("使用RTSP流作为输入")
            else:
                logger.warning("配置文件中摄像头数量不足，回退到本地视频文件")
                raise Exception("配置不足")
        except Exception as e:
            logger.warning(f"读取RTSP配置失败: {e}, 使用本地视频文件")
            RTSP_MQTT_AVAILABLE = False
    
    # 回退到本地视频文件
    if not RTSP_MQTT_AVAILABLE or not video_paths:
        logger.info("使用本地视频文件")
        # 设置默认的本地视频文件路径 - 使用实际存在的视频文件
        video_paths = {
            1: "/root/yolov5-7.0_lyngor1.17.0/videos/test_121.mp4",
            2: "/root/yolov5-7.0_lyngor1.17.0/videos/test_122.mp4",
            3: "/root/yolov5-7.0_lyngor1.17.0/videos/test_123.mp4"
        }
        
        # 检查视频文件是否存在
        for cam_id, video_path in video_paths.items():
            if not os.path.exists(video_path):
                logger.error(f"Camera{cam_id} 视频文件不存在: {video_path}")
                logger.error("请确保视频文件存在或提供正确的RTSP源配置")
                sys.exit(1)
            else:
                logger.info(f"Camera{cam_id}: {video_path}")
    
    detect_areas = {
        1: [np.array([[0, 720], [226, 324], [576, 77], [714, 77], [1278, 390], [1280, 720]], dtype=np.int32),
            np.array([[218,324], [472,149], [366,141], [48,312]], dtype=np.int32)],
        2: [np.array([[0, 503], [0, 714], [1280, 720], [1280, 410], [800, 128],[471,133]], dtype=np.int32)],
        3: [np.array([[70, 720], [1030, 720], [934, 166], [90, 166]], dtype=np.int32)]
    }

    signal.signal(signal.SIGINT, cancel_process)

    # 2. 初始化核心组件
    perf_monitor = PerformanceMonitor()

    # 2.0 初始化摄像头管理器
    camera_manager = CameraManager(video_paths, cancel_flag)
    queues = camera_manager.create_queues(maxsize=10)

    # 2.1 初始化雷达地理过滤器 (第一道关卡)
    logger.info("初始化雷达地理过滤器")
    radar_filter = RadarDataFilter()
    logger.info("✓ RadarDataFilter初始化成功")
    
    # 2.2 初始化雷达融合模块 (第二道关卡：融合处理)
    logger.info("初始化雷达融合模块")
    radar_fusion_enabled = False
    radar_data_loader = None
    radar_fusion_processors = {}  # 按摄像头存储融合处理器
    radar_stream_iterator = None  # 流式加载器迭代器
    
    # 雷达数据文件路径 (可配置)
    radar_data_path = '/root/yolov5-7.0_lyngor1.17.0/project-simple-video/videos/radar_data.jsonl'
        
    try:
        if os.path.exists(radar_data_path):
            # ✅ 改进：使用流式加载器替代批量加载
            logger.info(f"初始化流式雷达加载器: {radar_data_path}")
            radar_data_loader = StreamingRadarLoader(radar_data_path)
            radar_stream_iterator = radar_data_loader.stream_radar_frames()
            
            # 为每个摄像头初始化独立的融合处理器
            for camera_id in [1, 2, 3]:
                radar_fusion_processors[camera_id] = RadarVisionFusionProcessor(
                    fusion_area_geo=None,  # 使用融合区域判断已在GlobalID分配时完成
                    lat_offset=0.0,
                    lon_offset=0.0,
                    enable_lane_filtering=True,  # 禁用车道过滤（过滤太严格，导致匹配率低）
                    camera_id=camera_id  # 传入摄像头ID，用于调整阈值
                )
                logger.info(f"C{camera_id} 雷达融合处理器初始化成功")
            
            radar_fusion_enabled = True
            logger.info(f"✅ 流式雷达融合模块初始化成功 (不进行预加载，按需流式加载)")
        else:
            logger.warning(f"雷达数据文件不存在: {radar_data_path}")
            logger.warning("将不使用雷达融合功能")
    except Exception as e:
        logger.warning(f"雷达融合模块初始化失败: {e}")
        logger.warning("将不使用雷达融合功能")
        radar_fusion_enabled = False
    
    # 初始化雷达融合协调器
    radar_fusion_orchestrator = None
    radar_filter = None
    if radar_fusion_enabled:
        try:
            radar_filter = RadarDataFilter()
            radar_fusion_orchestrator = RadarFusionOrchestrator(
                radar_data_loader, radar_filter, radar_fusion_processors
            )
            logger.info("雷达融合协调器已初始化")
        except Exception as e:
            logger.warning(f"雷达融合协调器初始化失败: {e}")
            radar_fusion_orchestrator = None
    
    # 初始化统计：直接输出的雷达数据
    radar_direct_output_count = 0  # 区外直接输出
    radar_fusion_count = 0  # 区内融合
    
    # DEBUG: 跟踪器输入统计
    tracker_input_stats = {1: {'total': 0, 'empty': 0, 'non_empty': 0, 'total_dets': 0}, 
                          2: {'total': 0, 'empty': 0, 'non_empty': 0, 'total_dets': 0},
                          3: {'total': 0, 'empty': 0, 'non_empty': 0, 'total_dets': 0}}
    
    # 初始化MQTT发布器
    mqtt_publisher = None
    if RTSP_MQTT_AVAILABLE:
        try:
            mqtt_publisher = MqttPublisher("config/mqtt_config.ini")
            mqtt_publisher.connect()
            logger.info("MQTT发布器已连接")
        except Exception as e:
            logger.warning(f"MQTT连接失败: {e}, 将使用JSON文件保存")
            mqtt_publisher = None
    
    # 初始化结果输出管理器（三路缓冲和时间对齐）
    fusion_system = CrossCameraFusion()
    result_output_manager = ResultOutputManager(
        fusion_system=fusion_system,
        mqtt_publisher=mqtt_publisher,
        time_threshold=0.5  # 时间阈值（秒）
    )
    logger.info("结果输出管理器已初始化 - 三路缓冲和时间对齐模式")
    
    class TrackerArgs: # 新ByteTracker 所需参数
        def __init__(self):
            self.track_thresh = 0.5    # 跟踪置信度阈值，过低会导致噪声
            self.track_buffer = 30     # 增加缓冲区大小以保留轨迹历史
            self.match_thresh = 0.8    # 匹配阈值
            self.mot20 = False         # MOT20数据集标志
    
    tracker_args = TrackerArgs()
    trackers = {i: BYTETracker(tracker_args, frame_rate=Config.FPS) for i in [1, 2, 3]}
    logger.info("已启用优化版ByteTracker - 交替跟踪模式") 

    # 3. 创建并启动 SDK 推理进程 (生产者)
    # 使用 CameraManager 处理RTSP连接测试和进程启动
    camera_manager.test_all_rtsp_connections()
    camera_manager.start_all_cameras(create_sdk_worker_process)
    processes = camera_manager.get_processes()

    # 3.5. 启动预热阶段：等待所有摄像头队列都有数据
    preheat_success = camera_manager.wait_for_preheat(timeout=30)


    # 4. 主循环：时间戳融合逻辑 (消费者)
    
    # 🔧 改造：移除帧同步，改为单路独立处理
    logger.info("融合主循环启动 - 单路处理模式")
    logger.info("处理模式: 单路独立处理 + 后期三路匹配")
    logger.info("="*60)
    
    # 初始化单路结果存储
    camera_results = {1: [], 2: [], 3: []}  # 存储每个摄像头的处理结果

    try:
        current_frame = 0
        radar_id_map = {}  # 全局radar_id_map
        
        # 🔧 新增：性能统计（整体处理时间）
        frame_times = deque(maxlen=300)  # 保留最近300帧的处理时间
        component_times = {
            'queue_processing': deque(maxlen=300),
            'frame_processing': deque(maxlen=300),
            'matching_processing': deque(maxlen=300),
            'radar_fusion': deque(maxlen=300),
            'store_single_camera': deque(maxlen=300),  # 新增：存储单路结果
            'result_buffer': deque(maxlen=300),
            'json_mqtt': deque(maxlen=300),  # 新增：JSON和MQTT处理
            'total_frame': deque(maxlen=300)
        }
        
        while not cancel_flag.value:
            # 🔧 新增：记录整个帧处理的开始时间
            frame_start_time = time.time()
            
            # A. 从所有队列中获取结果，单路独立处理
            perf_monitor.start_timer('queue_processing')
            current_frame_results = {}
            
            for camera_id in [1, 2, 3]:
                queue_size = queues[camera_id].qsize()
                perf_monitor.record_queue_stats(camera_id, queue_size, 'read')
                
                try:
                    # 非阻塞获取单个帧
                    result = queues[camera_id].get_nowait()
                    perf_monitor.add_counter('queue_operations')
                    current_frame_results[camera_id] = result
                except multiprocessing.queues.Empty:
                    # 该摄像头暂无新帧，跳过
                    pass
                except Exception as e:
                    logger.error(f"C{camera_id} 队列读取异常: {e}")
            
            queue_processing_time = perf_monitor.end_timer('queue_processing')
            
            # 如果没有任何摄像头有新帧，短暂等待
            if not current_frame_results:
                time.sleep(0.01)
                continue
            
            # 增加帧计数
            current_frame += 1
            
            # 📊 性能监控：记录每帧处理情况
            perf_monitor.add_counter('frames_processed')
                
            all_frame_detections = []
            perf_monitor.start_timer('frame_processing')
            
            for camera_id, result in current_frame_results.items():
                perf_monitor.start_timer(f'camera_{camera_id}_processing')
                
                # 1. 类别过滤：只跟踪车辆，排除非车辆类别
                raw_detections = [d for d in result['detections'] 
                                if d['class'] in Config.VEHICLE_CLASSES and d['class'] not in Config.EXCLUDE_CLASSES]
                # 📊 性能监控：记录检测数量
                perf_monitor.add_counter('detections_processed', len(raw_detections))
                
                # 2. NMS 和 跟踪器输入格式转换
                # 性能优化：批量数据转换，避免循环，减少2-3ms处理时间
                perf_monitor.start_timer('nms_processing')
                det_for_nms = [{'box': d['box'], 'confidence': d['confidence'], 'class': d['class']} for d in raw_detections]
                nms_detections = DetectionUtils.non_max_suppression(det_for_nms)
                perf_monitor.end_timer('nms_processing')
                
                # 3. 区域过滤 (先过滤后跟踪)
                perf_monitor.start_timer('area_filtering')
                filtered_nms_detections = filter_by_detect_areas(nms_detections, detect_areas[camera_id])
                perf_monitor.end_timer('area_filtering')
                
                # 4. 使用批量处理函数，提升性能
                perf_monitor.start_timer('tracker_input_preparation')
                tracker_input_tensor, box_to_class = batch_prepare_tracker_input(filtered_nms_detections)
                perf_monitor.end_timer('tracker_input_preparation')
                
                # 限制最大跟踪目标数量，避免匈牙利算法性能问题
                if len(tracker_input_tensor) > 50:
                    tracker_input_tensor = tracker_input_tensor[:50]
                    logger.warning(f"C{camera_id} 目标数量过多({len(filtered_nms_detections)})，限制为50个")

                # 5. 局部跟踪 (BYTETracker)
                perf_monitor.start_timer(f'tracker_update_{camera_id}')
                img_info = [Config.IMAGE_HEIGHT, Config.IMAGE_WIDTH]  # [height, width]
                img_size = (Config.IMAGE_HEIGHT, Config.IMAGE_WIDTH)  # (width, height)
                
                # DEBUG: 检查tracker输入
                debug_input_count = len(tracker_input_tensor) if tracker_input_tensor is not None else 0
                tracker_input_stats[camera_id]['total'] += 1
                tracker_input_stats[camera_id]['total_dets'] += debug_input_count
                if debug_input_count == 0:
                    tracker_input_stats[camera_id]['empty'] += 1
                else:
                    tracker_input_stats[camera_id]['non_empty'] += 1
                
                # 打印跟踪前的调试信息
                # 🔧 优化：DEBUG输出从每100帧改为每500帧
                if current_frame % 500 == 0:
                    logger.debug(f"C{camera_id} F{current_frame} 跟踪前: 原始={len(raw_detections)}, NMS={len(nms_detections)}, 过滤={len(filtered_nms_detections)}, 输入={debug_input_count}")
                
                tracked_objects = trackers[camera_id].update(tracker_input_tensor, img_info, img_size)
                tracker_time = perf_monitor.end_timer(f'tracker_update_{camera_id}')
                # 📊 性能监控：记录每次跟踪器更新
                perf_monitor.add_counter('tracker_updates')
                
                perf_monitor.record_fusion_stats(f'tracker_update_{camera_id}', tracker_time, {
                    'input_count': len(filtered_nms_detections),
                    'output_count': len(tracked_objects)
                })

                # 6. 跟踪结果转换 (性能优化版)
                # 优化点：保留原始类别信息，提升跟踪精度，使用box_to_class映射 ✅ FIX
                tracked_detections = batch_convert_track_results(tracked_objects, result, camera_id, current_frame, filtered_nms_detections, box_to_class)
                
                # 7. 跨摄像头融合处理 - 新的融合逻辑
                # 从同步帧中获取时间戳
                timestamp = result.get('timestamp', None)
                global_targets, local_targets = fusion_system.process_detections(tracked_detections, camera_id, timestamp, perf_monitor)
                
                # 存储此帧的全局和本地目标
                all_frame_detections.append({
                    'camera_id': camera_id,
                    'global_targets': global_targets,
                    'local_targets': local_targets
                })
                
                camera_processing_time = perf_monitor.end_timer(f'camera_{camera_id}_processing')
                # 📊 性能监控：记录每帧的摄像头处理统计
                perf_monitor.record_fusion_stats(f'camera_{camera_id}_processing', camera_processing_time, {
                    'raw_detections': len(raw_detections),
                    'tracked_detections': len(tracked_detections),
                    'filtered_detections': len(filtered_nms_detections),
                    'global_targets': len(global_targets),
                    'local_targets': len(local_targets)
                })
            
            frame_processing_time = perf_monitor.end_timer('frame_processing')
            
            # 收集所有全局和本地目标
            all_global_targets = []
            all_local_targets = []
            for frame_data in all_frame_detections:
                all_global_targets.extend(frame_data['global_targets'])
                all_local_targets.extend(frame_data['local_targets'])
            
            # C. 跨摄像头融合：匹配全局和本地目标
            perf_monitor.start_timer('matching_processing')
            active_global_targets = list(fusion_system.global_targets.values())
            fusion_system._perform_matching(all_local_targets, active_global_targets, perf_monitor)
            
            # 更新全局状态
            fusion_system.update_global_state(all_global_targets, all_local_targets)
            matching_time = perf_monitor.end_timer('matching_processing')

            # D. 流式加载并处理雷达数据 (按需加载，无需预加载所有数据)
            # 🔧 优化：只在有新视觉帧时才读取一个雷达帧，避免内循环持续读取
            if radar_fusion_enabled and radar_stream_iterator is not None:
                try:
                    # 只读取一个雷达帧（避免在单个视觉帧内读取多个雷达帧导致性能下降）
                    try:
                        radar_timestamp, radar_objects = next(radar_stream_iterator)
                        
                        # 添加到所有处理器的缓冲区（融合处理器会在process_frame时使用）
                        for processor in radar_fusion_processors.values():
                            processor.add_radar_data(radar_timestamp, radar_objects)
                    
                    except StopIteration:
                        # 雷达数据读完了
                        radar_stream_iterator = None
                except Exception as e:
                    logger.warning(f"流式加载雷达数据失败: {e}")
            
            # D. 雷达融合处理 (使用协调器)
            radar_id_map = {}
            direct_radar_outputs = []
            
            if radar_fusion_enabled and radar_fusion_orchestrator:
                perf_monitor.start_timer('radar_fusion_processing')  # 🔧 新增：计时开始
                radar_id_map, direct_radar_outputs = radar_fusion_orchestrator.process_radar_fusion(
                    current_frame, current_frame_results,
                    all_global_targets, all_local_targets,
                    perf_monitor
                )
                radar_fusion_time = perf_monitor.end_timer('radar_fusion_processing')  # 🔧 新增：计时结束
                
                # 更新统计信息
                radar_direct_output_count += len(direct_radar_outputs)
                radar_fusion_count += sum(1 for _ in radar_id_map.values())
            
            # D.1 添加单路处理结果到缓冲区（存储已融合的GlobalTarget）
            perf_monitor.start_timer('store_single_camera_results')
            for camera_id in [1, 2, 3]:
                if camera_id in current_frame_results:
                    result = current_frame_results[camera_id]
                    original_timestamp = result.get('timestamp', time.time())
                    
                    # 将字符串时间戳转换为浮点数
                    if isinstance(original_timestamp, str):
                        try:
                            from datetime import datetime
                            try:
                                dt = datetime.strptime(original_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                            except ValueError:
                                parts = original_timestamp.split('.')
                                if len(parts) == 2:
                                    second_part = parts[0]
                                    ms_part = parts[1]
                                    us_part = ms_part.ljust(6, '0')
                                    ts_with_us = f"{second_part}.{us_part}"
                                    dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                                else:
                                    raise ValueError("时间戳格式错误")
                            original_timestamp = dt.timestamp()
                        except Exception as e:
                            logger.warning(f"时间戳转换失败: {e}，使用当前时间")
                            original_timestamp = time.time()
                    
                    # 获取该摄像头的全局目标（已融合的最终结果）
                    camera_global_targets = [t for t in all_global_targets if t.camera_id == camera_id]
                    
                    # 获取该摄像头的radar_ids（使用global_id作为key）
                    camera_radar_ids = {t.global_id: radar_id_map.get(t.global_id) for t in camera_global_targets}
                    
                    # 添加到结果缓冲区（替代原来的 fusion_system.store_single_camera_result）
                    result_output_manager.add_single_camera_result(
                        camera_id, original_timestamp, camera_global_targets, camera_radar_ids
                    )
            
            perf_monitor.end_timer('store_single_camera_results')
            
            # D.2 每一帧都处理缓冲区中的结果（实时输出三路融合结果）
            perf_monitor.start_timer('result_buffer_processing')
            
            # 添加直接输出的雷达数据到处理器
            if direct_radar_outputs:
                result_output_manager.add_radar_data(direct_radar_outputs)
            
            # 每一帧都尝试处理缓冲区中的结果
            output_count = 0
            while result_output_manager.process_and_output():
                output_count += 1
            
            if output_count > 0:
                logger.info(f"Frame {current_frame}: 输出 {output_count} 组三路融合结果")
            
            # 独立输出融合区外的雷达数据（不依赖三路匹配）
            if result_output_manager.output_pending_radar_data():
                logger.info(f"Frame {current_frame}: 输出融合区外的雷达直接数据")
            
            # 定期记录缓冲区状态（每100帧）
            if current_frame > 0 and current_frame % 100 == 0:
                buffer_status = result_output_manager.get_buffer_status()
                logger.info(f"缓冲区状态: C1={buffer_status['c1_size']} "
                           f"C2={buffer_status['c2_size']} C3={buffer_status['c3_size']}")
            
            result_buffer_time = perf_monitor.end_timer('result_buffer_processing')
            
            # ✅ 关键修改：不再生成每帧的JSON
            # 现在只通过 ResultBuffer 的三路匹配输出结果
            # 之前的 generate_json_data() 流程已被替代
            perf_monitor.start_timer('json_mqtt_processing')
            
            # 保留 fusion_system.next_frame() 用于内部计数
            fusion_system.next_frame()
            
            json_mqtt_time = perf_monitor.end_timer('json_mqtt_processing')
            
            # 🔧 新增：记录整个帧的处理时间
            # 注意：perf_monitor.end_timer() 返回的是毫秒，需要转换回秒
            total_frame_time = time.time() - frame_start_time
            frame_times.append(total_frame_time)
            component_times['queue_processing'].append(queue_processing_time / 1000.0)  # 转秒
            component_times['frame_processing'].append(frame_processing_time / 1000.0)
            component_times['matching_processing'].append(matching_time / 1000.0)
            if radar_fusion_enabled:
                # 🔧 修复：从 perf_data 中读取雷达融合处理时间
                try:
                    radar_fusion_time_ms = perf_monitor.perf_data.get('radar_fusion_processing', {}).get('total_ms', 0)
                    component_times['radar_fusion'].append(radar_fusion_time_ms / 1000.0)
                except:
                    component_times['radar_fusion'].append(0)
            # 获取存储单路结果的时间（检查perf_monitor中是否有记录）
            try:
                store_time_ms = perf_monitor.perf_data.get('store_single_camera_results', {}).get('total_ms', 0)
                component_times['store_single_camera'].append(store_time_ms / 1000.0)
            except:
                component_times['store_single_camera'].append(0)
            component_times['result_buffer'].append(result_buffer_time / 1000.0)
            # 获取JSON和MQTT处理时间
            try:
                json_time_ms = perf_monitor.perf_data.get('json_mqtt_processing', {}).get('total_ms', 0)
                component_times['json_mqtt'].append(json_time_ms / 1000.0)
            except:
                component_times['json_mqtt'].append(0)
            component_times['total_frame'].append(total_frame_time)
            
            # 🔧 新增：每100帧输出一次性能统计
            if current_frame > 0 and current_frame % 100 == 0:
                logger.info("="*70)
                logger.info(f"📊 性能统计 (截至Frame {current_frame}, 最近100帧)")
                logger.info("="*70)
                
                # 计算平均耗时
                if frame_times:
                    avg_total = mean(frame_times)
                    fps_actual = 1.0 / avg_total if avg_total > 0 else 0
                    logger.info(f"总处理时间: {avg_total*1000:.2f}ms/帧 (实际FPS: {fps_actual:.1f})")
                    
                    if component_times['queue_processing']:
                        logger.info(f"  ├─ 队列读取: {mean(component_times['queue_processing'])*1000:.2f}ms")
                    if component_times['frame_processing']:
                        logger.info(f"  ├─ 帧处理(跟踪+融合): {mean(component_times['frame_processing'])*1000:.2f}ms")
                    if component_times['matching_processing']:
                        logger.info(f"  ├─ 匹配处理: {mean(component_times['matching_processing'])*1000:.2f}ms")
                    if radar_fusion_enabled and component_times['radar_fusion']:
                        avg_radar = mean(component_times['radar_fusion'])
                        if avg_radar > 0:
                            logger.info(f"  ├─ 雷达融合: {avg_radar*1000:.2f}ms")
                    if component_times['store_single_camera']:
                        avg_store = mean(component_times['store_single_camera'])
                        if avg_store > 0:
                            logger.info(f"  ├─ 单路存储: {avg_store*1000:.2f}ms")
                    if component_times['result_buffer']:
                        logger.info(f"  ├─ 结果缓冲: {mean(component_times['result_buffer'])*1000:.2f}ms")
                    if component_times['json_mqtt']:
                        avg_json = mean(component_times['json_mqtt'])
                        if avg_json > 0:
                            logger.info(f"  └─ JSON/MQTT: {avg_json*1000:.2f}ms")
                    
                    # 实时性评估
                    required_fps = 30  # 目标30FPS
                    required_time = 1.0 / required_fps  # 约33.3ms
                    if avg_total > required_time:
                        logger.warning(f"⚠️  实时性告警：平均处理时间({avg_total*1000:.2f}ms) > 目标时间({required_time*1000:.1f}ms)")
                        logger.warning(f"    瓶颈可能在: 帧处理或匹配处理阶段")
                    else:
                        logger.info(f"✅ 可以达到 {fps_actual:.1f} FPS 的实时处理")
                
                logger.info("="*70)

            # D. 定期报告队列状态
            if current_frame > 0 and current_frame % 300 == 0:
                queue_sizes = {i: queues[i].qsize() for i in [1, 2, 3]}
                logger.info(f"队列状态 (截至帧 {current_frame})")
                for cam_id in [1, 2, 3]:
                    logger.info(f"C{cam_id}: 队列大小 {queue_sizes[cam_id]}")
        
        logger.info("所有处理完成")
        
        # 5. 保存融合结果（正常退出时）
        try:
            json_count = len(fusion_system.json_output_data) if fusion_system.json_output_data else 0
            logger.info(f"准备保存 {json_count} 帧的JSON数据")
            if json_count > 0:
                fusion_system.save_json_data("output_fusion_refactored.json")
            else:
                logger.warning("JSON数据列表为空")
        except Exception as e:
            logger.error(f"保存JSON数据失败: {e}")
            import traceback
            traceback.print_exc()
        
        # 6. 输出最终处理统计
        logger.info("="*60)
        logger.info("最终处理统计报告")
        
        processed_frames_count = fusion_system.frame_count
        logger.info(f"成功处理的帧数: {processed_frames_count}帧")
        
        # 雷达数据统计
        logger.info("="*60)
        logger.info("雷达数据处理统计")
        logger.info(f"直接输出的雷达数据: {radar_direct_output_count} 个")
        logger.info(f"送入融合系统的雷达数据: {radar_fusion_count} 个")
        logger.info(f"雷达数据总计: {radar_direct_output_count + radar_fusion_count} 个")
            
        logger.info("="*60)
        
        # 输出最终跟踪器优化统计
        logger.info("最终ByteTracker优化统计")
        logger.info("DEBUG - 跟踪器输入统计:")
        for cam_id in [1, 2, 3]:
            stats = tracker_input_stats[cam_id]
            avg_dets = stats['total_dets'] / max(stats['total'], 1)
            logger.info(f"C{cam_id}: 调用{stats['total']}次, 空输入{stats['empty']}次, 非空{stats['non_empty']}次, 总检测数{stats['total_dets']}, 平均{avg_dets:.1f}个/帧")
        
        for cam_id, tracker in trackers.items():
            stats = tracker.get_performance_stats()
            perf_improvement = stats.get('performance_improvement', 1.0)
            avg_tracking_time = stats.get('avg_tracking_time', 0.0)
            avg_prediction_time = stats.get('avg_prediction_time', 0.0)
            logger.info(f"C{cam_id}:")
            logger.info(f"  总帧数: {stats['total_frames']}")
            logger.info(f"  跟踪帧: {stats['tracking_frames']} ({stats['tracking_frames']/max(stats['total_frames'],1)*100:.1f}%)")
            logger.info(f"  预测帧: {stats['prediction_only_frames']} ({stats['prediction_only_frames']/max(stats['total_frames'],1)*100:.1f}%)")
            logger.info(f"  性能提升: {perf_improvement:.2f}x")
            logger.info(f"  平均跟踪耗时: {avg_tracking_time:.3f}s")
            logger.info(f"  平均预测耗时: {avg_prediction_time:.3f}s")
        logger.info("="*60)
    except Exception as e:
        logger.error(f"主程序执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 刷新所有缓冲区中的结果
        logger.info("程序结束，刷新缓冲区...")
        result_output_manager.flush_all()
        
        # 🔧 打印雷达融合统计信息
        if radar_fusion_orchestrator and hasattr(radar_fusion_orchestrator, 'print_overall_statistics'):
            try:
                radar_fusion_orchestrator.print_overall_statistics()
            except Exception as e:
                logger.warning(f"打印雷达融合统计失败: {e}")
                import traceback
                traceback.print_exc()
        
        # 🔧 修复：在finally块中保存JSON，确保即使异常退出也能保存数据
        logger.info("正在保存JSON数据")
        try:
            json_count = len(fusion_system.json_output_data) if fusion_system.json_output_data else 0
            logger.info(f"准备保存 {json_count} 帧的JSON数据")
            if json_count > 0:
                fusion_system.save_json_data("output_fusion_refactored.json")
            else:
                logger.warning("JSON数据列表为空，没有数据可保存")
                logger.warning("可能原因: 1) 程序异常退出 2) 没有检测到目标 3) 数据未正确添加")
        except Exception as e:
            logger.error(f"在finally块中保存JSON失败: {e}")
            import traceback
            traceback.print_exc()
        
        # 7. 清理资源
        camera_manager.stop_all_cameras()

        # ⏱️ 打印详细的每一步时间统计
        logger.info("=" * 70)
        logger.info("📊 详细的处理时间统计")
        logger.info("=" * 70)
        
        # 计算总体统计（基于最后的component_times）
        if 'component_times' in locals() and component_times:
            logger.info("各处理阶段平均耗时（最近100帧）：")
            
            # 队列处理时间
            if component_times['queue_processing']:
                queue_avg = mean(component_times['queue_processing']) * 1000
                queue_max = max(component_times['queue_processing']) * 1000
                queue_min = min(component_times['queue_processing']) * 1000
                logger.info(f"  ├─ 队列处理: 平均 {queue_avg:.2f}ms | 范围 [{queue_min:.2f}ms - {queue_max:.2f}ms]")
            
            # 帧处理时间（包含跟踪和融合）
            if component_times['frame_processing']:
                frame_avg = mean(component_times['frame_processing']) * 1000
                frame_max = max(component_times['frame_processing']) * 1000
                frame_min = min(component_times['frame_processing']) * 1000
                logger.info(f"  ├─ 帧处理(检测/跟踪/融合): 平均 {frame_avg:.2f}ms | 范围 [{frame_min:.2f}ms - {frame_max:.2f}ms]")
            
            # 匹配处理时间
            if component_times['matching_processing']:
                match_avg = mean(component_times['matching_processing']) * 1000
                match_max = max(component_times['matching_processing']) * 1000
                match_min = min(component_times['matching_processing']) * 1000
                logger.info(f"  ├─ 匹配处理(跨摄像头融合): 平均 {match_avg:.2f}ms | 范围 [{match_min:.2f}ms - {match_max:.2f}ms]")
            
            # 雷达融合时间
            if radar_fusion_enabled and component_times['radar_fusion']:
                radar_avg = mean(component_times['radar_fusion']) * 1000
                radar_max = max(component_times['radar_fusion']) * 1000
                radar_min = min(component_times['radar_fusion']) * 1000
                logger.info(f"  ├─ 雷达融合处理: 平均 {radar_avg:.2f}ms | 范围 [{radar_min:.2f}ms - {radar_max:.2f}ms]")
            
            # 单路存储处理时间（新增）
            if component_times['store_single_camera']:
                store_avg = mean(component_times['store_single_camera']) * 1000
                store_max = max(component_times['store_single_camera']) * 1000
                store_min = min(component_times['store_single_camera']) * 1000
                if store_avg > 0:
                    logger.info(f"  ├─ 单路结果存储: 平均 {store_avg:.2f}ms | 范围 [{store_min:.2f}ms - {store_max:.2f}ms]")
            
            # 结果缓冲处理时间
            if component_times['result_buffer']:
                buffer_avg = mean(component_times['result_buffer']) * 1000
                buffer_max = max(component_times['result_buffer']) * 1000
                buffer_min = min(component_times['result_buffer']) * 1000
                logger.info(f"  ├─ 结果缓冲处理: 平均 {buffer_avg:.2f}ms | 范围 [{buffer_min:.2f}ms - {buffer_max:.2f}ms]")
            
            # JSON和MQTT处理时间（新增）
            if component_times['json_mqtt']:
                json_avg = mean(component_times['json_mqtt']) * 1000
                json_max = max(component_times['json_mqtt']) * 1000
                json_min = min(component_times['json_mqtt']) * 1000
                if json_avg > 0:
                    logger.info(f"  └─ JSON/MQTT处理: 平均 {json_avg:.2f}ms | 范围 [{json_min:.2f}ms - {json_max:.2f}ms]")
            
            # 总处理时间
            if component_times['total_frame']:
                total_avg = mean(component_times['total_frame']) * 1000
                total_max = max(component_times['total_frame']) * 1000
                total_min = min(component_times['total_frame']) * 1000
                actual_fps = 1000.0 / total_avg if total_avg > 0 else 0
                logger.info(f"\n  总处理时间: 平均 {total_avg:.2f}ms | 范围 [{total_min:.2f}ms - {total_max:.2f}ms] | 实际FPS: {actual_fps:.1f}")
        
        logger.info("=" * 70)

        # 断开MQTT连接
        if mqtt_publisher:
            try:
                mqtt_publisher.disconnect()
                logger.info("MQTT连接已断开")
            except:
                pass
        
        # 8. 记录程序结束信息
        logger.info("=" * 70)
        logger.info("融合系统已停止")
        logger.info(f"日志文件: {os.path.abspath('logs/fusion_system.log')}")
        logger.info("=" * 70)
                
        logger.info("资源清理完成")