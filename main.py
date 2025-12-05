#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDK版多摄像头融合系统 - 重构版
职责分离: 
1. 子进程 (yolov5_SDK): 仅负责视频读取、SDK推理、结果入队列 (使用 SDKinfer_ffmpeg.py)
2. 主进程 (main): 负责跟踪(BYTETracker)、区域过滤、跨摄像头融合、帧同步 
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

# 配置logging - 只输出到文件，不输出到终端
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('fusion_system.log', mode='w', encoding='utf-8')  # mode='w' 每次运行时清空日志
    ]
)
logger = logging.getLogger(__name__)

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
    from rtsp_reader import RTSPStreamReader
    from mqtt_publisher import MqttPublisher  
    from config_reader import ConfigReader
    RTSP_MQTT_AVAILABLE = True
except ImportError as e:
    logger.warning(f"无法导入RTSP/MQTT模块: {e}, 将使用本地视频模式")
    RTSP_MQTT_AVAILABLE = False

# 🔧 移除FFmpeg相关导入，改用直接计算时间戳
# from Timestamp_sync import FFmpegTimeStampProvider, FFmpegTimestampFrameSynchronizer
from Basic import Config, DetectionUtils, GeometryUtils, PerformanceMonitor
from TargetTrack import TargetBuffer
from Fusion import CrossCameraFusion
from RadarVisionFusion import RadarVisionFusionProcessor, RadarDataLoader, OutputObject
from CameraManager import CameraManager

# 创建共享布尔值用于停止运行线程
cancel_flag = multiprocessing.Value('b', False)
# --- 辅助函数 (从SDKinfer.py移过来) ---

def cancel_process(signum, frame):
    """取消处理信号"""
    global cancel_flag
    cancel_flag.value = True
    logger.info("收到停止信号，正在退出...")

def filter_by_detect_areas(detections: List[dict], areas: List[np.ndarray]) -> List[dict]:
    """根据检测区域过滤检测结果"""
    filtered_detections = []
    for detection in detections:
        # 使用边界框的底部中心点作为判断点
        x1, y1, x2, y2 = detection['box']
        center_x, center_y = int((x1 + x2) / 2), int(y2) 
        in_detect_area = any(cv2.pointPolygonTest(area, (center_x, center_y), False) >= 0 
                           for area in areas)
        if in_detect_area:
            filtered_detections.append(detection)
    return filtered_detections

def batch_prepare_tracker_input(nms_detections: List[dict]) -> Tuple[np.ndarray, Dict[str, str]]:
    """
    批量准备跟踪器输入，优化性能 - 使用numpy避免torch依赖
    同时返回检测框到类别的映射字典
    """
    if not nms_detections:
        return np.empty((0, 5), dtype=np.float32), {}
    
    # 批量提取数据，避免循环
    # BYTETracker 期望的格式: [x1, y1, x2, y2, score]
    boxes_scores = np.array([[d['box'][0], d['box'][1], d['box'][2], d['box'][3], d['confidence']] 
                              for d in nms_detections], dtype=np.float32)
    
    # 创建映射字典：key为box的字符串表示，value为类别名称
    # 这样可以通过box坐标快速查找对应的类别
    box_to_class = {}
    for i, det in enumerate(nms_detections):
        box_key = f"{det['box'][0]:.1f}_{det['box'][1]:.1f}_{det['box'][2]:.1f}_{det['box'][3]:.1f}"
        box_to_class[box_key] = det['class']
    
    # 跟踪器只需要前5列：[x1, y1, x2, y2, score]
    tracker_input_array = boxes_scores.astype(np.float32)
    return tracker_input_array, box_to_class

def batch_convert_track_results(tracked_objects: List, result: dict, camera_id: int, current_frame: int, 
                               original_detections: List[dict] = None, box_to_class: Dict[str, str] = None) -> List[dict]:
    """
    批量转换跟踪结果，优化性能并保留原始类别信息
    box_to_class: 检测框到类别的映射字典
    """
    tracked_detections = []
    
    # 添加调试信息
    if current_frame % 100 == 0 and len(tracked_objects) > 0:
        logger.debug(f"C{camera_id} Frame {current_frame}: {len(tracked_objects)} tracked objects")
    
    for track in tracked_objects:
        # 高效转换tlwh到tlbr
        tlwh = track.tlwh
        tlbr = [tlwh[0], tlwh[1], tlwh[0] + tlwh[2], tlwh[1] + tlwh[3]]
        
        # 尝试从box_to_class映射中获取类别信息（最准确的方法）
        class_name = 'car'  # 默认值
        
        if box_to_class:
            # 尝试通过IoU匹配找到对应的原始检测
            best_iou = 0
            best_class = None
            best_box_key = None
            
            for box_key, class_str in box_to_class.items():
                # 解析box_key
                coords = box_key.split('_')
                if len(coords) == 4:
                    try:
                        orig_box = [float(c) for c in coords]
                        iou = GeometryUtils.calculate_iou(tlbr, orig_box)
                        if iou > best_iou and iou > 0.1:  # 降低IoU阈值
                            best_iou = iou
                            best_class = class_str
                            best_box_key = box_key
                    except:
                        continue
            
            if best_class:
                class_name = best_class
                if box_to_class and best_box_key:
                    # 删除已使用的映射，避免重复使用
                    del box_to_class[best_box_key]
            else:
                # 如果IoU匹配失败，尝试通过距离匹配
                min_distance = float('inf')
                best_class_dist = None
                
                for box_key, class_str in box_to_class.items():
                    coords = box_key.split('_')
                    if len(coords) == 4:
                        try:
                            orig_box = [float(c) for c in coords]
                            orig_center = [(orig_box[0] + orig_box[2]) / 2, (orig_box[1] + orig_box[3]) / 2]
                            track_center = [(tlbr[0] + tlbr[2]) / 2, (tlbr[1] + tlbr[3]) / 2]
                            distance = ((orig_center[0] - track_center[0]) ** 2 + 
                                       (orig_center[1] - track_center[1]) ** 2) ** 0.5
                            
                            if distance < min_distance and distance < 100:  # 100像素内的最近匹配
                                min_distance = distance
                                best_class_dist = class_str
                        except:
                            continue
                
                if best_class_dist:
                    class_name = best_class_dist
        
        elif original_detections:
            # 备选方案：使用原始检测列表
            best_iou = 0
            best_match = None
            for orig_det in original_detections:
                iou = GeometryUtils.calculate_iou(tlbr, orig_det['box'])
                if iou > best_iou and iou > 0.1:
                    best_iou = iou
                    best_match = orig_det
            
            if best_match:
                class_name = best_match['class']
            else:
                # 距离匹配
                min_distance = float('inf')
                for orig_det in original_detections:
                    orig_center = [(orig_det['box'][0] + orig_det['box'][2]) / 2, 
                                  (orig_det['box'][1] + orig_det['box'][3]) / 2]
                    track_center = [(tlbr[0] + tlbr[2]) / 2, (tlbr[1] + tlbr[3]) / 2]
                    distance = ((orig_center[0] - track_center[0]) ** 2 + 
                               (orig_center[1] - track_center[1]) ** 2) ** 0.5
                    
                    if distance < min_distance and distance < 100:
                        min_distance = distance
                        class_name = orig_det['class']
        
        # 计算目标底部中心点（用于融合区域判断）
        center_x = int((tlbr[0] + tlbr[2]) / 2)
        center_y = int(tlbr[3])
        pixel_point = (center_x, center_y)
        
        # 检查是否在雷视融合区域内
        in_fusion_area = GeometryUtils.is_in_radar_vision_fusion_area(pixel_point, camera_id)
        
        detection = {
            'box': tlbr,
            'confidence': track.score,
            'class': class_name,  # 保留原始类别信息
            'track_id': track.track_id,
            'local_id': track.track_id,
            'center_point': [(tlbr[0] + tlbr[2]) / 2, (tlbr[1] + tlbr[3]) / 2],
            'timestamp': result.get('timestamp', time.time()),
            'camera_id': camera_id,
            'in_fusion_area': in_fusion_area  # 新增：标记是否在融合区域内
        }
        tracked_detections.append(detection)
    
    return tracked_detections

# 🔧 修改：使用初始视频时间 + frame_id/fps 计算时间戳
def create_sdk_worker_process(camera_id: int, video_path: str, result_queue: multiprocessing.Queue):
    """创建并运行一个独立的 SDK 推理子进程 (生产者)"""
    
    # 确保子进程能找到 SDKinfer 模块
    from SDKinfer import yolov5_SDK, infer_process_attr
    from Basic import Config
    
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
        3: [np.array([[70, 720], [1030, 720], [934, 166], [90, 166]], dtype=np.int32)]}
        
    signal.signal(signal.SIGINT, cancel_process)
    
    # 2. 初始化核心组件
    # 【新增】指定融合策略：'original' 或 'improved'
    fusion_strategy = "improved"  # 可改为 "original" 使用原始融合逻辑
    fusion_system = CrossCameraFusion(fusion_strategy=fusion_strategy)
    perf_monitor = PerformanceMonitor()
    logger.info(f"使用融合策略: {fusion_strategy}")
    
    # 2.0 初始化摄像头管理器
    camera_manager = CameraManager(video_paths, cancel_flag)
    queues = camera_manager.create_queues(maxsize=10)
    
    # 2.1 初始化雷达融合模块
    logger.info("初始化雷达融合模块")
    radar_fusion_enabled = False
    radar_data_loader = None
    radar_fusion_processors = {}  # 按摄像头存储融合处理器
    
    # 雷达数据文件路径 (可配置)
    radar_data_path = '/root/yolov5-7.0_lyngor1.17.0/project-simple-video/videos/radar_data.jsonl'
        
    try:
        if os.path.exists(radar_data_path):
            # 初始化雷达数据加载器
            radar_data_loader = RadarDataLoader(radar_data_path)
            if radar_data_loader.load():
                # 为每个摄像头初始化独立的融合处理器
                for camera_id in [1, 2, 3]:
                    radar_fusion_processors[camera_id] = RadarVisionFusionProcessor(
                        fusion_area_geo=None,  # 使用融合区域判断已在GlobalID分配时完成
                        lat_offset=-0.00000165,
                        lon_offset=0.0000450
                    )
                    
                    # 将该摄像头的雷达数据添加到对应的处理器
                    camera_timestamps = radar_data_loader.get_camera_timestamps(camera_id)
                    for ts in camera_timestamps:
                        radar_objs = radar_data_loader.get_radar_data_by_camera(camera_id, ts)
                        radar_fusion_processors[camera_id].add_radar_data(ts, radar_objs)
                    
                    logger.info(f"C{camera_id} 雷达融合处理器初始化成功, 雷达数据帧数: {len(camera_timestamps)}")
                
                radar_fusion_enabled = True
                logger.info(f"雷达融合模块初始化成功")
            else:
                logger.warning("雷达数据加载失败，将不使用雷达融合")
        else:
            logger.warning(f"雷达数据文件不存在: {radar_data_path}")
            logger.warning("将不使用雷达融合功能")
    except Exception as e:
        logger.warning(f"雷达融合模块初始化失败: {e}")
        logger.warning("将不使用雷达融合功能")
        radar_fusion_enabled = False
    
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
    
    # 🔧 时间戳配置：完全基于时间戳同步，不依赖帧号
    logger.info("同步方式: 纯时间戳同步 (不依赖帧号)")
    
    # 🔧 改造：移除帧同步，改为单路独立处理
    logger.info("融合主循环启动 - 单路处理模式")
    logger.info("处理模式: 单路独立处理 + 后期三路匹配")
    logger.info("="*60)
    
    # 初始化单路结果存储
    camera_results = {1: [], 2: [], 3: []}  # 存储每个摄像头的处理结果

    try:
        current_frame = 0
        radar_id_map = {}  # 全局radar_id_map
        
        while not cancel_flag.value:
            
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

            # D. 雷达融合处理 (按摄像头同步融合)
            radar_id_map = {}
            if radar_fusion_enabled and radar_fusion_processors:
                perf_monitor.start_timer('radar_fusion_processing')
                
                # 按摄像头进行雷达融合
                for camera_id in [1, 2, 3]:
                    if camera_id not in radar_fusion_processors:
                        continue
                    
                    # 收集该摄像头的所有目标
                    vision_objects = []
                    
                    # 处理全局目标
                    for global_target in all_global_targets:
                        if global_target.camera_id != camera_id:
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
                        confidence = global_target.confidence_history[-1] if global_target.confidence_history else 0.0
                        
                        vision_obj = OutputObject(
                            timestamp="",
                            cameraid=global_target.camera_id,
                            type_name=global_target.class_name,
                            confidence=confidence,
                            track_id=global_target.global_id,
                            lon=lng,
                            lat=lat
                        )
                        vision_objects.append(vision_obj)
                    
                    # 处理本地目标 (已匹配的)
                    for local_target in all_local_targets:
                        if local_target.camera_id != camera_id:
                            continue
                        if not local_target.matched_global_id:
                            continue
                        
                        if local_target.current_bev_pos[0] == 0.0 and local_target.current_bev_pos[1] == 0.0:
                            continue
                        
                        geo_result = GeometryUtils.bev_to_geo(local_target.current_bev_pos[0], local_target.current_bev_pos[1])
                        if not geo_result:
                            continue
                        
                        lng, lat = geo_result
                        
                        # 检查是否已经添加过这个 global_id
                        if not any(v.track_id == local_target.matched_global_id for v in vision_objects):
                            vision_obj = OutputObject(
                                timestamp="",
                                cameraid=local_target.camera_id,
                                type_name=local_target.class_name,
                                confidence=local_target.confidence,
                                track_id=local_target.matched_global_id,
                                lon=lng,
                                lat=lat
                            )
                            vision_objects.append(vision_obj)
                    
                    # 执行该摄像头的雷达融合 - 使用原始时间戳
                    if vision_objects:
                        # 获取该摄像头的原始时间戳
                        if camera_id in current_frame_results:
                            result = current_frame_results[camera_id]
                            original_timestamp = result.get('timestamp', time.time())
                        else:
                            # 如果没有该摄像头的结果，使用当前时间
                            logger.warning(f"C{camera_id} 没有当前帧结果，使用当前时间作为时间戳")
                            original_timestamp = time.time()
                        if isinstance(original_timestamp, str):
                            # 如果是字符串，需要转换为浮点数
                            # 注意：时间戳格式是 'YYYY-MM-DD HH:MM:SS.mmm' (3位毫秒，不是6位微秒)
                            try:
                                from datetime import datetime
                                # 方法1：先尝试3位毫秒格式
                                try:
                                    dt = datetime.strptime(original_timestamp, '%Y-%m-%d %H:%M:%S.%f')
                                except ValueError:
                                    # 方法2：如果失败，说明可能是3位毫秒，需要补充到6位
                                    # 分割秒和毫秒部分
                                    parts = original_timestamp.split('.')
                                    if len(parts) == 2:
                                        second_part = parts[0]
                                        ms_part = parts[1]
                                        # 补充到6位微秒
                                        us_part = ms_part.ljust(6, '0')
                                        ts_with_us = f"{second_part}.{us_part}"
                                        dt = datetime.strptime(ts_with_us, '%Y-%m-%d %H:%M:%S.%f')
                                    else:
                                        raise ValueError("时间戳格式错误")
                                original_timestamp = dt.timestamp()
                            except Exception as e:
                                logger.warning(f"时间戳转换失败: {original_timestamp}, 错误: {e}")
                                original_timestamp = time.time()
                        
                        updated_vision_objects = radar_fusion_processors[camera_id].process_frame(original_timestamp, vision_objects)
                        
                        # 构建 radar_id_map (key使用global_id，即vision_obj.track_id)
                        # vision_obj.track_id 已经是 global_id（见上面创建vision_objects的代码）
                        for vision_obj in updated_vision_objects:
                            if vision_obj.radar_id is not None:
                                # 直接使用 track_id 作为 key（track_id 就是 global_id）
                                radar_id_map[vision_obj.track_id] = vision_obj.radar_id
                                if current_frame % 100 == 0:
                                    logger.debug(f"Frame {current_frame} C{camera_id}: 雷达ID映射 track_id={vision_obj.track_id} -> radar_id={vision_obj.radar_id}")
                        
                        # 统计信息
                        matched_count = sum(1 for v in updated_vision_objects if v.radar_id is not None)
                        if current_frame % 100 == 0 and matched_count > 0:
                            logger.info(f"Frame {current_frame} C{camera_id}: 雷达匹配 {matched_count}/{len(updated_vision_objects)} 个目标，radar_id_map大小={len(radar_id_map)}")
                
                perf_monitor.end_timer('radar_fusion_processing')
            
            # D.1 存储单路处理结果，用于后期三路匹配
            perf_monitor.start_timer('store_single_camera_results')
            for camera_id in [1, 2, 3]:
                if camera_id in current_frame_results:
                    result = current_frame_results[camera_id]
                    original_timestamp = result.get('timestamp', time.time())
                    
                    # 获取该摄像头的本地目标
                    camera_local_targets = [t for t in all_local_targets if t.camera_id == camera_id]
                    
                    # 获取该摄像头的radar_ids
                    camera_radar_ids = {t.local_id: radar_id_map.get(t.local_id) for t in camera_local_targets}
                    
                    # 存储结果
                    fusion_system.store_single_camera_result(camera_id, original_timestamp, camera_local_targets, camera_radar_ids)
            
            perf_monitor.end_timer('store_single_camera_results')
            
            # D.2 定期进行三路匹配（每处理100帧）
            if current_frame > 0 and current_frame % 100 == 0:
                perf_monitor.start_timer('cross_camera_matching')
                try:
                    global_targets_from_matching, unmatched_local_targets = fusion_system.match_cross_camera_targets(time_window=0.5)
                    if global_targets_from_matching:
                        logger.info(f"Frame {current_frame}: 三路匹配找到 {len(global_targets_from_matching)} 个全局目标")
                except Exception as e:
                    logger.error(f"三路匹配异常: {e}")
                perf_monitor.end_timer('cross_camera_matching')
            
            # E. 生成JSON数据并尝试发送MQTT
            perf_monitor.start_timer('json_mqtt_processing')
            
            # 获取当前帧的时间戳（使用最新的摄像头时间戳）
            frame_timestamp = None
            for camera_id in [1, 2, 3]:
                if camera_id in current_frame_results:
                    result = current_frame_results[camera_id]
                    ts = result.get('timestamp', None)
                    if ts is not None:
                        frame_timestamp = ts
                        break
            
            perf_monitor.start_timer('json_generation')
            json_data = fusion_system.generate_json_data(all_global_targets, all_local_targets, radar_id_map, frame_timestamp)
            perf_monitor.end_timer('json_generation')
            
            # 检查是否为空帧（participants为空）
            participants = json_data.get('participant', [])
            if len(participants) == 0:
                # 空帧不输出，跳过MQTT发送和JSON保存
                perf_monitor.end_timer('json_mqtt_processing')
                fusion_system.next_frame()
                continue
            
            mqtt_sent = False
            if mqtt_publisher:
                perf_monitor.start_timer('mqtt_publish')
                try:
                    mqtt_sent = mqtt_publisher.publish_rsm(participants)
                    if mqtt_sent:
                        # 📊 性能监控：记录MQTT成功发送
                        perf_monitor.add_counter('mqtt_sends')
                    else:
                        # 📊 性能监控：记录MQTT发送失败
                        perf_monitor.add_counter('mqtt_failures')
                except Exception as e:
                    logger.error(f"MQTT发送异常: {e}")
                    perf_monitor.add_counter('mqtt_failures')
                finally:
                    perf_monitor.end_timer('mqtt_publish')
            
            # 🔧 修复：无论MQTT是否成功，都保存JSON数据（用于调试和备份）
            # 只保存非空帧
            fusion_system.json_output_data.append(json_data)          
            json_mqtt_time = perf_monitor.end_timer('json_mqtt_processing')       
            fusion_system.next_frame()

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

        # 断开MQTT连接
        if mqtt_publisher:
            try:
                mqtt_publisher.disconnect()
                logger.info("MQTT连接已断开")
            except:
                pass
                
        logger.info("资源清理完成")