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
from collections import defaultdict, deque
from statistics import mean, median
sys.path.append('/usr/local/lynxi/sdk/sdk-samples/python')

# 配置logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('fusion_system.log', encoding='utf-8')
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

from Timestamp_sync import FFmpegTimeStampProvider, FFmpegTimestampFrameSynchronizer
from Basic import Config, DetectionUtils, GeometryUtils, PerformanceMonitor
from TargetTrack import TargetBuffer
from Fusion import CrossCameraFusion
from FrameSynchronizer import FrameLossPrevention
from RadarVisionFusion import RadarVisionFusionProcessor, RadarDataLoader, OutputObject

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
        print(f"🔍 C{camera_id} Frame {current_frame}: {len(tracked_objects)} tracked objects, box_to_class={len(box_to_class) if box_to_class else 0}")
    
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
        
        detection = {
            'box': tlbr,
            'confidence': track.score,
            'class': class_name,  # 保留原始类别信息
            'track_id': track.track_id,
            'local_id': track.track_id,
            'center_point': [(tlbr[0] + tlbr[2]) / 2, (tlbr[1] + tlbr[3]) / 2],
            'timestamp': result.get('timestamp', time.time()),
            'frame_number': result.get('frame_number', current_frame),
            'camera_id': camera_id,
            'sync_id': result.get('sync_id', f"C{camera_id}_F{current_frame}")
        }
        tracked_detections.append(detection)
    
    return tracked_detections

# 🔧 修改：移除了 timestamp_provider 参数
def create_sdk_worker_process(camera_id: int, video_path: str, result_queue: multiprocessing.Queue):
    """创建并运行一个独立的 SDK 推理子进程 (生产者)"""
    
    # 确保子进程能找到 SDKinfer_ffmpeg 模块
    from SDKinfer_ffmpeg import yolov5_SDK, infer_process_attr
    
    try:
        print(f"🔧 Camera{camera_id} 子进程启动，准备初始化SDK...")
        print(f"   视频源: {video_path[:80] if len(video_path) > 80 else video_path}")
        
        attr = infer_process_attr()
        attr.url = video_path
        attr.device_id = 0
        attr.chan_id = camera_id - 1
        attr.plugin_path = "/usr/local/lynxi/sdk/sdk-samples/plugin/obj/libYolov5Plugin.so"
        attr.model_path = "/root/yolov5-7.0_lyngor1.17.0/best_yolov5s_onnx/Net_0/"
        
        print(f"🔧 Camera{camera_id} 初始化yolov5_SDK (V13_PyAV)...")
        print(f"   ⚠️  如果出现 'av.open' 错误，说明无法打开视频源")
        print(f"   请检查: 1) RTSP URL是否正确 2) 网络连接 3) 视频文件是否存在")
        
        # 🔧 改进：移除传递 timestamp_provider
        # 注意：SDK初始化可能会在这里失败，如果RTSP连接不可用
        worker = yolov5_SDK(attr, result_queue) 
        print(f"✅ Camera{camera_id} yolov5_SDK初始化成功")
        worker.run(cancel_flag) # 运行在子进程中
        print(f"✅ Camera{camera_id} 子进程正常退出")
        
    except Exception as e:
        error_msg = str(e)
        print(f"\n❌ Camera{camera_id} SDK进程启动或运行失败")
        print(f"   错误信息: {error_msg}")
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
                    print(f"📷 摄像头 {i}: {camera['name']} - {camera['rtsp_url']}")
                print("✅ 使用RTSP流作为输入")
            else:
                print("⚠️  配置文件中摄像头数量不足，回退到本地视频文件")
                raise Exception("配置不足")
        except Exception as e:
            print(f"⚠️  读取RTSP配置失败: {e}, 使用本地视频文件")
            RTSP_MQTT_AVAILABLE = False
    
    # 回退到本地视频文件
    if not RTSP_MQTT_AVAILABLE or not video_paths:
        print("使用本地视频文件")
        # 设置默认的本地视频文件路径 - 使用实际存在的视频文件
        video_paths = {
            1: "/root/yolov5-7.0_lyngor1.17.0/videos/test_121.mp4",
            2: "/root/yolov5-7.0_lyngor1.17.0/videos/test_122.mp4",
            3: "/root/yolov5-7.0_lyngor1.17.0/videos/test_123.mp4"
        }
        
        # 检查视频文件是否存在
        for cam_id, video_path in video_paths.items():
            if not os.path.exists(video_path):
                print(f"❌ Camera{cam_id} 视频文件不存在: {video_path}")
                print(f"   请确保视频文件存在或提供正确的RTSP源配置")
                sys.exit(1)
            else:
                print(f"📷 Camera{cam_id}: {video_path}")
    
    detect_areas = {
        1: [np.array([[0, 720], [226, 324], [576, 77], [714, 77], [1278, 390], [1280, 720]], dtype=np.int32),
            np.array([[218,324], [472,149], [366,141], [48,312]], dtype=np.int32)],
        2: [np.array([[0, 503], [0, 714], [1280, 720], [1280, 410], [800, 128],[471,133]], dtype=np.int32)],
        3: [np.array([[70, 720], [1030, 720], [934, 166], [90, 166]], dtype=np.int32)]}
        
    signal.signal(signal.SIGINT, cancel_process)
    
    # 2. 初始化核心组件
    fusion_system = CrossCameraFusion()
    queues = {i: multiprocessing.Queue(maxsize=10) for i in [1, 2, 3]}
    perf_monitor = PerformanceMonitor()
    
    # 2.1 初始化雷达融合模块
    print("\n🔧 初始化雷达融合模块...")
    radar_fusion_enabled = False
    radar_fusion_processor = None
    radar_data_loader = None
    
    # 雷达数据文件路径 (可配置)
    radar_data_path = 'c:/Users/zhenghuiwen1/Desktop/project_simple/radar_vision/radar_data_85_aligned.jsonl'
    
    # 融合区域配置 (可选，如果需要区域过滤)
    # 这里使用一个通用的融合区域，或者可以为每个摄像头配置不同的区域
    fusion_area_geo = [
        [113.583894894, 23.530394880],
        [113.584462681, 23.530850485],
        [113.584032327, 23.530886446],
        [113.583922645, 23.530898319]
    ]
    
    try:
        if os.path.exists(radar_data_path):
            # 初始化雷达数据加载器
            radar_data_loader = RadarDataLoader(radar_data_path)
            if radar_data_loader.load():
                # 初始化雷达融合处理器
                radar_fusion_processor = RadarVisionFusionProcessor(
                    fusion_area_geo=fusion_area_geo,
                    lat_offset=-0.00000165,
                    lon_offset=0.0000450
                )
                
                # 将所有雷达数据添加到处理器
                for ts in radar_data_loader.get_all_timestamps():
                    radar_objs = radar_data_loader.get_radar_data(ts)
                    radar_fusion_processor.add_radar_data(ts, radar_objs)
                
                radar_fusion_enabled = True
                print(f"✅ 雷达融合模块初始化成功")
                print(f"   雷达数据帧数: {len(radar_data_loader.get_all_timestamps())}")
            else:
                print("⚠️  雷达数据加载失败，将不使用雷达融合")
        else:
            print(f"⚠️  雷达数据文件不存在: {radar_data_path}")
            print("   将不使用雷达融合功能")
    except Exception as e:
        print(f"⚠️  雷达融合模块初始化失败: {e}")
        print("   将不使用雷达融合功能")
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
            print("✅ MQTT发布器已连接")
        except Exception as e:
            print(f"⚠️  MQTT连接失败: {e}, 将使用JSON文件保存")
            mqtt_publisher = None
    
    class TrackerArgs: # 新ByteTracker 所需参数
        def __init__(self):
            self.track_thresh = 0.5    # 跟踪置信度阈值，过低会导致噪声
            self.track_buffer = 30     # 增加缓冲区大小以保留轨迹历史
            self.match_thresh = 0.8    # 匹配阈值
            self.mot20 = False         # MOT20数据集标志
    
    tracker_args = TrackerArgs()
    trackers = {i: BYTETracker(tracker_args, frame_rate=Config.FPS) for i in [1, 2, 3]} # 局部跟踪器运行在主进程
    print("✅ 已启用优化版ByteTracker - 交替跟踪模式") 

    # 🔧 新增：RTSP连接测试函数
    def test_rtsp_connection(rtsp_url: str, timeout: int = 5) -> bool:
        """测试RTSP连接是否可用"""
        try:
            import cv2
            # 尝试使用 PyAV (SDKinfer_ffmpeg.py 的依赖) 来测试，更一致
            try:
                import av
                av.logging.set_level(av.logging.ERROR)
                container = av.open(rtsp_url, 'r', options={'rtsp_transport': 'tcp', 'stimeout': str(timeout * 1000000)}, timeout=timeout)
                container.decode(video=0)
                container.close()
                return True
            except ImportError:
                # 回退到 OpenCV
                cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                # 设置超时
                start_time = time.time()
                ret = False
                while time.time() - start_time < timeout:
                    ret, frame = cap.read()
                    if ret:
                        break
                    time.sleep(0.1)
                cap.release()
                return ret
        except Exception as e:
            return False
    
    # 🔧 新增：在SDK初始化前测试RTSP连接
    print("\n🔍 测试RTSP连接 (使用PyAV/OpenCV)...")
    rtsp_connection_status = {}
    for cam_id in [1, 2, 3]:
        video_path = video_paths[cam_id]
        # 只测试RTSP URL，不测试本地文件
        if video_path.startswith('rtsp://'):
            print(f"  测试 Camera{cam_id}: {video_path[:60]}...")
            is_connected = test_rtsp_connection(video_path, timeout=5)
            rtsp_connection_status[cam_id] = is_connected
            if is_connected:
                print(f"  ✅ Camera{cam_id} RTSP连接成功")
            else:
                print(f"  ❌ Camera{cam_id} RTSP连接失败 - 无法连接到: {video_path}")
        else:
            # 本地视频文件，跳过测试
            rtsp_connection_status[cam_id] = True
            print(f"  ✅ Camera{cam_id} 使用本地视频文件: {video_path}")
    # 3. 创建并启动 SDK 推理进程 (生产者)
    processes = []
    print("\n🚀 启动SDK版多摄像头融合系统")
    
    # 🔧 新增：检查RTSP连接状态，给出警告
    failed_cameras = [cam_id for cam_id, status in rtsp_connection_status.items() if not status]
    if failed_cameras:
        print(f"⚠️  警告: {len(failed_cameras)}个摄像头RTSP连接测试失败: {failed_cameras}")
        time.sleep(2)  # 给用户时间阅读警告
    
    for camera_id in [1, 2, 3]:
        video_path = video_paths[camera_id]
        if not rtsp_connection_status.get(camera_id, True):
            print(f"⚠️  Camera{camera_id} RTSP连接测试失败，但仍尝试启动SDK...")
        
        # 🔧 修改：移除 timestamp_providers 参数
        process = multiprocessing.Process(
            target=create_sdk_worker_process,
            args=(camera_id, video_path, queues[camera_id]),
            daemon=True
        )
        processes.append(process)
        process.start()
        print(f"🔄 启动Camera{camera_id} SDK推理进程...")
        
    print("✅ 所有SDK推理进程已启动")

    # 3.5. 启动预热阶段：等待所有摄像头队列都有数据
    print("\n⏱️  进入预热阶段，等待所有摄像头推送第一帧数据...")
    PREHEAT_TIMEOUT = 30  # 30秒超时
    start_time = time.time()
    ready_cameras = {i: False for i in [1, 2, 3]}
    last_report_time = time.time()
    
    while time.time() - start_time < PREHEAT_TIMEOUT:
        all_ready = True
        for cam_id in [1, 2, 3]:
            if not ready_cameras[cam_id] and not queues[cam_id].empty():
                ready_cameras[cam_id] = True
                print(f"✅ 摄像头 C{cam_id} 已就绪！")
        
        if all(ready_cameras.values()):
            print("🎉 所有摄像头均已就绪，预热完成！")
            break
        
        # 每5秒打印一次状态
        current_time = time.time()
        if current_time - last_report_time >= 5:
            elapsed = int(current_time - start_time)
            queue_sizes = {i: queues[i].qsize() for i in [1, 2, 3]}
            process_status = {i: (p.is_alive() and "✓运行中" or "✗已停止") for i, p in enumerate(processes)}
            print(f"⏳ 预热进度 ({elapsed}s/{PREHEAT_TIMEOUT}s): 队列 {queue_sizes}, 进程状态 {[process_status.get(i) for i in range(3)]}")
            print(f"   准备就绪: C1={ready_cameras[1]}, C2={ready_cameras[2]}, C3={ready_cameras[3]}")
            last_report_time = current_time
        
        time.sleep(0.5)
    else:
        print("❌ 预热超时！")
        for cam_id, is_ready in ready_cameras.items():
            if not is_ready:
                print(f"  - 摄像头 C{cam_id} 未能在 {PREHEAT_TIMEOUT} 秒内推送数据。")
        print("程序将继续运行，但可能会出现同步问题。")


    # 4. 主循环：时间戳融合逻辑 (消费者)
    current_frame = 0
    
    # 🔧 新增：设置摄像头起始时间戳（绝对时间格式）
    print("\n🔧 配置摄像头时间戳...")
    FFmpegTimeStampProvider.set_all_camera_start_datetimes(Config.CAMERA_START_DATETIMES)
    print("✅ 摄像头时间戳配置完成")
    
    # 初始化FFmpeg时间戳帧同步器
    frame_synchronizer = FFmpegTimestampFrameSynchronizer(
        num_cameras=3, 
        timestamp_tolerance_ms=4000  # 启动容忍度（毫秒），用于Warmup阶段对齐起跑线
    )
    # 🔧 更新：使用绝对时间戳同步 - Warmup + 动态丢弃策略
    sync_mode = "绝对时间戳同步 - Warmup + 动态丢弃策略"
    
    frame_loss_prevention = FrameLossPrevention()
    
    print("\n--- 融合主循环启动 ---")
    print(f"🎯 同步模式：{sync_mode}")
    print("="*60)

    try:
        last_sync_report = time.time()
        no_sync_count = 0  # 计数连续没有同步帧的次数
        
        while not cancel_flag.value:
            
            # A. 从所有队列中获取结果并送入同步器
            perf_monitor.start_timer('queue_processing')
            for camera_id in [1, 2, 3]:
                queue_size = queues[camera_id].qsize()
                perf_monitor.record_queue_stats(camera_id, queue_size, 'read')
                
                while True: # 使用 get_nowait() 快速清空队列，避免阻塞
                    try:
                        result = queues[camera_id].get_nowait()
                        frame_id = result['frame_id']
                        perf_monitor.add_counter('queue_operations')
                        
                        # 防丢帧检测
                        if frame_loss_prevention.check_frame_sequence(camera_id, frame_id):
                            # 添加到FFmpeg时间戳同步器
                            frame_synchronizer.add_frame(camera_id, result)
                        
                    except multiprocessing.queues.Empty:
                        break # 队列为空，退出内层循环
                    except Exception as e:
                        print(f"❌ C{camera_id} 队列读取异常: {e}")
                        break
            
            queue_processing_time = perf_monitor.end_timer('queue_processing')
            
            # B. 获取时间戳同步的帧
            synchronized_frames, sync_frame_number = frame_synchronizer.get_synchronized_frames()
            
            if not synchronized_frames:
                # 没有可同步的帧，短暂等待，避免CPU空转
                no_sync_count += 1
                
                # 🔧 改进：更详细的调试信息
                if no_sync_count % 20 == 0:
                    current_time = time.time()
                    buffer_status = frame_synchronizer.get_buffer_status()
                    queue_sizes = {i: queues[i].qsize() for i in [1, 2, 3]}
                    # 🔧 修复：使用正确的属性名warmup_complete
                    warmup_complete = getattr(frame_synchronizer, 'warmup_complete', False)
                    warmup_status = "✅完成" if warmup_complete else "⏳进行中"
                    
                    print(f"⏱️  等待同步... (连续{no_sync_count}个周期)")
                    print(f"   队列大小: C1={queue_sizes[1]}, C2={queue_sizes[2]}, C3={queue_sizes[3]}")
                    print(f"   缓冲区: {buffer_status}")
                    print(f"   Warmup状态: {warmup_status}")
                    
                    # 检查进程状态
                    alive_count = sum(1 for p in processes if p.is_alive())
                    print(f"   SDK进程: {alive_count}/3 运行中")
                
                # 🔧 改进：更长的超时时间，因为Warmup阶段可能需要更多时间
                if no_sync_count > 500:  # 提高到500个周期（约2.5秒）
                    print(f"\n❌ 警告：已连续{no_sync_count}个周期无法同步")
                    print(f"   详细诊断信息:")
                    # 🔧 修复：使用正确的属性名warmup_complete
                    warmup_complete = getattr(frame_synchronizer, 'warmup_complete', False)
                    print(f"   1. Warmup状态: {'✅完成' if warmup_complete else '❌未完成'}")
                    if not warmup_complete:
                        print(f"      等待所有摄像头对齐起跑线...")
                    buffer_status = frame_synchronizer.get_buffer_status()
                    print(f"   2. 缓冲区状态: {buffer_status}")
                    queue_sizes = {i: queues[i].qsize() for i in [1, 2, 3]}
                    print(f"   3. 队列大小: {queue_sizes}")
                    alive_count = sum(1 for p in processes if p.is_alive())
                    print(f"   4. SDK进程状态: {alive_count}/3 运行中")

                    
                    # 检查是否有进程已停止
                    if alive_count < 3:
                        print("   检测到SDK进程已停止，主循环退出。")
                        break
                
                # 如果所有进程都停止了，也退出
                if all(not p.is_alive() for p in processes):
                    print("❌ 所有SDK子进程已停止，主循环退出。")
                    break

                time.sleep(0.005) 
                continue
            
            # 重置无同步计数
            no_sync_count = 0

            # C. 只要有数据就进行融合
            current_frame = sync_frame_number
            current_frame_results = synchronized_frames
            # 📊 性能监控：记录每帧同步和处理情况
            perf_monitor.add_counter('frames_synchronized')
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
                
                # 3. 区域过滤 (先过滤后跟踪，与main_combined保持一致) ✅ FIX
                perf_monitor.start_timer('area_filtering')
                filtered_nms_detections = filter_by_detect_areas(nms_detections, detect_areas[camera_id])
                perf_monitor.end_timer('area_filtering')
                
                # 4. 使用批量处理函数，提升性能
                perf_monitor.start_timer('tracker_input_preparation')
                tracker_input_tensor, box_to_class = batch_prepare_tracker_input(filtered_nms_detections)
                perf_monitor.end_timer('tracker_input_preparation')
                
                # 限制最大跟踪目标数量，避免匈牙利算法性能问题
                if len(tracker_input_tensor) > 50:  # 限制最多50个目标
                    tracker_input_tensor = tracker_input_tensor[:50]
                    print(f"⚠️  C{camera_id} 目标数量过多({len(filtered_nms_detections)})，限制为50个")

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
                    print(f"  📊 C{camera_id} F{current_frame} 跟踪前: 原始检测={len(raw_detections)}, NMS后={len(nms_detections)}, 过滤后={len(filtered_nms_detections)}, 跟踪器输入={debug_input_count}")
                
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
                global_targets, local_targets = fusion_system.process_detections(tracked_detections, camera_id, perf_monitor)
                
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

            # D. 雷达融合处理 (异步)
            radar_id_map = {}
            if radar_fusion_enabled and radar_fusion_processor:
                perf_monitor.start_timer('radar_fusion_processing')
                
                # 将所有目标转换为 OutputObject 格式
                vision_objects = []
                
                # 处理全局目标
                for global_target in all_global_targets:
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
                        timestamp="",  # 将在后面填充
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
                
                # 执行雷达融合
                if vision_objects:
                    vision_timestamp = ts  # 使用当前帧的时间戳
                    updated_vision_objects = radar_fusion_processor.process_frame(vision_timestamp, vision_objects)
                    
                    # 构建 radar_id_map
                    for vision_obj in updated_vision_objects:
                        if vision_obj.radar_id is not None:
                            radar_id_map[vision_obj.track_id] = vision_obj.radar_id
                    
                    # 统计信息
                    matched_count = sum(1 for v in updated_vision_objects if v.radar_id is not None)
                    if current_frame % 100 == 0 and matched_count > 0:
                        print(f"🔗 Frame {current_frame}: 雷达匹配 {matched_count}/{len(updated_vision_objects)} 个目标")
                
                perf_monitor.end_timer('radar_fusion_processing')
            
            # E. 生成JSON数据并尝试发送MQTT
            perf_monitor.start_timer('json_mqtt_processing')
            
            perf_monitor.start_timer('json_generation')
            json_data = fusion_system.generate_json_data(all_global_targets, all_local_targets, radar_id_map)
            perf_monitor.end_timer('json_generation')
            
            mqtt_sent = False
            if mqtt_publisher:
                perf_monitor.start_timer('mqtt_publish')
                try:
                    participants = json_data.get('participant', [])
                    mqtt_sent = mqtt_publisher.publish_rsm(participants)
                    if mqtt_sent:
                        # 📊 性能监控：记录MQTT成功发送
                        perf_monitor.add_counter('mqtt_sends')
                    else:
                        # 📊 性能监控：记录MQTT发送失败
                        perf_monitor.add_counter('mqtt_failures')
                except Exception as e:
                    print(f"❌ MQTT发送异常: {e}")
                    # 📊 性能监控：记录MQTT异常
                    perf_monitor.add_counter('mqtt_failures')
                finally:
                    perf_monitor.end_timer('mqtt_publish')
            
            # 🔧 修复：无论MQTT是否成功，都保存JSON数据（用于调试和备份）
            fusion_system.json_output_data.append(json_data)
            
            json_mqtt_time = perf_monitor.end_timer('json_mqtt_processing')
            
            # 打印处理信息
            #print(f"✅ 同步融合: Frame {current_frame} | 目标数: {len(all_frame_detections)} | MQTT: {'成功' if mqtt_sent else '失败/未配置'}")
            
            fusion_system.next_frame()

            # D. 定期报告丢帧情况
            if current_frame > 0 and current_frame % 300 == 0:
                missing_report = frame_loss_prevention.get_missing_frames_report()
                if missing_report:
                    print(f"\n📊 ----- 丢帧报告 (截至Frame {current_frame}) -----")
                    for cam_id, report in missing_report.items():
                        print(f"  C{cam_id}: 丢帧{report['missing_count']}个({report['loss_rate']:.2f}%), 重复{report['duplicate_count']}个")
                    print("-" * 45)
        
        print("\n🎯 所有处理完成 (或达到最大帧数)")
        
        # 5. 保存融合结果（正常退出时）
        try:
            json_count = len(fusion_system.json_output_data) if fusion_system.json_output_data else 0
            print(f"💾 准备保存 {json_count} 帧的JSON数据...")
            if json_count > 0:
                fusion_system.save_json_data("output_fusion_refactored.json")
            else:
                print("⚠️  警告: JSON数据列表为空")
        except Exception as e:
            print(f"❌ 保存JSON数据失败: {e}")
            import traceback
            traceback.print_exc()
        
        # 6. 输出最终同步统计
        print("\n" + "="*60)
        print("📊 最终同步统计报告:")
        final_stats = frame_loss_prevention.get_statistics()
        
        total_processed = sum(stat['total_processed'] for stat in final_stats.values())
        synchronized_frames_count = fusion_system.frame_count

        print(f"📈 处理概况:")
        print(f"  总接收帧数 (各摄像头合计): {total_processed}帧")
        print(f"  成功同步并处理的帧组: {synchronized_frames_count} 组")
        
        if total_processed > 0 and len(final_stats) > 0:
            avg_processed_per_cam = total_processed / len(final_stats)
            sync_rate = (synchronized_frames_count / max(avg_processed_per_cam, 1)) * 100
            print(f"  同步成功率 (估算): {sync_rate:.2f}%")
        
        buffer_status = frame_synchronizer.get_buffer_status()
        remaining_frames = sum(status['count'] for status in buffer_status.values())
        if remaining_frames > 0:
            print(f"⚠️  处理结束时缓冲区剩余: {remaining_frames}帧未处理")
            
        print("="*60)
        
        # 输出最终跟踪器优化统计
        print("\n📊 最终ByteTracker优化统计:")
        print("\n🔍 DEBUG - 跟踪器输入统计:")
        for cam_id in [1, 2, 3]:
            stats = tracker_input_stats[cam_id]
            avg_dets = stats['total_dets'] / max(stats['total'], 1)
            print(f"  C{cam_id}: 调用{stats['total']}次, 空输入{stats['empty']}次, 非空{stats['non_empty']}次, 总检测数{stats['total_dets']}, 平均{avg_dets:.1f}个/帧")
        print()
        
        for cam_id, tracker in trackers.items():
            stats = tracker.get_performance_stats()
            perf_improvement = stats.get('performance_improvement', 1.0)
            avg_tracking_time = stats.get('avg_tracking_time', 0.0)
            avg_prediction_time = stats.get('avg_prediction_time', 0.0)
            print(f"  C{cam_id}:")
            print(f"    总帧数: {stats['total_frames']}")
            print(f"    跟踪帧: {stats['tracking_frames']} ({stats['tracking_frames']/max(stats['total_frames'],1)*100:.1f}%)")
            print(f"    预测帧: {stats['prediction_only_frames']} ({stats['prediction_only_frames']/max(stats['total_frames'],1)*100:.1f}%)")
            print(f"    性能提升: {perf_improvement:.2f}x")
            print(f"    平均跟踪耗时: {avg_tracking_time:.3f}s")
            print(f"    平均预测耗时: {avg_prediction_time:.3f}s")
        print("="*60)
        
    except Exception as e:
        print(f"❌ 主程序执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 🔧 修复：在finally块中保存JSON，确保即使异常退出也能保存数据
        print("\n💾 正在保存JSON数据...")
        try:
            json_count = len(fusion_system.json_output_data) if fusion_system.json_output_data else 0
            print(f"   准备保存 {json_count} 帧的JSON数据")
            if json_count > 0:
                fusion_system.save_json_data("output_fusion_refactored.json")
            else:
                print("⚠️  警告: JSON数据列表为空，没有数据可保存")
                print(f"   可能原因: 1) 程序异常退出 2) 没有检测到目标 3) 数据未正确添加")
        except Exception as e:
            print(f"❌ 在finally块中保存JSON失败: {e}")
            import traceback
            traceback.print_exc()
        
        # 7. 清理资源
        cancel_flag.value = True # 确保所有进程停止
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join()

        # 断开MQTT连接
        if mqtt_publisher:
            try:
                mqtt_publisher.disconnect()
                print("✅ MQTT连接已断开")
            except:
                pass
                
        print("🧹 资源清理完成")