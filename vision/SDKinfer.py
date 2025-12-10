# --- SDK 推理类 (生产者，精简版) ---
import os
import sys
import time
import multiprocessing
import copy
import json
import logging
from datetime import datetime, timedelta
from collections import defaultdict, deque
from statistics import mean, median
sys.path.append('/usr/local/lynxi/sdk/sdk-samples/python')

import numpy as np
import cv2
from ctypes import *
import ctypes
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict
import re
import struct

logger = logging.getLogger(__name__)

# 导入SDK相关模块
import pycommon.common as common
import pylynchipsdk as sdk
from pycommon.infer_process import *
from pycommon.callback_data_struct import *
from pycommon.dump_json import *
from core.Basic import NAMES

class yolov5_SDK(infer_process):
    """
    精简版的 SDK 推理进程。只负责推理和将原始检测结果放入队列。
    不包含任何跟踪、BEV、融合逻辑。
    
    🔧 改进：使用初始视频时间 + frame_id/fps 计算时间戳
    """
    def __init__(self, attr, result_queue, start_datetime_str=None, fps=25.0):
        super().__init__(attr)
        self.class_num = self.model_desc.outputTensorAttrArray[0].dims[3] - 5
        self.anchor_size = self.model_desc.outputTensorAttrArray[0].dims[1]
        
        # 使用传入的队列
        self.result_queue = result_queue
        self.frame_count = 0
        
        # 🔧 时间戳计算参数
        self.fps = fps
        self.start_datetime = None
        if start_datetime_str:
            try:
                # 解析起始时间字符串
                if '.' in start_datetime_str:
                    self.start_datetime = datetime.strptime(start_datetime_str, "%Y-%m-%d %H:%M:%S.%f")
                else:
                    self.start_datetime = datetime.strptime(start_datetime_str, "%Y-%m-%d %H:%M:%S")
                logger.info(f"Camera{attr.chan_id + 1} 时间戳初始化: 起始时间={self.start_datetime.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}, FPS={fps}")
            except Exception as e:
                logger.warning(f"Camera{attr.chan_id + 1} 时间戳解析失败: {e}, 将使用系统时间")
                self.start_datetime = None
        
        # 🔧 新增：动态延迟机制参数
        self.camera_id = attr.chan_id + 1
        self.detections_history = deque(maxlen=30)  # 保留最近30帧的检测数统计
        self.base_delay = 0.030  # 基础延迟30ms (从20ms增加)
        self.max_extra_delay = 0.250  # 最大额外延迟250ms (从150ms增加)
        self.min_detections_threshold = 2  # 检测数少于此值时才加延迟
        
        # 分配内存
        self.boxes_info, ret = sdk.lyn_malloc(ctypes.sizeof(Box))
        if ret != 0:
            raise RuntimeError(f"Camera{self.attr.chan_id + 1}: 内存分配失败: {ret}")

    def update_class_name(self, class_name_path: str) -> None:
        """更新类别名称到插件中 - 从test_single_camera.py复制"""
        try:
            with open(class_name_path, 'r') as file:
                file_content = file.read()
                pattern = re.compile(r"^(x7|normal):([^,]+(,[^,]+)*)$", re.IGNORECASE)
                if not pattern.match(file_content):
                    logger.error(f'"{file_content}" is not right!')
                    os._exit(-1)
        except FileNotFoundError:
            logger.error(f"File at path '{class_name_path}' not found.")
            return None
        except IOError as e:
            logger.error(f"Error reading file at path '{class_name_path}': {e}")
            return None
        
        ary = np.fromfile(class_name_path)
        ptr = sdk.lyn_numpy_to_ptr(ary)
        device_ptr, ret = sdk.lyn_malloc(ary.nbytes)
        sdk.lyn_memcpy(device_ptr, ptr, ary.nbytes, sdk.ClientToServer)
        class_name_arg = struct.pack("Pi", pythonapi.PyCapsule_GetPointer(device_ptr, None), ary.nbytes)
        sdk.lyn_plugin_run_async(self.ipe_stream, self.plugin, "lynClassNameUpdata", class_name_arg, len(class_name_arg))
        sdk.lyn_synchronize_stream(self.ipe_stream)
        sdk.lyn_free(device_ptr)
        logger.info(f"Camera{self.attr.chan_id + 1} 更新类别名称: {class_name_path}")

    def copy_box_data_safely(self, boxes_info_ptr):
        
        try:
            # 复制 box data 
            if not boxes_info_ptr: return None
            pythonapi.PyCapsule_GetPointer.restype = c_void_p
            pythonapi.PyCapsule_GetPointer.argtypes = [py_object, c_char_p]
            host_buf_c = pythonapi.PyCapsule_GetPointer(boxes_info_ptr, None)
            if not host_buf_c: return None
            dst_img_size = ctypes.sizeof(Box)
            host_buf_arr = np.ones(dst_img_size, dtype=np.uint8)
            host_buf = sdk.lyn_numpy_to_ptr(host_buf_arr)
            ret = sdk.lyn_memcpy(
                host_buf, boxes_info_ptr, dst_img_size, 
                sdk.lyn_memcpy_dir_t.ServerToClient
            )
            if ret != 0: return None
            host_buf_c = pythonapi.PyCapsule_GetPointer(host_buf, None)
            box_data = ctypes.cast(host_buf_c, ctypes.POINTER(Box)).contents
            if hasattr(box_data, 'boxesnum') and box_data.boxesnum >= 0:
                return box_data
            else:
                return None
        except Exception as e:
            if self.frame_count % 50 == 0:
                logger.warning(f"Camera{self.attr.chan_id + 1} 复制box数据错误: {type(e).__name__}")
            return None

    def calculate_timestamp(self, frame_id: int) -> str:
        """计算帧的时间戳：start_datetime + (frame_id / fps)
        
        Returns:
            str: 时间戳字符串，格式 "YYYY-MM-DD HH:MM:SS.mmm"
        """
        if self.start_datetime is None:
            # 降级方案：使用系统时间
            current_time = datetime.now()
            return current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # 计算时间偏移：frame_id / fps 秒
        time_offset_seconds = frame_id / self.fps
        target_datetime = self.start_datetime + timedelta(seconds=time_offset_seconds)
        return target_datetime.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    def extract_detection_info(self, box_data):
      
        if not box_data: return None
        
        # 🔧 计算时间戳：初始时间 + (frame_id / fps)
        timestamp_str = self.calculate_timestamp(self.frame_count)
            
        frame_result = {
            'frame_id': self.frame_count,
            'camera_id': self.attr.chan_id + 1,
            'boxes_num': 0,  # 先设为0，后面更新
            'detections': [],
            'timestamp': timestamp_str,
            'rtsp_timestamp': timestamp_str  # 用于时间戳同步
        }
        
        try:
            boxes_num = box_data.boxesnum if hasattr(box_data, 'boxesnum') else 0
            if boxes_num < 0 or boxes_num > 10000:  # 异常检查
                return frame_result
                
            for i in range(boxes_num):
                try:
                    # 安全访问box结构
                    if not hasattr(box_data, 'boxes'):
                        break
                    
                    box = box_data.boxes[i]
                    if box is None:
                        continue
                    
                    # 初始化默认值
                    label_str = "unknown"
                    class_id = 0
                    
                    # 安全提取标签
                    if hasattr(box, 'label') and box.label is not None:
                        try:
                            if isinstance(box.label, str):
                                label_str = box.label
                                try:
                                    class_id = int(box.label)
                                except (ValueError, TypeError):
                                    class_id = hash(box.label) % 1000
                            else:
                                class_id = int(box.label)
                                # 修复：检查class_id的范围，包括负数情况
                                if 0 <= class_id < len(NAMES):
                                    label_str = NAMES[class_id]
                                else:
                                    label_str = f"class_{class_id}"
                        except (ValueError, UnicodeError, TypeError, IndexError) as e:
                            label_str = "unknown"
                            class_id = 0
                    
                    # 安全提取检测框坐标
                    try:
                        x_min = float(box.xmin) if hasattr(box, 'xmin') else 0.0
                        y_min = float(box.ymin) if hasattr(box, 'ymin') else 0.0
                        x_max = float(box.xmax) if hasattr(box, 'xmax') else 0.0
                        y_max = float(box.ymax) if hasattr(box, 'ymax') else 0.0
                        conf = float(box.score) if hasattr(box, 'score') else 0.0
                    except (ValueError, TypeError, AttributeError):
                        continue  # 跳过无效的检测框
                    
                    detection = {
                        'box': [x_min, y_min, x_max, y_max],
                        'confidence': conf,
                        'class': label_str, 
                    }
                    frame_result['detections'].append(detection)
                    
                except (IndexError, AttributeError, TypeError) as e:
                    # 单个box处理失败，继续处理下一个
                    if self.frame_count % 50 == 0:
                        logger.debug(f"Camera{self.attr.chan_id + 1} 处理box[{i}]错误")
                    continue
        
        except Exception as e:
            # 完整的try块捕获所有异常
            if self.frame_count % 50 == 0:
                logger.warning(f"Camera{self.attr.chan_id + 1} extract_detection_info异常")
        
        # 更新实际检测数量
        frame_result['boxes_num'] = len(frame_result['detections'])
        return frame_result

    def process_box_data_callback(self, params):
        """处理SDK后处理的结果回调函数 - 改进版本，添加更完善的错误处理"""
        try:
            # 安全的参数提取
            if not isinstance(params, (list, tuple)) or len(params) < 2:
                if self.frame_count % 50 == 0:
                    logger.warning(f"Camera{self.attr.chan_id + 1} callback参数格式错误")
                return 0
            
            boxes_info = params[0]
            frame_count = params[1]
            
            if not boxes_info:
                return 0
            
            # 复制box数据到主机内存
            dst_img_size = ctypes.sizeof(Box)
            host_buf_arr = np.ones(dst_img_size, dtype=np.uint8)
            host_buf = sdk.lyn_numpy_to_ptr(host_buf_arr)
            ret = sdk.lyn_memcpy(
                host_buf, boxes_info, dst_img_size, 
                sdk.lyn_memcpy_dir_t.ServerToClient
            )
            if ret != 0:
                if frame_count % 30 == 0:
                    logger.error(f"Camera{self.attr.chan_id + 1} memcpy失败: {ret}")
                return 0
            
            # 获取box数据指针
            pythonapi.PyCapsule_GetPointer.restype = c_void_p
            pythonapi.PyCapsule_GetPointer.argtypes = [py_object, c_char_p]
            host_buf_c = pythonapi.PyCapsule_GetPointer(host_buf, None)
            box_data = ctypes.cast(host_buf_c, ctypes.POINTER(Box)).contents
            
            # 提取检测信息
            frame_result = self.extract_detection_info_from_box(box_data, frame_count)
            
            if frame_result:
                try:
                    self.result_queue.put(copy.deepcopy(frame_result), timeout=1)
                    if frame_count % 100 == 0:
                        logger.debug(f"C{self.attr.chan_id + 1} F{frame_count}: {frame_result['boxes_num']} boxes")
                except Exception as e:
                    if frame_count % 30 == 0:
                        logger.error(f"Camera{self.attr.chan_id + 1} 队列PUT失败")
            else:
                # 空帧也需要入队
                try:
                    self.result_queue.put({
                        'frame_id': frame_count, 
                        'camera_id': self.attr.chan_id + 1,
                        'boxes_num': 0, 
                        'detections': []
                    }, timeout=1)
                except:
                    pass
            
            return 0
        except Exception as e:
            if self.frame_count % 50 == 0:
                logger.error(f"Camera{self.attr.chan_id + 1} callback异常: {type(e).__name__}")
            return 0

    def extract_detection_info_from_box(self, box_data, frame_count):
        """从box数据中提取检测信息"""
        if not box_data: return None
        
        # 🔧 计算时间戳：初始时间 + (frame_id / fps)
        timestamp_str = self.calculate_timestamp(frame_count)
            
        frame_result = {
            'frame_id': frame_count,
            'camera_id': self.attr.chan_id + 1,
            'boxes_num': box_data.boxesnum if hasattr(box_data, 'boxesnum') else 0,
            'detections': [],
            'timestamp': timestamp_str,
            'rtsp_timestamp': timestamp_str  # 用于时间戳同步
        }
        
        try:
            boxes_num = box_data.boxesnum if hasattr(box_data, 'boxesnum') else 0
            if boxes_num < 0 or boxes_num > 10000:  # 异常检查
                return frame_result
            
            for i in range(boxes_num):
                try:
                    box = box_data.boxes[i]
                    if box is None:
                        continue
                    
                    # 初始化默认值
                    label_str = "unknown"
                    class_id = 0
                    
                    # 安全提取标签
                    if hasattr(box, 'label') and box.label is not None:
                        try:
                            if isinstance(box.label, str):
                                label_str = box.label
                                try:
                                    class_id = int(box.label)
                                except (ValueError, TypeError):
                                    class_id = hash(box.label) % 1000
                            else:
                                class_id = int(box.label)
                                # 检查class_id的范围
                                if 0 <= class_id < len(NAMES):
                                    label_str = NAMES[class_id]
                                else:
                                    label_str = f"class_{class_id}"
                        except (ValueError, UnicodeError, TypeError, IndexError):
                            label_str = "unknown"
                            class_id = 0
                    
                    # 安全提取检测框坐标
                    try:
                        x_min = float(box.xmin) if hasattr(box, 'xmin') else 0.0
                        y_min = float(box.ymin) if hasattr(box, 'ymin') else 0.0
                        x_max = float(box.xmax) if hasattr(box, 'xmax') else 0.0
                        y_max = float(box.ymax) if hasattr(box, 'ymax') else 0.0
                        conf = float(box.score) if hasattr(box, 'score') else 0.0
                    except (ValueError, TypeError, AttributeError):
                        continue
                    
                    detection = {
                        'box': [x_min, y_min, x_max, y_max],
                        'confidence': conf,
                        'class': label_str, 
                    }
                    frame_result['detections'].append(detection)
                    
                except (IndexError, AttributeError, TypeError):
                    continue
        
        except Exception as e:
            if frame_count % 30 == 0:
                logger.warning(f"Camera{self.attr.chan_id + 1} extract_detection_info_from_box异常")
        
        # 更新实际检测数量
        frame_result['boxes_num'] = len(frame_result['detections'])
        return frame_result

    def plugin_process(self, apu_output_data, cb_data):
        """处理后处理回调 - 改进版本，更安全的参数处理"""
        try:
            # 等待 apu 处理完成
            ret = sdk.lyn_record_event(self.apu_stream, self.apu_event)
            if ret != 0: 
                if self.frame_count % 50 == 0:
                    logger.error(f"Camera{self.attr.chan_id + 1} lyn_record_event失败")
                return
            ret = sdk.lyn_stream_wait_event(self.plugin_stream, self.apu_event)
            if ret != 0: 
                if self.frame_count % 50 == 0:
                    logger.error(f"Camera{self.attr.chan_id + 1} lyn_stream_wait_event失败")
                return
            
            # 获取指针 - 安全的指针转换
            pythonapi.PyCapsule_GetPointer.restype = c_void_p
            pythonapi.PyCapsule_GetPointer.argtypes = [py_object, c_char_p]
            apu_data_ptr = pythonapi.PyCapsule_GetPointer(apu_output_data, None)
            boxes_info_ptr = pythonapi.PyCapsule_GetPointer(self.boxes_info, None)
            
            if not apu_data_ptr or not boxes_info_ptr:
                if self.frame_count % 50 == 0:
                    logger.error(f"Camera{self.attr.chan_id + 1} 指针转换失败")
                return

            # 获取图像和模型尺寸（与main_combined.py保持一致）
            original_width = self.codec_para.width
            original_height = self.codec_para.height
            model_width = self.model_width
            model_height = self.model_height
            
            # 调试信息
            if self.frame_count % 500 == 0:
                logger.debug(f"C{self.attr.chan_id + 1} 尺寸: {original_width}x{original_height} -> {model_width}x{model_height}")
            
            # 执行后处理 - 使用正确的参数格式
            post_para = struct.pack(
                '6IH2f?2P',
                original_width,     # 原始图像宽度
                original_height,    # 原始图像高度
                model_width,        # 模型输入宽度
                model_height,       # 模型输入高度
                self.class_num,     # 类别数量
                500,                # nmsTopK
                self.anchor_size,   # anchor尺寸
                0.25,              # score threshold
                0.45,              # nms threshold
                True,              # is pad resize
                apu_data_ptr,      # APU输出数据指针
                boxes_info_ptr,    # 检测框信息指针
            )
            
            ret = sdk.lyn_plugin_run_async(
                self.plugin_stream, self.plugin, "lynPostProcess", post_para, len(post_para)
            )
            if ret != 0: 
                if self.frame_count % 50 == 0:
                    logger.error(f"Camera{self.attr.chan_id + 1} lyn_plugin_run_async失败")
                return
            
            # 使用回调函数处理结果（与main_combined.py保持一致）
            ret = sdk.lyn_stream_add_callback(
                self.plugin_stream,
                self.process_box_data_callback,
                [self.boxes_info, self.frame_count],  # 传递必要的参数
            )
            if ret != 0: 
                if self.frame_count % 50 == 0:
                    logger.error(f"Camera{self.attr.chan_id + 1} lyn_stream_add_callback失败")
                return
            
            self.frame_count += 1

            # 释放内存
            ret = sdk.lyn_stream_add_async_callback(
                self.plugin_stream, free_to_pool_callback, [self.apu_output_mem_pool, apu_output_data]
            )
            
        except Exception as e:
            if self.frame_count % 50 == 0:
                logger.error(f"Camera{self.attr.chan_id + 1} plugin_process异常: {type(e).__name__}")
            pass

    def run(self, cancel_flag):
        # 调用父类的 run 方法启动所有线程
        super().run(cancel_flag)
        
        logger.info(f"Camera{self.attr.chan_id + 1} SDK线程已启动")
        
        try:
            while not cancel_flag.value:
                time.sleep(0.1)
        except KeyboardInterrupt:
            logger.info(f"Camera{self.attr.chan_id + 1} 收到中断信号")
        finally:
            self.close()
            logger.info(f"Camera{self.attr.chan_id + 1} SDK进程退出")


# --- 辅助函数 ---

def cancel_process(signum, frame):
    """取消处理信号"""
    global cancel_flag
    cancel_flag.value = True
    logger.info("收到停止信号")

def create_sdk_worker_process(camera_id: int, video_path: str, result_queue: multiprocessing.Queue, start_datetime_str=None, fps=25.0):
    """创建并运行一个独立的 SDK 推理子进程 (生产者)"""
    try:
        logger.info(f"Camera{camera_id} 子进程启动")
        
        attr = infer_process_attr()
        attr.url = video_path
        attr.device_id = 0
        attr.chan_id = camera_id - 1
        attr.plugin_path = "/usr/local/lynxi/sdk/sdk-samples/plugin/obj/libYolov5Plugin.so"
        attr.model_path = "/root/yolov5-7.0_lyngor1.17.0/best_yolov5s_onnx/Net_0/"
        attr.show_type = 2
        attr.output_path = ""
        
        logger.info(f"Camera{camera_id} 初始化yolov5_SDK")
        worker = yolov5_SDK(attr, result_queue, start_datetime_str=start_datetime_str, fps=fps) 
        logger.info(f"Camera{camera_id} yolov5_SDK初始化成功")
        
        # 更新类别名称 - 在运行前调用
        class_name_path = "/usr/local/lynxi/sdk/sdk-samples/data/class.txt"
        if os.path.exists(class_name_path):
            worker.update_class_name(class_name_path)
        else:
            logger.warning(f"Camera{camera_id} 类别名称文件不存在")
        
        logger.info(f"Camera{camera_id} 开始运行")
        worker.run(cancel_flag)
        logger.info(f"Camera{camera_id} 子进程退出")
        
    except Exception as e:
        logger.error(f"Camera{camera_id} SDK进程失败: {e}")
        import traceback
        traceback.print_exc()
        # 确保进程能退出
        os._exit(1)

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

def batch_prepare_tracker_input(nms_detections: List[dict]) -> np.ndarray:
    """批量准备跟踪器输入，优化性能 - 使用numpy避免torch依赖"""
    if not nms_detections:
        return np.empty((0, 6), dtype=np.float32)
    
    # 批量提取数据，避免循环
    boxes_scores = np.array([[d['box'][0], d['box'][1], d['box'][2], d['box'][3], d['confidence']] for d in nms_detections])
    labels = np.array([NAMES.index(d['class']) if d['class'] in NAMES else 0 for d in nms_detections])
    
    # 合并为跟踪器输入格式 [x1, y1, x2, y2, score, label]
    tracker_input_array = np.column_stack([boxes_scores, labels]).astype(np.float32)
    return tracker_input_array

def batch_convert_track_results(tracked_objects: List, result: dict, camera_id: int, current_frame: int, 
                               original_detections: List[dict] = None) -> List[dict]:
    """批量转换跟踪结果，优化性能并保留原始类别信息"""
    tracked_detections = []
    
    for track in tracked_objects:
        # 高效转换tlwh到tlbr
        tlwh = track.tlwh
        tlbr = [tlwh[0], tlwh[1], tlwh[0] + tlwh[2], tlwh[1] + tlwh[3]]
        
        # 尝试从原始检测中匹配类别信息
        class_name = 'vehicle'  # 默认值
        if original_detections:
            # 通过IoU匹配找到对应的原始检测
            best_iou = 0
            for orig_det in original_detections:
                iou = GeometryUtils.calculate_iou(tlbr, orig_det['box'])
                if iou > best_iou and iou > 0.3:  # IoU阈值
                    best_iou = iou
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
