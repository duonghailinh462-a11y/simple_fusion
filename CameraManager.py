#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头管理器模块
负责SDK进程管理、队列管理、预热阶段和RTSP连接测试
"""

import os
import sys
import time
import logging
import multiprocessing
from typing import Dict, List, Tuple

logger = logging.getLogger(__name__)


class CameraManager:
    """
    摄像头管理器 - 负责多摄像头的进程和队列管理
    
    职责：
    - 创建和管理SDK推理子进程
    - 管理摄像头队列
    - 执行RTSP连接测试
    - 管理预热阶段
    - 处理进程生命周期
    """
    
    def __init__(self, video_paths: Dict[int, str], cancel_flag: multiprocessing.Value):
        """
        初始化摄像头管理器
        
        Args:
            video_paths: 摄像头ID到视频路径的映射 {1: path1, 2: path2, 3: path3}
            cancel_flag: 全局取消标志
        """
        self.video_paths = video_paths
        self.cancel_flag = cancel_flag
        self.queues: Dict[int, multiprocessing.Queue] = {}
        self.processes: List[multiprocessing.Process] = []
        self.rtsp_connection_status: Dict[int, bool] = {}
        
        logger.info("CameraManager 初始化完成")
    
    def create_queues(self, maxsize: int = 10) -> Dict[int, multiprocessing.Queue]:
        """
        为所有摄像头创建队列
        
        Args:
            maxsize: 队列最大大小
            
        Returns:
            摄像头ID到队列的映射
        """
        self.queues = {i: multiprocessing.Queue(maxsize=maxsize) for i in [1, 2, 3]}
        logger.info(f"已创建3个队列 (maxsize={maxsize})")
        return self.queues
    
    def test_rtsp_connection(self, rtsp_url: str, timeout: int = 5) -> bool:
        """
        测试RTSP连接是否可用
        
        Args:
            rtsp_url: RTSP URL
            timeout: 超时时间（秒）
            
        Returns:
            连接是否成功
        """
        try:
            import cv2
            # 尝试使用 PyAV 来测试，更一致
            try:
                import av
                av.logging.set_level(av.logging.ERROR)
                container = av.open(
                    rtsp_url, 'r',
                    options={'rtsp_transport': 'tcp', 'stimeout': str(timeout * 1000000)},
                    timeout=timeout
                )
                container.decode(video=0)
                container.close()
                return True
            except ImportError:
                # 回退到 OpenCV
                cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
            logger.debug(f"RTSP连接测试异常: {e}")
            return False
    
    def test_all_rtsp_connections(self) -> Dict[int, bool]:
        """
        测试所有RTSP连接
        
        Returns:
            摄像头ID到连接状态的映射
        """
        logger.info("测试RTSP连接 (使用PyAV/OpenCV)")
        self.rtsp_connection_status = {}
        
        for cam_id in [1, 2, 3]:
            video_path = self.video_paths[cam_id]
            
            # 只测试RTSP URL，不测试本地文件
            if video_path.startswith('rtsp://'):
                logger.info(f"测试 Camera{cam_id}: {video_path[:60]}...")
                is_connected = self.test_rtsp_connection(video_path, timeout=5)
                self.rtsp_connection_status[cam_id] = is_connected
                
                if is_connected:
                    logger.info(f"Camera{cam_id} RTSP连接成功")
                else:
                    logger.error(f"Camera{cam_id} RTSP连接失败: {video_path}")
            else:
                # 本地视频文件，跳过测试
                self.rtsp_connection_status[cam_id] = True
                logger.info(f"Camera{cam_id} 使用本地视频文件: {video_path}")
        
        return self.rtsp_connection_status
    
    def start_all_cameras(self, worker_process_func):
        """
        启动所有摄像头的SDK推理进程
        
        Args:
            worker_process_func: 工作进程函数 (create_sdk_worker_process)
        """
        logger.info("启动SDK版多摄像头融合系统")
        
        # 检查RTSP连接状态，给出警告
        failed_cameras = [
            cam_id for cam_id, status in self.rtsp_connection_status.items() 
            if not status
        ]
        if failed_cameras:
            logger.warning(f"警告: {len(failed_cameras)}个摄像头RTSP连接测试失败: {failed_cameras}")
            time.sleep(2)
        
        # 启动所有摄像头进程
        for camera_id in [1, 2, 3]:
            video_path = self.video_paths[camera_id]
            
            if not self.rtsp_connection_status.get(camera_id, True):
                logger.warning(f"Camera{camera_id} RTSP连接测试失败，但仍尝试启动SDK")
            
            process = multiprocessing.Process(
                target=worker_process_func,
                args=(camera_id, video_path, self.queues[camera_id]),
                daemon=True
            )
            self.processes.append(process)
            process.start()
            logger.info(f"启动Camera{camera_id} SDK推理进程")
        
        logger.info("所有SDK推理进程已启动")
    
    def wait_for_preheat(self, timeout: int = 30) -> bool:
        """
        等待预热阶段完成（所有摄像头都推送了第一帧数据）
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            是否成功完成预热
        """
        logger.info("进入预热阶段，等待所有摄像头推送第一帧数据")
        
        start_time = time.time()
        ready_cameras = {i: False for i in [1, 2, 3]}
        last_report_time = time.time()
        
        while time.time() - start_time < timeout:
            # 检查队列是否有数据
            for cam_id in [1, 2, 3]:
                if not ready_cameras[cam_id] and not self.queues[cam_id].empty():
                    ready_cameras[cam_id] = True
                    logger.info(f"摄像头 C{cam_id} 已就绪")
            
            # 所有摄像头都就绪
            if all(ready_cameras.values()):
                logger.info("所有摄像头均已就绪，预热完成")
                return True
            
            # 每5秒打印一次状态
            current_time = time.time()
            if current_time - last_report_time >= 5:
                elapsed = int(current_time - start_time)
                queue_sizes = {i: self.queues[i].qsize() for i in [1, 2, 3]}
                process_status = [p.is_alive() for p in self.processes]
                alive_count = sum(1 for p in self.processes if p.is_alive())
                
                logger.info(f"预热进度 ({elapsed}s/{timeout}s): 队列 {queue_sizes}")
                logger.info(f"准备就绪: C1={ready_cameras[1]}, C2={ready_cameras[2]}, C3={ready_cameras[3]}")
                logger.info(f"SDK进程: {alive_count}/3 运行中")
                
                last_report_time = current_time
            
            time.sleep(0.5)
        
        # 预热超时
        logger.error("预热超时！")
        for cam_id, is_ready in ready_cameras.items():
            if not is_ready:
                logger.error(f"摄像头 C{cam_id} 未能在 {timeout} 秒内推送数据")
        
        logger.warning("程序将继续运行，但可能会出现同步问题")
        return False
    
    def get_queues(self) -> Dict[int, multiprocessing.Queue]:
        """获取所有队列"""
        return self.queues
    
    def get_processes(self) -> List[multiprocessing.Process]:
        """获取所有进程"""
        return self.processes
    
    def get_process_status(self) -> Dict[int, bool]:
        """
        获取所有进程的状态
        
        Returns:
            摄像头ID到进程状态的映射 (True=运行中, False=已停止)
        """
        return {i: self.processes[i-1].is_alive() for i in [1, 2, 3]}
    
    def stop_all_cameras(self):
        """停止所有摄像头进程"""
        logger.info("停止所有摄像头进程")
        
        self.cancel_flag.value = True
        
        for i, process in enumerate(self.processes, 1):
            if process.is_alive():
                logger.info(f"终止Camera{i}进程")
                process.terminate()
                process.join(timeout=5)
                if process.is_alive():
                    logger.warning(f"Camera{i}进程未能正常终止，强制杀死")
                    process.kill()
        
        logger.info("所有摄像头进程已停止")
    
    def get_queue_status(self) -> Dict[int, int]:
        """
        获取所有队列的大小
        
        Returns:
            摄像头ID到队列大小的映射
        """
        return {i: self.queues[i].qsize() for i in [1, 2, 3]}
    
    def is_all_alive(self) -> bool:
        """检查所有进程是否都在运行"""
        return all(p.is_alive() for p in self.processes)
    
    def is_any_alive(self) -> bool:
        """检查是否有任何进程在运行"""
        return any(p.is_alive() for p in self.processes)
    
    def get_alive_count(self) -> int:
        """获取运行中的进程数"""
        return sum(1 for p in self.processes if p.is_alive())
