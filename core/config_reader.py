#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置读取器
用于读取camera_config.ini和mqtt_config.ini配置文件
"""

import configparser
import os
from typing import List, Dict, Optional

class ConfigReader:
    """配置读取器类"""
    
    def __init__(self, config_dir: str = "config"):
        """
        初始化配置读取器
        
        Args:
            config_dir: 配置文件目录路径
        """
        self.config_dir = config_dir
        
    def read_camera_config(self, config_file: str = "camera_config.ini") -> List[Dict[str, str]]:
        """
        读取摄像头配置文件
        
        Args:
            config_file: 摄像头配置文件名
            
        Returns:
            List[Dict]: 摄像头配置列表，每个字典包含name, rtsp_url, enabled
        """
        config_path = os.path.join(self.config_dir, config_file)
        cameras = []
        
        if not os.path.exists(config_path):
            print(f"⚠️  配置文件不存在: {config_path}")
            return cameras
            
        try:
            config = configparser.ConfigParser()
            config.read(config_path, encoding='utf-8')
            
            for section in config.sections():
                if section.startswith('Camera'):
                    camera_info = {
                        'name': config.get(section, 'name', fallback='Unknown'),
                        'rtsp_url': config.get(section, 'rtsp_url', fallback=''),
                        'enabled': config.getboolean(section, 'enabled', fallback=False)
                    }
                    cameras.append(camera_info)
                    print(f"📷 读取摄像头配置: {camera_info['name']} - {camera_info['rtsp_url']} (启用: {camera_info['enabled']})")
                    
        except Exception as e:
            print(f"❌ 读取摄像头配置失败: {e}")
            
        return cameras
    
    def get_enabled_cameras(self, config_file: str = "camera_config.ini") -> List[Dict[str, str]]:
        """
        获取启用的摄像头配置
        
        Args:
            config_file: 摄像头配置文件名
            
        Returns:
            List[Dict]: 启用的摄像头配置列表
        """
        all_cameras = self.read_camera_config(config_file)
        enabled_cameras = [cam for cam in all_cameras if cam['enabled']]
        print(f"✅ 找到 {len(enabled_cameras)} 个启用的摄像头")
        return enabled_cameras
    
    def get_camera_urls(self, config_file: str = "camera_config.ini") -> List[str]:
        """
        获取所有启用的摄像头URL列表
        
        Args:
            config_file: 摄像头配置文件名
            
        Returns:
            List[str]: 摄像头URL列表
        """
        enabled_cameras = self.get_enabled_cameras(config_file)
        urls = [cam['rtsp_url'] for cam in enabled_cameras if cam['rtsp_url']]
        print(f"📡 获取到 {len(urls)} 个摄像头URL")
        return urls
    
    def read_mqtt_config(self, config_file: str = "mqtt_config.ini") -> Optional[Dict[str, str]]:
        """
        读取MQTT配置文件
        
        Args:
            config_file: MQTT配置文件名
            
        Returns:
            Dict: MQTT配置字典，包含broker, port, client_id等
        """
        config_path = os.path.join(self.config_dir, config_file)
        
        if not os.path.exists(config_path):
            print(f"⚠️  MQTT配置文件不存在: {config_path}")
            return None
            
        try:
            config = configparser.ConfigParser()
            config.read(config_path, encoding='utf-8')
            
            mqtt_config = {}
            if 'MQTT' in config:
                mqtt_config = {
                    'broker': config.get('MQTT', 'broker', fallback='localhost'),
                    'port': config.getint('MQTT', 'port', fallback=1883),
                    'client_id': config.get('MQTT', 'client_id', fallback='GRG16'),
                    'username': config.get('MQTT', 'username', fallback=''),
                    'password': config.get('MQTT', 'password', fallback=''),
                    'topic_template': config.get('MQTT', 'topic_template', fallback='GRGUpload/{client_id}/RSM')
                }
                print(f"📡 读取MQTT配置: {mqtt_config['broker']}:{mqtt_config['port']}")
            else:
                print("❌ 配置文件中未找到[MQTT]部分")
                
        except Exception as e:
            print(f"❌ 读取MQTT配置失败: {e}")
            return None
            
        return mqtt_config

# 测试函数
def test_config_reader():
    """测试配置读取器"""
    print("🔍 测试配置读取器...")
    
    reader = ConfigReader()
    
    # 测试摄像头配置读取
    print("\n=== 摄像头配置测试 ===")
    cameras = reader.read_camera_config()
    print(f"总共读取到 {len(cameras)} 个摄像头配置")
    
    enabled_cameras = reader.get_enabled_cameras()
    print(f"其中 {len(enabled_cameras)} 个已启用")
    
    urls = reader.get_camera_urls()
    print(f"获取到 {len(urls)} 个URL:")
    for i, url in enumerate(urls, 1):
        print(f"  {i}. {url}")
    
    # 测试MQTT配置读取
    print("\n=== MQTT配置测试 ===")
    mqtt_config = reader.read_mqtt_config()
    if mqtt_config:
        print("MQTT配置读取成功")
    else:
        print("MQTT配置读取失败")

if __name__ == "__main__":
    test_config_reader()

