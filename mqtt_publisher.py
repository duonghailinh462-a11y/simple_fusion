# -*- coding: utf-8 -*-
"""
author： pony
date: 2025/03/21
action：使用rtsp协议读取视频流，通过mqtt协议实时推送内容
"""

import paho.mqtt.client as mqtt
import time
import json
import logging
import threading
from typing import Dict, Any, List
from configparser import ConfigParser

# 尝试导入 numpy（如果可用）
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class NumpyJSONEncoder(json.JSONEncoder):
    """自定义JSON编码器，处理NumPy类型"""
    def default(self, obj):
        if HAS_NUMPY:
            if isinstance(obj, (np.integer, np.int_, np.intc, np.intp, np.int8,
                                np.int16, np.int32, np.int64, np.uint8, np.uint16,
                                np.uint32, np.uint64)):
                return int(obj)
            elif isinstance(obj, (np.floating, np.float_, np.float16, np.float32, np.float64)):
                return float(obj)
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.bool_):
                return bool(obj)
        return super(NumpyJSONEncoder, self).default(obj)


class MqttPublisher:
    """
    MQTT 数据发布器，支持连接、发布 JSON 消息
    """

    def __init__(self, config_file: str):
        self.config_file = config_file
        self.config = self._load_config()
        self.client: mqtt.Client = None
        self.lock = threading.Lock()
        self.connected = False

    def _load_config(self) -> Dict[str, str]:
        """加载配置文件"""
        parser = ConfigParser()
        try:
            # 显式以 UTF-8 编码读取文件 ✅ 关键修改
            with open(self.config_file, 'r', encoding='utf-8') as f:
                parser.read_file(f)

            if not parser.has_section("MQTT"):
                raise ValueError("配置文件缺少 [MQTT] 节")

            config = dict(parser.items("MQTT"))

            # 转换端口为 int
            config['port'] = int(config.get('port', 1883))

            return config
        except Exception as e:
            logger.error(f"加载 MQTT 配置失败: {e}")
            raise

    def connect(self):
        """连接到 MQTT 服务器，并等待连接成功"""
        try:
            self.client = mqtt.Client(client_id=self.config['client_id'])
            username = self.config.get('username')
            password = self.config.get('password')
            if username:
                self.client.username_pw_set(username, password)

            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect

            self.client.connect(self.config['broker'], self.config['port'], keepalive=60)
            self.client.loop_start()

            # 等待连接成功（最多 10 秒）
            timeout = 10
            while not self.connected and timeout > 0:
                time.sleep(0.5)
                timeout -= 0.5
            if not self.connected:
                raise TimeoutError("MQTT 连接超时")

        except Exception as e:
            logger.error(f"MQTT 连接失败: {e}")
            raise

    def _on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        if rc == 0:
            logger.info(f"MQTT 连接成功: {self.config['broker']}:{self.config['port']}")
            self.connected = True
        else:
            logger.error(f"MQTT 连接失败，返回码: {rc}")
            self.connected = False

    def _on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self.connected = False
        if rc != 0:
            logger.warning("MQTT 意外断开连接")
        else:
            logger.info("MQTT 已断开")

    def publish_rsm(self, participants: List[Dict[str, Any]]):
        """
        发布轨迹数据（RSM 格式）
        :param participants: 参与者列表，格式见下方
        """
        if not self.connected:
            logger.warning("MQTT 未连接，跳过发布")
            return False

        # 构造消息
        message = {
            "reportTime": int(time.time() * 1000),  # 毫秒时间戳
            "participant": participants
        }

        # 生成主题（支持 client_id 变量）
        topic = self.config['topic_template'].format(client_id=self.config['client_id'])

        try:
            payload = json.dumps(message, ensure_ascii=False, cls=NumpyJSONEncoder)
            result = self.client.publish(topic, payload, qos=1)  # QoS=1 确保送达

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.info(f"消息已发布到主题: {topic}")
                return True
            else:
                logger.error(f"消息发布失败: {result.rc}")
                return False

        except Exception as e:
            logger.error(f"发布消息时发生异常: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        logger.info("MQTT 已断开")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()




