# 项目结构文档

## 📁 目录结构

```
/zhw/no-frame-sync/
├── main.py                              # 主程序入口（融合与同步的消费者）
├── main_1015.py                         # 备用主程序
│
├── core/                                # 核心模块（公共库）
│   ├── __init__.py
│   ├── Basic.py                        # 基础工具：配置、检测工具、几何工具、性能监控
│   ├── config_reader.py                # 配置文件读取器
│   ├── mqtt_publisher.py               # MQTT发布器
│   ├── Fusion.py                       # 跨摄像头融合逻辑
│   ├── FusionComponents.py             # 融合数据结构
│   ├── RadarVisionFusion.py            # 雷达视觉融合处理
│   ├── ResultBuffer.py                 # 结果缓冲和输出管理
│   └── fusion_debug.py                 # 融合过程调试器
│
├── vision/                              # 视觉模块（图像处理与跟踪）
│   ├── __init__.py
│   ├── SDKinfer.py                    # SDK推理类（生产者）
│   ├── rtsp_reader.py                 # RTSP流读取器
│   ├── CameraManager.py               # 摄像头管理器（进程管理）
│   └── TargetTrack.py                 # 目标跟踪缓冲
│
├── radar/                               # 雷达模块
│   ├── __init__.py
│   ├── RadarDataFilter.py             # 雷达数据筛选和转发
│   ├── RadarDataManager.py            # 雷达数据管理器
│   └── RadarFusionOrchestrator.py     # 雷达融合协调器 (v2新增)
│
├── data/                                # 数据模块
│   └── __init__.py
│
├── config/                              # 配置文件
│   ├── fusion_config.py               # 融合配置
│   ├── camera_config.ini              # 摄像头配置（RTSP URLs）
│   └── mqtt_config.ini                # MQTT配置
│
├── create_timelapse_video_optimized.py # 延时视频创建工具
│
├── README.md                            # 项目说明
├── RESULT_BUFFER_INTEGRATION_GUIDE.md  # 结果缓冲集成指南
└── PROJECT_STRUCTURE.md                # 本文档
```

## 🔄 导入规范

### 项目重构后的导入方式

所有子模块统一使用相对于项目根目录的导入路径：

#### ✅ 正确的导入方式

```python
# vision/SDKinfer.py
from core.Basic import NAMES

# radar/RadarDataFilter.py
from core.Basic import Config, GeometryUtils, CAMERA_MATRICES, ...

# main.py
from vision.rtsp_reader import RTSPStreamReader
from core.mqtt_publisher import MqttPublisher
from core.Basic import Config, DetectionUtils, GeometryUtils, PerformanceMonitor
from vision.TargetTrack import TargetBuffer
from core.Fusion import CrossCameraFusion
from core.RadarVisionFusion import RadarVisionFusionProcessor, RadarDataLoader, OutputObject
from vision.CameraManager import CameraManager
from core.ResultBuffer import ResultOutputManager
```

### ❌ 旧的导入方式（已弃用）

```python
# 不再使用这种方式
from Basic import NAMES
import Basic
```

## 📋 模块说明

### 核心模块 (core/)

#### `Basic.py`
- **Config**: 全局配置类（摄像头参数、FPS、图像尺寸等）
- **DetectionUtils**: 检测工具（NMS、IOU计算等）
- **GeometryUtils**: 几何工具（像素/BEV/地理坐标转换等）
- **PerformanceMonitor**: 性能监控器
- **NAMES**: YOLO类别名称常量
- **CAMERA_MATRICES**: 摄像头内参

#### `Fusion.py`
- **CrossCameraFusion**: 跨摄像头融合处理
- 全局目标跟踪（GlobalTarget）
- 本地目标跟踪（LocalTarget）
- 目标匹配逻辑

#### `RadarVisionFusion.py`
- **RadarVisionFusionProcessor**: 雷达视觉融合处理器
- **RadarDataLoader**: 雷达数据加载器
- **OutputObject**: 输出对象数据类

#### `ResultBuffer.py`
- **ResultOutputManager**: 结果缓冲和输出管理
- 三路融合结果缓冲
- 时间戳对齐
- MQTT/JSON输出

### 视觉模块 (vision/)

#### `SDKinfer.py`
- **yolov5_SDK**: SDK推理类（在子进程中运行）
- YOLO检测和推理
- 生产者角色：将检测结果放入队列

#### `CameraManager.py`
- **CameraManager**: 摄像头管理器
- 进程管理（启动/停止）
- RTSP连接测试
- 预热阶段处理

#### `TargetTrack.py`
- **TargetBuffer**: 目标跟踪缓冲

### 雷达模块 (radar/)

#### `RadarDataFilter.py`
- **RadarGeoFusionArea**: 融合区域地理范围
- **PixelToGeoConverter**: 像素到地理坐标转换
- 雷达数据筛选逻辑

#### `RadarFusionOrchestrator.py` (v2新增)
- **RadarFusionOrchestrator**: 雷达融合协调器
- 统一协调雷达融合流程
- 地理区域过滤、按摄像头融合、时间戳匹配
- 目标收集和转换

## 🔗 数据流

### 处理流程

```
1. 子进程 (生产者) - SDK推理
   ↓
   vision.SDKinfer.yolov5_SDK.run()
   ├─ 读取视频/RTSP流
   ├─ 进行YOLO检测
   └─ 将结果放入队列: {detections: [...], timestamp: ...}

2. 主进程 (消费者) - 融合与输出
   ↓
   main.py 主循环
   ├─ 从队列读取单路检测结果
   ├─ 局部跟踪 (BYTETracker)
   ├─ 跨摄像头融合 (CrossCameraFusion)
   ├─ 雷达融合 (RadarVisionFusionProcessor)
   ├─ 结果缓冲 (ResultOutputManager)
   └─ 三路匹配输出
```

### 关键数据结构

#### 检测结果 (detection)
```python
{
    'box': [x1, y1, x2, y2],           # 检测框
    'confidence': float,                # 置信度
    'class': str,                       # 类别名称
    'track_id': int,                    # 跟踪ID
    'center_point': [x, y],            # 中心点
    'timestamp': float,                # 时间戳
    'camera_id': int,                  # 摄像头ID
    'in_fusion_area': bool             # 是否在融合区域
}
```

#### 全局目标 (GlobalTarget)
```python
{
    'global_id': int,                  # 全局唯一ID
    'camera_id': int,                  # 来源摄像头
    'class_name': str,                 # 类别名称
    'bev_trajectory': [[x, y], ...],  # BEV轨迹
    'confidence': float,               # 置信度
    'radar_id': int or None           # 关联的雷达ID
}
```

## 🚀 启动方式

### 方式1：运行主程序
```bash
cd /zhw/no-frame-sync
python3 main.py
```

### 方式2：从特定帧启动
```bash
python3 main_1015.py
```

## 📊 性能监控

项目使用 `PerformanceMonitor` 类进行性能监控：

- 各阶段耗时统计
- 队列大小统计
- 检测数量统计
- 融合结果统计

监控结果输出到 `fusion_system.log` 文件。

## 🔧 配置文件

### `config/fusion_config.py`
融合系统的配置参数

### `config/camera_config.ini`
摄像头配置（RTSP URLs）

### `config/mqtt_config.ini`
MQTT服务器配置

## 📝 注意事项

1. **导入路径**: 所有导入都基于项目根目录，确保运行时在正确的目录
2. **多进程**: 子进程（SDK推理）与主进程（融合）分离，提高效率
3. **时间戳**: 使用统一的时间戳格式 `YYYY-MM-DD HH:MM:SS.mmm`
4. **日志**: 所有日志输出到 `fusion_system.log` 文件
5. **队列**: 使用 `multiprocessing.Queue` 实现进程间通信

## ✅ 验证

项目结构已验证，所有导入路径正确：
- 所有 `.py` 文件语法检查通过
- 导入依赖已正确解析
- 模块可正常加载


