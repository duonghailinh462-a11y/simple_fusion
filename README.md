# 全息十字路口多摄像头融合系统

## 项目概述

三路视频在T字路口中心区域的实时融合系统，用雷达数据补全进入路口前在车道上的轨迹。

**核心特性**：
- ✅ 三路摄像头实时融合
- ✅ 时间戳对齐和结果缓冲
- ✅ 雷达视觉融合
- ✅ 实时输出（每帧处理）
- ✅ 无结果丢弃

## 系统架构

### 数据流
```
SDK推理 → NMS → 区域过滤 → BYTETracker跟踪 → 单路处理
    ↓
雷视融合（用原始时间戳）
    ↓
结果缓冲 → 时间对齐 → 三路匹配 → 统一输出
    ↓
JSON生成 → MQTT发送
```

### 核心模块

| 模块 | 功能 | 文件 |
|------|------|------|
| SDK推理 | 视频读取、目标检测、NMS | `SDKinfer.py` |
| 跟踪 | BYTETracker目标跟踪 | `main.py` |
| 融合 | 跨摄像头目标融合 | `Fusion.py`, `FusionComponents.py` |
| 雷视融合 | 雷达数据与视觉目标匹配 | `RadarVisionFusion.py` |
| 结果缓冲 | 三路结果时间对齐 | `ResultBuffer.py` |
| 配置 | 工具类和配置管理 | `Basic.py` |

## 快速开始

### 1. 环境配置
```bash
# 配置摄像头RTSP URL和MQTT参数
config/camera_config.ini
config/mqtt_config.ini
```

### 2. 启动系统
```bash
python main.py
```

### 3. 输出结果
- JSON数据：`output_fusion_refactored.json`
- 日志文件：`fusion_system.log`

## 关键参数

### 时间对齐
- **time_threshold**: 0.5秒（三路时间差异阈值）
- **max_buffer_size**: 100个结果（缓冲区大小）
- **处理频率**: 每一帧都处理缓冲区

### 融合配置
- **TIME_WINDOW**: 60帧（融合时间窗口）
- **SPATIAL_THRESHOLD**: 400.0米（空间距离阈值）
- **FPS**: 25帧/秒

### 跟踪配置
- **TRACK_THRESH**: 0.5（跟踪置信度阈值）
- **MATCH_THRESH**: 0.8（匹配阈值）
- **MIN_FRAMES**: 10（最小帧数）

## 三个关键区域

### 1. detect_areas（检测区域）
- **用途**：过滤检测结果，只跟踪感兴趣的区域
- **坐标系**：像素坐标

### 2. RADAR_VISION_FUSION_AREAS（雷视融合区域）
- **用途**：标记车辆进入路口前的车道区域
- **作用**：在这些区域内的目标才分配globalid，用雷达补全轨迹
- **坐标系**：像素坐标

### 3. PUBLIC_AREA_BEV（路口中心区域）
- **用途**：多摄像头在路口中心的重合区域
- **作用**：在这个区域内的目标进行跨摄像头融合
- **坐标系**：BEV坐标

## 处理流程

```
1. SDK推理 → NMS → detect_areas过滤
2. BYTETracker跟踪
3. 跟踪结果转换 + 标记in_fusion_area
4. 在融合区域内的目标 → 分配globalid（用于雷视融合）
5. 在PUBLIC_AREA_BEV内的目标 → 跨摄像头融合
6. 有globalid的目标 → 进行雷达融合
7. 结果缓冲 → 时间对齐 → 三路匹配
8. JSON生成 → MQTT发送
```

## 输出格式

### JSON结构
```json
{
  "reportTime": 1700000000000,
  "participant": [
    {
      "pid": 1,
      "type": "car",
      "plate": "GID1",
      "heading": 0,
      "lon": 113.123456,
      "lat": 23.456789
    }
  ]
}
```

## 性能指标

- **处理延迟**：< 100ms/帧
- **缓冲区大小**：最多100个结果
- **实时输出**：每一帧都处理缓冲区
- **无结果丢弃**：程序结束时自动刷新所有缓冲区

## 故障排查

### 问题：缓冲区堆积
- **原因**：三路时间差异过大，超过了time_threshold
- **解决**：增大time_threshold参数（如1.0秒）

### 问题：某一路结果没有输出
- **原因**：该路的结果与其他两路时间差异过大
- **解决**：检查摄像头时间同步是否正确

### 问题：输出延迟过大
- **原因**：处理压力大，缓冲区处理不过来
- **解决**：优化融合逻辑或调整time_threshold

## 文件说明

- `main.py` - 主程序，协调整个系统
- `ResultBuffer.py` - 三路结果缓冲和时间对齐模块
- `Fusion.py` - 跨摄像头融合协调器
- `FusionComponents.py` - 融合组件（目标管理、匹配、轨迹融合）
- `RadarVisionFusion.py` - 雷达视觉融合
- `Basic.py` - 配置和工具类
- `SDKinfer.py` - SDK推理子进程
- `TargetTrack.py` - 目标跟踪数据结构
- `RESULT_BUFFER_INTEGRATION_GUIDE.md` - ResultBuffer集成指南

## 版本信息

- **当前版本**：1.0
- **状态**：可正常运行，融合效果待测试
- **最后更新**：2025-12-08

┌─────────────────────────────────────────────────┐
│ 车辆从 C1 主责区 移动到 C1-C2 交界区             │
└─────────────────────────────────────────────────┘

【C1 的视角】
  起始阶段：目标在 C1 主责区内出现
  ├─ C1 分配 GlobalID = 101
  └─ C1_GlobalTarget(GID=101) ✓

  进入交界区：
  ├─ C1_GlobalTarget(GID=101) 继续跟踪
  └─ 独立更新自己的轨迹

【C2 的视角】
  起始阶段：目标在 C1 主责区（C2 主责区外）
  ├─ C2 创建 LocalTarget(LID=5)  [未分配GID]
  └─ 因为起始点不在 C2 主责区，不分配GlobalID

  进入交界区：
  ├─ C2_LocalTarget(LID=5) 进入融合区
  ├─ 尝试匹配其他摄像头的 GlobalTarget
  └─ 匹配成功：C2_LocalTarget(LID=5) ↔ C1_GlobalTarget(GID=101)

【融合逻辑】
  融合区域内，C1的GlobalTarget更新方式：
  
  new_pos = 0.2 × C2_local_pos + 0.8 × C1_global_pos
            ↑                      ↑
         来自C2的观测          C1的上一帧位置
  
  ⚠️ 注意：
  - C1_GlobalTarget 依然是 GlobalTarget (GID=101)
  - C2_LocalTarget 依然是 LocalTarget (LID=5)
  - 只是用 C2 的观测来修正 C1 的 GlobalTarget 轨迹
  - 最终输出的ID = 101 (来自C1)