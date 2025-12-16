# 雷达数据源抽象层设计文档

## 概述

雷达数据源抽象层 (`core/radar_source_abstraction.py`) 是一个高度解耦的架构，提供：

1. **统一的数据源接口** - 所有数据源实现相同的接口
2. **多种实现** - 支持 JSONL 文件、流式读取、多摄像头映射等
3. **装饰器模式** - 灵活的功能扩展（缓存、过滤、转换）
4. **工厂模式** - 简便的实例创建和自动选择
5. **完全解耦** - 业务逻辑与数据来源完全独立

## 架构设计

### 类关系图

```
BaseRadarSource (抽象基类)
    ├── JSONLRadarSource (完整加载)
    ├── StreamingRadarSource (流式读取)
    ├── MultiCameraRadarSource (多摄像头)
    └── CachedRadarSource (装饰器 - 缓存)

RadarObject (数据对象)
RadarFrame (数据帧)
RadarSourceFactory (工厂)
```

## 核心组件

### 1. BaseRadarSource (抽象基类)

所有数据源实现的接口：

```python
class BaseRadarSource(ABC):
    @abstractmethod
    def initialize(self) -> bool:
        """初始化数据源"""
        pass
    
    @abstractmethod
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        """获取指定时间戳的数据帧"""
        pass
    
    @abstractmethod
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        """流式生成数据帧"""
        pass
    
    @abstractmethod
    def get_all_timestamps(self) -> List[str]:
        """获取所有时间戳"""
        pass
    
    @abstractmethod
    def close(self) -> None:
        """关闭数据源"""
        pass
    
    @abstractmethod
    def get_stats(self) -> Dict:
        """获取统计信息"""
        pass
```

### 2. JSONLRadarSource

**特点：**
- 一次性加载整个 JSONL 文件到内存
- 支持快速随机访问
- 适合中等规模数据集（< 500MB）

**使用场景：**
- 开发测试阶段
- 数据集相对较小
- 需要频繁随机访问

**示例：**
```python
source = JSONLRadarSource('radar_data.jsonl')
source.initialize()

# 获取特定时间戳的数据
frame = source.get_frame('2025-11-21 11:59:10.171')

# 获取所有时间戳
timestamps = source.get_all_timestamps()

source.close()
```

### 3. StreamingRadarSource

**特点：**
- 逐帧读取，按需生成
- 低内存占用（固定缓冲区）
- 支持处理大型文件

**使用场景：**
- 处理超大文件（> 500MB）
- 实时流处理
- 内存受限环境

**示例：**
```python
source = StreamingRadarSource('radar_data.jsonl', buffer_size=100)
source.initialize()

# 流式处理数据
for frame in source.stream_frames():
    # 处理每一帧
    print(f"{frame.timestamp}: {frame.get_object_count()} 个目标")

source.close()
```

### 4. MultiCameraRadarSource

**特点：**
- 自动按摄像头分离数据
- 支持按摄像头获取数据
- 包装其他数据源（装饰器模式）

**使用场景：**
- 多摄像头融合系统
- 需要按摄像头处理数据

**示例：**
```python
# 创建多摄像头数据源
source = RadarSourceFactory.create_multi_camera_source('radar.jsonl')
source.initialize()

# 获取摄像头 1 的所有时间戳
timestamps_c1 = source.get_timestamps_by_camera(1)

# 获取摄像头 1 的特定时间戳数据
frame = source.get_frame_by_camera(1, '2025-11-21 11:59:10.171')

# 流式处理摄像头 2 的数据
for frame in source.stream_frames_by_camera(2):
    # 处理
    pass

source.close()
```

### 5. CachedRadarSource (装饰器)

**特点：**
- 缓存频繁访问的帧
- LRU 淘汰策略
- 可配置缓存大小

**使用场景：**
- 频繁访问同一帧
- 减少磁盘 I/O
- 性能优化

**示例：**
```python
# 创建带缓存的数据源
base_source = JSONLRadarSource('radar.jsonl')
cached_source = CachedRadarSource(base_source, cache_size=500)
cached_source.initialize()

# 多次访问同一帧 - 第一次从磁盘，后续从缓存
for i in range(10):
    frame = cached_source.get_frame(timestamp)  # 后9次使用缓存

stats = cached_source.get_stats()
print(f"缓存命中率: {stats['cache_hit_rate']:.1%}")

cached_source.close()
```

## 数据结构

### RadarObject

单个雷达目标对象：

```python
class RadarObject:
    id: str                 # 雷达目标 ID
    latitude: float         # 纬度
    longitude: float        # 经度
    speed: float            # 速度
    azimuth: float          # 方位角
    lane: str               # 车道信息
    timestamp_str: str      # 原始时间戳字符串
    source_ip: str          # 数据源 IP（用于摄像头映射）
```

### RadarFrame

一个时间戳对应的所有雷达目标：

```python
class RadarFrame:
    timestamp: str          # 时间戳
    objects: List[RadarObject]  # 目标列表
    metadata: Dict          # 元数据
    
    @property
    def camera_id(self) -> Optional[int]:
        """从数据推断摄像头 ID"""
        pass
    
    def filter_by_camera(self, camera_id: int) -> RadarFrame:
        """按摄像头过滤"""
        pass
    
    def filter_by_lane(self, lane: str) -> List[RadarObject]:
        """按车道过滤"""
        pass
    
    def get_object_count(self) -> int:
        """获取目标数量"""
        pass
```

## 工厂模式

### RadarSourceFactory

简便创建数据源：

```python
# 创建 JSONL 数据源（带缓存）
source = RadarSourceFactory.create_jsonl_source('radar.jsonl', cached=True)

# 创建流式数据源
source = RadarSourceFactory.create_streaming_source('radar.jsonl', buffer_size=100)

# 创建多摄像头数据源
source = RadarSourceFactory.create_multi_camera_source('radar.jsonl', use_streaming=False)

# 自动选择（根据文件大小）
source = RadarSourceFactory.create_auto('radar.jsonl')
```

### 自动选择逻辑

| 文件大小 | 选择 | 原因 |
|---------|------|------|
| < 100MB | JSONLRadarSource | 加载快，随机访问快 |
| > 100MB | StreamingRadarSource | 内存占用少，适合大文件 |

## 使用指南

### 基本模式

```python
from core.radar_source_abstraction import RadarSourceFactory

# 1. 创建数据源
source = RadarSourceFactory.create_jsonl_source('radar_data.jsonl')

# 2. 初始化
if not source.initialize():
    raise RuntimeError("初始化失败")

# 3. 获取数据
timestamps = source.get_all_timestamps()
frame = source.get_frame(timestamps[0])

# 4. 处理数据
for obj in frame.objects:
    print(f"目标 {obj.id}: ({obj.latitude}, {obj.longitude})")

# 5. 关闭
source.close()
```

### 多摄像头模式

```python
from core.radar_source_abstraction import RadarSourceFactory

# 为每个摄像头创建独立的融合处理器
processors = {
    1: RadarVisionFusionProcessor(camera_id=1),
    2: RadarVisionFusionProcessor(camera_id=2),
    3: RadarVisionFusionProcessor(camera_id=3),
}

# 使用多摄像头数据源
source = RadarSourceFactory.create_multi_camera_source('radar.jsonl')
source.initialize()

# 为每个摄像头处理数据
for camera_id in range(1, 4):
    timestamps = source.get_timestamps_by_camera(camera_id)
    processor = processors[camera_id]
    
    for ts in timestamps:
        frame = source.get_frame_by_camera(camera_id, ts)
        processor.add_radar_data(ts, frame.objects)

source.close()
```

### 流式处理大文件

```python
from core.radar_source_abstraction import RadarSourceFactory

# 使用流式数据源处理超大文件
source = RadarSourceFactory.create_streaming_source('radar_data.jsonl', buffer_size=100)
source.initialize()

frame_count = 0
for frame in source.stream_frames():
    # 处理每一帧（内存占用恒定）
    process_frame(frame)
    
    frame_count += 1
    if frame_count % 100 == 0:
        print(f"已处理 {frame_count} 帧")

source.close()
```

## 集成到现有系统

### 与 RadarVisionFusionProcessor 集成

```python
from core.radar_source_abstraction import RadarSourceFactory
from core.RadarVisionFusion import RadarVisionFusionProcessor

# 创建数据源
radar_source = RadarSourceFactory.create_jsonl_source('radar.jsonl')
radar_source.initialize()

# 创建融合处理器
processor = RadarVisionFusionProcessor(camera_id=1)

# 添加雷达数据
for frame in radar_source.stream_frames():
    processor.add_radar_data(frame.timestamp, frame.objects)

# 处理视觉帧
vision_frame = get_vision_frame()
results = processor.process_frame(vision_frame.timestamp, vision_frame.objects)

radar_source.close()
```

### 与 StreamingFusionPipeline 集成

```python
from core.radar_source_abstraction import RadarSourceFactory
from core.StreamingDataLoader import StreamingFusionPipeline
from core.RadarVisionFusion import RadarVisionFusionProcessor

# 创建融合处理器
processors = {
    1: RadarVisionFusionProcessor(camera_id=1),
    2: RadarVisionFusionProcessor(camera_id=2),
    3: RadarVisionFusionProcessor(camera_id=3),
}

# 创建融合管线（使用新的数据源抽象层）
radar_source = RadarSourceFactory.create_multi_camera_source('radar.jsonl')
radar_source.initialize()

# 处理视觉帧
for camera_id, vision_timestamp, vision_objects in vision_stream:
    # 获取对应的雷达数据
    timestamps = radar_source.get_timestamps_by_camera(camera_id)
    # ... 处理 ...

radar_source.close()
```

## 性能特性

### 内存占用

| 数据源 | 内存占用 | 加载时间 | 随机访问 |
|--------|----------|---------|---------|
| JSONLRadarSource | O(n) - 完全加载 | ~2s/100MB | 快 |
| StreamingRadarSource | O(b) - 缓冲区固定 | ~0.1s | 慢 |
| MultiCameraRadarSource | O(n) | ~2s/100MB | 快 |
| CachedRadarSource | O(min(n,c)) - 缓存大小 | 可配置 | 很快 |

### 时间复杂度

| 操作 | JSONL | Streaming | MultiCamera |
|------|--------|-----------|-------------|
| initialize() | O(n) | O(1) | O(n) |
| get_frame() | O(1) | O(1)* | O(1) |
| stream_frames() | O(n) | O(n) | O(n) |
| get_all_timestamps() | O(1) | O(n)** | O(1) |

*Streaming 只能获取缓冲区内的帧
**Streaming 需要遍历整个文件

## 扩展指南

### 实现自定义数据源

```python
from core.radar_source_abstraction import BaseRadarSource, RadarFrame
from typing import List, Dict, Optional, Generator

class CustomRadarSource(BaseRadarSource):
    def __init__(self, config: Dict):
        self.config = config
        self.is_initialized = False
    
    def initialize(self) -> bool:
        # 初始化逻辑
        self.is_initialized = True
        return True
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        # 实现获取帧的逻辑
        pass
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        # 实现流式生成逻辑
        pass
    
    def get_all_timestamps(self) -> List[str]:
        # 实现获取所有时间戳
        pass
    
    def close(self) -> None:
        # 清理逻辑
        pass
    
    def get_stats(self) -> Dict:
        # 返回统计信息
        return {}
```

### 实现自定义装饰器

```python
from core.radar_source_abstraction import BaseRadarSource

class FilteredRadarSource(BaseRadarSource):
    """过滤装饰器 - 只返回特定摄像头的数据"""
    
    def __init__(self, source: BaseRadarSource, camera_id: int):
        self.source = source
        self.camera_id = camera_id
    
    def initialize(self) -> bool:
        return self.source.initialize()
    
    def get_frame(self, timestamp: str) -> Optional[RadarFrame]:
        frame = self.source.get_frame(timestamp)
        if frame:
            return frame.filter_by_camera(self.camera_id)
        return None
    
    def stream_frames(self) -> Generator[RadarFrame, None, None]:
        for frame in self.source.stream_frames():
            filtered = frame.filter_by_camera(self.camera_id)
            if filtered.get_object_count() > 0:
                yield filtered
    
    def get_all_timestamps(self) -> List[str]:
        return self.source.get_all_timestamps()
    
    def close(self) -> None:
        self.source.close()
    
    def get_stats(self) -> Dict:
        return self.source.get_stats()
```

## 总结

| 特性 | 描述 |
|------|------|
| **解耦性** | 业务逻辑与数据来源完全独立 |
| **扩展性** | 易于添加新的数据源实现或装饰器 |
| **灵活性** | 支持多种使用场景和数据源 |
| **性能** | 根据场景选择最优实现 |
| **易用性** | 统一接口和工厂模式简化使用 |
| **维护性** | 代码清晰，职责单一 |

## 相关文件

- `core/radar_source_abstraction.py` - 核心实现
- `examples/radar_source_integration_example.py` - 集成示例
- `test_radar_source_manager.py` - 单元测试

