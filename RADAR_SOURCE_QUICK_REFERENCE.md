# 🚀 雷达数据源 API 快速参考

## 快速选择

| 场景 | 推荐方案 | 代码 |
|------|--------|------|
| 小文件 + 随机访问 | JSONLRadarSource | `create_jsonl_source()` |
| 大文件 + 流式处理 | StreamingRadarSource | `create_streaming_source()` |
| 多摄像头系统 | MultiCameraRadarSource | `create_multi_camera_source()` |
| 频繁访问同数据 | JSONL + 缓存 | `create_jsonl_source(cached=True)` |
| 自动选择 | 自适应 | `create_auto()` |

---

## 3 行代码快速开始

### 从文件读取

```python
from core.radar_source_abstraction import RadarSourceFactory

source = RadarSourceFactory.create_auto('radar.jsonl')
source.initialize()
for frame in source.stream_frames():
    print(f"{frame.timestamp}: {len(frame.objects)} 个目标")
source.close()
```

### 实时 TCP 数据

```python
from radar.server_wrapper import RealtimeRadarServer

server = RealtimeRadarServer(camera_id=1)
server.start()
for frame in server.get_source().stream_frames():
    print(f"{frame.timestamp}: {len(frame.objects)} 个目标")
server.stop()
```

---

## API 速查表

### RadarFrame 对象

```python
frame.timestamp              # 时间戳字符串
frame.objects              # RadarObject 列表
frame.get_object_count()   # 获取目标数量
frame.camera_id            # 摄像头 ID
```

### RadarObject 对象

```python
obj.id                # 目标 ID
obj.latitude          # 纬度
obj.longitude         # 经度
obj.speed             # 速度 (m/s)
obj.azimuth           # 方位角 (度)
obj.lane              # 车道
obj.source_ip         # 数据源 IP
```

### BaseRadarSource 接口

```python
source.initialize()          # 初始化
source.get_frame(ts)         # 获取单帧
source.stream_frames()       # 流式获取
source.get_all_timestamps()  # 获取所有时间戳
source.get_stats()           # 获取统计信息
source.close()               # 关闭
```

---

## 工厂方法详解

### JSONLRadarSource（文件）

```python
source = RadarSourceFactory.create_jsonl_source(
    file_path='radar.jsonl',
    cached=True  # 启用缓存
)
source.initialize()
```

**特点**：
- ✅ 支持随机访问
- ✅ 支持缓存
- ❌ 需要一次性读入内存

**适用**：< 100MB 的雷达数据文件

### StreamingRadarSource（流）

```python
source = RadarSourceFactory.create_streaming_source(
    file_path='radar.jsonl',
    buffer_size=100
)
source.initialize()
```

**特点**：
- ✅ 低内存占用
- ✅ 支持大文件
- ❌ 不支持随机访问

**适用**：> 100MB 的雷达数据文件

### MultiCameraRadarSource（多摄像头）

```python
source = RadarSourceFactory.create_multi_camera_source(
    file_path='radar.jsonl'
)
source.initialize()

# 获取摄像头 1 的数据
for frame in source.stream_frames_by_camera(camera_id=1):
    # 处理
    pass
```

**特点**：
- ✅ 按摄像头分组
- ✅ 支持单摄像头查询
- ✅ 支持跨摄像头过滤

### RealtimeRadarSource（TCP 实时）

```python
from radar.server_wrapper import RealtimeRadarServer

server = RealtimeRadarServer(camera_id=1)
server.start()
source = server.get_source()

for frame in source.stream_frames():
    # 实时处理
    pass

server.stop()
```

**特点**：
- ✅ 实时 TCP 数据
- ✅ 自动 Protobuf 解码
- ✅ 队列缓冲

---

## 常见代码模式

### 获取特定时间戳

```python
source = RadarSourceFactory.create_jsonl_source('radar.jsonl')
source.initialize()

timestamps = source.get_all_timestamps()
frame = source.get_frame(timestamps[0])

for obj in frame.objects:
    print(f"目标 {obj.id}: {obj.speed}m/s")

source.close()
```

### 带错误处理的流式处理

```python
source = RadarSourceFactory.create_auto('radar.jsonl')
try:
    source.initialize()
    
    for frame in source.stream_frames():
        try:
            # 处理帧
            process(frame)
        except Exception as e:
            print(f"处理失败: {e}")
            continue
            
finally:
    source.close()
```

### 多摄像头独立处理

```python
source = RadarSourceFactory.create_multi_camera_source('radar.jsonl')
source.initialize()

for camera_id in [1, 2, 3]:
    frames = list(source.stream_frames_by_camera(camera_id))
    print(f"摄像头 {camera_id}: {len(frames)} 帧")

source.close()
```

### 监控性能

```python
source = RadarSourceFactory.create_auto('radar.jsonl')
source.initialize()

frame_count = 0
for frame in source.stream_frames():
    frame_count += 1

stats = source.get_stats()
print(f"总帧数: {stats.get('total_frames', 0)}")
print(f"总对象: {stats.get('total_objects', 0)}")

source.close()
```

---

## 常见错误

### ❌ 未初始化就使用

```python
source = RadarSourceFactory.create_jsonl_source('radar.jsonl')
frame = source.get_frame(ts)  # 会失败！
```

✅ **正确做法**：

```python
source = RadarSourceFactory.create_jsonl_source('radar.jsonl')
source.initialize()  # 必须初始化
frame = source.get_frame(ts)
```

### ❌ 流式数据源做随机访问

```python
source = RadarSourceFactory.create_streaming_source('radar.jsonl')
frame = source.get_frame(ts)  # 可能返回 None
```

✅ **正确做法**：

```python
source = RadarSourceFactory.create_streaming_source('radar.jsonl')
for frame in source.stream_frames():  # 使用流式方式
    process(frame)
```

### ❌ 忘记关闭

```python
source = RadarSourceFactory.create_auto('radar.jsonl')
source.initialize()
# 处理...
# 忘记 source.close()
```

✅ **正确做法**：

```python
source = RadarSourceFactory.create_auto('radar.jsonl')
try:
    source.initialize()
    # 处理...
finally:
    source.close()  # 一定要关闭
```

---

## 性能建议

| 场景 | 建议 |
|------|------|
| 文件 <50MB + 多次访问 | `create_jsonl_source(cached=True)` |
| 文件 >100MB | `create_streaming_source()` |
| 多个摄像头 | `create_multi_camera_source()` |
| 不确定 | `create_auto()` |

---

## 与融合处理器集成

```python
from radar.server_wrapper import RealtimeRadarServer
from core.RadarVisionFusion import RadarVisionFusionProcessor

# 启动实时数据源
server = RealtimeRadarServer(camera_id=1)
server.start()
source = server.get_source()

# 创建融合处理器
processor = RadarVisionFusionProcessor(camera_id=1)

# 流式处理
for frame in source.stream_frames():
    # 添加雷达数据
    processor.add_radar_data(frame.timestamp, frame.objects)
    
    # 处理视觉帧...
    # results = processor.process_frame(vision_ts, vision_objs)
    
    # 监控进度
    if processor.total_fused % 100 == 0:
        stats = processor.get_buffer_stats()
        print(f"融合: {processor.total_fused}, "
              f"缓冲: {stats['buffer_size']}")

server.stop()
```

---

## 更多信息

- 📖 详细设计: [`RADAR_SOURCE_ABSTRACTION.md`](RADAR_SOURCE_ABSTRACTION.md)
- 💻 实现代码: [`core/radar_source_abstraction.py`](core/radar_source_abstraction.py)
- 🧪 单元测试: [`test_radar_source_manager.py`](test_radar_source_manager.py)
