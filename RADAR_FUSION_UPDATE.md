# 雷达融合数据读取方式更新

## 问题分析

之前的实现存在以下问题：
1. **异步处理**: 雷达数据和视觉数据分离，雷达融合是事后处理
2. **全局融合**: 所有摄像头的数据混在一起，没有按摄像头区分
3. **重复筛选**: 雷达融合时重新筛选视觉检测目标，造成逻辑混乱

## 改进方案

### 1. 按摄像头读取雷达数据

在 `RadarDataLoader` 中添加摄像头级别的数据管理：

```python
# IP到摄像头ID的映射
RADAR_IP_TO_CAMERA = {
    '44.30.142.85': 2,  # C2
    '44.30.142.88': 1,  # C1
    '44.30.142.87': 3,  # C3
}

# 新增方法
def get_radar_data_by_camera(self, camera_id, timestamp):
    """获取指定摄像头和时间戳的雷达数据"""
    key = (camera_id, timestamp)
    return self.radar_data_by_camera.get(key, [])

def get_camera_timestamps(self, camera_id):
    """获取指定摄像头的所有时间戳"""
    return sorted(self.camera_timestamps.get(camera_id, set()))
```

### 2. 按摄像头初始化融合处理器

在 `main.py` 中为每个摄像头创建独立的融合处理器：

```python
# 为每个摄像头初始化独立的融合处理器
radar_fusion_processors = {}  # 按摄像头存储
for camera_id in [1, 2, 3]:
    radar_fusion_processors[camera_id] = RadarVisionFusionProcessor(...)
    
    # 将该摄像头的雷达数据添加到对应的处理器
    camera_timestamps = radar_data_loader.get_camera_timestamps(camera_id)
    for ts in camera_timestamps:
        radar_objs = radar_data_loader.get_radar_data_by_camera(camera_id, ts)
        radar_fusion_processors[camera_id].add_radar_data(ts, radar_objs)
```

### 3. 同步融合处理

在主处理循环中，按摄像头进行同步的雷达融合：

```python
# 按摄像头进行雷达融合
for camera_id in [1, 2, 3]:
    if camera_id not in radar_fusion_processors:
        continue
    
    # 收集该摄像头的所有目标
    vision_objects = []
    
    # 处理全局目标和本地目标（仅该摄像头的）
    for global_target in all_global_targets:
        if global_target.camera_id != camera_id:
            continue
        # ... 构建vision_obj ...
    
    # 执行该摄像头的雷达融合
    if vision_objects:
        updated_vision_objects = radar_fusion_processors[camera_id].process_frame(ts, vision_objects)
        # 构建 radar_id_map
        for vision_obj in updated_vision_objects:
            if vision_obj.radar_id is not None:
                radar_id_map[vision_obj.track_id] = vision_obj.radar_id
```

## 数据流改进

### 之前
```
SDK推理 → NMS → 跟踪 → GlobalID分配 → 跨摄像头融合
                                    ↓
                            [异步] 雷达融合（全局）
                                    ↓
                            JSON生成 → MQTT发送
```

### 之后
```
SDK推理 → NMS → 跟踪 → GlobalID分配 → 跨摄像头融合
                                    ↓
                            [同步] 按摄像头雷达融合
                                    ↓
                            JSON生成 → MQTT发送
```

## 关键改进点

| 方面 | 之前 | 之后 |
|------|------|------|
| **数据组织** | 全局混合 | 按摄像头分离 |
| **融合时机** | 事后异步 | 同步处理 |
| **处理器数量** | 1个全局 | 3个独立 |
| **目标筛选** | 重复筛选 | 单次筛选 |
| **逻辑清晰度** | 混乱 | 清晰 |

## 文件修改

### RadarVisionFusion.py
- ✅ 添加 `RADAR_IP_TO_CAMERA` 映射
- ✅ 添加 `get_radar_data_by_camera()` 方法
- ✅ 添加 `get_camera_timestamps()` 方法
- ✅ 添加 `camera_timestamps` 数据结构
- ✅ 改进 `load()` 方法支持按摄像头分类

### main.py
- ✅ 更新雷达融合初始化（按摄像头）
- ✅ 更新雷达数据路径为实际路径
- ✅ 改进雷达融合处理逻辑（同步、按摄像头）
- ✅ 移除异步处理的冗余代码

## 性能影响

- **内存**: 略有增加（3个处理器 vs 1个）
- **CPU**: 相同（同样的计算量，只是组织方式改变）
- **延迟**: 降低（同步处理避免了时间戳匹配的延迟）

## 向后兼容性

- ✅ 保留 `get_radar_data()` 和 `get_all_timestamps()` 方法
- ✅ 全局数据仍然可用（用于调试）
- ✅ 现有的 `RadarVisionFusionProcessor` API 不变

## 测试建议

1. 验证雷达数据按摄像头正确分类
2. 检查融合处理器是否为每个摄像头独立初始化
3. 确认雷达融合结果正确（radar_id_map）
4. 性能测试（帧率、内存占用）
