# GlobalID 分配逻辑重构总结

## 改造目标
将 GlobalID 分配逻辑从基于像素 Y 值阈值（y > 700）改为基于雷视融合区域的多边形判断。

## 改造内容

### 1. 配置层 (Basic.py)

#### 新增配置
```python
# FusionConfig 中新增
RADAR_VISION_FUSION_AREAS: Dict[int, np.ndarray] = {
    1: np.array([[110, 429], [0, 536], [0, 720], [1280, 720], [1280, 458]], dtype=np.int32),
    2: np.array([[0, 720], [1280, 720], [1280, 418], [109, 432]], dtype=np.int32),
    3: np.array([[328, 472], [186, 720], [1033, 720], [985, 468]], dtype=np.int32),
}
```

#### 新增工具方法
```python
# GeometryUtils 中新增
@staticmethod
def is_in_radar_vision_fusion_area(pixel_point: Tuple[int, int], camera_id: int) -> bool:
    """检查像素点是否在该摄像头的雷视融合区域内"""
```

### 2. 跟踪层 (TargetTrack.py)

#### 修改 analyze_trajectory_for_global_assignment 函数
**之前**: 基于 y > 700 的简单阈值判断
```python
def analyze_trajectory_for_global_assignment(pixel_track_history, min_trajectory_length=3, 
                                            pixel_bottom_threshold=700, pixel_top_threshold=1080):
    if pixel_bottom_threshold <= start_y <= pixel_top_threshold:
        return True
```

**之后**: 基于融合区域的多边形判断
```python
def analyze_trajectory_for_global_assignment(pixel_track_history, camera_id, min_trajectory_length=3):
    start_pos = pixel_track_history[0]
    return GeometryUtils.is_in_radar_vision_fusion_area(start_pos, camera_id)
```

### 3. 主程序层 (main.py)

#### 修改 batch_convert_track_results 函数
在返回的 detection 字典中新增 `in_fusion_area` 标记：
```python
# 计算目标底部中心点（用于融合区域判断）
center_x = int((tlbr[0] + tlbr[2]) / 2)
center_y = int(tlbr[3])
pixel_point = (center_x, center_y)

# 检查是否在雷视融合区域内
in_fusion_area = GeometryUtils.is_in_radar_vision_fusion_area(pixel_point, camera_id)

detection = {
    # ... 其他字段 ...
    'in_fusion_area': in_fusion_area  # 新增标记
}
```

### 4. 融合层 (Fusion.py & FusionComponents.py)

#### 修改 classify_targets 中的 GlobalID 分配调用
```python
# 之前
should_assign_gid = (
    camera_id != 2 and 
    analyze_trajectory_for_global_assignment(pixel_track_history, 
                                           min_trajectory_length=3,
                                           pixel_bottom_threshold=500,
                                           pixel_top_threshold=720)
)

# 之后
should_assign_gid = (
    camera_id != 2 and 
    analyze_trajectory_for_global_assignment(pixel_track_history, camera_id,
                                           min_trajectory_length=3)
)
```

#### 修改 create_local_target 中的融合区域判断
```python
# 使用detection中的in_fusion_area标记（如果存在），否则从像素坐标判断
if 'in_fusion_area' in detection:
    is_in_fusion_area = detection['in_fusion_area']
else:
    # 备选方案：从像素坐标判断
    is_in_fusion_area = GeometryUtils.is_in_radar_vision_fusion_area((center_x, center_y), camera_id)
```

## 数据流改进

### 之前的流程
```
SDK推理 → NMS → detect_areas过滤 → BYTETracker跟踪 
→ 跟踪结果转换 → 在classify_targets中用y>700判断是否分配globalid
→ 跨摄像头融合 → 雷达融合 → JSON生成
```

### 改进后的流程
```
SDK推理 → NMS → detect_areas过滤 → BYTETracker跟踪 
→ 跟踪结果转换 + 标记in_fusion_area
→ 在classify_targets中用融合区域判断是否分配globalid
→ 跨摄像头融合 → 雷达融合 → JSON生成
```

## 关键改进

1. **概念统一**: 不再有两套判断逻辑（y > 700 和 GPS融合区域），只有一套像素坐标融合区域判断
2. **精度提升**: 多边形区域比简单的 y 阈值更精确，能更准确地定义"车道区域"
3. **可维护性**: 融合区域与检测区域保持一致，配置更清晰
4. **灵活性**: 可以为不同摄像头定义不同的融合区域形状，不仅仅是矩形

## 测试方法

运行测试脚本验证融合区域判断：
```bash
python test_fusion_areas.py
```

该脚本会：
1. 测试各摄像头的融合区域判断逻辑
2. 生成融合区域的可视化图像
3. 验证测试点是否正确判断

## 注意事项

1. **融合区域 ⊆ 检测区域**: 融合区域应该是检测区域的子集
2. **C1/C3 才分配 GlobalID**: C2 摄像头不分配 GlobalID（只创建 LocalTarget）
3. **首次进入融合区域时分配**: 目标首次进入融合区域时分配 GlobalID，之后保持稳定
4. **GlobalID 保持不变**: 一旦分配，GlobalID 会一直伴随目标到消失

## 后续优化空间

1. 可以为每个摄像头配置多个融合区域（当前每个摄像头只有一个）
2. 可以根据目标方向动态选择融合区域
3. 可以添加融合区域的时间相关性（例如不同时间段使用不同的融合区域）
