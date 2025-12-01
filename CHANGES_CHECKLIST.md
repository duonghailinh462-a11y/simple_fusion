# GlobalID 分配逻辑改造 - 变更清单

## ✅ 已完成的改造

### 1. Basic.py 配置层
- [x] 在 `FusionConfig` 中添加 `RADAR_VISION_FUSION_AREAS` 配置
  - C1: [[110, 429], [0, 536], [0, 720], [1280, 720], [1280, 458]]
  - C2: [[0, 720], [1280, 720], [1280, 418], [109, 432]]
  - C3: [[328, 472], [186, 720], [1033, 720], [985, 468]]
- [x] 在 `Config` 类中添加 `RADAR_VISION_FUSION_AREAS` 属性访问器
- [x] 在 `GeometryUtils` 中添加 `is_in_radar_vision_fusion_area()` 静态方法

### 2. TargetTrack.py 跟踪层
- [x] 修改 `analyze_trajectory_for_global_assignment()` 函数
  - 移除 `pixel_bottom_threshold` 和 `pixel_top_threshold` 参数
  - 添加 `camera_id` 参数
  - 改为使用 `GeometryUtils.is_in_radar_vision_fusion_area()` 进行判断

### 3. main.py 主程序层
- [x] 在 `batch_convert_track_results()` 中添加融合区域判断
  - 计算目标底部中心点
  - 调用 `GeometryUtils.is_in_radar_vision_fusion_area()` 判断
  - 在返回的 detection 字典中添加 `in_fusion_area` 标记

### 4. Fusion.py 融合层
- [x] 修改 `classify_targets()` 中的 `analyze_trajectory_for_global_assignment()` 调用
  - 添加 `camera_id` 参数
  - 移除 `pixel_bottom_threshold` 和 `pixel_top_threshold` 参数

### 5. FusionComponents.py 融合组件层
- [x] 修改 `create_local_target()` 中的融合区域判断
  - 优先使用 detection 中的 `in_fusion_area` 标记
  - 备选方案：从像素坐标直接判断

## 📋 验证清单

### 代码检查
- [x] 所有导入都正确
- [x] 函数签名都已更新
- [x] 没有遗留的旧参数
- [x] Config 实例化正确

### 逻辑检查
- [x] 融合区域判断逻辑正确
- [x] GlobalID 分配条件正确（C1/C3 且在融合区域内）
- [x] LocalTarget 创建时正确标记融合区域

### 兼容性检查
- [x] 向后兼容性（detection 中的 in_fusion_area 是可选的）
- [x] 没有破坏现有的 PUBLIC_AREA_BEV 逻辑
- [x] C2 摄像头的特殊处理保持不变

## 🧪 测试建议

### 单元测试
1. 测试 `is_in_radar_vision_fusion_area()` 方法
   - 测试融合区域内的点
   - 测试融合区域外的点
   - 测试边界点

2. 测试 `analyze_trajectory_for_global_assignment()` 函数
   - 测试在融合区域内的轨迹
   - 测试在融合区域外的轨迹
   - 测试轨迹长度不足的情况

### 集成测试
1. 运行完整的视频处理流程
2. 检查 GlobalID 是否正确分配
3. 验证雷视融合是否正常工作

### 性能测试
1. 检查融合区域判断的性能开销
2. 确保不会显著增加处理时间

## 📝 注意事项

1. **融合区域 ⊆ 检测区域**: 融合区域应该是检测区域的子集
2. **C1/C3 才分配 GlobalID**: C2 摄像头不分配 GlobalID
3. **首次进入融合区域时分配**: 目标首次进入融合区域时分配 GlobalID
4. **GlobalID 保持不变**: 一旦分配，GlobalID 会一直伴随目标到消失

## 🔍 后续检查

- [ ] 运行测试脚本 `test_fusion_areas.py`
- [ ] 验证融合区域可视化图像
- [ ] 在实际视频上进行端到端测试
- [ ] 检查日志输出，确认 GlobalID 分配正确
- [ ] 验证雷视融合结果

## 📞 联系方式

如有问题，请参考 `REFACTOR_SUMMARY.md` 中的详细说明。
