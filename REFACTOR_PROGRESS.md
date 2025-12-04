# 无帧同步架构重构进度

## 分支信息
- **分支名**: `refactor/no-frame-sync`
- **创建时间**: 2025-12-03 09:58
- **目标**: 移除帧同步，实现单路处理 + 后期三路匹配

## 重构目标

### 当前问题
- 使用帧同步的sync_timestamp匹配雷达数据，导致时间戳偏差
- 雷达融合的null值过多
- 帧同步本身有缓存堆积问题

### 新架构
```
C1帧 → 单路处理 + 雷视匹配 → local_targets_c1 + radar_ids
C2帧 → 单路处理 + 雷视匹配 → local_targets_c2 + radar_ids
C3帧 → 单路处理 + 雷视匹配 → local_targets_c3 + radar_ids
                              ↓
                    三路视频匹配（时间+空间+特征）
                              ↓
                  global_targets + local_targets
                              ↓
                        JSON输出
```

## 重构步骤

### ✅ 第一阶段：准备工作
- [x] 创建git分支 `refactor/no-frame-sync`
- [x] 提交RadarVisionFusion.py的改动（MAX_TIME_DIFF增加到0.5秒）

### ✅ 第二阶段：改造main.py主循环
**目标**: 移除帧同步，实现单路独立处理

**已完成的改动**:
1. [x] 移除FrameSynchronizer初始化
2. [x] 改造主循环
   - 从 `frame_synchronizer.get_synchronized_frames()` 改为直接从队列获取
   - 改为 `for camera_id in [1,2,3]` 的循环结构
3. [x] 改造帧处理逻辑
   - 用 `result['timestamp']` 替代 `sync_timestamp`
   - 单路处理，存储结果到 `camera_results[camera_id]`
4. [x] 移除所有frame_synchronizer引用

**关键代码位置**:
- 初始化帧同步器: main.py 第430-458行
- 主循环获取帧: main.py 第468-551行
- 帧处理: main.py 第556-650行
- 雷达融合: main.py 第670-755行

### ✅ 第三阶段：在Fusion.py中实现三路匹配
**目标**: 添加后期跨摄像头匹配逻辑

**已完成的方法**:
1. [x] `store_single_camera_result()` - 存储单路处理结果
2. [x] `match_cross_camera_targets()` - 三路匹配
3. [x] `can_match_targets()` - 匹配条件判断

**在main.py中的集成**:
1. [x] 在雷达融合后存储单路结果
2. [x] 定期（每100帧）进行三路匹配

### ⏳ 第四阶段：测试和调优
- 单路处理测试
- 雷视融合准确率测试
- 三路匹配准确率测试
- 性能对比

## 关键设计决策

### 1. 单路结果存储
```python
camera_results = {
    1: [
        {'timestamp': 1234.567, 'local_targets': [...], 'radar_ids': {...}},
        {'timestamp': 1234.600, 'local_targets': [...], 'radar_ids': {...}},
        ...
    ],
    2: [...],
    3: [...]
}
```

### 2. 后期匹配触发时机
- 选项A: 定时触发（每处理100帧）
- 选项B: 缓冲区满时触发
- 选项C: 程序结束时触发

**建议**: 选项A（每处理100帧）- 平衡实时性和准确性

### 3. 匹配条件
- 时间条件: `|ts1 - ts2| < 0.5秒`
- 空间条件: 地理位置相近（在PUBLIC_AREA_BEV内）
- 特征条件: 类别相同、大小相近（0.7-1.3倍）

## 预期改进

| 指标 | 当前 | 预期 |
|------|------|------|
| 雷达融合null率 | 高 | 低 |
| 时间戳匹配准确率 | 低 | 高 |
| 缓存堆积 | 有 | 无 |
| 处理延迟 | 高 | 低 |

## 注意事项

1. **向后兼容性**: 需要保留JSON输出格式不变
2. **性能**: 确保三路匹配不会成为性能瓶颈
3. **测试**: 需要充分测试各种场景
4. **回滚方案**: 如果出现问题，可以快速回到原分支

## 提交记录

- `5292bca`: refactor: 增加MAX_TIME_DIFF到0.5秒，添加诊断日志
- `506460e`: refactor: 移除帧同步，改为单路独立处理 + 用原始时间戳匹配雷达
- `418c5b3`: refactor: 在Fusion.py中添加三路匹配逻辑
- `aa38de1`: refactor: 在main.py中集成三路匹配，定期存储单路结果并进行匹配
- `35db462`: refactor: 删除不再使用的FrameSynchronizer.py

## 关键改动说明

### main.py改动总结
- **移除**: FrameSynchronizer初始化和所有帧同步逻辑（约100行）
- **改造**: 主循环改为单路独立处理，每个摄像头的帧独立获取和处理
- **改进**: 雷达融合使用原始时间戳而不是sync_timestamp
- **简化**: 移除了复杂的缓冲区管理和同步等待逻辑

### 核心变化
```python
# 旧：帧同步 + 融合
synchronized_frames, sync_timestamp = frame_synchronizer.get_synchronized_frames()
for camera_id, result in synchronized_frames.items():
    vision_timestamp = sync_timestamp  # 使用同步时间戳
    
# 新：单路处理 + 后期三路匹配
for camera_id in [1, 2, 3]:
    result = queues[camera_id].get_nowait()
    original_timestamp = result['timestamp']  # 使用原始时间戳
    
    # 存储单路结果
    fusion_system.store_single_camera_result(camera_id, original_timestamp, local_targets, radar_ids)

# 定期进行三路匹配
if current_frame % 100 == 0:
    global_targets, unmatched_local_targets = fusion_system.match_cross_camera_targets()
```

### Fusion.py新增方法

#### store_single_camera_result()
存储单路处理结果到内部缓冲区，用于后期三路匹配

#### can_match_targets(target1, target2)
判断两个目标是否可以匹配，条件：
- 类别相同
- 地理位置相近（距离 < 5米）
- 大小相近（0.7-1.3倍）

#### match_cross_camera_targets(time_window=0.5)
进行三路视频匹配，返回全局目标和未匹配的本地目标

---

**最后更新**: 2025-12-03 10:02
