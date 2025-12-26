# 阶段1实现：基于时间戳的视觉队列缓冲对齐

## 概述
实现了"基于雷达的缓冲对齐"方案的第一阶段，通过添加视觉数据队列和时间戳对齐，解决目标批量消失问题。

## 核心改变

### 1. 视觉数据队列 (`core/Fusion.py`)
**新增组件：**
- `self.vision_frame_queue`: 按时间戳索引的视觉帧队列
- `self.vision_queue_max_size = 200`: 最多保留200个时间戳的视觉数据（约5-10秒）

**新增方法：**
- `_enqueue_vision_frame()`: 将视觉检测数据入队
- `_get_vision_frame_by_timestamp()`: 从队列中按时间戳获取视觉数据

**工作流程：**
```
视觉数据进来 → _enqueue_vision_frame() → 存入 vision_frame_queue
                                        ↓
                                    等待雷达数据
                                        ↓
                                  雷达数据到达
                                        ↓
                              从队列取出对应时间戳的视觉数据
                                        ↓
                                    进行融合
                                        ↓
                              输出带有融合时间戳的结果
```

### 2. 时间戳对齐 (`core/ResultBuffer.py`)
**修改内容：**
- `_perform_triple_matching()` 中的 `reportTime` 现在使用融合时的时间戳（`last_seen_timestamp`）
- `participant` 对象的 `timestamp` 字段也使用融合时的时间戳

**改变的影响：**
- 之前：所有输出都使用当前系统时间，导致时间戳与实际数据时间不符
- 现在：所有输出都使用融合时的时间戳，确保时间一致性

## 为什么这样做能解决问题

### 根本原因分析
当前的"目标批量消失"问题是由以下原因造成的：

1. **时间不对齐**：
   - 视觉数据：T时刻
   - 雷达数据：T-1秒（延迟）
   - 输出时间戳：当前时间（T+Δt）

2. **清理策略冲突**：
   - 当雷达数据到达时，视觉数据已经被处理并输出
   - 如果雷达延迟太长，视觉目标会被清理
   - 导致所有目标同时消失

### 解决方案
通过视觉队列缓冲：
1. 视觉数据不立即处理，而是入队等待
2. 当雷达数据到达时，从队列中取出对应时间戳的视觉数据
3. 进行融合并输出带有融合时间戳的结果
4. 避免了因为时间不对齐导致的目标清理

## 实现细节

### 视觉队列管理
```python
# 入队时自动清理过期数据
if len(self.vision_frame_queue) > self.vision_queue_max_size:
    oldest_ts = min(self.vision_frame_queue.keys())
    del self.vision_frame_queue[oldest_ts]
```

### 时间戳规范化
- 支持多种时间戳格式：字符串、Unix时间戳（float/int）
- 统一转换为 `'YYYY-MM-DD HH:MM:SS.mmm'` 格式

### 时间戳对齐逻辑
```python
# 从 global_targets 中获取融合时的时间戳
fusion_timestamp = None
for gt in unique_global_targets.values():
    if gt.last_seen_timestamp:
        fusion_timestamp = gt.last_seen_timestamp
        break

# 用于 reportTime 和 participant.timestamp
if fusion_timestamp:
    # 使用融合时的时间戳
else:
    # 回退到当前时间
```

## 预期效果

### 短期效果（立即）
- ✅ 避免因为时间不对齐导致的目标清理
- ✅ 所有输出数据都带有一致的时间戳
- ✅ 减少目标批量消失的频率

### 长期效果（需要验证）
- 完全消除目标批量消失现象
- 提高目标跟踪的稳定性
- 改善显示端的用户体验

## 测试方法

1. **运行系统**：
   ```bash
   python main.py
   ```

2. **观察输出**：
   - 检查 `output_fusion_refactored.json` 中的时间戳
   - 验证目标是否仍然批量消失

3. **分析日志**：
   ```bash
   grep "VISION_QUEUE" fusion_system.log
   ```

4. **运行分析脚本**：
   ```bash
   python analyze_disappear_detailed.py
   ```

## 后续步骤（阶段2）

如果阶段1有效，可以考虑：
1. 完全改变架构，使视觉处理延迟到雷达数据到达
2. 优化队列大小和清理策略
3. 添加更详细的时间戳同步日志
4. 验证显示端是否正确处理历史时间戳

## 文件修改清单

- `core/Fusion.py`：添加视觉队列和相关方法
- `core/ResultBuffer.py`：修改时间戳对齐逻辑
