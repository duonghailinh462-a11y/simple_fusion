# 融合策略管理指南

## 概述

使用**策略模式**管理融合系统的两个版本：
- **原始融合策略** (OriginalFusionStrategy)
- **改进融合策略** (ImprovedFusionStrategy)

## 快速开始

### 1. 切换融合策略

在 `main.py` 中修改：

```python
# 使用改进策略（推荐）
fusion_strategy = "improved"

# 或使用原始策略
fusion_strategy = "original"

fusion_system = CrossCameraFusion(fusion_strategy=fusion_strategy)
```

### 2. 运行程序

```bash
python main.py
```

程序会在日志中输出当前使用的策略：
```
使用融合策略: improved
CrossCameraFusion初始化完成 (重构版) - 使用策略: ImprovedFusionStrategy
```

---

## 两个策略的详细对比

### 原始融合策略 (Original)

**特点**：
- ✅ 简单、快速
- ✅ 计算复杂度低
- ❌ 无方向判断
- ❌ 无FIFO排队

**匹配逻辑**：
```
C1 LocalTarget ↔ C2 GlobalTarget: 时间窗口匹配
C3 LocalTarget → C2 GlobalTarget: 时间窗口匹配
```

**适用场景**：
- 简单场景
- 目标进入时间差异明显
- 计算资源受限

**参数**：
- `time_window`: 融合时间窗口（默认60帧）

---

### 改进融合策略 (Improved)

**特点**：
- ✅ 多维度考虑
- ✅ 减少误匹配
- ✅ 更好处理复杂场景
- ❌ 计算复杂度中等
- ❌ 需要更多内存

**匹配逻辑**：
```
C1 LocalTarget ↔ C2 GlobalTarget: 时间窗口匹配（保持不变）
C3 LocalTarget → C2 GlobalTarget: FIFO排队匹配（新增）
```

**新增机制**：

1. **方向判断**
   - C2的LocalTarget只有向上移动（Y值变小）时
   - 才会被加入到 `c2_buffer_from_c3` 队列
   - 这样C3的目标可以通过FIFO方式与C2匹配

2. **FIFO排队**
   - C3的GlobalTarget按队列顺序与C2的LocalTarget匹配
   - 遵循"先进先出"原则
   - 自动处理过期条目和类别不匹配情况

**适用场景**：
- 复杂场景
- 多目标同时进入
- 需要更高的融合准确率

**参数**：
- `time_window`: 融合时间窗口（默认60帧）
- `pixel_y_direction_threshold`: 像素Y值变化阈值（默认50像素）
- `max_retention_frames`: C2缓冲区最大保留帧数（默认100帧）

---

## 代码结构

### 文件组织与职责

```
FusionComponents.py          # 融合组件库
├── TargetManager            # 目标管理（GlobalID分配、颜色管理）
├── MatchingEngine           # 匹配引擎（执行具体的匹配操作）
│   ├── match_time_window()           # 时间窗口匹配
│   ├── match_c3_to_c2_fifo()         # FIFO排队匹配
│   └── add_c2_to_buffer()            # C2缓冲区管理
└── TrajectoryMerger         # 轨迹融合（平滑加权融合）

FusionStrategies.py          # 融合策略模块
├── FusionMatchingStrategy   # 抽象基类（定义策略接口）
├── OriginalFusionStrategy   # 原始策略（纯时间窗口匹配）
│   └── perform_matching()   # 流程控制 + 委托给 MatchingEngine
├── ImprovedFusionStrategy   # 改进策略（时间窗口 + FIFO + 方向判断）
│   ├── perform_matching()   # 流程控制 + 委托给 MatchingEngine
│   ├── add_c2_to_buffer()   # 委托给 MatchingEngine
│   └── get_metrics()        # 委托给 MatchingEngine
└── FusionStrategyFactory    # 策略工厂（创建策略实例）

Fusion.py                     # 融合协调器
├── CrossCameraFusion        # 主协调器
│   ├── __init__()           # 策略注入
│   ├── classify_targets()   # GlobalID/LocalID分配
│   ├── _perform_matching()  # 委托给策略执行
│   ├── _smoothly_merge_trajectory()  # 轨迹融合
│   ├── update_global_state()         # 目标确认
│   └── generate_json_data()          # JSON输出

main.py                       # 主程序
└── fusion_strategy = "improved"  # 策略选择
```

### 架构设计模式

#### 1. 策略模式 (Strategy Pattern)
```
CrossCameraFusion
    ↓ (使用)
FusionMatchingStrategy (抽象)
    ↑ (实现)
    ├── OriginalFusionStrategy
    └── ImprovedFusionStrategy
```

#### 2. 委托模式 (Delegation Pattern)
```
FusionStrategies (策略)
    ↓ (委托)
FusionComponents.MatchingEngine (执行)
    ↓ (使用)
TargetManager, TrajectoryMerger
```

### 关键类和方法

#### FusionMatchingStrategy (抽象基类)

```python
class FusionMatchingStrategy(ABC):
    """融合匹配策略的抽象接口"""
    
    @abstractmethod
    def perform_matching(self, 
                        local_targets_this_frame: List[LocalTarget],
                        active_global_targets: List[GlobalTarget],
                        local_to_global: dict,
                        frame_count: int,
                        perf_monitor=None) -> dict:
        """
        执行匹配逻辑
        
        Returns:
            {
                'local_to_global': 更新后的绑定字典,
                'locked_global_ids': 本帧锁定的GlobalID集合
            }
        """
        pass
    
    @abstractmethod
    def get_strategy_name(self) -> str:
        """获取策略名称"""
        pass
```

#### OriginalFusionStrategy (原始策略)

```python
class OriginalFusionStrategy(FusionMatchingStrategy):
    """原始融合策略 - 纯时间窗口匹配"""
    
    def perform_matching(self, ...):
        # 流程控制：
        # 1. 预处理全局目标的融合区状态
        # 2. 遍历本地目标进行匹配
        # 3. 调用 MatchingEngine.match_time_window() 执行匹配
        # 4. 返回匹配结果
```

#### ImprovedFusionStrategy (改进策略)

```python
class ImprovedFusionStrategy(FusionMatchingStrategy):
    """改进融合策略 - 时间窗口 + FIFO + 方向判断"""
    
    def __init__(self, ...):
        # 创建 MatchingEngine 实例
        self.matching_engine = MatchingEngine()
    
    def perform_matching(self, ...):
        # 流程控制：
        # 1. 预处理全局目标的融合区状态
        # 2. 遍历本地目标进行匹配
        # 3. 根据摄像头类型选择匹配方式：
        #    - C3: 调用 MatchingEngine.match_time_window()
        #    - C1/C2: 调用 MatchingEngine.match_time_window()
        # 4. 返回匹配结果
    
    def add_c2_to_buffer(self, ...):
        # 委托给 MatchingEngine
        self.matching_engine.add_c2_to_buffer(...)
    
    def get_metrics(self):
        # 委托给 MatchingEngine
        return self.matching_engine.metrics.copy()
```

#### MatchingEngine (匹配引擎)

```python
class MatchingEngine:
    """执行具体的匹配操作"""
    
    def match_time_window(self, local_target, active_global_targets, time_window):
        """基于时间窗口的匹配"""
        # 遍历候选，选择时间差最小的
        # 返回最佳匹配的 GlobalTarget
    
    def match_c3_to_c2_fifo(self, c3_global_target, frame_count):
        """C3 -> C2 FIFO排队匹配"""
        # 从 C2 缓冲区队头取出一个条目
        # 检查过期和类别兼容性
        # 返回匹配的 C2 local_id
    
    def add_c2_to_buffer(self, local_target, frame_count):
        """将 C2 目标添加到缓冲区"""
        # 创建缓冲区条目
        # 加入到 deque
```

#### CrossCameraFusion (融合协调器)

```python
class CrossCameraFusion:
    """融合系统的主协调器"""
    
    def __init__(self, fusion_strategy: str = "improved"):
        # 创建策略实例
        self.fusion_strategy = FusionStrategyFactory.create_strategy(
            fusion_strategy, ...
        )
        # 创建组件
        self.target_manager = TargetManager()
        self.trajectory_merger = TrajectoryMerger()
    
    def _perform_matching(self, local_targets_this_frame, 
                         active_global_targets, perf_monitor=None):
        # 【关键】委托给策略执行匹配
        result = self.fusion_strategy.perform_matching(
            local_targets_this_frame,
            active_global_targets,
            self.local_to_global,
            self.frame_count,
            perf_monitor
        )
        
        # 处理已绑定的LocalTarget（融合轨迹）
        for local_target in local_targets_this_frame:
            if lookup_key in self.local_to_global:
                # 调用 TrajectoryMerger 进行轨迹融合
                self._smoothly_merge_trajectory(...)
```

#### FusionStrategyFactory (工厂类)

```python
class FusionStrategyFactory:
    """融合策略工厂 - 创建策略实例"""
    
    @staticmethod
    def create_strategy(strategy_name: str, **kwargs) -> FusionMatchingStrategy:
        if strategy_name == "original":
            return OriginalFusionStrategy(
                time_window=kwargs.get('time_window', 60)
            )
        elif strategy_name == "improved":
            return ImprovedFusionStrategy(
                time_window=kwargs.get('time_window', 60),
                pixel_y_direction_threshold=kwargs.get('pixel_y_direction_threshold', 50),
                max_retention_frames=kwargs.get('max_retention_frames', 100)
            )
```

---

## 使用示例

### 示例1：使用原始策略

```python
from Fusion import CrossCameraFusion

# 创建融合系统，使用原始策略
fusion_system = CrossCameraFusion(fusion_strategy="original")

# 处理检测结果
global_targets, local_targets = fusion_system.process_detections(
    detections, camera_id, timestamp, perf_monitor
)
```

### 示例2：使用改进策略

```python
from Fusion import CrossCameraFusion

# 创建融合系统，使用改进策略
fusion_system = CrossCameraFusion(fusion_strategy="improved")

# 处理检测结果
global_targets, local_targets = fusion_system.process_detections(
    detections, camera_id, timestamp, perf_monitor
)
```

### 示例3：对比两个策略

```python
import time

# 创建两个融合系统
fusion_original = CrossCameraFusion(fusion_strategy="original")
fusion_improved = CrossCameraFusion(fusion_strategy="improved")

# 处理相同的数据
for frame_data in data_stream:
    # 原始策略
    start = time.time()
    gt_orig, lt_orig = fusion_original.process_detections(...)
    time_orig = time.time() - start
    
    # 改进策略
    start = time.time()
    gt_impr, lt_impr = fusion_improved.process_detections(...)
    time_impr = time.time() - start
    
    print(f"原始: {len(gt_orig)} 全局目标, {time_orig:.3f}s")
    print(f"改进: {len(gt_impr)} 全局目标, {time_impr:.3f}s")
```

---

## 性能对比

### 计算复杂度

| 方面 | 原始策略 | 改进策略 |
|------|--------|--------|
| 时间复杂度 | O(n*m) | O(n*m + k) |
| 空间复杂度 | O(1) | O(k) |
| 缓冲区 | 无 | 有 |

其中：
- n: LocalTarget数量
- m: GlobalTarget数量
- k: C2缓冲区大小

### 准确率

| 场景 | 原始策略 | 改进策略 |
|------|--------|--------|
| 简单场景 | ✅ 高 | ✅ 高 |
| 多目标同时进入 | ❌ 低 | ✅ 高 |
| 复杂交叉场景 | ❌ 低 | ✅ 中等 |

---

## 调试和监控

### 查看当前策略

```python
# 获取策略名称
strategy_name = fusion_system.fusion_strategy.get_strategy_name()
print(f"当前策略: {strategy_name}")
```

### 获取匹配统计（仅改进策略）

```python
# 获取FIFO匹配统计
if hasattr(fusion_system.fusion_strategy, 'get_metrics'):
    metrics = fusion_system.fusion_strategy.get_metrics()
    print(f"FIFO匹配成功: {metrics['fifo_match_c3_c2']}")
    print(f"缓冲区过期: {metrics['fifo_pop_stale']}")
    print(f"类别不匹配: {metrics['fifo_pop_mismatch']}")
```

### 日志输出

程序会输出策略相关的日志：

```
OriginalFusionStrategy 初始化: time_window=60
ImprovedFusionStrategy 初始化: time_window=60, pixel_y_threshold=50
使用融合策略: improved
CrossCameraFusion初始化完成 (重构版) - 使用策略: ImprovedFusionStrategy
```

---

## 常见问题

### Q1: 如何在运行时切换策略？

A: 目前需要在 `main.py` 中修改 `fusion_strategy` 变量，然后重新运行程序。后续可以添加配置文件支持动态切换。

### Q2: 改进策略的缓冲区会不会导致内存溢出？

A: 不会。缓冲区有自动清理机制：
- 过期条目会自动移除（`max_retention_frames`）
- 类别不匹配的条目会自动丢弃
- 内存占用有界

### Q3: 两个策略的输出结果会不同吗？

A: 可能会不同。改进策略的FIFO排队和方向判断可能导致不同的匹配结果。这是正常的，改进策略通常会产生更准确的结果。

### Q4: 如何添加新的融合策略？

A: 
1. 创建新类继承 `FusionMatchingStrategy`
2. 实现 `perform_matching()` 和 `get_strategy_name()` 方法
3. 在 `FusionStrategyFactory.create_strategy()` 中添加新策略
4. 在 `main.py` 中指定新策略名称

---

## 总结

| 方面 | 原始策略 | 改进策略 |
|------|--------|--------|
| **复杂度** | 低 | 中等 |
| **准确率** | 中等 | 高 |
| **适用场景** | 简单 | 复杂 |
| **推荐** | 基准对比 | 生产环境 |

**建议**：
- 开发和测试阶段：使用原始策略作为基准
- 生产环境：使用改进策略获得更好的融合效果
- 性能敏感场景：根据实际需求选择
