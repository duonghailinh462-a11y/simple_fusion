# 帧号同步改造 - refactor/frame-number-sync

## 概述

本次改造将系统的同步方式从**时间戳同步**改为**帧号同步**。三路视频流的帧号相同时，认为它们是同步的，打包送入SDK处理。时间戳由公式计算得出，确保准确性。

## 核心改变

### 同步方式对比

| 方面 | 原方式（时间戳同步） | 新方式（帧号同步） |
|------|------------------|------------------|
| **同步键** | 时间戳（浮点数） | 帧号（整数） |
| **同步条件** | 时间戳在时间窗口内 | 三路帧号相同 |
| **时间戳来源** | 帧同步时的参考时间戳 | 公式计算：`start_timestamp + frame_number/fps` |
| **精度** | 依赖时间窗口大小 | 确定性强，无容差 |
| **复杂度** | 较高（需要时间窗口匹配） | 简单（直接帧号匹配） |

## 改造详情

### 1. FrameSynchronizer.py

**改动内容：**

#### 初始化参数
```python
# 原来
def __init__(self, num_cameras=3, time_window=0.5, camera_start_times=None)

# 改为
def __init__(self, num_cameras=3, fps=25, start_timestamp=None)
```

#### 缓冲区管理
- **缓冲区键**：从时间戳 → 帧号（整数）
- **缓冲区结构**：`frame_buffers[camera_id][frame_number] = frame_data`

#### 同步逻辑
```python
def get_synchronized_frames(self):
    """
    基于帧号同步
    - 寻找三路都有的帧号
    - 时间戳由公式计算：start_timestamp + frame_number/fps
    """
```

**关键改动：**
- 移除时间戳解析和时间窗口匹配逻辑
- 改为帧号直接匹配
- 时间戳计算改为公式计算

#### 缓冲区状态
```python
def get_buffer_status(self):
    # 返回帧号范围信息
    status[camera_id] = {
        'count': len(frame_numbers),
        'min_frame_number': frame_numbers[0],
        'max_frame_number': frame_numbers[-1],
        'frame_span': frame_numbers[-1] - frame_numbers[0]
    }
```

### 2. SDKinfer.py

**改动内容：**

在 `extract_detection_info()` 方法中添加 `frame_number` 字段：

```python
frame_result = {
    'frame_number': self.frame_count,  # 新增：用于帧号同步
    'frame_id': self.frame_count,      # 保留：用于兼容性
    'camera_id': self.attr.chan_id + 1,
    'boxes_num': 0,
    'detections': [],
    'timestamp': timestamp_str,
    'rtsp_timestamp': timestamp_str
}
```

**说明：**
- `frame_number`：新增字段，用于帧号同步
- `frame_id`：保留字段，用于向后兼容性

### 3. main.py

**改动内容：**

#### 初始化FrameSynchronizer
```python
# 获取起始时间戳
start_timestamp = None
if Config.CAMERA_START_DATETIMES:
    first_camera_time = Config.CAMERA_START_DATETIMES.get(1, ...)
    try:
        start_datetime = datetime.strptime(first_camera_time, "%Y-%m-%d %H:%M:%S.%f")
        start_timestamp = start_datetime.timestamp()
    except Exception as e:
        logger.warning(f"时间戳解析失败: {e}, 将使用系统时间")
        start_timestamp = time.time()

# 初始化帧同步器
frame_synchronizer = StrictFrameSynchronizer(
    num_cameras=3, 
    fps=Config.FPS,
    start_timestamp=start_timestamp
)
```

#### 日志更新
- 同步模式日志改为：`"帧号同步 - 三路帧号相同时同步，时间戳=start_timestamp + frame_number/fps"`
- 缓冲区状态日志改为显示帧号范围
- 超时警告日志改为帧号相关信息

**示例：**
```
等待同步 (连续50个周期) - 缓冲区帧号范围: {1: '[100-150]', 2: '[95-145]', 3: '[98-148]'}
```

## 时间戳计算

### 公式
```
sync_timestamp = start_timestamp + (frame_number / fps)
```

### 示例
```python
start_timestamp = 1700000000.0  # 起始时间戳（秒）
fps = 25                         # 帧率
frame_number = 100              # 当前帧号

sync_timestamp = 1700000000.0 + (100 / 25)
               = 1700000000.0 + 4.0
               = 1700000004.0
```

## 优势

✅ **简单高效**
- 帧号是离散的、明确的整数
- 不需要时间窗口容差
- 同步条件清晰：三路帧号相同

✅ **时间戳准确**
- 时间戳由公式计算，确定性强
- 不依赖于帧同步时的参考时间戳
- 避免时间戳漂移问题

✅ **性能优化**
- 缓冲区管理更简单
- 帧号匹配比时间戳匹配更快
- 减少不必要的时间戳解析

## 前提条件

⚠️ **重要：使用本方案前需要确保：**

1. **三路视频帧号同步**
   - 三路视频的帧号应该相同（同一时刻）
   - 用户需要在读流时对齐视频起始时间

2. **准确的fps配置**
   - 需要在 `Basic.py` 中正确设置 `FPS` 值
   - 默认值为 25

3. **正确的起始时间**
   - 需要在 `Basic.py` 的 `CAMERA_START_DATETIMES` 中设置三路摄像头的起始时间
   - 格式：`"YYYY-MM-DD HH:MM:SS.mmm"`

## 使用方式

### 1. 配置起始时间

在 `Basic.py` 中修改：

```python
class TimestampConfig:
    def __post_init__(self):
        if self.CAMERA_START_DATETIMES is None:
            self.CAMERA_START_DATETIMES = {
                1: "2025-11-21 11:59:10.097",  # C1的起始时间
                2: "2025-11-21 11:59:12.200",  # C2的起始时间
                3: "2025-11-21 11:59:10.281",  # C3的起始时间
            }
```

### 2. 确保帧号同步

在读取视频流时，确保三路视频的帧号同步。这通常需要在 `SDKinfer.py` 或视频读取模块中实现。

### 3. 运行程序

程序会自动基于帧号进行同步：

```bash
python main.py
```

## 调试建议

### 检查帧号同步

查看日志中的缓冲区帧号范围：

```
缓冲区帧号范围: {1: '[100-150]', 2: '[100-150]', 3: '[100-150]'}
```

如果三路的帧号范围不同，说明帧号未同步。

### 验证时间戳计算

在日志中查看同步时间戳，验证是否符合公式：

```
sync_timestamp = start_timestamp + (frame_number / fps)
```

### 监控同步状态

- 如果连续无法同步，检查缓冲区状态日志
- 如果摄像头缓冲区为空，检查SDK推理进程是否正常运行
- 如果帧号范围不同，检查三路视频是否同步

## 文件变更统计

```
 3 files changed, 115 insertions(+), 89 deletions(-)
 
 FrameSynchronizer.py: 改为基于帧号同步
 SDKinfer.py: 添加 frame_number 字段
 main.py: 更新初始化和日志
```

## 向后兼容性

- ✅ 保留 `frame_id` 字段用于兼容性
- ✅ 现有的时间戳字段仍然存在
- ✅ 其他模块无需修改

## 相关分支

- **基于分支**：`refactor/data-processing-v2`
- **新分支**：`refactor/frame-number-sync`
- **其他相关分支**：
  - `refactor/fusion-strategy-pattern`：融合策略模式
  - `feature/frame-sync-stable`：帧同步稳定版

## 后续工作

- [ ] 在实际数据上验证帧号同步效果
- [ ] 测试三路视频帧号同步的稳定性
- [ ] 对比时间戳同步和帧号同步的性能差异
- [ ] 根据实际情况调整缓冲区大小

## 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。
