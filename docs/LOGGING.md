# 日志系统说明

## 概述

融合系统采用**统一日志系统**，将所有日志信息集中到单一文件中，便于追踪和调试。

## 日志文件位置

- **主日志文件**: `logs/fusion_system.log`
- **日志位置**: 项目根目录下的 `logs/` 目录

## 日志内容

日志文件包含以下信息：

### 1. 系统启动信息
```
2025-12-10 15:03:03,577 - __main__ - INFO - ============================================================
2025-12-10 15:03:03,577 - __main__ - INFO - 融合系统启动
2025-12-10 15:03:03,577 - __main__ - INFO - ============================================================
2025-12-10 15:03:03,577 - __main__ - INFO - 日志文件: /zhw/no-frame-sync/logs/fusion_system.log
```

### 2. 摄像头和模块初始化日志
```
2025-12-10 15:03:03,579 - __main__ - INFO - 摄像头1: Main Entrance - /path/to/video
2025-12-10 15:03:09,829 - Fusion - INFO - CrossCameraFusion初始化完成
```

### 3. 融合和跟踪过程日志
- BYTETracker 跟踪信息
- 目标匹配决策
- 融合算法日志（来自FusionDebugger）

### 4. **输出文件路径信息** ⭐
```
======================================================================
✅ JSON数据已保存
   文件路径: /zhw/no-frame-sync/output_fusion_refactored.json
   数据条目: 1234 条
   文件大小: 45.67 KB
======================================================================
```

### 5. 程序结束信息
```
======================================================================
融合系统已停止
日志文件: /zhw/no-frame-sync/logs/fusion_system.log
======================================================================
```

## 查看日志

### 实时查看
```bash
tail -f logs/fusion_system.log
```

### 查看最后100行
```bash
tail -100 logs/fusion_system.log
```

### 搜索特定信息
```bash
# 查看输出路径
grep "文件路径:" logs/fusion_system.log

# 查看所有错误
grep "ERROR" logs/fusion_system.log

# 查看特定摄像头的日志
grep "C1" logs/fusion_system.log
```

## 日志级别

- **INFO**: 正常程序流程、初始化、完成状态
- **WARNING**: 非关键的警告信息（如MQTT连接失败）
- **ERROR**: 错误信息和异常
- **DEBUG**: 详细的调试信息

## 日志配置

日志系统由 `core/logger_config.py` 统一管理：

```python
# 获取logger
from core.logger_config import get_logger
logger = get_logger(__name__)

# 初始化日志系统（main.py自动调用）
from core.logger_config import FusionLogger
FusionLogger.setup()
```

## 历史改进

### v1.1 更新（2025-12-10）
- ✅ 统一日志系统：合并 `fusion_debug.log` 到 `fusion_system.log`
- ✅ 完整的输出路径信息：包含文件路径、大小、条目数
- ✅ 改进的日志可读性：添加分隔符和格式化
- ✅ 减少日志文件数量：从2个日志文件改为1个

### 之前版本
- 运行时生成两个日志文件
- 输出文件路径信息不完整

