"""
统一日志配置模块
- 单一日志文件：logs/fusion_system.log
- 包含所有模块的日志（包括调试信息）
- 输出路径在日志中明确标记
"""

import logging
import os
import json
from datetime import datetime
from pathlib import Path


class FusionLogger:
    """统一的融合系统日志管理器"""
    
    # 类级别的logger（全局唯一）
    _logger = None
    _handlers_initialized = False
    
    # 配置
    LOG_DIR = Path(__file__).parent.parent / 'logs'
    LOG_FILE = LOG_DIR / 'fusion_system.log'
    
    # 🔧 模块级日志开关配置
    ENABLE_RADAR_FUSION_LOG = False  # 是否输出雷达融合日志（RadarVisionFusion + RadarFusionOrchestrator）
    ENABLE_DEBUG_LOG = False          # 是否输出调试日志（包含详细的成本矩阵计算）
    ENABLE_RADAR_FILTER_LOG = False   # 是否输出雷达过滤日志（RadarDataFilter的批量过滤日志）
    
    @classmethod
    def setup(cls, enable_radar_fusion=True, enable_debug=False, enable_radar_filter=False):
        """
        一次性设置日志系统（由main.py调用）
        
        Args:
            enable_radar_fusion: 是否启用雷达融合日志输出
                - RadarVisionFusion: 融合尝试、匹配结果、耗时统计
                - RadarFusionOrchestrator: 地理区域过滤、摄像头融合、总体统计
            enable_debug: 是否启用调试级别日志（包含详细信息）
                - 成本矩阵计算、车道检查、坐标诊断、时间戳匹配详情
            enable_radar_filter: 是否启用雷达过滤日志输出
                - RadarDataFilter: 批量过滤统计、融合区内/外数据信息
        """
        if cls._handlers_initialized:
            return cls._logger
        
        # 保存配置
        cls.ENABLE_RADAR_FUSION_LOG = enable_radar_fusion
        cls.ENABLE_DEBUG_LOG = enable_debug
        cls.ENABLE_RADAR_FILTER_LOG = enable_radar_filter
        
        # 创建日志目录
        cls.LOG_DIR.mkdir(exist_ok=True)
        
        # 配置root logger
        root_logger = logging.getLogger()
        log_level = logging.DEBUG if enable_debug else logging.INFO
        root_logger.setLevel(log_level)
        
        # 清除已有的handlers（防止重复）
        root_logger.handlers.clear()
        
        # 文件处理器 - 写入所有日志到单一文件
        file_handler = logging.FileHandler(
            cls.LOG_FILE, 
            mode='w',  # 清空模式，每次运行先清空
            encoding='utf-8'
        )
        file_handler.setLevel(log_level)
        
        # 日志格式
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        
        root_logger.addHandler(file_handler)
        
        # 保存logger引用
        cls._logger = logging.getLogger(__name__)
        cls._handlers_initialized = True
        
        # 记录初始化信息
        cls._logger.info("=" * 70)
        cls._logger.info("融合系统启动")
        cls._logger.info("=" * 70)
        cls._logger.info(f"日志文件: {cls.LOG_FILE}")
        cls._logger.info(f"启动时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        if enable_radar_fusion:
            cls._logger.info("✅ 雷达融合日志已启用（包括：")
            cls._logger.info("   - RadarVisionFusion: 融合尝试、匹配结果、耗时")
            cls._logger.info("   - RadarFusionOrchestrator: 地理过滤、摄像头融合、统计）")
        else:
            cls._logger.info("❌ 雷达融合日志已禁用")
        if enable_debug:
            cls._logger.info("✅ 调试日志已启用（包括：")
            cls._logger.info("   - 成本矩阵计算、车道检查、坐标诊断、时间戳详情）")
        else:
            cls._logger.info("❌ 调试日志已禁用")
        if enable_radar_filter:
            cls._logger.info("✅ 雷达过滤日志已启用（包括：")
            cls._logger.info("   - RadarDataFilter: 批量过滤统计、融合区内/外数据）")
        else:
            cls._logger.info("❌ 雷达过滤日志已禁用")
        cls._logger.info("=" * 70)
        
        return cls._logger
    
    @classmethod
    def get_logger(cls, name: str = None) -> logging.Logger:
        """获取logger实例"""
        if not cls._handlers_initialized:
            cls.setup()
        return logging.getLogger(name)
    
    @classmethod
    def log_output_path(cls, output_file: str):
        """记录输出文件路径信息"""
        abs_path = os.path.abspath(output_file)
        logger = cls.get_logger(__name__)
        logger.info("=" * 70)
        logger.info(f"📁 输出文件: {abs_path}")
        logger.info(f"   文件大小: 等待生成...")
        logger.info("=" * 70)
    
    @classmethod
    def log_save_result(cls, output_file: str, count: int, size_kb: float):
        """记录保存结果"""
        abs_path = os.path.abspath(output_file)
        logger = cls.get_logger(__name__)
        logger.info("=" * 70)
        logger.info(f"✅ 数据已保存: {abs_path}")
        logger.info(f"   数据条目: {count} 条")
        logger.info(f"   文件大小: {size_kb:.2f} KB")
        logger.info("=" * 70)
    
    @classmethod
    def log_program_end(cls):
        """记录程序结束"""
        logger = cls.get_logger(__name__)
        logger.info("=" * 70)
        logger.info(f"融合系统已停止")
        logger.info(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info(f"日志文件: {cls.LOG_FILE}")
        logger.info("=" * 70)


# 便捷函数
def get_logger(name: str = None) -> logging.Logger:
    """获取logger实例"""
    return FusionLogger.get_logger(name)


def setup_logger():
    """初始化日志系统"""
    return FusionLogger.setup()

