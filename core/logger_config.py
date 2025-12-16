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
    
    @classmethod
    def setup(cls):
        """一次性设置日志系统（由main.py调用）"""
        if cls._handlers_initialized:
            return cls._logger
        
        # 创建日志目录
        cls.LOG_DIR.mkdir(exist_ok=True)
        
        # 配置root logger
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.DEBUG)
        
        # 清除已有的handlers（防止重复）
        root_logger.handlers.clear()
        
        # 文件处理器 - 写入所有日志到单一文件
        file_handler = logging.FileHandler(
            cls.LOG_FILE, 
            mode='w',  # 清空模式，每次运行先清空
            encoding='utf-8'
        )
        file_handler.setLevel(logging.DEBUG)
        
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

