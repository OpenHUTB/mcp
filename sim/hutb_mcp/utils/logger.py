"""
HUTB MCP 日志系统

提供统一的日志记录功能，支持文件和控制台输出。
"""

import logging
import sys
from pathlib import Path
from typing import Optional

# 日志格式
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

# 全局日志器缓存
_loggers: dict = {}


def setup_logging(
    level: str = "INFO",
    log_file: Optional[str] = None,
    log_format: str = LOG_FORMAT
) -> None:
    """
    设置全局日志配置
    
    Args:
        level: 日志级别 (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: 日志文件路径，None 表示不输出到文件
        log_format: 日志格式字符串
    """
    # 获取日志级别
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    
    # 配置根日志器
    root_logger = logging.getLogger()
    root_logger.setLevel(numeric_level)
    
    # 清除现有处理器
    root_logger.handlers.clear()
    
    # 创建格式器
    formatter = logging.Formatter(log_format, datefmt=LOG_DATE_FORMAT)
    
    # 添加控制台处理器
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)
    
    # 添加文件处理器（如果指定）
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)


def get_logger(name: str) -> logging.Logger:
    """
    获取指定名称的日志器
    
    Args:
        name: 日志器名称，通常使用模块名
        
    Returns:
        配置好的日志器实例
    """
    if name not in _loggers:
        logger = logging.getLogger(name)
        _loggers[name] = logger
    return _loggers[name]


# 默认初始化日志系统
def init_default_logging():
    """使用默认配置初始化日志系统"""
    from ..config import get_config
    cfg = get_config()
    setup_logging(
        level=cfg.log.level,
        log_file=cfg.log.log_file
    )


# 模块级日志器
logger = get_logger("hutb_mcp")
