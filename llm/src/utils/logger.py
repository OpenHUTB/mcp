"""
日志配置工具
提供统一的日志管理功能
"""

import sys
from loguru import logger
from src.config import config

def setup_logger():
    """设置日志配置"""
    # 移除默认的处理器
    logger.remove()
    
    # 添加控制台处理器
    logger.add(
        sys.stdout,
        level=config.LOG_LEVEL,
        format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
        colorize=True
    )
    
    # 添加文件处理器（可选）
    logger.add(
        "logs/github_mcp_{time:YYYY-MM-DD}.log",
        level="DEBUG",
        format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {name}:{function}:{line} - {message}",
        rotation="1 day",
        retention="7 days"
    )
    
    return logger

# 创建全局logger实例
app_logger = setup_logger() 