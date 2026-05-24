"""
HUTB Simulator MCP - 基于 FastMCP 的人车仿真器交互控制系统

本模块提供与 HUTB 仿真器（基于 CARLA/Unreal Engine）的 MCP 协议交互接口，
支持车辆控制、无人机控制、编辑器操作、天气控制和传感器管理等功能。

Author: 徐杨杨
Version: 0.1.0
"""

__version__ = "0.1.0"
__author__ = "徐杨杨"

from .connection import HutbConnection, get_connection
from .server import mcp, main

__all__ = [
    "HutbConnection",
    "get_connection", 
    "mcp",
    "main",
    "__version__",
]
