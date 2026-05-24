"""
HUTB MCP 工具模块

包含所有 MCP 工具的实现：
- vehicle_tools: 车辆控制工具
- air_tools: 无人机控制工具  
- editor_tools: 编辑器控制工具
- weather_tools: 天气控制工具
- sensor_tools: 传感器管理工具
"""

from .vehicle_tools import register_vehicle_tools
from .air_tools import register_air_tools
from .editor_tools import register_editor_tools
from .weather_tools import register_weather_tools
from .sensor_tools import register_sensor_tools

__all__ = [
    "register_vehicle_tools",
    "register_air_tools",
    "register_editor_tools",
    "register_weather_tools",
    "register_sensor_tools",
]
