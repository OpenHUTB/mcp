"""
HUTB MCP 服务器主模块

基于 FastMCP 框架实现的 HUTB 仿真器交互控制服务器。
"""

import logging
import sys
from contextlib import asynccontextmanager
from typing import AsyncIterator, Dict, Any

from mcp.server.fastmcp import FastMCP

from .config import get_config
from .connection import get_connection, reset_connection
from .utils.logger import setup_logging, get_logger

# 初始化日志
cfg = get_config()
setup_logging(level=cfg.log.level, log_file=cfg.log.log_file)
logger = get_logger("hutb_mcp.server")


@asynccontextmanager
async def server_lifespan(server: FastMCP) -> AsyncIterator[Dict[str, Any]]:
    """
    服务器生命周期管理
    
    在服务器启动时建立与仿真器的连接，
    在服务器关闭时清理资源并断开连接。
    """
    logger.info("HUTB MCP 服务器启动中...")
    
    try:
        # 尝试连接仿真器
        conn = get_connection()
        if conn.is_connected():
            logger.info(f"成功连接到 HUTB 仿真器，当前地图: {conn.map.name}")
        else:
            logger.warning("无法连接到 HUTB 仿真器，服务器将在无连接模式下运行")
    except Exception as e:
        logger.error(f"连接仿真器时出错: {e}")
    
    try:
        yield {}
    finally:
        # 清理资源
        logger.info("HUTB MCP 服务器关闭中...")
        try:
            reset_connection()
            logger.info("资源清理完成")
        except Exception as e:
            logger.error(f"清理资源时出错: {e}")
        logger.info("HUTB MCP 服务器已关闭")


# 创建 FastMCP 服务器实例
mcp = FastMCP(
    name=cfg.mcp.server_name,
    instructions="基于 FastMCP 的 HUTB 人车仿真器交互控制系统，提供车辆控制、无人机控制、编辑器操作、天气控制和传感器管理等功能。",
    lifespan=server_lifespan
)


# 注册所有工具模块
from .tools.editor_tools import register_editor_tools
from .tools.vehicle_tools import register_vehicle_tools
from .tools.weather_tools import register_weather_tools
from .tools.sensor_tools import register_sensor_tools
from .tools.air_tools import register_air_tools

register_editor_tools(mcp)
register_vehicle_tools(mcp)
register_weather_tools(mcp)
register_sensor_tools(mcp)
register_air_tools(mcp)


# 注册健康检查工具
@mcp.tool()
def health_check() -> Dict[str, Any]:
    """
    健康检查
    
    检查服务器和仿真器连接状态。
    
    Returns:
        健康状态信息
    """
    try:
        conn = get_connection()
        status = conn.get_status()
        return {
            "success": True,
            "server": cfg.mcp.server_name,
            "version": cfg.mcp.server_version,
            "simulator": status
        }
    except Exception as e:
        return {
            "success": False,
            "server": cfg.mcp.server_name,
            "version": cfg.mcp.server_version,
            "error": str(e)
        }


# 注册提示信息
@mcp.prompt()
def info():
    """HUTB MCP 服务器工具和最佳实践信息"""
    return """
    # HUTB MCP 服务器工具指南
    
    ## 车辆控制工具
    - `get_vehicle_blueprints()` - 获取可用车辆蓝图列表
    - `spawn_vehicle(blueprint_id, location, rotation)` - 生成车辆
    - `destroy_vehicle(vehicle_id)` - 销毁车辆
    - `set_vehicle_autopilot(vehicle_id, enabled)` - 设置自动驾驶
    - `apply_vehicle_control(vehicle_id, throttle, steer, brake, ...)` - 应用车辆控制
    - `get_vehicle_state(vehicle_id)` - 获取车辆状态
    
    ## 编辑器控制工具
    - `get_actors_in_level()` - 获取场景中所有 Actor
    - `get_current_map()` - 获取当前地图名称
    - `load_map(map_name)` - 加载地图 (推荐 Town10 或 Town01)
    - `spawn_actor(actor_type, location, rotation)` - 生成 Actor
    - `delete_actor(actor_id)` - 删除 Actor
    - `set_actor_transform(actor_id, location, rotation)` - 设置 Actor 变换
    - `get_actor_properties(actor_id)` - 获取 Actor 属性
    
    ## 天气控制工具
    - `get_weather()` - 获取当前天气参数
    - `set_weather(cloudiness, precipitation, ...)` - 设置天气参数
    - `set_weather_preset(preset)` - 应用天气预设 (clear, cloudy, rainy, foggy, stormy)
    - `set_time_of_day(hour, minute)` - 设置时间
    - `set_extreme_weather(weather_type)` - 设置极端天气 (heavy_rain, dense_fog, blizzard, sandstorm)
    
    ## 传感器管理工具
    - `attach_camera(vehicle_id, camera_type, ...)` - 附加相机 (rgb, depth, semantic_segmentation)
    - `attach_lidar(vehicle_id, ...)` - 附加激光雷达
    - `attach_radar(vehicle_id, ...)` - 附加雷达
    - `destroy_sensor(sensor_id)` - 销毁传感器
    - `list_vehicle_sensors(vehicle_id)` - 列出车辆传感器
    
    ## 无人机控制工具
    - `spawn_drone(location, rotation)` - 生成无人机
    - `get_drone_state(drone_id)` - 获取无人机状态
    - `set_drone_destination(drone_id, destination)` - 设置目标位置
    - `drone_hover(drone_id)` - 悬停
    - `drone_land(drone_id)` - 降落
    - `destroy_drone(drone_id)` - 销毁无人机
    
    ## 系统工具
    - `health_check()` - 健康检查
    
    ## 最佳实践
    
    ### 测试场景
    - 优先使用 Town10 场景进行测试
    - Town01 作为备选测试场景
    
    ### 车辆控制
    - 生成车辆前先获取可用蓝图列表
    - 使用自动驾驶进行基础测试
    - 手动控制时注意参数范围 (throttle/brake: 0-1, steer: -1 到 1)
    
    ### 天气测试
    - 使用预设快速切换天气
    - 极端天气用于压力测试
    - 注意天气对传感器的影响
    
    ### 资源管理
    - 及时销毁不需要的 Actor 和传感器
    - 服务器关闭时会自动清理资源
    """


def main():
    """主入口函数"""
    logger.info(f"启动 {cfg.mcp.server_name} v{cfg.mcp.server_version}")
    logger.info(f"仿真器地址: {cfg.hutb.host}:{cfg.hutb.port}")
    
    # 打印已注册的工具
    print(f"[HUTB-MCP] 启动 {cfg.mcp.server_name} v{cfg.mcp.server_version}")
    print(f"[HUTB-MCP] 仿真器地址: {cfg.hutb.host}:{cfg.hutb.port}")
    print("[HUTB-MCP] 已注册工具模块:")
    print("  - 编辑器控制 (editor_tools)")
    print("  - 车辆控制 (vehicle_tools)")
    print("  - 天气控制 (weather_tools)")
    print("  - 传感器管理 (sensor_tools)")
    print("  - 无人机控制 (air_tools)")
    print("[HUTB-MCP] 等待 AI 客户端连接...")
    
    # 运行服务器
    mcp.run(transport='stdio')


if __name__ == "__main__":
    main()
