"""
无人机控制工具模块

提供无人机生成、飞行控制等功能。
注意：HUTB/CARLA 原生不支持无人机，此模块提供基于 Walker 的模拟实现。
"""

import logging
from typing import Dict, List, Any, Optional
from mcp.server.fastmcp import FastMCP, Context

logger = logging.getLogger("hutb_mcp.tools.air")


def register_air_tools(mcp: FastMCP) -> None:
    """注册无人机控制工具"""
    
    @mcp.tool()
    def spawn_drone(
        ctx: Context,
        location: List[float],
        rotation: List[float] = None
    ) -> Dict[str, Any]:
        """
        生成无人机（模拟实现）
        
        注意：CARLA 原生不支持无人机，此功能使用特殊 Actor 模拟。
        
        Args:
            location: 生成位置 [x, y, z]
            rotation: 旋转角度 [pitch, yaw, roll]，默认 [0, 0, 0]
            
        Returns:
            生成结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            rotation = rotation or [0, 0, 0]
            
            # 使用静态道具模拟无人机
            # 在实际项目中，可以使用自定义的无人机模型
            blueprint = conn.blueprint_library.find("static.prop.box01")
            
            if not blueprint:
                # 尝试其他可用的静态道具
                props = list(conn.blueprint_library.filter("static.prop.*"))
                if props:
                    blueprint = props[0]
                else:
                    return {"success": False, "error": "未找到可用的无人机蓝图"}
            
            # 创建变换
            transform = carla.Transform(
                carla.Location(x=location[0], y=location[1], z=location[2]),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])
            )
            
            # 生成无人机
            drone = conn.world.spawn_actor(blueprint, transform)
            
            if drone:
                conn.track_actor(drone.id)
                logger.info(f"生成无人机成功: {drone.id}")
                return {
                    "success": True,
                    "drone_id": drone.id,
                    "location": location,
                    "note": "无人机为模拟实现，使用静态道具代替"
                }
            else:
                return {"success": False, "error": "生成无人机失败"}
                
        except Exception as e:
            logger.error(f"生成无人机失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def get_drone_state(ctx: Context, drone_id: int) -> Dict[str, Any]:
        """
        获取无人机状态
        
        Args:
            drone_id: 无人机 ID
            
        Returns:
            无人机状态信息
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            drone = conn.world.get_actor(drone_id)
            if not drone:
                return {"success": False, "error": f"未找到无人机: {drone_id}"}
            
            transform = drone.get_transform()
            velocity = drone.get_velocity()
            
            result = {
                "success": True,
                "drone_id": drone_id,
                "location": {
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z
                },
                "rotation": {
                    "pitch": transform.rotation.pitch,
                    "yaw": transform.rotation.yaw,
                    "roll": transform.rotation.roll
                },
                "velocity": {
                    "x": velocity.x,
                    "y": velocity.y,
                    "z": velocity.z
                },
                "altitude": transform.location.z
            }
            
            logger.info(f"获取无人机 {drone_id} 状态成功")
            return result
            
        except Exception as e:
            logger.error(f"获取无人机状态失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_drone_destination(
        ctx: Context,
        drone_id: int,
        destination: List[float]
    ) -> Dict[str, Any]:
        """
        设置无人机目标位置
        
        Args:
            drone_id: 无人机 ID
            destination: 目标位置 [x, y, z]
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            drone = conn.world.get_actor(drone_id)
            if not drone:
                return {"success": False, "error": f"未找到无人机: {drone_id}"}
            
            # 直接设置位置（简化实现）
            # 在实际项目中，应该实现平滑的飞行路径
            current_transform = drone.get_transform()
            current_transform.location = carla.Location(
                x=destination[0],
                y=destination[1],
                z=destination[2]
            )
            drone.set_transform(current_transform)
            
            logger.info(f"设置无人机 {drone_id} 目标位置: {destination}")
            return {
                "success": True,
                "drone_id": drone_id,
                "destination": destination,
                "note": "位置已直接设置，实际项目中应实现平滑飞行"
            }
            
        except Exception as e:
            logger.error(f"设置无人机目标位置失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def drone_hover(ctx: Context, drone_id: int) -> Dict[str, Any]:
        """
        无人机悬停
        
        Args:
            drone_id: 无人机 ID
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            drone = conn.world.get_actor(drone_id)
            if not drone:
                return {"success": False, "error": f"未找到无人机: {drone_id}"}
            
            # 静态道具默认就是悬停状态
            transform = drone.get_transform()
            
            logger.info(f"无人机 {drone_id} 悬停中")
            return {
                "success": True,
                "drone_id": drone_id,
                "status": "hovering",
                "location": {
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z
                }
            }
            
        except Exception as e:
            logger.error(f"无人机悬停失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def drone_land(ctx: Context, drone_id: int) -> Dict[str, Any]:
        """
        无人机降落
        
        Args:
            drone_id: 无人机 ID
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            drone = conn.world.get_actor(drone_id)
            if not drone:
                return {"success": False, "error": f"未找到无人机: {drone_id}"}
            
            # 将无人机降落到地面
            transform = drone.get_transform()
            transform.location.z = 0.5  # 降落到接近地面的高度
            drone.set_transform(transform)
            
            logger.info(f"无人机 {drone_id} 降落成功")
            return {
                "success": True,
                "drone_id": drone_id,
                "status": "landed",
                "location": {
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z
                }
            }
            
        except Exception as e:
            logger.error(f"无人机降落失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def destroy_drone(ctx: Context, drone_id: int) -> Dict[str, Any]:
        """
        销毁无人机
        
        Args:
            drone_id: 无人机 ID
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            drone = conn.world.get_actor(drone_id)
            if not drone:
                return {"success": False, "error": f"未找到无人机: {drone_id}"}
            
            drone.destroy()
            conn.untrack_actor(drone_id)
            
            logger.info(f"销毁无人机成功: {drone_id}")
            return {"success": True, "drone_id": drone_id}
            
        except Exception as e:
            logger.error(f"销毁无人机失败: {e}")
            return {"success": False, "error": str(e)}
    
    logger.info("无人机控制工具注册完成")
