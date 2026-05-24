"""
车辆控制工具模块

提供车辆生成、控制、状态获取等功能。
"""

import logging
from typing import Dict, List, Any, Optional
from mcp.server.fastmcp import FastMCP, Context

logger = logging.getLogger("hutb_mcp.tools.vehicle")


def register_vehicle_tools(mcp: FastMCP) -> None:
    """注册车辆控制工具"""
    
    @mcp.tool()
    def get_vehicle_blueprints(ctx: Context) -> List[str]:
        """
        获取所有可用的车辆蓝图列表
        
        Returns:
            车辆蓝图 ID 列表
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                logger.error("未连接到仿真器")
                return []
            
            blueprints = conn.blueprint_library.filter("vehicle.*")
            result = [bp.id for bp in blueprints]
            
            logger.info(f"获取到 {len(result)} 个车辆蓝图")
            return result
            
        except Exception as e:
            logger.error(f"获取车辆蓝图失败: {e}")
            return []
    
    @mcp.tool()
    def spawn_vehicle(
        ctx: Context,
        blueprint_id: str,
        location: List[float],
        rotation: List[float] = None
    ) -> Dict[str, Any]:
        """
        生成车辆
        
        Args:
            blueprint_id: 车辆蓝图 ID，如 "vehicle.tesla.model3"
            location: 生成位置 [x, y, z]
            rotation: 旋转角度 [pitch, yaw, roll]，默认 [0, 0, 0]
            
        Returns:
            生成结果，包含车辆 ID
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            rotation = rotation or [0, 0, 0]
            
            # 查找蓝图
            blueprint = conn.blueprint_library.find(blueprint_id)
            if not blueprint:
                return {"success": False, "error": f"未找到车辆蓝图: {blueprint_id}"}
            
            # 设置随机颜色（如果支持）
            if blueprint.has_attribute("color"):
                color = blueprint.get_attribute("color").recommended_values
                if color:
                    blueprint.set_attribute("color", color[0])
            
            # 创建变换
            transform = carla.Transform(
                carla.Location(x=location[0], y=location[1], z=location[2]),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])
            )
            
            # 生成车辆
            vehicle = conn.world.spawn_actor(blueprint, transform)
            
            if vehicle:
                conn.track_vehicle(vehicle.id)
                logger.info(f"生成车辆成功: {vehicle.id} ({blueprint_id})")
                return {
                    "success": True,
                    "vehicle_id": vehicle.id,
                    "blueprint": blueprint_id,
                    "location": location
                }
            else:
                return {"success": False, "error": "生成车辆失败，可能位置有碰撞"}
                
        except Exception as e:
            logger.error(f"生成车辆失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def destroy_vehicle(ctx: Context, vehicle_id: int) -> Dict[str, Any]:
        """
        销毁车辆
        
        Args:
            vehicle_id: 车辆 ID
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            vehicle.destroy()
            conn.untrack_vehicle(vehicle_id)
            
            logger.info(f"销毁车辆成功: {vehicle_id}")
            return {"success": True, "vehicle_id": vehicle_id}
            
        except Exception as e:
            logger.error(f"销毁车辆失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_vehicle_autopilot(
        ctx: Context,
        vehicle_id: int,
        enabled: bool
    ) -> Dict[str, Any]:
        """
        设置车辆自动驾驶模式
        
        Args:
            vehicle_id: 车辆 ID
            enabled: 是否启用自动驾驶
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            vehicle.set_autopilot(enabled)
            
            logger.info(f"设置车辆 {vehicle_id} 自动驾驶: {enabled}")
            return {
                "success": True,
                "vehicle_id": vehicle_id,
                "autopilot": enabled
            }
            
        except Exception as e:
            logger.error(f"设置自动驾驶失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def apply_vehicle_control(
        ctx: Context,
        vehicle_id: int,
        throttle: float = 0.0,
        steer: float = 0.0,
        brake: float = 0.0,
        hand_brake: bool = False,
        reverse: bool = False
    ) -> Dict[str, Any]:
        """
        应用车辆控制
        
        Args:
            vehicle_id: 车辆 ID
            throttle: 油门 (0.0-1.0)
            steer: 转向 (-1.0 到 1.0，负值左转，正值右转)
            brake: 刹车 (0.0-1.0)
            hand_brake: 手刹
            reverse: 倒车
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            # 限制参数范围
            throttle = max(0.0, min(1.0, throttle))
            steer = max(-1.0, min(1.0, steer))
            brake = max(0.0, min(1.0, brake))
            
            # 创建控制对象
            control = carla.VehicleControl(
                throttle=throttle,
                steer=steer,
                brake=brake,
                hand_brake=hand_brake,
                reverse=reverse
            )
            
            vehicle.apply_control(control)
            
            logger.info(f"应用车辆 {vehicle_id} 控制: throttle={throttle}, steer={steer}, brake={brake}")
            return {
                "success": True,
                "vehicle_id": vehicle_id,
                "control": {
                    "throttle": throttle,
                    "steer": steer,
                    "brake": brake,
                    "hand_brake": hand_brake,
                    "reverse": reverse
                }
            }
            
        except Exception as e:
            logger.error(f"应用车辆控制失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def get_vehicle_state(ctx: Context, vehicle_id: int) -> Dict[str, Any]:
        """
        获取车辆状态
        
        Args:
            vehicle_id: 车辆 ID
            
        Returns:
            车辆状态信息
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            acceleration = vehicle.get_acceleration()
            angular_velocity = vehicle.get_angular_velocity()
            
            # 计算速度大小 (km/h)
            speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            
            result = {
                "success": True,
                "vehicle_id": vehicle_id,
                "type_id": vehicle.type_id,
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
                "speed_kmh": speed,
                "acceleration": {
                    "x": acceleration.x,
                    "y": acceleration.y,
                    "z": acceleration.z
                },
                "angular_velocity": {
                    "x": angular_velocity.x,
                    "y": angular_velocity.y,
                    "z": angular_velocity.z
                }
            }
            
            logger.info(f"获取车辆 {vehicle_id} 状态成功")
            return result
            
        except Exception as e:
            logger.error(f"获取车辆状态失败: {e}")
            return {"success": False, "error": str(e)}
    
    logger.info("车辆控制工具注册完成")
