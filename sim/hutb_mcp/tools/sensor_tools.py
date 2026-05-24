"""
传感器管理工具模块

提供相机、激光雷达、雷达等传感器的创建和管理功能。
"""

import logging
from typing import Dict, List, Any, Optional
from mcp.server.fastmcp import FastMCP, Context

logger = logging.getLogger("hutb_mcp.tools.sensor")


def register_sensor_tools(mcp: FastMCP) -> None:
    """注册传感器管理工具"""
    
    @mcp.tool()
    def attach_camera(
        ctx: Context,
        vehicle_id: int,
        camera_type: str = "rgb",
        location: List[float] = None,
        rotation: List[float] = None,
        image_size_x: int = 800,
        image_size_y: int = 600,
        fov: float = 90.0
    ) -> Dict[str, Any]:
        """
        为车辆附加相机传感器
        
        Args:
            vehicle_id: 车辆 ID
            camera_type: 相机类型，可选值: rgb, depth, semantic_segmentation
            location: 相对于车辆的位置 [x, y, z]，默认 [0, 0, 2]
            rotation: 相对于车辆的旋转 [pitch, yaw, roll]，默认 [0, 0, 0]
            image_size_x: 图像宽度
            image_size_y: 图像高度
            fov: 视场角
            
        Returns:
            传感器信息
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            location = location or [0, 0, 2]
            rotation = rotation or [0, 0, 0]
            
            # 获取车辆
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            # 确定相机蓝图
            camera_blueprints = {
                "rgb": "sensor.camera.rgb",
                "depth": "sensor.camera.depth",
                "semantic_segmentation": "sensor.camera.semantic_segmentation"
            }
            
            if camera_type not in camera_blueprints:
                return {
                    "success": False,
                    "error": f"未知相机类型: {camera_type}，可选值: {list(camera_blueprints.keys())}"
                }
            
            # 获取蓝图
            blueprint = conn.blueprint_library.find(camera_blueprints[camera_type])
            
            # 设置属性
            blueprint.set_attribute("image_size_x", str(image_size_x))
            blueprint.set_attribute("image_size_y", str(image_size_y))
            blueprint.set_attribute("fov", str(fov))
            
            # 创建变换
            transform = carla.Transform(
                carla.Location(x=location[0], y=location[1], z=location[2]),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])
            )
            
            # 附加传感器
            sensor = conn.world.spawn_actor(blueprint, transform, attach_to=vehicle)
            
            if sensor:
                conn.track_sensor(sensor.id)
                logger.info(f"附加相机成功: {sensor.id} ({camera_type})")
                return {
                    "success": True,
                    "sensor_id": sensor.id,
                    "type": camera_type,
                    "parent_id": vehicle_id,
                    "image_size": [image_size_x, image_size_y],
                    "fov": fov
                }
            else:
                return {"success": False, "error": "附加相机失败"}
                
        except Exception as e:
            logger.error(f"附加相机失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def attach_lidar(
        ctx: Context,
        vehicle_id: int,
        location: List[float] = None,
        rotation: List[float] = None,
        channels: int = 32,
        range: float = 100.0,
        points_per_second: int = 56000,
        rotation_frequency: float = 10.0,
        upper_fov: float = 10.0,
        lower_fov: float = -30.0
    ) -> Dict[str, Any]:
        """
        为车辆附加激光雷达传感器
        
        Args:
            vehicle_id: 车辆 ID
            location: 相对于车辆的位置 [x, y, z]，默认 [0, 0, 2.5]
            rotation: 相对于车辆的旋转 [pitch, yaw, roll]，默认 [0, 0, 0]
            channels: 激光通道数
            range: 探测范围（米）
            points_per_second: 每秒点数
            rotation_frequency: 旋转频率（Hz）
            upper_fov: 上视场角
            lower_fov: 下视场角
            
        Returns:
            传感器信息
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            location = location or [0, 0, 2.5]
            rotation = rotation or [0, 0, 0]
            
            # 获取车辆
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            # 获取蓝图
            blueprint = conn.blueprint_library.find("sensor.lidar.ray_cast")
            
            # 设置属性
            blueprint.set_attribute("channels", str(channels))
            blueprint.set_attribute("range", str(range))
            blueprint.set_attribute("points_per_second", str(points_per_second))
            blueprint.set_attribute("rotation_frequency", str(rotation_frequency))
            blueprint.set_attribute("upper_fov", str(upper_fov))
            blueprint.set_attribute("lower_fov", str(lower_fov))
            
            # 创建变换
            transform = carla.Transform(
                carla.Location(x=location[0], y=location[1], z=location[2]),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])
            )
            
            # 附加传感器
            sensor = conn.world.spawn_actor(blueprint, transform, attach_to=vehicle)
            
            if sensor:
                conn.track_sensor(sensor.id)
                logger.info(f"附加激光雷达成功: {sensor.id}")
                return {
                    "success": True,
                    "sensor_id": sensor.id,
                    "type": "lidar",
                    "parent_id": vehicle_id,
                    "channels": channels,
                    "range": range
                }
            else:
                return {"success": False, "error": "附加激光雷达失败"}
                
        except Exception as e:
            logger.error(f"附加激光雷达失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def attach_radar(
        ctx: Context,
        vehicle_id: int,
        location: List[float] = None,
        rotation: List[float] = None,
        horizontal_fov: float = 30.0,
        vertical_fov: float = 10.0,
        range: float = 100.0,
        points_per_second: int = 1500
    ) -> Dict[str, Any]:
        """
        为车辆附加雷达传感器
        
        Args:
            vehicle_id: 车辆 ID
            location: 相对于车辆的位置 [x, y, z]，默认 [2, 0, 1]
            rotation: 相对于车辆的旋转 [pitch, yaw, roll]，默认 [0, 0, 0]
            horizontal_fov: 水平视场角
            vertical_fov: 垂直视场角
            range: 探测范围（米）
            points_per_second: 每秒点数
            
        Returns:
            传感器信息
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            location = location or [2, 0, 1]
            rotation = rotation or [0, 0, 0]
            
            # 获取车辆
            vehicle = conn.world.get_actor(vehicle_id)
            if not vehicle:
                return {"success": False, "error": f"未找到车辆: {vehicle_id}"}
            
            # 获取蓝图
            blueprint = conn.blueprint_library.find("sensor.other.radar")
            
            # 设置属性
            blueprint.set_attribute("horizontal_fov", str(horizontal_fov))
            blueprint.set_attribute("vertical_fov", str(vertical_fov))
            blueprint.set_attribute("range", str(range))
            blueprint.set_attribute("points_per_second", str(points_per_second))
            
            # 创建变换
            transform = carla.Transform(
                carla.Location(x=location[0], y=location[1], z=location[2]),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])
            )
            
            # 附加传感器
            sensor = conn.world.spawn_actor(blueprint, transform, attach_to=vehicle)
            
            if sensor:
                conn.track_sensor(sensor.id)
                logger.info(f"附加雷达成功: {sensor.id}")
                return {
                    "success": True,
                    "sensor_id": sensor.id,
                    "type": "radar",
                    "parent_id": vehicle_id,
                    "horizontal_fov": horizontal_fov,
                    "vertical_fov": vertical_fov,
                    "range": range
                }
            else:
                return {"success": False, "error": "附加雷达失败"}
                
        except Exception as e:
            logger.error(f"附加雷达失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def destroy_sensor(ctx: Context, sensor_id: int) -> Dict[str, Any]:
        """
        销毁传感器
        
        Args:
            sensor_id: 传感器 ID
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            sensor = conn.world.get_actor(sensor_id)
            if not sensor:
                return {"success": False, "error": f"未找到传感器: {sensor_id}"}
            
            sensor.destroy()
            conn.untrack_sensor(sensor_id)
            
            logger.info(f"销毁传感器成功: {sensor_id}")
            return {"success": True, "sensor_id": sensor_id}
            
        except Exception as e:
            logger.error(f"销毁传感器失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def list_vehicle_sensors(ctx: Context, vehicle_id: int) -> List[Dict[str, Any]]:
        """
        列出车辆上的所有传感器
        
        Args:
            vehicle_id: 车辆 ID
            
        Returns:
            传感器列表
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return []
            
            # 获取所有传感器类型的 Actor
            sensors = conn.world.get_actors().filter("sensor.*")
            
            result = []
            for sensor in sensors:
                # 检查是否附加到指定车辆
                parent = sensor.parent
                if parent and parent.id == vehicle_id:
                    transform = sensor.get_transform()
                    result.append({
                        "id": sensor.id,
                        "type_id": sensor.type_id,
                        "location": {
                            "x": transform.location.x,
                            "y": transform.location.y,
                            "z": transform.location.z
                        },
                        "attributes": dict(sensor.attributes)
                    })
            
            logger.info(f"车辆 {vehicle_id} 有 {len(result)} 个传感器")
            return result
            
        except Exception as e:
            logger.error(f"列出传感器失败: {e}")
            return []
    
    logger.info("传感器管理工具注册完成")
