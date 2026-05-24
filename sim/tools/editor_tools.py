"""
编辑器控制工具模块

提供场景编辑、Actor 管理、地图加载等功能。
"""

import logging
from typing import Dict, List, Any, Optional
from mcp.server.fastmcp import FastMCP, Context

logger = logging.getLogger("hutb_mcp.tools.editor")


def register_editor_tools(mcp: FastMCP) -> None:
    """注册编辑器控制工具"""
    
    @mcp.tool()
    def get_actors_in_level(ctx: Context) -> List[Dict[str, Any]]:
        """
        获取当前场景中所有 Actor 的列表
        
        Returns:
            Actor 列表，每个 Actor 包含 id、type_id、name 等信息
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                logger.error("未连接到仿真器")
                return []
            
            actors = conn.world.get_actors()
            result = []
            
            for actor in actors:
                result.append({
                    "id": actor.id,
                    "type_id": actor.type_id,
                    "name": actor.attributes.get("role_name", ""),
                    "location": {
                        "x": actor.get_location().x,
                        "y": actor.get_location().y,
                        "z": actor.get_location().z
                    }
                })
            
            logger.info(f"获取到 {len(result)} 个 Actor")
            return result
            
        except Exception as e:
            logger.error(f"获取 Actor 列表失败: {e}")
            return []
    
    @mcp.tool()
    def get_current_map(ctx: Context) -> str:
        """
        获取当前加载的地图名称
        
        Returns:
            地图名称字符串
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                logger.error("未连接到仿真器")
                return ""
            
            map_name = conn.map.name
            logger.info(f"当前地图: {map_name}")
            return map_name
            
        except Exception as e:
            logger.error(f"获取地图名称失败: {e}")
            return ""
    
    @mcp.tool()
    def load_map(ctx: Context, map_name: str) -> Dict[str, Any]:
        """
        加载指定地图
        
        Args:
            map_name: 地图名称，如 "Town10" 或 "Town01"
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            logger.info(f"正在加载地图: {map_name}")
            
            # 清理当前资源
            conn.cleanup_all()
            
            # 加载新地图
            conn.client.load_world(map_name)
            
            # 更新世界和地图引用
            conn.world = conn.client.get_world()
            conn.map = conn.world.get_map()
            conn.blueprint_library = conn.world.get_blueprint_library()
            
            logger.info(f"地图 {map_name} 加载成功")
            return {"success": True, "map": map_name}
            
        except Exception as e:
            logger.error(f"加载地图失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def spawn_actor(
        ctx: Context,
        actor_type: str,
        location: List[float],
        rotation: List[float] = None
    ) -> Dict[str, Any]:
        """
        在场景中生成 Actor
        
        Args:
            actor_type: Actor 类型，如 "static.prop.bench"
            location: 位置 [x, y, z]
            rotation: 旋转 [pitch, yaw, roll]，默认 [0, 0, 0]
            
        Returns:
            生成结果，包含 Actor ID
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            rotation = rotation or [0, 0, 0]
            
            # 查找蓝图
            blueprint = conn.blueprint_library.find(actor_type)
            if not blueprint:
                return {"success": False, "error": f"未找到蓝图: {actor_type}"}
            
            # 创建变换
            transform = carla.Transform(
                carla.Location(x=location[0], y=location[1], z=location[2]),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])
            )
            
            # 生成 Actor
            actor = conn.world.spawn_actor(blueprint, transform)
            
            if actor:
                conn.track_actor(actor.id)
                logger.info(f"生成 Actor 成功: {actor.id} ({actor_type})")
                return {
                    "success": True,
                    "actor_id": actor.id,
                    "type": actor_type,
                    "location": location
                }
            else:
                return {"success": False, "error": "生成 Actor 失败"}
                
        except Exception as e:
            logger.error(f"生成 Actor 失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def delete_actor(ctx: Context, actor_id: int) -> Dict[str, Any]:
        """
        删除指定 Actor
        
        Args:
            actor_id: Actor ID
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            actor = conn.world.get_actor(actor_id)
            if not actor:
                return {"success": False, "error": f"未找到 Actor: {actor_id}"}
            
            actor.destroy()
            conn.untrack_actor(actor_id)
            
            logger.info(f"删除 Actor 成功: {actor_id}")
            return {"success": True, "actor_id": actor_id}
            
        except Exception as e:
            logger.error(f"删除 Actor 失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_actor_transform(
        ctx: Context,
        actor_id: int,
        location: List[float] = None,
        rotation: List[float] = None
    ) -> Dict[str, Any]:
        """
        设置 Actor 的位置和旋转
        
        Args:
            actor_id: Actor ID
            location: 新位置 [x, y, z]
            rotation: 新旋转 [pitch, yaw, roll]
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            actor = conn.world.get_actor(actor_id)
            if not actor:
                return {"success": False, "error": f"未找到 Actor: {actor_id}"}
            
            current_transform = actor.get_transform()
            
            # 更新位置
            if location:
                current_transform.location = carla.Location(
                    x=location[0], y=location[1], z=location[2]
                )
            
            # 更新旋转
            if rotation:
                current_transform.rotation = carla.Rotation(
                    pitch=rotation[0], yaw=rotation[1], roll=rotation[2]
                )
            
            actor.set_transform(current_transform)
            
            logger.info(f"设置 Actor {actor_id} 变换成功")
            return {
                "success": True,
                "actor_id": actor_id,
                "location": [
                    current_transform.location.x,
                    current_transform.location.y,
                    current_transform.location.z
                ],
                "rotation": [
                    current_transform.rotation.pitch,
                    current_transform.rotation.yaw,
                    current_transform.rotation.roll
                ]
            }
            
        except Exception as e:
            logger.error(f"设置 Actor 变换失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def get_actor_properties(ctx: Context, actor_id: int) -> Dict[str, Any]:
        """
        获取 Actor 的属性信息
        
        Args:
            actor_id: Actor ID
            
        Returns:
            Actor 属性字典
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            actor = conn.world.get_actor(actor_id)
            if not actor:
                return {"success": False, "error": f"未找到 Actor: {actor_id}"}
            
            transform = actor.get_transform()
            velocity = actor.get_velocity()
            
            result = {
                "success": True,
                "id": actor.id,
                "type_id": actor.type_id,
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
                "attributes": dict(actor.attributes)
            }
            
            logger.info(f"获取 Actor {actor_id} 属性成功")
            return result
            
        except Exception as e:
            logger.error(f"获取 Actor 属性失败: {e}")
            return {"success": False, "error": str(e)}
    
    logger.info("编辑器控制工具注册完成")
