"""
HUTB 仿真器连接管理模块

提供与 HUTB/CARLA 仿真器的连接管理功能，包括：
- 连接建立和断开
- 自动重连机制
- 资源追踪和清理
"""

import logging
from dataclasses import dataclass, field
from typing import List, Optional, Any, Dict

try:
    import carla
except ImportError:
    carla = None
    
from .config import get_config
from .utils.logger import get_logger

logger = get_logger("hutb_mcp.connection")


@dataclass
class HutbConnection:
    """
    HUTB 仿真器连接管理类
    
    负责与 HUTB/CARLA 仿真器的连接管理，包括连接建立、断开、重连，
    以及生成资源的追踪和清理。
    """
    host: str = "localhost"
    port: int = 2000
    timeout: float = 10.0
    
    # CARLA 客户端和世界对象
    client: Any = None
    world: Any = None
    map: Any = None
    blueprint_library: Any = None
    
    # 资源追踪列表
    spawned_vehicles: List[int] = field(default_factory=list)
    spawned_sensors: List[int] = field(default_factory=list)
    spawned_actors: List[int] = field(default_factory=list)
    
    # 连接状态
    _connected: bool = False
    
    def __post_init__(self):
        """初始化后处理"""
        if carla is None:
            logger.warning("CARLA 模块未安装，部分功能将不可用")
    
    def connect(self) -> bool:
        """
        建立与仿真器的连接
        
        Returns:
            连接是否成功
        """
        if carla is None:
            logger.error("CARLA 模块未安装，无法连接仿真器")
            return False
            
        logger.info(f"正在连接 HUTB 仿真器: {self.host}:{self.port}")
        
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
            
            # 获取世界和地图
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.blueprint_library = self.world.get_blueprint_library()
            
            self._connected = True
            logger.info(f"成功连接到 HUTB 仿真器，当前地图: {self.map.name}")
            return True
            
        except Exception as e:
            logger.error(f"连接 HUTB 仿真器失败: {e}")
            self._connected = False
            return False
    
    def disconnect(self) -> None:
        """断开与仿真器的连接并清理资源"""
        logger.info("正在断开 HUTB 仿真器连接...")
        
        try:
            # 清理所有生成的资源
            self.cleanup_all()
            
            # 重置连接状态
            self.client = None
            self.world = None
            self.map = None
            self.blueprint_library = None
            self._connected = False
            
            logger.info("已断开 HUTB 仿真器连接")
            
        except Exception as e:
            logger.error(f"断开连接时出错: {e}")
    
    def reconnect(self) -> bool:
        """
        重新连接仿真器
        
        Returns:
            重连是否成功
        """
        logger.info("正在重新连接 HUTB 仿真器...")
        self.disconnect()
        return self.connect()
    
    def is_connected(self) -> bool:
        """
        检查连接状态
        
        Returns:
            是否已连接
        """
        if not self._connected or self.client is None:
            return False
            
        try:
            # 尝试获取世界快照来验证连接
            if self.world:
                self.world.get_snapshot()
                return True
        except Exception:
            self._connected = False
            
        return False
    
    def cleanup_all(self) -> None:
        """清理所有生成的资源"""
        if not self.client or carla is None:
            return
            
        logger.info("正在清理所有生成的资源...")
        
        try:
            # 销毁传感器
            for sensor_id in self.spawned_sensors:
                try:
                    actor = self.world.get_actor(sensor_id)
                    if actor:
                        actor.destroy()
                except Exception as e:
                    logger.warning(f"销毁传感器 {sensor_id} 失败: {e}")
            
            # 销毁车辆
            for vehicle_id in self.spawned_vehicles:
                try:
                    actor = self.world.get_actor(vehicle_id)
                    if actor:
                        actor.destroy()
                except Exception as e:
                    logger.warning(f"销毁车辆 {vehicle_id} 失败: {e}")
            
            # 销毁其他 Actor
            for actor_id in self.spawned_actors:
                try:
                    actor = self.world.get_actor(actor_id)
                    if actor:
                        actor.destroy()
                except Exception as e:
                    logger.warning(f"销毁 Actor {actor_id} 失败: {e}")
            
            # 清空追踪列表
            self.spawned_sensors.clear()
            self.spawned_vehicles.clear()
            self.spawned_actors.clear()
            
            logger.info("资源清理完成")
            
        except Exception as e:
            logger.error(f"清理资源时出错: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取连接状态信息
        
        Returns:
            包含连接状态的字典
        """
        return {
            "connected": self.is_connected(),
            "host": self.host,
            "port": self.port,
            "map": self.map.name if self.map else None,
            "spawned_vehicles": len(self.spawned_vehicles),
            "spawned_sensors": len(self.spawned_sensors),
            "spawned_actors": len(self.spawned_actors),
        }
    
    def track_vehicle(self, vehicle_id: int) -> None:
        """追踪生成的车辆"""
        if vehicle_id not in self.spawned_vehicles:
            self.spawned_vehicles.append(vehicle_id)
    
    def track_sensor(self, sensor_id: int) -> None:
        """追踪生成的传感器"""
        if sensor_id not in self.spawned_sensors:
            self.spawned_sensors.append(sensor_id)
    
    def track_actor(self, actor_id: int) -> None:
        """追踪生成的 Actor"""
        if actor_id not in self.spawned_actors:
            self.spawned_actors.append(actor_id)
    
    def untrack_vehicle(self, vehicle_id: int) -> None:
        """取消追踪车辆"""
        if vehicle_id in self.spawned_vehicles:
            self.spawned_vehicles.remove(vehicle_id)
    
    def untrack_sensor(self, sensor_id: int) -> None:
        """取消追踪传感器"""
        if sensor_id in self.spawned_sensors:
            self.spawned_sensors.remove(sensor_id)
    
    def untrack_actor(self, actor_id: int) -> None:
        """取消追踪 Actor"""
        if actor_id in self.spawned_actors:
            self.spawned_actors.remove(actor_id)


# 全局连接实例
_connection: Optional[HutbConnection] = None


def get_connection() -> HutbConnection:
    """
    获取全局连接实例
    
    Returns:
        HutbConnection 实例
    """
    global _connection
    
    if _connection is None:
        cfg = get_config()
        _connection = HutbConnection(
            host=cfg.hutb.host,
            port=cfg.hutb.port,
            timeout=cfg.hutb.timeout
        )
    
    # 如果未连接，尝试连接
    if not _connection.is_connected():
        _connection.connect()
    
    return _connection


def reset_connection() -> None:
    """重置全局连接"""
    global _connection
    if _connection:
        _connection.disconnect()
    _connection = None
