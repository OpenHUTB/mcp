#!/usr/bin/env python3
"""
FastMCP HUTB Assistant - 使用FastMCP框架的模拟器接口
集成Deepseek AI模型，支持自然语言使用 HUTB 模拟器
使用 FastMCP 装饰器方式实现 MCP 工具调用机制
"""

import socket
import sys
import json
import re
from pathlib import Path
from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse
import uvicorn
import aiohttp
from typing import Optional
import carla
import subprocess
import os
import xml.etree.ElementTree as ET
import random
import math
from pathlib import Path
from fastapi.responses import FileResponse
# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir/ "src"))

from fastmcp import FastMCP
from src.github_client import GitHubClient
from src.config import config               
from src.utils.logger import app_logger

# 创建FastMCP实例
mcp = FastMCP("AI智能助手")


class CarlaClient:
    """CARLA客户端封装类"""

    def __init__(self):
        self.client = None
        self.world = None
        self.actors = []
        self.tick_task = None
        self.is_ticking = False
        # 视频录制相关
        self.is_recording = False
        self.recording_task = None
        self.recording_output_path = None
        self.recording_fps = 30
        self.recording_frame_count = 0
        self.video_writer = None
        self.camera_sensor = None
        self.image_queue = None
        # 视角控制相关
        self.current_view_mode = "spectator"  # spectator, third_person, first_person, overhead, bystander
        self.view_target = None  # 当前视角跟随的目标
        self.view_follow_task = None  # 视角跟随任务
        self.is_view_following = False  # 是否正在跟随视角
        # 修复3+8: 行人状态管理
        self.walker_controllers = {}   # {walker_id: controller_actor}
        self.walker_goals = {}         # {walker_id: {'last_loc':..., 'stuck':0, 'target':...}}
        self.walker_check_interval = 0

    async def connect(self, host='localhost', port=2000):
        """连接CARLA服务器"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10)
            self.world = self.client.get_world()
            if self.world is None:
                app_logger.error("❌ CARLA返回了空的world对象")
                return False
            app_logger.info("✅ CARLA服务器连接成功")
            return True
        except Exception as e:
            app_logger.error(f"❌ 连接CARLA失败: {str(e)}")
            return False
        
    async def load_world(self, map_name='Town05'):
        """加载指定地图"""
        try:
            if self.client is None:
                app_logger.error("❌ 未连接到CARLA服务器")
                return False
            # 已是目标地图则跳过（重复加载同一地图也会重建世界，耗时且可能超时）
            current = self.world.get_map().name if self.world is not None else ''
            if map_name in current:
                app_logger.info(f"✅ 地图已是 {map_name}，无需重复加载")
                return True
            # 大地图加载常超过10s，临时调高超时时间
            self.client.set_timeout(60)
            self.world = self.client.load_world(map_name)
            self.client.set_timeout(10)
            app_logger.info(f"✅ 地图加载成功: {map_name}")
            return True
        except Exception as e:
            app_logger.error(f"❌ 加载地图失败: {str(e)}")
            return False

    async def set_synchronous_mode(self, enabled=True, fixed_delta_seconds=0.02):
        """设置同步模式 - 参考tuto_G_pedestrian_navigation.py"""
        try:
            if self.world is None:
                app_logger.error("❌ 未连接到CARLA服务器")
                return False
            
            settings = self.world.get_settings()
            settings.synchronous_mode = enabled
            if enabled:
                settings.fixed_delta_seconds = fixed_delta_seconds
            else:
                settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            
            if enabled:
                app_logger.info(f"✅ 同步模式已启用，固定时间步长: {fixed_delta_seconds}s")
            else:
                app_logger.info("✅ 同步模式已禁用")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置同步模式失败: {str(e)}")
            return False

    async def start_tick_loop(self):
        """启动后台tick循环，确保世界持续运行，并同步更新视角"""
        import asyncio
        if self.is_ticking:
            app_logger.info("⚠️ tick循环已在运行")
            return
        
        # 先启用同步模式
        await self.set_synchronous_mode(True, 0.05)
        
        self.is_ticking = True
        app_logger.info("🔄 启动后台tick循环")
        
        async def tick_loop():
            while self.is_ticking and self.world:
                try:
                    self.world.tick()
                    self.check_and_fix_stuck_walkers()
                    
                    # 🔑 关键修复：在每次tick后立即更新视角跟随
                    if self.is_view_following and self.view_target and self.view_target.is_alive:
                        if self.current_view_mode == "third_person":
                            self._update_third_person_view(self.view_target)
                        elif self.current_view_mode == "first_person":
                            self._update_first_person_view(self.view_target)
                        elif self.current_view_mode == "overhead":
                            self._update_overhead_view(self.view_target)
                    
                    await asyncio.sleep(0.05)
                except Exception as e:
                    app_logger.warning(f"⚠️ tick时出错: {e}")
                    await asyncio.sleep(0.1)
        
        self.tick_task = asyncio.create_task(tick_loop())

    async def stop_tick_loop(self):
        """停止后台tick循环"""
        self.is_ticking = False
        if self.tick_task:
            self.tick_task.cancel()
            try:
                await self.tick_task
            except asyncio.CancelledError:
                pass
            self.tick_task = None
        
        # 禁用同步模式
        await self.set_synchronous_mode(False)
        
        app_logger.info("🛑 停止后台tick循环")

    async def spawn_vehicles(self, vehicle_type='model3', count=1, autopilot=False):
        """生成多辆车辆，确保在车道内，返回详细ID列表"""
        count = int(count)  # ← 新增：强制转int，防止LLM传字符串
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        vehicle_blueprints = {
            'model3': 'vehicle.tesla.model3', 'a2': 'vehicle.audi.a2',
            'etron': 'vehicle.audi.etron', 'tt': 'vehicle.audi.tt',
            'grandtourer': 'vehicle.bmw.grandtourer', 'i8': 'vehicle.bmw.i8',
            'mini': 'vehicle.bmw.mini', 'impala': 'vehicle.chevrolet.impala',
            'c3': 'vehicle.citroen.c3', 'charger_police': 'vehicle.dodge.charger_police',
            'charger2020': 'vehicle.dodge.charger2020', 'mustang': 'vehicle.ford.mustang',
            'crown': 'vehicle.ford.crown', 'wrangler_rubicon': 'vehicle.jeep.wrangler_rubicon',
            'mkz_2017': 'vehicle.lincoln.mkz_2017', 'mkz_2020': 'vehicle.lincoln.mkz_2020',
            'benz_coupe': 'vehicle.mercedes.benz_coupe', 'cabrio': 'vehicle.mercedes.cabrio',
            'ccc': 'vehicle.mercedes.ccc', 'cooper_s': 'vehicle.mini.cooper_s',
            'micra': 'vehicle.nissan.micra', 'patrol': 'vehicle.nissan.patrol',
            'leon': 'vehicle.seat.leon', 't2': 'vehicle.volkswagen.t2',
            't3': 'vehicle.volkswagen.t3',
        }
        
        blueprint_path = vehicle_blueprints.get(vehicle_type.lower(), f'vehicle.tesla.{vehicle_type}')
        
        # 获取有效生成点
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        if not valid_points:
            app_logger.warning("⚠️ 没有可用的车辆生成点")
            return []
        
        blueprint_library = self.world.get_blueprint_library()
        spawned_vehicles = []
        
        # 遍历所有可用点，直到生成够 count 辆
        for i, transform in enumerate(valid_points):
            if len(spawned_vehicles) >= count:
                break
            
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if bp.id.startswith('vehicle.')]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    continue
                
                if blueprint.has_attribute('color'):
                    r, g, b = random.randint(0,255), random.randint(0,255), random.randint(0,255)
                    blueprint.set_attribute('color', f"{r},{g},{b}")
                
                if blueprint.has_attribute('role_name'):
                    blueprint.set_attribute('role_name', 'autopilot')
                
                # 小偏移避免位置被占（±0.5米，自动驾驶启动后会自动修正到车道中心）
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_loc = carla.Location(
                    x=transform.location.x + offset_x,
                    y=transform.location.y + offset_y,
                    z=transform.location.z + 0.5
                )
                new_transform = carla.Transform(new_loc, transform.rotation)
                vehicle = self.world.try_spawn_actor(blueprint, new_transform)
                
                if vehicle:
                    if autopilot:
                        vehicle.set_autopilot(True)
                    self.actors.append(vehicle)
                    spawned_vehicles.append(vehicle)
                    app_logger.info(f"🚗 [第{len(spawned_vehicles)}辆] ID={vehicle.id} | {blueprint.id} | 位置=({transform.location.x:.1f}, {transform.location.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 位置 ({transform.location.x:.1f}, {transform.location.y:.1f}) 被占用，尝试下一个点")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成出错: {e}")
                continue
        
        # 启动后台tick循环（视角跟随和自动驾驶都依赖它）
        if spawned_vehicles and not self.is_ticking:
            await self.start_tick_loop()
            app_logger.info("🔄 已启动后台tick循环")
        
        app_logger.info(f"✅ 共生成 {len(spawned_vehicles)} 辆车，ID列表: {[v.id for v in spawned_vehicles]}")
        return spawned_vehicles

    async def spawn_bicycles(self, bicycle_type='crossbike', count=1):
        count = int(count)  # ← 新增
        """生成多辆自行车，返回详细ID列表
        
        支持的自行车类型:
        - crossbike: BH Crossbike
        - century: Diamondback Century
        - omafiets: Gazelle Omafiets
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        bicycle_blueprints = {
            'crossbike': 'vehicle.bh.crossbike',
            'century': 'vehicle.diamondback.century',
            'omafiets': 'vehicle.gazelle.omafiets',
        }
        
        blueprint_path = bicycle_blueprints.get(bicycle_type.lower(), f'vehicle.bh.{bicycle_type}')
        
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        if not valid_points:
            app_logger.warning("⚠️ 没有可用的自行车生成点")
            return []
        
        blueprint_library = self.world.get_blueprint_library()
        spawned_bicycles = []
        
        for i, transform in enumerate(valid_points):
            if len(spawned_bicycles) >= count:
                break
            
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    # 回退到任意自行车蓝图
                    blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                                  if 'crossbike' in bp.id or 'diamondback' in bp.id or 'gazelle' in bp.id or 'bike' in bp.id]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    continue
                
                # 自行车通常没有颜色属性，跳过颜色设置
                # 小偏移避免位置被占
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_loc = carla.Location(
                    x=transform.location.x + offset_x,
                    y=transform.location.y + offset_y,
                    z=transform.location.z + 0.5
                )
                new_transform = carla.Transform(new_loc, transform.rotation)
                bicycle = self.world.try_spawn_actor(blueprint, new_transform)
                
                if bicycle:
                    self.actors.append(bicycle)
                    spawned_bicycles.append(bicycle)
                    app_logger.info(f"🚲 [第{len(spawned_bicycles)}辆] ID={bicycle.id} | {blueprint.id} | 位置=({transform.location.x:.1f}, {transform.location.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 位置 ({transform.location.x:.1f}, {transform.location.y:.1f}) 被占用，尝试下一个点")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成自行车出错: {e}")
                continue
        
        # 启动后台tick循环
        if spawned_bicycles and not self.is_ticking:
            await self.start_tick_loop()
            app_logger.info("🔄 已启动后台tick循环")
        
        app_logger.info(f"✅ 共生成 {len(spawned_bicycles)} 辆自行车，ID列表: {[b.id for b in spawned_bicycles]}")
        return spawned_bicycles     

    async def spawn_motorcycles(self, motorcycle_type='ninja', count=1):
        count = int(count)  # ← 新增
        """生成多辆摩托车，返回详细ID列表
        
        支持的摩托车类型:
        - ninja: Kawasaki Ninja
        - yzf: Yamaha YZF
        - low_rider: Harley-Davidson Low Rider
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        motorcycle_blueprints = {
            'ninja': 'vehicle.kawasaki.ninja',
            'yzf': 'vehicle.yamaha.yzf',
            'low_rider': 'vehicle.harley-davidson.low_rider',
        }
        
        blueprint_path = motorcycle_blueprints.get(motorcycle_type.lower(), f'vehicle.kawasaki.{motorcycle_type}')
        
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        if not valid_points:
            app_logger.warning("⚠️ 没有可用的摩托车生成点")
            return []
        
        blueprint_library = self.world.get_blueprint_library()
        spawned_motorcycles = []
        
        for i, transform in enumerate(valid_points):
            if len(spawned_motorcycles) >= count:
                break
            
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                                  if 'ninja' in bp.id or 'yzf' in bp.id or 'low_rider' in bp.id or 'harley' in bp.id]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    continue
                
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_loc = carla.Location(
                    x=transform.location.x + offset_x,
                    y=transform.location.y + offset_y,
                    z=transform.location.z + 0.5
                )
                new_transform = carla.Transform(new_loc, transform.rotation)
                motorcycle = self.world.try_spawn_actor(blueprint, new_transform)
                
                if motorcycle:
                    self.actors.append(motorcycle)
                    spawned_motorcycles.append(motorcycle)
                    app_logger.info(f"🏍️ [第{len(spawned_motorcycles)}辆] ID={motorcycle.id} | {blueprint.id} | 位置=({transform.location.x:.1f}, {transform.location.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 位置 ({transform.location.x:.1f}, {transform.location.y:.1f}) 被占用，尝试下一个点")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成摩托车出错: {e}")
                continue
        
        if spawned_motorcycles and not self.is_ticking:
            await self.start_tick_loop()
            app_logger.info("🔄 已启动后台tick循环")
        
        app_logger.info(f"✅ 共生成 {len(spawned_motorcycles)} 辆摩托车，ID列表: {[m.id for m in spawned_motorcycles]}")
        return spawned_motorcycles

    async def spawn_props(self, prop_type='cone', count=1, location=None):
        count = int(count)  # ← 新增
        """生成静态道具（施工警示牌、三角警示牌、路障等）
        
        支持的道具类型:
        - cone: 施工锥/路锥
        - barrier: 路障/护栏
        - warning: 三角警示牌/交通警示牌
        - construction: 施工警示牌（如果有）
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        prop_blueprints = {
            'cone': 'static.prop.constructioncone',
            'barrier': 'static.prop.barrier',
            'warning': 'static.prop.trafficwarning',
            'construction': 'static.prop.constructioncone',  # 回退到施工锥
        }
        
        blueprint_path = prop_blueprints.get(prop_type.lower(), f'static.prop.{prop_type}')
        blueprint_library = self.world.get_blueprint_library()
        spawned_props = []
        
        for i in range(count):
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    # 回退到任意静态道具
                    blueprints = [bp for bp in blueprint_library.filter('static.prop.*') 
                                  if 'cone' in bp.id or 'barrier' in bp.id or 'warning' in bp.id]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    app_logger.warning(f"⚠️ 找不到道具蓝图: {blueprint_path}")
                    continue
                
                # 如果指定了位置，使用指定位置；否则在车辆附近随机放置
                if location:
                    spawn_loc = carla.Location(
                        x=location.x + random.uniform(-2.0, 2.0),
                        y=location.y + random.uniform(-2.0, 2.0),
                        z=location.z + 0.1
                    )
                else:
                    # 默认在地图原点附近
                    spawn_loc = carla.Location(
                        x=random.uniform(-50, 50),
                        y=random.uniform(-50, 50),
                        z=0.1
                    )
                
                spawn_transform = carla.Transform(spawn_loc, carla.Rotation())
                prop = self.world.try_spawn_actor(blueprint, spawn_transform)
                
                if prop:
                    self.actors.append(prop)
                    spawned_props.append(prop)
                    app_logger.info(f"🚧 [第{len(spawned_props)}个] ID={prop.id} | {blueprint.id} | 位置=({spawn_loc.x:.1f}, {spawn_loc.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 道具生成失败，位置可能被占用")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成道具出错: {e}")
                continue
        
        app_logger.info(f"✅ 共生成 {len(spawned_props)} 个道具，ID列表: {[p.id for p in spawned_props]}")
        return spawned_props

    async def spawn_overturned_vehicle(self, vehicle_type='model3', location=None):
        """生成仰翻的车辆（多重容错：遍历所有点 → 随机位置 → 手动空位）"""
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return None
        
        try:
            vehicle_type = str(vehicle_type).lower()
            
            vehicle_blueprints = {
                'model3': 'vehicle.tesla.model3',
                'mustang': 'vehicle.ford.mustang',
                'a2': 'vehicle.audi.a2',
                'charger_police': 'vehicle.dodge.charger_police',
            }
            
            blueprint_path = vehicle_blueprints.get(vehicle_type, 'vehicle.tesla.model3')
            blueprint_library = self.world.get_blueprint_library()
            blueprint = blueprint_library.find(blueprint_path)
            
            if blueprint is None:
                app_logger.error(f"❌ 找不到蓝图: {blueprint_path}")
                return None
            
            # 收集所有可能的生成位置
            spawn_locations = []
            
            # 1. 使用传入的位置
            if location:
                spawn_locations.append(location)
            
            # 2. 使用地图车辆生成点
            map_spawn_points = self.world.get_map().get_spawn_points()
            if map_spawn_points:
                random.shuffle(map_spawn_points)
                for sp in map_spawn_points[:20]:  # 取前20个随机点
                    spawn_locations.append(sp.location)
            
            # 3. 使用导航随机位置
            for _ in range(10):
                nav_loc = self.world.get_random_location_from_navigation()
                if nav_loc:
                    spawn_locations.append(nav_loc)
            
            # 4. 手动指定几个空位（地图中心附近）
            for offset in [(0,0), (10,0), (-10,0), (0,10), (0,-10), (20,20), (-20,-20)]:
                spawn_locations.append(carla.Location(x=offset[0], y=offset[1], z=0.5))
            
            if not spawn_locations:
                app_logger.error("❌ 没有任何可用生成位置")
                return None
            
            # 目标翻转姿态
            overturned_rotation = carla.Rotation(pitch=0, yaw=random.uniform(0, 360), roll=180)
            
            # 遍历所有位置尝试生成
            for idx, spawn_loc in enumerate(spawn_locations):
                try:
                    # 确保z轴有足够高度
                    spawn_loc.z = max(spawn_loc.z, 0.5)
                    
                    # 方案A：直接仰翻 spawn
                    spawn_transform = carla.Transform(spawn_loc, overturned_rotation)
                    vehicle = self.world.try_spawn_actor(blueprint, spawn_transform)
                    
                    if vehicle:
                        vehicle.set_simulate_physics(False)
                        self.actors.append(vehicle)
                        app_logger.info(f"🚗💥 仰翻车辆生成成功: {vehicle_type} (ID: {vehicle.id}, 尝试{idx+1}次)")
                        return vehicle
                    
                    # 方案B：正常 spawn 后翻转
                    normal_transform = carla.Transform(spawn_loc, carla.Rotation())
                    vehicle = self.world.try_spawn_actor(blueprint, normal_transform)
                    
                    if vehicle:
                        vehicle.set_simulate_physics(False)
                        final_loc = vehicle.get_location()
                        vehicle.set_transform(carla.Transform(final_loc, overturned_rotation))
                        self.actors.append(vehicle)
                        app_logger.info(f"🚗💥 仰翻车辆生成成功(翻转): {vehicle_type} (ID: {vehicle.id}, 尝试{idx+1}次)")
                        return vehicle
                        
                except Exception as e:
                    app_logger.warning(f"⚠️ 位置{idx+1} ({spawn_loc.x:.1f}, {spawn_loc.y:.1f}) 失败: {e}")
                    continue
            
            app_logger.error(f"❌ 仰翻车辆生成彻底失败，已尝试{len(spawn_locations)}个位置")
            return None
            
        except Exception as e:
            app_logger.error(f"❌ 生成仰翻车辆异常: {e}")
            import traceback
            app_logger.error(traceback.format_exc())
            return None

    async def spawn_vehicle(self, vehicle_type='model3'):
        """生成单辆车辆（兼容旧接口）"""
        vehicles = await self.spawn_vehicles(vehicle_type, count=1)
        return vehicles[0] if vehicles else None

    async def set_weather(self, weather_type='clear'):
        """设置天气"""
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return False
        
        weather_presets = {
            'clear': carla.WeatherParameters(
                cloudiness=0, precipitation=0, precipitation_deposits=0,
                wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=75,
                fog_density=0, fog_distance=0, wetness=0
            ),
            'rain': carla.WeatherParameters(
                cloudiness=100, precipitation=80, precipitation_deposits=50,
                wind_intensity=30, sun_azimuth_angle=0, sun_altitude_angle=15,
                fog_density=10, fog_distance=100, wetness=60
            ),
            'fog': carla.WeatherParameters(
                cloudiness=80, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=30,
                fog_density=90, fog_distance=50, wetness=20
            ),
            'light_fog': carla.WeatherParameters(  # ← 新增：薄雾
                cloudiness=60, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=45,
                fog_density=25, fog_distance=80, wetness=10, fog_falloff=2.0
            ),
            'snow': carla.WeatherParameters(
                cloudiness=80, precipitation=60, precipitation_deposits=80,
                wind_intensity=20, sun_azimuth_angle=0, sun_altitude_angle=10,
                fog_density=20, fog_distance=200, wetness=30
            ),
            'night': carla.WeatherParameters(
                cloudiness=20, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
                fog_density=0, fog_distance=0, wetness=0
            ),
        }
        if weather_type in weather_presets:
            self.world.set_weather(weather_presets[weather_type])
            app_logger.info(f"✅ 天气已设置为 {weather_type}")
            return True
        return False

    async def get_traffic_lights(self):
        """获取交通灯状态"""
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return []
        
        lights = [light for light in self.world.get_actors() if 'traffic_light' in light.type_id]
        return lights[:5]  # 只返回前5个

    async def spawn_pedestrians(self, pedestrian_type='pedestrian', count=1, speed=None):
        """生成多个行人，返回生成的行人列表和最后一个行人
        
        支持的行人类型:
        - pedestrian: 普通行人
        - elderly: 老年人
        - child: 儿童
        - police: 警察
        - business: 商务人士
        - jogger: 慢跑者
        
        参数:
        - speed: 行人移动速度（m/s），默认为1.4（正常步行速度），慢跑者默认为2.8
        
        数据量支持: 取决于地图大小，通常支持10-100+个行人
        """
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return []
        
        try:
            # 行人类型到蓝图编号的映射
            pedestrian_blueprint_map = {
                'police': ['0030', '0032'],
                'child': ['0009', '0010', '0011', '0012', '0013', '0014', '0048', '0049'],
                'elderly': ['0020', '0021', '0022', '0023', '0024', '0025'],
                'business': ['0027', '0028', '0029'],
                'pedestrian': ['0001', '0002', '0003', '0004', '0005', '0006', '0007', '0008', 
                               '0015', '0016', '0017', '0018', '0019', '0026', '0031', '0033', 
                               '0034', '0035', '0036', '0037', '0038', '0039', '0040', '0041', 
                               '0042', '0043', '0044', '0045', '0046', '0047'],
                'jogger': ['0001', '0002', '0003', '0004', '0005', '0006', '0007', '0008', 
                           '0015', '0016', '0017', '0018', '0019', '0026', '0031', '0033', 
                           '0034', '0035', '0036', '0037', '0038', '0039', '0040', '0041', 
                           '0042', '0043', '0044', '0045', '0046', '0047']
            }
            
            # 根据行人类型设置默认速度
            if speed is None:
                if pedestrian_type == 'jogger':
                    speed = 2.8  # 慢跑者默认速度
                elif pedestrian_type == 'elderly':
                    speed = 1.0  # 老年人默认速度较慢
                else:
                    speed = 1.4  # 正常步行速度
            
            # 获取当前行人类型对应的蓝图编号列表
            blueprint_numbers = pedestrian_blueprint_map.get(pedestrian_type, pedestrian_blueprint_map['pedestrian'])
            
            # 检查地图是否支持行人导航
            spawn_location = self.world.get_random_location_from_navigation()
            if spawn_location:
                app_logger.info(f"✅ 地图支持行人导航，测试位置: {spawn_location}")
            else:
                app_logger.warning("⚠️ 警告: 地图可能不支持行人导航，get_random_location_from_navigation()返回None")
                app_logger.warning("⚠️ 建议: 尝试加载Town05地图（client.load_world('Town05')）")
            
            # 获取控制器蓝图
            controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            if not controller_bp:
                app_logger.error("❌ 无法找到行人控制器蓝图")
                return []
            
            spawned_pedestrians = []
            
            for i in range(count):
                try:
                    # 从指定类型的蓝图编号中随机选择一个
                    blueprint_number = random.choice(blueprint_numbers)
                    blueprint_id = f'walker.pedestrian.{blueprint_number}'
                    
                    # 查找指定的行人蓝图
                    blueprint_library = self.world.get_blueprint_library()
                    pedestrian_bp = blueprint_library.find(blueprint_id)
                    
                    if not pedestrian_bp:
                        app_logger.error(f"❌ 无法找到行人蓝图: {blueprint_id}")
                        continue
                    
                    app_logger.info(f"📋 尝试生成行人，蓝图: {pedestrian_bp.id}")
                    
                    # 如果是老年人，随机设置轮椅
                    if pedestrian_type == 'elderly':
                        if pedestrian_bp.has_attribute('can_use_wheelchair'):
                            if random.random() < 0.3:  # 30%的概率使用轮椅
                                pedestrian_bp.set_attribute('use_wheelchair', 'True')
                                app_logger.info(f"♿ 为老年人设置轮椅")
                    
                    # 尝试多个位置生成行人
                    spawn_success = False
                    for attempt in range(3):  # 尝试3次
                        try:
                            # 随机生成位置
                            spawn_location = self.world.get_random_location_from_navigation()
                            if not spawn_location:
                                # 如果无法获取随机位置，使用默认位置
                                spawn_location = carla.Location(x=-134 + i*2, y=78.1, z=1.18)
                            
                            spawn_transform = carla.Transform(spawn_location)
                            app_logger.info(f"📍 尝试在位置生成: {spawn_location}")
                            
                            # 生成行人
                            pedestrian = self.world.try_spawn_actor(pedestrian_bp, spawn_transform)
                            if pedestrian:
                                app_logger.info(f"✅ 行人生成成功: {pedestrian.id}")
                                
                                # 为行人设置AI控制器 - 参考tuto_G_pedestrian_navigation.py
                                try:
                                    # 使用行人的变换作为控制器的生成位置
                                    controller = self.world.spawn_actor(controller_bp, pedestrian.get_transform(), pedestrian)
                                    if controller:
                                        app_logger.info(f"✅ 控制器生成成功: {controller.id}")
                                        # 启动控制器并给它一个随机位置
                                        controller.start()
                                        target_location = self.world.get_random_location_from_navigation()
                                        if target_location is None:
                                            # 如果导航网格返回None，用行人当前位置附近
                                            current_loc = pedestrian.get_location()
                                            target_location = carla.Location(
                                                x=current_loc.x + random.uniform(-20, 20),
                                                y=current_loc.y + random.uniform(-20, 20),
                                                z=current_loc.z
                                            )
                                        controller.go_to_location(target_location)
                                        controller.set_max_speed(speed)
                                        app_logger.info(f"🎯 为行人设置随机目标位置，速度: {speed} m/s")
                                        
                                        # 修复3+8: 保存控制器引用
                                        self.walker_controllers[pedestrian.id] = controller
                                        self.walker_goals[pedestrian.id] = {
                                            'last_location': pedestrian.get_location(),
                                            'stuck_count': 0,
                                            'target': target_location
                                        }

                                        # 存储控制器和行人的关联关系
                                        self.actors.append(pedestrian)
                                        self.actors.append(controller)
                                        spawned_pedestrians.append(pedestrian)
                                        app_logger.info(f"🚶 生成第{i+1}个行人: {pedestrian_bp.id} (ID: {pedestrian.id})")
                                        
                                        # 将世界移动几帧，让行人生成 - 参考tuto_G_pedestrian_navigation.py
                                        for frame in range(0, 5):
                                            try:
                                                self.world.tick()
                                            except Exception as tick_error:
                                                app_logger.warning(f"⚠️ 推进世界时出错: {tick_error}")
                                                continue
                                        
                                        spawn_success = True
                                        break
                                    else:
                                        # 如果控制器生成失败，销毁行人
                                        if pedestrian.is_alive:
                                            pedestrian.destroy()
                                        app_logger.error(f"❌ 为第{i+1}个行人创建控制器失败")
                                except Exception as ctrl_error:
                                    # 如果控制器生成失败，销毁行人
                                    if pedestrian.is_alive:
                                        pedestrian.destroy()
                                    app_logger.error(f"❌ 控制器生成异常: {ctrl_error}")
                            else:
                                app_logger.warning(f"⚠️ 尝试 {attempt+1}/3: 生成行人失败，位置可能被占用")
                                
                        except Exception as loc_error:
                            app_logger.error(f"❌ 位置生成时出错: {loc_error}")
                            continue
                    
                    if not spawn_success:
                        app_logger.error(f"❌ 生成第{i+1}个行人失败，已尝试3个位置")
                        
                except Exception as e:
                    app_logger.error(f"❌ 生成第{i+1}个行人时出错: {str(e)}")
                    continue
            
            app_logger.info(f"✅ 共生成{len(spawned_pedestrians)}个行人")
            
            # 启动后台tick循环，确保行人持续移动
            if spawned_pedestrians and not self.is_ticking:
                await self.start_tick_loop()
                app_logger.info("🔄 已启动后台tick循环，行人将开始移动")
            
            # 再次确保所有行人控制器都有目标位置
            if spawned_pedestrians:
                import asyncio
                await asyncio.sleep(0.5)  # 等待一小段时间让控制器初始化
                for pedestrian in spawned_pedestrians:
                    try:
                        # 获取行人的控制器
                        controller = pedestrian.get_control()
                        if controller:
                            # 重新设置随机目标位置
                            target_location = self.world.get_random_location_from_navigation()
                            if target_location:
                                # 通过walker的controller来设置目标
                                walker_controller = None
                                for actor in self.world.get_actors():
                                    if 'controller.ai.walker' in actor.type_id:
                                        # 检查这个控制器是否附着到当前行人
                                        try:
                                            if hasattr(actor, 'parent') and actor.parent == pedestrian:
                                                walker_controller = actor
                                                break
                                        except:
                                            pass
                                
                                if walker_controller:
                                    walker_controller.go_to_location(target_location)
                                    walker_controller.set_max_speed(speed)
                                    app_logger.info(f"🚶 为行人 {pedestrian.id} 重新设置目标位置，速度: {speed} m/s")
                    except Exception as e:
                        app_logger.warning(f"⚠️ 为行人设置目标时出错: {e}")
                        continue
            
            return spawned_pedestrians
            
        except Exception as e:
            app_logger.error(f"❌ 生成行人失败: {str(e)}")
            return []

    async def spawn_pedestrian(self, pedestrian_type='walker', speed=None):
        """生成单个行人（兼容旧接口）"""
        pedestrians = await self.spawn_pedestrians(pedestrian_type, count=1, speed=speed)
        return pedestrians[0] if pedestrians else None

    def set_spectator_view(self, target_actor):
        """将视角对准目标actor"""
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False
        
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            
            # 设置相机位置在目标actor前方5米，上方2米处
            # 这样可以从正面看到行人
            camera_location = carla.Location(
                x=target_transform.location.x + 5.0,  # 前方5米
                y=target_transform.location.y,
                z=target_transform.location.z + 2.0
            )
            
            # 计算相机朝向，指向行人
            # yaw=180.0 让相机朝向行人方向
            camera_rotation = carla.Rotation(
                pitch=-15.0,  # 略微向下看
                yaw=180.0,    # 朝向行人
                roll=0.0
            )
            
            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  视角已对准actor {target_actor.id}")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置视角失败: {str(e)}")
            return False

    async def setup_autopilot(self, enable=True, radius=0.0):
        """设置车辆自动驾驶模式"""
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return False
        
        try:
            vehicles = [actor for actor in self.world.get_actors() if 'vehicle' in actor.type_id]
            
            enabled_count = 0
            for vehicle in vehicles:
                if radius > 0:
                    spectator = self.world.get_spectator()
                    if spectator.get_location().distance(vehicle.get_location()) > radius:
                        continue
                
                # 直接启用自动驾驶，CARLA 会自动走默认 Traffic Manager
                vehicle.set_autopilot(enable)
                enabled_count += 1
                app_logger.info(f"🚗 车辆 {vehicle.id} 自动驾驶已{'启用' if enable else '禁用'}")
            
            app_logger.info(f"✅ {'启用' if enable else '禁用'}了 {enabled_count} 辆车的自动驾驶")
            return True
            
        except Exception as e:
            app_logger.error(f"❌ 设置自动驾驶失败: {str(e)}")
            return False

    async def setup_pedestrian_movement(self, enable=True, radius=0.0):
        """设置行人自动移动
        
        Args:
            enable: 是否启用行人移动
            radius: 移动范围半径（米），0表示全图
        """
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return False
        
        try:
            # 获取所有行人控制器 - 参考官方文档
            controllers = [actor for actor in self.world.get_actors() if 'controller.ai.walker' in actor.type_id]
            
            updated_count = 0
            for controller in controllers:
                try:
                    if enable:
                        # 启用控制器并设置随机目标位置
                        controller.start()
                        target_location = self.world.get_random_location_from_navigation()
                        if target_location:
                            controller.go_to_location(target_location)
                            app_logger.info(f"🚶 行人控制器 {controller.id} 已启用并设置目标: {target_location}")
                            updated_count += 1
                        else:
                            app_logger.warning(f"⚠️ 无法获取随机目标位置")
                    else:
                        # 禁用控制器
                        controller.stop()
                        app_logger.info(f"🚶 行人控制器 {controller.id} 已禁用")
                        updated_count += 1
                except Exception as ctrl_error:
                    app_logger.error(f"❌ 操作控制器 {controller.id} 时出错: {ctrl_error}")
                    continue
            
            app_logger.info(f"✅ {'启用' if enable else '禁用'}了 {updated_count} 个行人控制器")
            return True
            
        except Exception as e:
            app_logger.error(f"❌ 设置行人移动失败: {str(e)}")
            return False

     # ============ 修复6: 获取车道内有效生成点 ============
    def get_valid_vehicle_spawn_points(self, safe_mode=True):
        """获取有效的车辆生成点（仅在Driving车道内）"""
        carla_map = self.world.get_map()
        all_spawn_points = carla_map.get_spawn_points()
        valid_points = []
        for sp in all_spawn_points:
            waypoint = carla_map.get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            if waypoint is None or waypoint.lane_type != carla.LaneType.Driving:
                continue
            if safe_mode:
                nearby = [a for a in self.world.get_actors().filter("vehicle.*")
                          if a.get_location().distance(sp.location) < 5.0]
                if nearby:
                    continue
            valid_points.append(waypoint.transform)
        return valid_points

     # ============ 修复7: 批量生成车辆并返回详细ID ============
    async def batch_spawn_vehicles_with_id(self, count=10, blueprint_filter="vehicle.*", autopilot=True):
        """批量生成车辆，返回带ID的详细列表"""
        result = {"total": count, "success": 0, "failed": 0, "vehicles": []}
        blueprints = [bp for bp in self.world.get_blueprint_library().filter(blueprint_filter)
                      if bp.id.startswith("vehicle.")]
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        
        for i in range(min(count, len(valid_points))):
            blueprint = random.choice(blueprints)
            color = "default"
            if blueprint.has_attribute("color"):
                color = f"{random.randint(0,255)},{random.randint(0,255)},{random.randint(0,255)}"
                blueprint.set_attribute("color", color)
            transform = valid_points[i]
            try:
                vehicle = self.world.try_spawn_actor(blueprint, transform)
                if vehicle:
                    if autopilot:
                        vehicle.set_autopilot(True)
                    self.actors.append(vehicle)
                    info = {
                        "index": i + 1,
                        "id": vehicle.id,
                        "type_id": vehicle.type_id,
                        "blueprint": blueprint.id,
                        "color": color,
                        "location": {
                            "x": round(transform.location.x, 2),
                            "y": round(transform.location.y, 2),
                            "z": round(transform.location.z, 2)
                        },
                        "autopilot": autopilot
                    }
                    result["vehicles"].append(info)
                    result["success"] += 1
            except Exception as e:
                result["failed"] += 1
        
        app_logger.info("=" * 50)
        app_logger.info(f"[修复7] 批量生成: 成功={result['success']}, 失败={result['failed']}")
        for v in result["vehicles"]:
            app_logger.info(f"  [{v['index']}] ID={v['id']:>4} | {v['type_id']:<35} | ({v['location']['x']}, {v['location']['y']})")
        app_logger.info("=" * 50)
        return result

        # ============ 修复5: 参数化生成actor ============
    async def spawn_vehicles_with_params(self, params):
        """根据参数面板生成actor"""
        blueprint_library = self.world.get_blueprint_library()
        carla_map = self.world.get_map()
        spawned = []
        
        if params.actor_type == "vehicle":
            blueprints = [bp for bp in blueprint_library.filter(params.blueprint_filter) if bp.id.startswith("vehicle.")]
        else:
            blueprints = blueprint_library.filter("walker.pedestrian.*")
        
        if not blueprints:
            app_logger.warning("⚠️ [参数化] 未找到匹配蓝图")
            return spawned
        
        spawn_locations = []
        if params.reference_actor_id is not None:
            ref_actor = self.world.get_actor(params.reference_actor_id)
            if ref_actor and ref_actor.is_alive:
                ref_loc = ref_actor.get_transform().location
                app_logger.info(f"🔍 [参数化] 参照物位置: ({ref_loc.x:.1f}, {ref_loc.y:.1f})")
                for i in range(params.count):
                    angle_rad = math.radians(params.relative_angle + i * (360 / max(params.count, 1)))
                    dist = params.relative_distance
                    sx = ref_loc.x + dist * math.cos(angle_rad) + random.uniform(-3.0, 3.0)
                    sy = ref_loc.y + dist * math.sin(angle_rad) + random.uniform(-3.0, 3.0)
                    spawn_loc = carla.Location(x=sx, y=sy, z=ref_loc.z + 0.5)
                    
                    if params.lane_type == "Driving":
                        wp = carla_map.get_waypoint(spawn_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                    elif params.lane_type == "Sidewalk":
                        wp = carla_map.get_waypoint(spawn_loc, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
                    else:
                        wp = carla_map.get_waypoint(spawn_loc, project_to_road=True)
                    
                    if wp:
                        # 在waypoint位置基础上再加偏移，避免被占用
                        final_loc = carla.Location(
                            x=wp.transform.location.x + random.uniform(-2.0, 2.0),
                            y=wp.transform.location.y + random.uniform(-2.0, 2.0),
                            z=wp.transform.location.z + 0.5
                        )
                        spawn_locations.append(final_loc)
                        app_logger.info(f"🔍 [参数化] 位置{i+1}: waypoint+偏移=({final_loc.x:.1f}, {final_loc.y:.1f})")
                    else:
                        spawn_locations.append(spawn_loc)
                        app_logger.info(f"🔍 [参数化] 位置{i+1}: 原始位置=({spawn_loc.x:.1f}, {spawn_loc.y:.1f})")
            else:
                app_logger.warning(f"⚠️ [参数化] 参照物ID={params.reference_actor_id} 不存在，回退到地图生成点")
                valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
                for i in range(min(params.count, len(valid_points))):
                    spawn_locations.append(valid_points[i].location)
        else:
            valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
            for i in range(min(params.count, len(valid_points))):
                spawn_locations.append(valid_points[i].location)
        
        app_logger.info(f"🔍 [参数化] 请求{params.count}辆, 实际位置数{len(spawn_locations)}")
        
        for i, loc in enumerate(spawn_locations):
            blueprint = random.choice(blueprints)
            if params.actor_type == "vehicle" and blueprint.has_attribute("color"):
                r, g, b = random.randint(0,255), random.randint(0,255), random.randint(0,255)
                blueprint.set_attribute('color', f"{r},{g},{b}")
            
            transform = carla.Transform(loc, carla.Rotation())
            try:
                actor = self.world.try_spawn_actor(blueprint, transform)
                if actor:
                    if params.initial_speed > 0:
                        actor.set_target_velocity(carla.Vector3D(x=params.initial_speed, y=0, z=0))
                    if params.actor_type == "vehicle" and params.autopilot:
                        actor.set_autopilot(True)
                    self.actors.append(actor)
                    spawned.append(actor)
                    app_logger.info(f"🚗 [参数化] 第{i+1}辆成功: ID={actor.id}")
                else:
                    app_logger.warning(f"⚠️ [参数化] 第{i+1}辆失败: 位置被占用 ({loc.x:.1f}, {loc.y:.1f})")
            except Exception as e:
                app_logger.warning(f"❌ [参数化] 第{i+1}辆异常: {e}")
        
        app_logger.info(f"🔍 [参数化] 完成: 请求{params.count}辆, 成功{len(spawned)}辆")
        return spawned

    # ============================================================
    # 第1周场景任务: 高速进出匝道 / 城市车道合并 / 分合流路口 / 辅路
    # ============================================================
    def _get_driving_waypoints(self, distance=4.0):
        """获取地图上所有 Driving 车道 waypoint"""
        carla_map = self.world.get_map()
        return [wp for wp in carla_map.generate_waypoints(distance)
                if wp.lane_type == carla.LaneType.Driving]

    @staticmethod
    def _group_waypoints_by_road(waypoints):
        """按 (road_id, section_id) 分组"""
        groups = {}
        for wp in waypoints:
            groups.setdefault((wp.road_id, wp.section_id), []).append(wp)
        return groups

    @staticmethod
    def _road_length(waypoints):
        """用 s 值估计道路长度"""
        s_values = [wp.s for wp in waypoints]
        return max(s_values) - min(s_values) if s_values else 0.0

    def _follow_lane_all(self, start_wp, step=10.0, max_hops=25, max_visited=300):
        """沿车道向前多分支跟随（路口/分流处自动分叉），
        返回 {road_id: 首个到达该 road 的 waypoint}"""
        reached = {}
        frontier = [start_wp]
        visited = set()
        for _ in range(max_hops):
            new_frontier = []
            for wp in frontier:
                key = (wp.road_id, wp.section_id, round(wp.s, 0), wp.lane_id)
                if key in visited:
                    continue
                visited.add(key)
                reached.setdefault(wp.road_id, wp)
                try:
                    new_frontier.extend(wp.next(step))
                except Exception:
                    pass
            frontier = [w for w in new_frontier
                        if (w.road_id, w.section_id, round(w.s, 0), w.lane_id) not in visited]
            if not frontier or len(visited) >= max_visited:
                break
        return reached

    def _lane_terminal(self, start_wp, step=4.0, max_hops=80):
        """沿同一 road 同一 lane_id 跟随直到无法继续，返回该车道终点 waypoint"""
        current = start_wp
        visited = set()
        for _ in range(max_hops):
            key = (current.road_id, current.section_id, round(current.s, 0), current.lane_id)
            if key in visited:
                break
            visited.add(key)
            try:
                nxt = current.next(step)
            except Exception:
                return current
            same = [w for w in nxt if w.road_id == current.road_id and w.lane_id == current.lane_id]
            if not same:
                return current
            current = same[0]
        return current

    def _spawn_vehicle_on_waypoint(self, waypoint, autopilot=True, tag=""):
        """在指定 waypoint 生成一辆车（位置被占用则向后找空位），返回 actor 或 None"""
        blueprint_library = self.world.get_blueprint_library()
        blueprints = [bp for bp in blueprint_library.filter("vehicle.*") if bp.id.startswith("vehicle.")]
        if not blueprints:
            return None
        candidates = [waypoint]
        candidates.extend(waypoint.previous(8.0) or [])
        candidates.extend(waypoint.previous(16.0) or [])
        candidates.extend(waypoint.previous(24.0) or [])
        for wp in candidates:
            loc = wp.transform.location
            if any(a.get_location().distance(loc) < 6.0 for a in self.world.get_actors().filter("vehicle.*")):
                continue
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                blueprint.set_attribute("color", f"{random.randint(0,255)},{random.randint(0,255)},{random.randint(0,255)}")
            try:
                actor = self.world.try_spawn_actor(blueprint, wp.transform)
            except Exception as e:
                app_logger.warning(f"⚠️ [场景{tag}] 生成异常: {e}")
                continue
            if actor:
                if autopilot:
                    actor.set_autopilot(True)
                self.actors.append(actor)
                return actor
        return None

    def _set_spectator_overhead(self, location, height=45.0):
        """把观察者相机设置为某位置的俯视视角"""
        try:
            spectator = self.world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=location.x, y=location.y, z=location.z + height),
                carla.Rotation(pitch=-90.0)))
        except Exception as e:
            app_logger.warning(f"⚠️ 设置俯视视角失败: {e}")

    async def _ensure_map(self, map_name):
        """如果指定了地图且与当前地图不同，则加载（会清空已有 actor）"""
        if not map_name:
            return True
        try:
            current = self.world.get_map().name
            if map_name in current:
                return True
            app_logger.info(f"🗺️ [场景] 加载地图 {map_name} (当前: {current})")
            return await self.load_world(map_name)
        except Exception as e:
            app_logger.error(f"❌ [场景] 加载地图失败: {e}")
            return False

    async def scenario_highway_ramp(self, ramp_type='on', vehicle_count=4, map_name=None):
        """高速-进出匝道场景。
        ramp_type='on': 匝道车辆汇入主路；'off': 主路车辆驶出匝道。
        自动寻找多车道高速主路与单车道匝道的连接点布设车辆。
        """
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        recommended = "Town04/Town06"
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(4.0)
        groups = self._group_waypoints_by_road(waypoints)
        main_road_ids = set(k[0] for k, v in groups.items() if len({w.lane_id for w in v}) >= 2)
        if not main_road_ids:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到多车道道路（高速），建议使用 {recommended}"}

        # 单车道匝道候选（长度 20~500m）
        ramp_candidates = []
        for (road_id, section_id), wps in groups.items():
            if len({w.lane_id for w in wps}) != 1:
                continue
            length = self._road_length(wps)
            if 20.0 <= length <= 500.0:
                ramp_candidates.append(sorted(wps, key=lambda w: w.s))
        if not ramp_candidates:
            return {"success": False, "error": f"当前地图 {current_map} 未找到匝道结构，建议使用 {recommended}"}

        scene = None
        if ramp_type == 'on':
            # 从匝道入口前向跟随，能找到主路 → 汇入型匝道
            for wps in ramp_candidates:
                reached = self._follow_lane_all(wps[0], step=10.0, max_hops=30)
                main_hits = [rid for rid in reached if rid in main_road_ids]
                if main_hits:
                    scene = {"ramp_wps": wps, "point": reached[main_hits[0]], "reached": reached}
                    break
        else:
            # 从主路各车道前向跟随（多分支），能找到匝道 → 驶出型匝道
            for (road_id, section_id), wps in groups.items():
                if road_id not in main_road_ids:
                    continue
                for wp in wps[::max(1, len(wps) // 6)]:
                    reached = self._follow_lane_all(wp, step=10.0, max_hops=25)
                    ramp_hits = [rid for rid in reached
                                 if rid not in main_road_ids
                                 and any(rid == c[0].road_id for c in ramp_candidates)]
                    if ramp_hits:
                        ramp_first = reached[ramp_hits[0]]
                        ramp_wps = next(c for c in ramp_candidates if c[0].road_id == ramp_hits[0])
                        scene = {"ramp_wps": ramp_wps, "point": wp, "fork_wp": ramp_first}
                        break
                if scene:
                    break

        if not scene:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到{'汇入' if ramp_type == 'on' else '驶出'}型匝道，建议换用 {recommended}"}

        spawned = []
        desc = []
        # 匝道侧车辆
        ramp_count = max(1, vehicle_count // 3)
        ramp_wps = scene["ramp_wps"]
        step_idx = max(1, len(ramp_wps) // (ramp_count + 1))
        for i in range(ramp_count):
            wp = ramp_wps[min((i + 1) * step_idx, len(ramp_wps) - 1)]
            actor = self._spawn_vehicle_on_waypoint(wp, autopilot=True, tag="匝道")
            if actor:
                spawned.append(actor)
                desc.append(f"匝道车 ID={actor.id} @road{wp.road_id} s={wp.s:.0f}")
        # 主路车辆：在汇流/分流点之前的主路车道上
        main_count = max(1, vehicle_count - ramp_count)
        point = scene["point"]
        lane_offsets = [0.0, 8.0, 16.0, 24.0]
        for i in range(main_count):
            back = lane_offsets[i % len(lane_offsets)]
            prev = point.previous(back) if back > 0 else [point]
            if not prev:
                continue
            wp = prev[0]
            actor = self._spawn_vehicle_on_waypoint(wp, autopilot=True, tag="主路")
            if actor:
                spawned.append(actor)
                desc.append(f"主路车 ID={actor.id} @road{wp.road_id} lane={wp.lane_id} s={wp.s:.0f}")

        self._set_spectator_overhead(point.transform.location, height=50.0)
        ramp_desc = "汇入(on-ramp)" if ramp_type == 'on' else "驶出(off-ramp)"
        return {
            "success": bool(spawned),
            "scenario": f"高速-进出匝道({ramp_desc})",
            "map": current_map,
            "merge_point": {"x": round(point.transform.location.x, 1),
                            "y": round(point.transform.location.y, 1)},
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_lane_merge(self, vehicle_count=4, map_name=None):
        """城市-车道合并场景：找到车道数减少（车道消失）的位置，
        在消失车道与延续车道上布设车辆，演示汇流。"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(4.0)
        groups = self._group_waypoints_by_road(waypoints)

        # 找"车道消失点"：同路段中某车道的终点 s 明显短于其他车道
        # （该车道提前结束=汇入邻道，其他车道可走到路段末尾）
        drop_point = None
        drop_lane_id = None
        cont_lane_wp = None
        for (road_id, section_id), wps in groups.items():
            if len({w.lane_id for w in wps}) < 2:
                continue
            if self._road_length(wps) < 40.0:
                continue
            by_lane = {}
            for w in wps:
                by_lane.setdefault(w.lane_id, []).append(w)
            road_max_s = max(w.s for w in wps)
            terminals = {}
            for lane_id, lane_wps in by_lane.items():
                start = min(lane_wps, key=lambda w: w.s)
                terminals[lane_id] = self._lane_terminal(start)
            for lane_id, term in terminals.items():
                if term.s >= road_max_s - 8.0:
                    continue  # 能走到路段末尾，不是消失车道
                # 存在另一条能走到接近路段末尾的车道 → 确认车道消失
                for other_id, other_term in terminals.items():
                    if other_id != lane_id and other_term.s >= road_max_s - 8.0:
                        drop_point, drop_lane_id = term, lane_id
                        cont_lane_wp = min(by_lane[other_id], key=lambda w: abs(w.s - term.s))
                        break
                if drop_point:
                    break
            if drop_point:
                break

        if not drop_point:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到车道消失（合并）结构，建议换用 Town04/Town05/Town10"}

        spawned = []
        desc = []
        # 消失车道上的车（将被挤入邻道）
        vanish_count = max(1, vehicle_count // 2)
        vanish_prev = drop_point.previous(30.0) or drop_point.previous(20.0) or [drop_point]
        for i in range(vanish_count):
            base = vanish_prev[min(i, len(vanish_prev) - 1)]
            actor = self._spawn_vehicle_on_waypoint(base, autopilot=True, tag="消失车道")
            if actor:
                spawned.append(actor)
                desc.append(f"消失车道车 ID={actor.id} lane={drop_lane_id} s={base.s:.0f}")
        # 延续车道上的车
        cont_count = max(1, vehicle_count - vanish_count)
        if cont_lane_wp is not None:
            for back in (15.0, 25.0, 35.0)[:cont_count]:
                prev = cont_lane_wp.previous(back)
                if not prev:
                    continue
                actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="延续车道")
                if actor:
                    spawned.append(actor)
                    desc.append(f"延续车道车 ID={actor.id} lane={cont_lane_wp.lane_id} s={prev[0].s:.0f}")

        self._set_spectator_overhead(drop_point.transform.location, height=40.0)
        return {
            "success": bool(spawned),
            "scenario": "城市-车道合并",
            "map": current_map,
            "merge_point": {"x": round(drop_point.transform.location.x, 1),
                            "y": round(drop_point.transform.location.y, 1),
                            "road_id": drop_point.road_id,
                            "dropped_lane": drop_lane_id},
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_diverge_merge(self, vehicle_count=4, map_name=None):
        """城市-分合流路口场景：找到有多个进出口臂的路口，
        在各进口臂布设车辆，经路口分流后从不同出口驶出/合流。"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        carla_map = self.world.get_map()
        waypoints = self._get_driving_waypoints(6.0)
        junction_wp = next((w for w in waypoints if w.is_junction), None)
        if not junction_wp:
            return {"success": False, "error": f"当前地图 {current_map} 未找到路口，建议换用 Town05/Town10"}

        # 找一个 >=3 臂的路口
        junction = None
        arms = []
        visited_junctions = set()
        for w in waypoints:
            if not w.is_junction:
                continue
            j = w.get_junction()
            if j.id in visited_junctions:
                continue
            visited_junctions.add(j.id)
            try:
                pairs = j.get_waypoints(carla.LaneType.Driving)
            except Exception:
                continue
            entries = {}
            for entry_wp, exit_wp in pairs:
                entries.setdefault(entry_wp.road_id, entry_wp)
            if len(entries) >= 3:
                junction, arms = j, list(entries.values())
                break
        if not junction:
            junction = junction_wp.get_junction()
            pairs = junction.get_waypoints(carla.LaneType.Driving)
            arms = []
            for entry_wp, exit_wp in pairs:
                if all(entry_wp.road_id != a.road_id for a in arms):
                    arms.append(entry_wp)

        spawned = []
        desc = []
        center = junction.bounding_box.location
        per_arm = max(1, vehicle_count // max(1, len(arms)))
        for arm in arms[:4]:
            for i in range(per_arm):
                back = 25.0 + i * 12.0
                prev = arm.previous(back)
                if not prev:
                    prev = [arm]
                actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="路口")
                if actor:
                    spawned.append(actor)
                    desc.append(f"进口臂车 ID={actor.id} road{arm.road_id}")
        self._set_spectator_overhead(center, height=55.0)
        return {
            "success": bool(spawned),
            "scenario": "城市-分合流路口",
            "map": current_map,
            "junction_id": junction.id,
            "arms": len(arms),
            "center": {"x": round(center.x, 1), "y": round(center.y, 1)},
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_side_road(self, vehicle_count=4, map_name=None):
        """城市-辅路场景：找到与主路平行且近距离的道路（辅路），
        在主路和辅路上同时布设车辆。"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)

        # 找一对平行且间距 8~45m 的不同道路
        pair = None
        stride = max(1, len(waypoints) // 120)
        for w1 in waypoints[::stride]:
            if self._road_length(self._group_waypoints_by_road(waypoints).get((w1.road_id, w1.section_id), [])) < 60.0:
                continue
            best = None
            best_d = 999.0
            for w2 in waypoints:
                if w2.road_id == w1.road_id:
                    continue
                d = w2.transform.location.distance(w1.transform.location)
                if not (8.0 <= d <= 45.0) or d >= best_d:
                    continue
                yaw1 = w1.transform.rotation.yaw % 360
                yaw2 = w2.transform.rotation.yaw % 360
                diff = abs(yaw1 - yaw2) % 360
                diff = min(diff, 360 - diff)
                if diff < 25.0:
                    best, best_d = w2, d
            if best:
                pair = (w1, best, best_d)
                break

        if not pair:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到与主路平行的辅路结构，建议换用 Town05/Town10"}

        main_wp, side_wp, side_dist = pair
        spawned = []
        desc = []

        def _spread(wp, count, tag):
            actors = []
            lane_wps = [w for w in waypoints
                        if w.road_id == wp.road_id and w.lane_id == wp.lane_id]
            lane_wps.sort(key=lambda w: w.s)
            if not lane_wps:
                lane_wps = [wp]
            step_idx = max(1, len(lane_wps) // (count + 1))
            for i in range(count):
                idx = min((i + 1) * step_idx, len(lane_wps) - 1)
                actor = self._spawn_vehicle_on_waypoint(lane_wps[idx], autopilot=True, tag=tag)
                if actor:
                    actors.append(actor)
                    desc.append(f"{tag}车 ID={actor.id} road{wp.road_id} s={lane_wps[idx].s:.0f}")
            return actors

        main_count = max(1, (vehicle_count + 1) // 2)
        side_count = max(1, vehicle_count - main_count)
        spawned.extend(_spread(main_wp, main_count, "主路"))
        spawned.extend(_spread(side_wp, side_count, "辅路"))

        mid = carla.Location(x=(main_wp.transform.location.x + side_wp.transform.location.x) / 2,
                             y=(main_wp.transform.location.y + side_wp.transform.location.y) / 2,
                             z=main_wp.transform.location.z)
        self._set_spectator_overhead(mid, height=50.0)
        return {
            "success": bool(spawned),
            "scenario": "城市-辅路",
            "map": current_map,
            "main_road_id": main_wp.road_id,
            "side_road_id": side_wp.road_id,
            "parallel_distance_m": round(side_dist, 1),
            "spawned_count": len(spawned),
            "details": desc
        }



        # ============ 修复8: 行人停止/恢复移动 ============
    def stop_walker(self, walker_id):
        controller = self.walker_controllers.get(walker_id)
        if controller and controller.is_alive:
            controller.stop()
            app_logger.info(f"[修复8] 行人 {walker_id} 已停止")
            return True
        return False

    def resume_walker(self, walker_id, new_target=None):
        walker = self.world.get_actor(walker_id)
        controller = self.walker_controllers.get(walker_id)
        if not walker or not walker.is_alive:
            app_logger.warning(f"[修复8] 行人 {walker_id} 不存在")
            return False
        if not controller or not controller.is_alive:
            controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            controller = self.world.spawn_actor(controller_bp, carla.Transform(), walker)
            self.walker_controllers[walker_id] = controller
        if new_target is None:
            new_target = self.world.get_random_location_from_navigation()
        if not new_target:
            current_loc = walker.get_location()
            new_target = carla.Location(
                x=current_loc.x + random.uniform(-20, 20),
                y=current_loc.y + random.uniform(-20, 20),
                z=current_loc.z
            )
        controller.go_to_location(new_target)
        controller.set_max_speed(random.uniform(1.0, 2.0))
        controller.start()
        if walker_id in self.walker_goals:
            self.walker_goals[walker_id]['target'] = new_target
            self.walker_goals[walker_id]['stuck_count'] = 0
            self.walker_goals[walker_id]['last_location'] = walker.get_location()
        app_logger.info(f"[修复8] 行人 {walker_id} 已恢复移动")
        return True

    def stop_all_walkers(self):
        count = 0
        for walker_id, controller in self.walker_controllers.items():
            if controller and controller.is_alive:
                controller.stop()
                count += 1
        app_logger.info(f"[修复8] 共停止 {count} 个行人")
        return count

    def resume_all_walkers(self):
        count = 0
        for walker_id in list(self.walker_controllers.keys()):
            if self.resume_walker(walker_id):
                count += 1
        app_logger.info(f"[修复8] 共恢复 {count} 个行人")
        return count

    async def cleanup(self):
        """清理环境"""
        # 停止后台tick循环
        await self.stop_tick_loop()

        # 停止视角跟随
        await self.stop_view_follow()

        # 停止视频录制
        await self.stop_recording()

        # 先关闭所有车辆自动驾驶，避免Traffic Manager与destroy竞争导致原生崩溃
        for actor in list(self.actors):
            try:
                if actor.is_alive and 'vehicle' in actor.type_id:
                    actor.set_autopilot(False)
            except Exception:
                pass

        for actor in self.actors:
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception as e:
                app_logger.warning(f"⚠️ 销毁actor {getattr(actor, 'id', '?')} 失败: {e}")
        self.actors = []
        app_logger.info("🧹 清理所有CARLA actor")

        # ============ 修复3: 行人卡住检测与自动修复 ============
    def check_and_fix_stuck_walkers(self):
        """每帧检测行人是否卡住，若卡住则重新设置目标"""
        self.walker_check_interval += 1
        if self.walker_check_interval < 30:
            return
        self.walker_check_interval = 0
        
        for walker_id, info in list(self.walker_goals.items()):
            walker = self.world.get_actor(walker_id)
            if walker is None or not walker.is_alive:
                del self.walker_goals[walker_id]
                if walker_id in self.walker_controllers:
                    del self.walker_controllers[walker_id]
                continue
            
            current_loc = walker.get_location()
            last_loc = info['last_location']
            distance = current_loc.distance(last_loc)
            
            if distance < 0.15:
                info['stuck_count'] += 1
            else:
                info['stuck_count'] = 0
                info['last_location'] = current_loc
            
            if info['stuck_count'] >= 3:
                controller = self.walker_controllers.get(walker_id)
                if controller and controller.is_alive:
                    new_target = self.world.get_random_location_from_navigation()
                    if new_target:
                        controller.go_to_location(new_target)
                        info['target'] = new_target
                        info['stuck_count'] = 0
                        info['last_location'] = current_loc
                        app_logger.info(f"[修复3] 行人 {walker_id} 原地踏步，已重新设置目标")
                    else:
                        fallback = carla.Location(
                            x=current_loc.x + random.uniform(-15, 15),
                            y=current_loc.y + random.uniform(-15, 15),
                            z=current_loc.z
                        )
                        controller.go_to_location(fallback)
                        info['target'] = fallback
                        info['stuck_count'] = 0
                        info['last_location'] = current_loc
                        app_logger.info(f"[修复3] 行人 {walker_id} 原地踏步，已设置备用目标")

    # ============ 视角控制功能 ============

    def set_third_person_view(self, target_actor, distance=5.0, height=2.0, offset_angle=0):
        """设置第三人称视角（跟随视角）

        Args:
            target_actor: 目标actor（车辆或行人）
            distance: 相机与目标的距离（米）
            height: 相机高度（米）
            offset_angle: 水平偏移角度（度）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            # 计算相机位置（在目标后方指定距离和高度）
            yaw_rad = math.radians(target_transform.rotation.yaw + offset_angle + 180)  # +180 表示在目标后方
            camera_x = target_location.x + distance * math.cos(yaw_rad)
            camera_y = target_location.y + distance * math.sin(yaw_rad)
            camera_z = target_location.z + height

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)

            # 计算相机朝向，指向目标
            camera_rotation = carla.Rotation(
                pitch=-15.0,  # 略微向下看
                yaw=target_transform.rotation.yaw + offset_angle,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  第三人称视角已设置 - 目标: {target_actor.id}, 距离: {distance}m, 高度: {height}m")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置第三人称视角失败: {str(e)}")
            return False

    def set_first_person_view(self, target_actor, offset_x=0.3, offset_y=0.0, offset_z=1.2):
        """设置第一人称视角（驾驶员/行人视角）

        Args:
            target_actor: 目标actor（车辆或行人）
            offset_x: 前后偏移（米），默认0.3米（稍微向前）
            offset_y: 左右偏移（米）
            offset_z: 高度偏移（米），默认1.2米（眼睛高度）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            # 计算相机位置（在目标位置，考虑旋转）
            yaw_rad = math.radians(target_transform.rotation.yaw)
            # 相机位置：在目标前方offset_x处（行人/车辆朝向的方向）
            camera_x = target_location.x + offset_x * math.cos(yaw_rad) - offset_y * math.sin(yaw_rad)
            camera_y = target_location.y + offset_x * math.sin(yaw_rad) + offset_y * math.cos(yaw_rad)
            # 高度：目标位置高度 + 眼睛高度偏移
            camera_z = target_location.z + offset_z

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)

            # 相机朝向与目标相同
            camera_rotation = carla.Rotation(
                pitch=0.0,  # 平视
                yaw=target_transform.rotation.yaw,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  第一人称视角已设置 - 目标: {target_actor.id}, 高度: {camera_z:.2f}m")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置第一人称视角失败: {str(e)}")
            return False

    def set_overhead_view(self, target_actor=None, height=30.0):
        """设置俯视视角（鸟瞰视角）

        Args:
            target_actor: 目标actor，如果为None则使用地图中心
            height: 相机高度（米）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            self.view_target = target_actor  # 修复4: 保存追踪目标

            if target_actor:
                target_location = target_actor.get_transform().location
            else:
                # 使用地图中心或默认位置
                target_location = carla.Location(x=0, y=0, z=0)

            camera_location = carla.Location(
                x=target_location.x,
                y=target_location.y,
                z=target_location.z + height
            )

            camera_rotation = carla.Rotation(
                pitch=-90.0,  # 垂直向下看
                yaw=0.0,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  俯视视角已设置 - 高度: {height}m" + 
                          (f"，追踪目标ID={target_actor.id}" if target_actor else ""))
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置俯视视角失败: {str(e)}")
            return False

    def set_free_view(self, location=None, rotation=None):
        """设置自由视角（观察者视角）

        Args:
            location: 相机位置，如果为None则使用默认位置
            rotation: 相机旋转，如果为None则使用默认旋转

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()

            if location is None:
                location = carla.Location(x=0, y=0, z=50)
            if rotation is None:
                rotation = carla.Rotation(pitch=-45, yaw=0, roll=0)

            camera_transform = carla.Transform(location, rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  自由视角已设置 - 位置: ({location.x}, {location.y}, {location.z})")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置自由视角失败: {str(e)}")
            return False

    def rotate_view_around_target(self, target_actor, angle_degrees, distance=5.0, height=2.0):
        """围绕目标旋转视角

        Args:
            target_actor: 目标actor
            angle_degrees: 旋转角度（度）
            distance: 相机与目标的距离（米）
            height: 相机高度（米）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            target_location = target_actor.get_transform().location


            angle_rad = math.radians(angle_degrees)
            camera_x = target_location.x + distance * math.cos(angle_rad)
            camera_y = target_location.y + distance * math.sin(angle_rad)
            camera_z = target_location.z + height

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)

            # 计算朝向目标的旋转
            yaw = angle_degrees + 180  # 朝向中心
            camera_rotation = carla.Rotation(pitch=-15, yaw=yaw, roll=0)

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  视角已旋转到 {angle_degrees}°")
            return True
        except Exception as e:
            app_logger.error(f"❌ 旋转视角失败: {str(e)}")
            return False

    async def set_bystander_view(self):
        """设置旁观者视角（默认观察者视角，不跟随任何目标）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            # 停止之前的视角跟随
            await self.stop_view_follow()

            spectator = self.world.get_spectator()

            # 获取地图的推荐观察者位置
            spawn_points = self.world.get_map().get_spawn_points()
            if spawn_points:
                # 使用第一个生成点作为参考，在其上方设置观察者
                ref_point = spawn_points[0].location
                location = carla.Location(x=ref_point.x, y=ref_point.y, z=ref_point.z + 50)
            else:
                location = carla.Location(x=0, y=0, z=50)

            rotation = carla.Rotation(pitch=-45, yaw=0, roll=0)
            camera_transform = carla.Transform(location, rotation)
            spectator.set_transform(camera_transform)

            # 清除当前视角目标
            self.view_target = None
            self.current_view_mode = "bystander"

            app_logger.info(f"👁️  旁观者视角已设置 - 位置: ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置旁观者视角失败: {str(e)}")
            return False

    def start_view_follow(self, view_mode, target_actor):
        """启用视角跟随（实际更新逻辑已合并到tick_loop中）
        
        Args:
            view_mode: 视角模式 - third_person, first_person, overhead
            target_actor: 要跟随的目标actor
        """
        self.is_view_following = True
        self.view_target = target_actor
        self.current_view_mode = view_mode

        app_logger.info(f"🎯 视角跟随已启用 - 模式: {view_mode}, 目标: {target_actor.id}")

    async def stop_view_follow(self):
        """停止视角跟随"""
        if self.is_view_following:
            self.is_view_following = False
            app_logger.info("🛑 停止视角跟随")

        if self.view_follow_task and not self.view_follow_task.done():
            self.view_follow_task.cancel()
            try:
                await self.view_follow_task
            except asyncio.CancelledError:
                pass
            self.view_follow_task = None

    def _update_third_person_view(self, target_actor, distance=5.0, height=2.0):
        """更新第三人称视角位置（用于跟随）"""
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            yaw_rad = math.radians(target_transform.rotation.yaw + 180)
            camera_x = target_location.x + distance * math.cos(yaw_rad)
            camera_y = target_location.y + distance * math.sin(yaw_rad)
            camera_z = target_location.z + height

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
            camera_rotation = carla.Rotation(
                pitch=-15.0,
                yaw=target_transform.rotation.yaw,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        except Exception as e:
            app_logger.warning(f"⚠️ 更新第三人称视角出错: {e}")

    def _update_first_person_view(self, target_actor, offset_x=0.3, offset_y=0.0, offset_z=1.2):
        """更新第一人称视角位置（用于跟随）"""
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            yaw_rad = math.radians(target_transform.rotation.yaw)
            camera_x = target_location.x + offset_x * math.cos(yaw_rad) - offset_y * math.sin(yaw_rad)
            camera_y = target_location.y + offset_x * math.sin(yaw_rad) + offset_y * math.cos(yaw_rad)
            camera_z = target_location.z + offset_z

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
            camera_rotation = carla.Rotation(
                pitch=0.0,
                yaw=target_transform.rotation.yaw,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        except Exception as e:
            app_logger.warning(f"⚠️ 更新第一人称视角出错: {e}")


    def get_all_pedestrians(self):
        """获取当前世界中所有行人列表"""
        if self.world is None:
            return []

        pedestrians = []
        try:
            for actor in self.world.get_actors():
                if 'walker' in actor.type_id and 'controller' not in actor.type_id:
                    type_name = self._get_pedestrian_type_name(actor.type_id)
                    pedestrians.append({
                        'id': actor.id,
                        'type_id': actor.type_id,
                        'type_name': type_name
                    })
        except Exception as e:
            app_logger.error(f"❌ 获取行人列表失败: {str(e)}")

        return pedestrians

    def _update_overhead_view(self, target_actor, height=30.0):
        """更新俯视视角位置（用于跟随目标）"""
        try:
            spectator = self.world.get_spectator()
            target_location = target_actor.get_transform().location
            camera_location = carla.Location(
                x=target_location.x, y=target_location.y, z=target_location.z + height
            )
            camera_rotation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        except Exception as e:
            app_logger.warning(f"⚠️ 更新俯视视角出错: {e}")

    def get_all_pedestrians(self):
        """获取当前世界中所有行人列表

        Returns:
            list: 行人信息列表，每个元素包含 (id, type_id, type_name)
        """
        if self.world is None:
            return []

        pedestrians = []
        try:
            for actor in self.world.get_actors():
                if 'walker' in actor.type_id and 'controller' not in actor.type_id:
                    # 提取行人类型名称
                    type_name = self._get_pedestrian_type_name(actor.type_id)
                    pedestrians.append({
                        'id': actor.id,
                        'type_id': actor.type_id,
                        'type_name': type_name
                    })
        except Exception as e:
            app_logger.error(f"❌ 获取行人列表失败: {str(e)}")

        return pedestrians

    def _get_pedestrian_type_name(self, type_id):
        """根据type_id获取行人类型中文名称"""
        # 从蓝图ID中提取编号
        import re
        match = re.search(r'walker\.pedestrian\.(\d+)', type_id)
        if match:
            blueprint_number = match.group(1)
            # 根据编号判断类型
            if blueprint_number in ['0030', '0032']:
                return "警察"
            elif blueprint_number in ['0009', '0010', '0011', '0012', '0013', '0014', '0048', '0049']:
                return "儿童"
            elif blueprint_number in ['0020', '0021', '0022', '0023', '0024', '0025']:
                return "老年人"
            elif blueprint_number in ['0027', '0028', '0029']:
                return "商务人士"
            else:
                return "普通行人"
        return "未知类型"

    def get_all_vehicles(self):
        """获取当前世界中所有车辆列表

        Returns:
            list: 车辆信息列表，每个元素包含 (id, type_id, type_name)
        """
        if self.world is None:
            return []

        vehicles = []
        try:
            for actor in self.world.get_actors():
                if 'vehicle' in actor.type_id:
                    # 提取车辆类型名称
                    type_name = self._get_vehicle_type_name(actor.type_id)
                    vehicles.append({
                        'id': actor.id,
                        'type_id': actor.type_id,
                        'type_name': type_name
                    })
        except Exception as e:
            app_logger.error(f"❌ 获取车辆列表失败: {str(e)}")

        return vehicles

    def _get_vehicle_type_name(self, type_id):
        """根据type_id获取车辆类型中文名称"""
        # 车辆类型映射表
        vehicle_types = {
            'model3': '特斯拉 Model 3',
            'a2': '奥迪 A2',
            'etron': '奥迪 e-tron',
            'tt': '奥迪 TT',
            'grandtourer': '宝马 Grand Tourer',
            'i8': '宝马 i8',
            'mini': '宝马 Mini',
            'impala': '雪佛兰 Impala',
            'c3': '雪铁龙 C3',
            'charger_police': '道奇 Charger Police',
            'charger2020': '道奇 Charger 2020',
            'mustang': '福特 Mustang',
            'crown': '福特 Crown',
            'wrangler_rubicon': '吉普 Wrangler Rubicon',
            'mkz_2017': '林肯 MKZ 2017',
            'mkz_2020': '林肯 MKZ 2020',
            'benz_coupe': '奔驰 Coupe',
            'cabrio': '奔驰 Cabrio',
            'ccc': '奔驰 CCC',
            'cooper_s': 'Mini Cooper S',
            'micra': '日产 Micra',
            'patrol': '日产 Patrol',
            'leon': '西雅特 Leon',
            't2': '大众 T2',
            't3': '大众 T3',
        }
        # 从type_id中提取车辆型号
        for key, name in vehicle_types.items():
            if key in type_id.lower():
                return name
        return "未知车辆"

    # ============ 视频录制功能 ============

    async def start_recording(self, fps=30, output_path=None):
        """开始视频录制 - 从当前窗口视角录制

        Args:
            fps: 帧率
            output_path: 输出文件路径，如果为None则自动生成

        Returns:
            bool: 是否成功开始录制
        """
        import os
        import datetime

        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法开始录制")
            return False

        if self.is_recording:
            app_logger.warning("⚠️ 已经在录制中，请先停止当前录制")
            return False

        try:
            # 设置输出路径
            if output_path is None:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = "recordings"
                os.makedirs(output_dir, exist_ok=True)
                output_path = os.path.join(output_dir, f"carla_recording_{timestamp}.mp4")

            self.recording_output_path = output_path
            self.recording_fps = fps
            self.recording_frame_count = 0
            self.is_recording = True

            # 创建相机传感器（使用spectator视角）
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1920')
            camera_bp.set_attribute('image_size_y', '1080')
            camera_bp.set_attribute('fov', '110')

            # 初始位置在spectator位置
            spectator = self.world.get_spectator()
            camera_transform = spectator.get_transform()
            self.camera_sensor = self.world.spawn_actor(camera_bp, camera_transform)

            # 创建图像队列
            import queue
            self.image_queue = queue.Queue()
            self.camera_sensor.listen(self.image_queue.put)

            # 启动录制任务
            import asyncio
            self.recording_task = asyncio.create_task(self._recording_loop())

            app_logger.info(f"🎥 开始录制 - 输出: {output_path}, 帧率: {fps}fps, 分辨率: 1920x1080")
            return True

        except Exception as e:
            app_logger.error(f"❌ 开始录制失败: {str(e)}")
            return False

    async def _recording_loop(self):
        """录制循环 - 持续捕获帧并写入视频"""
        import asyncio

        frame_interval = 1.0 / self.recording_fps

        while self.is_recording:
            try:
                # 更新相机位置到当前spectator位置
                if self.world and self.camera_sensor:
                    spectator = self.world.get_spectator()
                    camera_transform = spectator.get_transform()
                    self.camera_sensor.set_transform(camera_transform)

                # 获取图像
                if self.image_queue and not self.image_queue.empty():
                    image = self.image_queue.get()
                    # 转换为numpy数组
                    import numpy as np
                    import cv2
                    array = np.frombuffer(image.raw_data, dtype=np.uint8)
                    array = array.reshape((image.height, image.width, 4))
                    array = array[:, :, :3]
                    img_rgb = array[:, :, ::-1]
                    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

                    # 写入视频文件
                    if self.video_writer is None:
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        self.video_writer = cv2.VideoWriter(
                            self.recording_output_path,
                            fourcc,
                            self.recording_fps,
                            (image.width, image.height)
                        )
                        app_logger.info(f"📹 视频写入器已创建")

                    self.video_writer.write(img_bgr)
                    self.recording_frame_count += 1

                await asyncio.sleep(frame_interval)

            except Exception as e:
                app_logger.warning(f"⚠️ 录制帧捕获出错: {e}")
                await asyncio.sleep(frame_interval)

    async def stop_recording(self):
        """停止视频录制

        Returns:
            str: 操作结果信息
        """
        import asyncio

        if not self.is_recording:
            return "未在录制中"

        try:
            self.is_recording = False

            # 等待录制任务结束
            if self.recording_task:
                try:
                    await asyncio.wait_for(self.recording_task, timeout=2.0)
                except asyncio.TimeoutError:
                    self.recording_task.cancel()

            # 释放视频写入器
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
                app_logger.info(f"📹 视频写入器已释放")

            # 停止相机监听
            if self.camera_sensor:
                self.camera_sensor.stop()

            # 清理相机传感器
            if self.camera_sensor:
                if self.camera_sensor.is_alive:
                    self.camera_sensor.destroy()
                self.camera_sensor = None

            self.image_queue = None

            result = f"✅ 录制已停止，共录制 {self.recording_frame_count} 帧，已保存至: {self.recording_output_path}"
            app_logger.info(result)

            self.recording_frame_count = 0
            return result

        except Exception as e:
            app_logger.error(f"❌ 停止录制失败: {str(e)}")
            return f"停止录制失败: {str(e)}"

    async def switch_view_mode(self, view_mode, target_actor_id=None):
        """切换视角模式

        Args:
            view_mode: 视角模式 - third_person, first_person, overhead, free, bystander
            target_actor_id: 目标actor ID

        Returns:
            str: 操作结果信息
        """
        import asyncio

        if self.world is None:
            return "❌ 未连接到CARLA服务器"

        # 旁观者视角不需要目标
        if view_mode == "bystander":
            await self.set_bystander_view()
            return "✅ 已切换到旁观者视角"

        # 确定目标actor
        target_actor = None
        if target_actor_id:
            target_actor = self.world.get_actor(target_actor_id)
        elif self.view_target:
            target_actor = self.view_target
        elif self.actors:
            for actor in reversed(self.actors):
                if 'vehicle' in actor.type_id or 'walker' in actor.type_id:
                    target_actor = actor
                    break

        if target_actor:
            self.view_target = target_actor

        # 设置视角
        if view_mode == "third_person":
            if target_actor:
                # 先停止之前的视角跟随
                await self.stop_view_follow()
                # 先设置一次视角
                self.set_third_person_view(target_actor)
                self.current_view_mode = "third_person"
                # 启动视角跟随（普通方法，直接调用）
                self.start_view_follow("third_person", target_actor)
                result = f"✅ 已切换到第三人称视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "❌ 第三人称视角需要指定目标"

        elif view_mode == "first_person":
            if target_actor:
                # 先停止之前的视角跟随
                await self.stop_view_follow()
                # 先设置一次视角
                self.set_first_person_view(target_actor)
                self.current_view_mode = "first_person"
                # 启动视角跟随（普通方法，直接调用）
                self.start_view_follow("first_person", target_actor)
                result = f"✅ 已切换到第一人称视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "❌ 第一人称视角需要指定目标"

        elif view_mode == "overhead":
            # 停止之前的跟随
            await self.stop_view_follow()
            self.set_overhead_view(target_actor)
            self.current_view_mode = "overhead"
            # 如果有目标，启动跟随
            if target_actor:
                self.start_view_follow("overhead", target_actor)
                result = f"✅ 已切换到俯视视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "✅ 已切换到俯视视角"

        elif view_mode == "free":
            # 停止之前的跟随
            await self.stop_view_follow()
            self.set_free_view()
            self.current_view_mode = "free"
            result = "✅ 已切换到自由视角"

        else:
            result = f"❌ 未知的视角模式: {view_mode}"

        return result


# 全局客户端实例
carla_client = CarlaClient()

async def connect_carla_impl(host: str = 'localhost', port: int = 2000) -> str:
    success = await carla_client.connect(host, port)
    # 双重保险：确认 world 真的获取到了
    if success and carla_client.world is None:
        try:
            carla_client.world = carla_client.client.get_world()
        except:
            pass
    if carla_client.world is None:
        return "❌ CARLA连接异常：无法获取world对象，请确认服务器已启动"
    return "✅ CARLA服务器连接成功"


async def spawn_vehicle_impl(query: str, count: int = 1, **kwargs) -> str:
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    # 强制转int，防止LLM传字符串进来
    count = int(count)
    
    vehicles = await carla_client.spawn_vehicles(query, count=count)
    if vehicles:
        if len(vehicles) == 1:
            return f"✅ 已生成1辆{query}车辆 (ID: {vehicles[0].id})"
        else:
            last_vehicle = vehicles[-1]
            return f"✅ 已生成{len(vehicles)}辆{query}车辆，最后一辆车ID: {last_vehicle.id}"
    return "❌ 车辆生成失败，请确保CARLA服务器已连接且地图有可用生成点"

async def spawn_bicycle_impl(query: str, count: int = 1, **kwargs) -> str:
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    count = int(count)  # ← 强制转int
    
    bicycles = await carla_client.spawn_bicycles(query, count=count)
    if bicycles:
        if len(bicycles) == 1:
            return f"✅ 已生成1辆{query}自行车 (ID: {bicycles[0].id})"
        else:
            ids = [b.id for b in bicycles]
            return f"✅ 已生成{len(bicycles)}辆{query}自行车，ID列表: {ids}"
    return "❌ 自行车生成失败，请确保CARLA服务器已连接且地图有可用生成点"

async def spawn_motorcycle_impl(query: str, count: int = 1, **kwargs) -> str:
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    count = int(count)  # ← 强制转int
    
    motorcycles = await carla_client.spawn_motorcycles(query, count=count)
    if motorcycles:
        if len(motorcycles) == 1:
            return f"✅ 已生成1辆{query}摩托车 (ID: {motorcycles[0].id})"
        else:
            ids = [m.id for m in motorcycles]
            return f"✅ 已生成{len(motorcycles)}辆{query}摩托车，ID列表: {ids}"
    return "❌ 摩托车生成失败"

async def spawn_prop_impl(query: str, count: int = 1, target_id: int = None, **kwargs) -> str:
    """（实际功能：生成道具/警示牌，可指定放在某个actor后方）"""
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    count = int(count)
    
    # 如果指定了 target_id，计算目标后方位置
    location = None
    if target_id is not None:
        target_actor = None
        for actor in carla_client.world.get_actors():
            if actor.id == int(target_id):
                target_actor = actor
                break
        
        if target_actor is None:
            return f"❌ 找不到目标 actor (ID: {target_id})，无法放置道具"
        
        # 获取目标位置和朝向，计算后方5米处
        target_loc = target_actor.get_location()
        target_rot = target_actor.get_transform().rotation
        
        # 将 yaw 转换为弧度，计算后方偏移
        import math
        yaw_rad = math.radians(target_rot.yaw)
        # 后方 = 当前位置 - 朝向向量 * 距离
        behind_x = target_loc.x - math.cos(yaw_rad) * 5.0
        behind_y = target_loc.y - math.sin(yaw_rad) * 5.0
        behind_z = target_loc.z + 0.1
        
        location = carla.Location(x=behind_x, y=behind_y, z=behind_z)
        app_logger.info(f"🚧 道具将放置在目标 {target_id} 后方5米处 ({behind_x:.1f}, {behind_y:.1f})")
    
    props = await carla_client.spawn_props(query, count=count, location=location)
    if props:
        if len(props) == 1:
            return f"✅ 已生成1个{query}道具 (ID: {props[0].id})" + (f"，位于目标 {target_id} 后方" if target_id else "")
        else:
            ids = [p.id for p in props]
            return f"✅ 已生成{len(props)}个{query}道具，ID列表: {ids}" + (f"，位于目标 {target_id} 后方" if target_id else "")
    return "❌ 道具生成失败"

async def spawn_overturned_vehicle_impl(vehicle_type: str = 'model3', **kwargs) -> str:
    """（实际功能：生成仰翻车辆）"""
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    vehicle = await carla_client.spawn_overturned_vehicle(vehicle_type)
    if vehicle:
        return f"✅ 已生成仰翻的{vehicle_type} (ID: {vehicle.id})，物理已禁用以保持姿态"
    return "❌ 仰翻车辆生成失败"

async def set_weather_impl(weather_type: str) -> str:
    """（实际功能：设置天气）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    weather_presets = {'clear': '晴天', 'rain': '雨天', 'fog': '雾天', 'snow': '雪天', 'night': '夜晚'}
    success = await carla_client.set_weather(weather_type.lower())
    return f"✅ 天气已设置为 {weather_presets.get(weather_type.lower(), weather_type)}" if success else "❌ 不支持的天气类型"


async def get_traffic_lights_impl(query: str, **kwargs) -> str:
    """（实际功能：获取交通灯信息）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    lights = await carla_client.get_traffic_lights()
    if not lights:
        return "🚦 未找到交通灯或无法获取交通灯信息"
    
    result = ["🚦 交通灯状态:"]
    for i, light in enumerate(lights, 1):
        state = "绿色" if light.state == carla.TrafficLightState.Green else \
            "红色" if light.state == carla.TrafficLightState.Red else \
                "黄色"
        result.append(f"{i}. {light.type_id} - {state} (位置: {light.get_location()})")
    return "\n".join(result)


async def cleanup_scene_impl(**kwargs) -> str:
    """（实际功能：清理环境）"""
    await carla_client.cleanup()
    return "✅ 已清理所有车辆和物体"


async def spawn_pedestrian_impl(query: str, count: int = 1, speed: float = None, **kwargs) -> str:
    """（实际功能：生成行人）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    count = int(count)  # ← 强制转int
    
    pedestrians = await carla_client.spawn_pedestrians(query, count=count, speed=speed)
    if pedestrians:
        speed_info = f"，速度: {speed} m/s" if speed is not None else ""
        if len(pedestrians) == 1:
            return f"✅ 已生成1个{query}行人 (ID: {pedestrians[0].id}){speed_info}，行人已开始自动行走"
        else:
            last_pedestrian = pedestrians[-1]
            return f"✅ 已生成{len(pedestrians)}个{query}行人，最后一个行人ID: {last_pedestrian.id}{speed_info}，所有行人已开始自动行走"
    return "❌ 行人生成失败，请确保CARLA服务器已连接且地图有可用导航点"


async def setup_autopilot_impl(enable: bool = True, radius: float = 0.0, **kwargs) -> str:
    """（实际功能：设置车辆自动驾驶）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    success = await carla_client.setup_autopilot(enable, radius)
    if success:
        return f"✅ 车辆自动驾驶已{'启用' if enable else '禁用'}"
    return "❌ 设置自动驾驶失败"


async def setup_pedestrian_movement_impl(enable: bool = True, radius: float = 0.0, **kwargs) -> str:
    """（实际功能：设置行人自动移动）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    success = await carla_client.setup_pedestrian_movement(enable, radius)
    if success:
        return f"✅ 行人自动移动已{'启用' if enable else '禁用'}"
    return "❌ 设置行人移动失败"


# ============ 视角控制和视频录制实现函数 ============

async def switch_view_impl(view_mode: str, target_actor_id: int = None, **kwargs) -> str:
    """（实际功能：切换视角模式）"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    result = await carla_client.switch_view_mode(view_mode, target_actor_id)
    return result


async def start_recording_impl(fps: int = 30, **kwargs) -> str:
    """（实际功能：开始视频录制）"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    success = await carla_client.start_recording(fps=fps)

    if success:
        return f"🎥 开始录制 - 帧率: {fps}fps。录制过程中可以自由切换视角。"
    return "❌ 开始录制失败"


async def stop_recording_impl(**kwargs) -> str:
    """（实际功能：停止视频录制）"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"

    result = await carla_client.stop_recording()
    return result


# ============ 第1周场景任务: 底层实现函数 ============
def _format_scenario_result(result) -> str:
    """统一格式化场景构建结果"""
    if not result.get("success"):
        return f"❌ {result.get('scenario', '场景')}构建失败: {result.get('error', '未知错误')}"
    lines = [
        f"✅ 场景「{result['scenario']}」构建完成（地图: {result['map']}）",
        f"🚗 成功生成 {result['spawned_count']} 辆车（已开启自动驾驶汇流行驶）",
    ]
    if "merge_point" in result:
        mp = result["merge_point"]
        lines.append(f"📍 关键位置: ({mp['x']}, {mp['y']})")
    if "junction_id" in result:
        lines.append(f"📍 路口ID: {result['junction_id']}，进口臂数: {result['arms']}")
    if "parallel_distance_m" in result:
        lines.append(f"📍 主路 road{result['main_road_id']} / 辅路 road{result['side_road_id']}，平行间距 {result['parallel_distance_m']}m")
    if result.get("details"):
        lines.append("🚙 明细:")
        lines.extend(f"  • {d}" for d in result["details"])
    return "\n".join(lines)


async def scenario_highway_ramp_impl(ramp_type: str = 'on', vehicle_count: int = 4,
                                     map_name: Optional[str] = None, **kwargs) -> str:
    """高速-进出匝道场景底层实现"""
    result = await carla_client.scenario_highway_ramp(ramp_type, vehicle_count, map_name)
    return _format_scenario_result(result)


async def scenario_lane_merge_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    """城市-车道合并场景底层实现"""
    result = await carla_client.scenario_lane_merge(vehicle_count, map_name)
    return _format_scenario_result(result)


async def scenario_diverge_merge_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    """城市-分合流路口场景底层实现"""
    result = await carla_client.scenario_diverge_merge(vehicle_count, map_name)
    return _format_scenario_result(result)


async def scenario_side_road_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    """城市-辅路场景底层实现"""
    result = await carla_client.scenario_side_road(vehicle_count, map_name)
    return _format_scenario_result(result)


# ============ FastMCP 工具装饰器版本 ============

@mcp.tool()
async def connect_carla(host: str = 'localhost', port: int = 2000) -> str:
    """（实际功能：连接CARLA）"""
    return await connect_carla_impl(host, port)


@mcp.tool()
async def spawn_vehicle(query: str, count: int = 1) -> str:
    """（实际功能：生成车辆）"""
    return await spawn_vehicle_impl(query, count=count)

@mcp.tool()
async def spawn_bicycle(query: str, count: int = 1) -> str:
    """（实际功能：生成自行车）
    
    支持类型: crossbike(BH Crossbike), century(Diamondback Century), omafiets(Gazelle Omafiets)
    """
    return await spawn_bicycle_impl(query, count=count)

@mcp.tool()
async def spawn_motorcycle(query: str, count: int = 1) -> str:
    """（实际功能：生成摩托车）
    
    支持类型: ninja(Kawasaki Ninja), yzf(Yamaha YZF), low_rider(Harley-Davidson Low Rider)
    """
    return await spawn_motorcycle_impl(query, count=count)

@mcp.tool()
async def spawn_prop(query: str, count: int = 1) -> str:
    """（实际功能：生成道具/警示牌）
    
    支持类型: cone(施工锥), barrier(路障), warning(三角警示牌/交通警示牌)
    """
    return await spawn_prop_impl(query, count=count)

@mcp.tool()
async def spawn_overturned_vehicle(vehicle_type: str = 'model3') -> str:
    """（实际功能：生成仰翻的车辆）
    
    支持类型: model3(特斯拉Model3), mustang(福特野马)等
    """
    return await spawn_overturned_vehicle_impl(vehicle_type)

@mcp.tool()
async def set_weather(weather_type: str) -> str:
    """设置仿真天气环境。weather_type 支持:
        clear(晴天),
        rain(雨天),
        fog(雾天),
        snow(雪天),
        night(夜晚/弱光)"""
    return await set_weather_impl(weather_type)


@mcp.tool()
async def get_traffic_lights(query: str, user_type: Optional[str] = None) -> str:
    """（实际功能：获取交通灯）"""
    return await get_traffic_lights_impl(query)


@mcp.tool()
async def cleanup_scene(language: Optional[str] = None, period: str = "daily") -> str:
    """（实际功能：清理环境）"""
    return await cleanup_scene_impl()


@mcp.tool()
async def spawn_pedestrian(query: str, count: int = 1, speed: float = None) -> str:
    """（实际功能：生成行人）"""
    return await spawn_pedestrian_impl(query, count=count, speed=speed)


@mcp.tool()
async def setup_autopilot(enable: bool = True, radius: float = 0.0) -> str:
    """（实际功能：设置车辆自动驾驶）"""
    return await setup_autopilot_impl(enable, radius=radius)


@mcp.tool()
async def setup_pedestrian_movement(enable: bool = True, radius: float = 0.0) -> str:
    """（实际功能：设置行人自动移动）"""
    return await setup_pedestrian_movement_impl(enable, radius=radius)


@mcp.tool()
async def switch_view(view_mode: str = "third_person", target_actor_id: int = None) -> str:
    """（实际功能：切换视角）"""
    return await switch_view_impl(view_mode, target_actor_id)


@mcp.tool()
async def start_recording(fps: int = 30) -> str:
    """（实际功能：开始录制视频）"""
    return await start_recording_impl(fps)


@mcp.tool()
async def stop_recording() -> str:
    """（实际功能：停止录制视频）"""
    return await stop_recording_impl()


@mcp.tool()
async def scenario_highway_ramp(ramp_type: str = "on", vehicle_count: int = 4,
                                map_name: Optional[str] = None) -> str:
    """（实际功能：高速-进出匝道场景）
    在高速公路主路与匝道的汇流/分流点自动布设车辆。
    ramp_type: "on"(匝道汇入) / "off"(主路驶出匝道)
    """
    return await scenario_highway_ramp_impl(ramp_type, vehicle_count, map_name)


@mcp.tool()
async def scenario_lane_merge(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：城市-车道合并场景）
    自动寻找车道消失（车道数减少）位置，在消失车道和延续车道布设车辆演示汇流。
    """
    return await scenario_lane_merge_impl(vehicle_count, map_name)


@mcp.tool()
async def scenario_diverge_merge(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：城市-分合流路口场景）
    找到多臂路口，在各进口臂布设车辆，经路口分流/合流。
    """
    return await scenario_diverge_merge_impl(vehicle_count, map_name)


@mcp.tool()
async def scenario_side_road(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：城市-辅路场景）
    自动寻找与主路平行的辅路，在主路和辅路上同时布设车辆。
    """
    return await scenario_side_road_impl(vehicle_count, map_name)



# ============ AI助手类（集成Deepseek AI） ============

class FastMCPGitHubAssistant:
    """FastMCP GitHub AI助手 - 集成Deepseek AI与FastMCP工具"""

    VEHICLE_TYPES = {
        "model3": "Tesla Model 3",
        "a2": "Audi A2",
        "etron": "Audi e-tron",
        "tt": "Audi TT",
        "grandtourer": "BMW Grand Tourer",
        "i8": "BMW i8",
        "mini": "BMW Mini",
        "impala": "Chevrolet Impala",
        "c3": "Citroen C3",
        "charger_police": "Dodge Charger Police",
        "charger2020": "Dodge Charger 2020",
        "mustang": "Ford Mustang",
        "crown": "Ford Crown",
        "wrangler_rubicon": "Jeep Wrangler Rubicon",
        "mkz_2017": "Lincoln MKZ 2017",
        "mkz_2020": "Lincoln MKZ 2020",
        "benz_coupe": "Mercedes-Benz Coupe",
        "cabrio": "Mercedes-Benz Cabrio",
        "ccc": "Mercedes-Benz CCC",
        "cooper_s": "Mini Cooper S",
        "micra": "Nissan Micra",
        "patrol": "Nissan Patrol",
        "leon": "Seat Leon",
        "t2": "Volkswagen T2",
        "t3": "Volkswagen T3",
        "crossbike": "BH Crossbike",
        "century": "Diamondback Century",
        "omafiets": "Gazelle Omafiets"
    }

    VEHICLE_TYPE_MAP = {
        "特斯拉": "model3",
        "特斯拉model3": "model3",
        "model3": "model3",
        "奥迪": "a2",
        "奥迪a2": "a2",
        "a2": "a2",
        "奥迪etron": "etron",
        "etron": "etron",
        "奥迪tt": "tt",
        "tt": "tt",
        "宝马": "grandtourer",
        "宝马grandtourer": "grandtourer",
        "grandtourer": "grandtourer",
        "宝马i8": "i8",
        "i8": "i8",
        "宝马mini": "mini",
        "mini": "mini",
        "雪佛兰": "impala",
        "雪佛兰impala": "impala",
        "impala": "impala",
        "雪铁龙": "c3",
        "雪铁龙c3": "c3",
        "c3": "c3",
        "道奇": "charger2020",
        "道奇警车": "charger_police",
        "charger_police": "charger_police",
        "道奇charger": "charger2020",
        "charger2020": "charger2020",
        "福特": "mustang",
        "福特野马": "mustang",
        "野马": "mustang",
        "mustang": "mustang",
        "福特crown": "crown",
        "crown": "crown",
        "吉普": "wrangler_rubicon",
        "吉普牧马人": "wrangler_rubicon",
        "牧马人": "wrangler_rubicon",
        "wrangler_rubicon": "wrangler_rubicon",
        "林肯": "mkz_2020",
        "林肯mkz2017": "mkz_2017",
        "mkz_2017": "mkz_2017",
        "林肯mkz2020": "mkz_2020",
        "mkz_2020": "mkz_2020",
        "奔驰": "benz_coupe",
        "奔驰轿跑": "benz_coupe",
        "benz_coupe": "benz_coupe",
        "奔驰敞篷": "cabrio",
        "cabrio": "cabrio",
        "奔驰ccc": "ccc",
        "ccc": "ccc",
        "迷你": "cooper_s",
        "迷你cooper": "cooper_s",
        "cooper_s": "cooper_s",
        "日产": "patrol",
        "日产micra": "micra",
        "micra": "micra",
        "日产patrol": "patrol",
        "patrol": "patrol",
        "西雅特": "leon",
        "西雅特leon": "leon",
        "leon": "leon",
        "大众": "t2",
        "大众t2": "t2",
        "t2": "t2",
        "大众t3": "t3",
        "t3": "t3",
        "自行车": "crossbike",
        "单车": "crossbike",
        "山地自行车": "crossbike",
        "crossbike": "crossbike",
        "公路自行车": "century",
        "century": "century",
        "荷兰自行车": "omafiets",
        "omafiets": "omafiets"
    }

    PEDESTRIAN_TYPES = {
        "pedestrian": "普通行人",
        "elderly": "老年人",
        "child": "儿童",
        "police": "警察",
        "business": "商务人士",
        "jogger": "慢跑者"
    }

    PEDESTRIAN_TYPE_MAP = {
        "普通行人": "pedestrian",
        "行人": "pedestrian",
        "人": "pedestrian",
        "老年人": "elderly",
        "老人": "elderly",
        "儿童": "child",
        "小孩": "child",
        "孩子": "child",
        "警察": "police",
        "警官": "police",
        "商务人士": "business",
        "商人": "business",
        "白领": "business",
        "慢跑者": "jogger",
        "跑步者": "jogger",
        "跑步的人": "jogger"
    }

    def __init__(self):
        # 将FastMCP工具转换为标准MCP工具格式供AI使用
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "connect_carla",
                    "description": "连接CARLA服务器",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "host": {"type": "string", "description": "CARLA服务器地址", "default": "localhost"},
                            "port": {"type": "integer", "description": "CARLA服务器端口", "default": 2000}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_vehicle",
                    "description": "生成指定类型和数量的车辆。支持类型: model3(Tesla), a2/etron/tt(Audi), grandtourer/i8/mini(BMW), impala(Chevrolet), c3(Citroen), charger_police/charger2020(Dodge), mustang/crown(Ford), wrangler_rubicon(Jeep), mkz_2017/mkz_2020(Lincoln), benz_coupe/cabrio/ccc(Mercedes), cooper_s(Mini), micra/patrol(Nissan), leon(Seat), t2/t3(Volkswagen)。数据量: 取决于地图生成点数量，通常支持10-100+辆车",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "车辆型号，如model3, mustang, a2等", "enum": ["model3", "a2", "etron", "tt", "grandtourer", "i8", "mini", "impala", "c3", "charger_police", "charger2020", "mustang", "crown", "wrangler_rubicon", "mkz_2017", "mkz_2020", "benz_coupe", "cabrio", "ccc", "cooper_s", "micra", "patrol", "leon", "t2", "t3"]},
                            "count": {"type": "integer", "description": "生成车辆数量，默认为1", "default": 1}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_bicycle",
                    "description": "生成自行车。支持类型: crossbike(BH Crossbike), century(Diamondback Century), omafiets(Gazelle Omafiets)。数据量: 取决于地图生成点数量，通常支持1-20辆",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "自行车型号，如crossbike, century, omafiets", "enum": ["crossbike", "century", "omafiets"]},
                            "count": {"type": "integer", "description": "生成自行车数量，默认为1", "default": 1}
                        },
                        "required": ["query"]
                    }
                }
            },
                        {
                "type": "function",
                "function": {
                    "name": "spawn_motorcycle",
                    "description": "生成摩托车。支持类型: ninja(Kawasaki Ninja), yzf(Yamaha YZF), low_rider(Harley-Davidson Low Rider)。数据量: 取决于地图生成点数量，通常支持1-20辆",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "摩托车型号，如ninja, yzf, low_rider", "enum": ["ninja", "yzf", "low_rider"]},
                            "count": {"type": "integer", "description": "生成摩托车数量，默认为1", "default": 1}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_prop",
                    "description": "生成静态道具/警示牌，可指定放在某个actor（如仰翻车辆）后方。支持类型: cone(施工锥), barrier(路障), warning(三角警示牌)。数据量: 通常支持1-50个",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "道具类型，如cone, barrier, warning", "enum": ["cone", "barrier", "warning"]},
                            "count": {"type": "integer", "description": "生成道具数量，默认为1", "default": 1},
                            "target_id": {"type": "integer", "description": "目标actor ID，道具将放置在该目标后方5米处。如仰翻车辆的ID", "default": None}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_overturned_vehicle",
                    "description": "生成仰翻/侧翻的车辆，用于模拟事故场景。支持类型: model3(特斯拉Model3), mustang(福特野马), a2(奥迪A2)",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_type": {"type": "string", "description": "车辆类型，如model3, mustang, a2", "enum": ["model3", "mustang", "a2"], "default": "model3"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_weather",
                    "description": "设置天气（clear/rain/fog）",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "weather_type": {"type": "string", "enum": ["clear", "rain", "fog", "snow", "night"]}
                        },
                        "required": ["weather_type"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_traffic_lights",
                    "description": "获取交通灯状态",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "固定值traffic"}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "cleanup_scene",
                    "description": "清理仿真环境",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_pedestrian",
                    "description": "生成指定类型和数量的行人。支持类型: pedestrian(普通行人), elderly(老年人), child(儿童), police(警察), business(商务人士), jogger(慢跑者)。数据量: 取决于地图大小，通常支持10-100+个行人",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "行人类型，如pedestrian, elderly, child等", "enum": ["pedestrian", "elderly", "child", "police", "business", "jogger"]},
                            "count": {"type": "integer", "description": "生成行人数量，默认为1", "default": 1},
                            "speed": {"type": "number", "description": "行人移动速度（m/s），默认根据类型自动设置：普通行人1.4，老年人1.0，慢跑者2.8"}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "setup_autopilot",
                    "description": "设置车辆自动驾驶模式，可指定范围半径",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enable": {"type": "boolean", "description": "是否启用自动驾驶，默认为true", "default": True},
                            "radius": {"type": "number", "description": "自动驾驶范围半径（米），0表示全图，默认为0", "default": 0.0}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "setup_pedestrian_movement",
                    "description": "设置行人自动移动，可指定范围半径",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enable": {"type": "boolean", "description": "是否启用行人移动，默认为true", "default": True},
                            "radius": {"type": "number", "description": "移动范围半径（米），0表示全图，默认为0", "default": 0.0}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "switch_view",
                    "description": "切换视角模式。支持 third_person(第三人称跟随视角), first_person(第一人称视角), overhead(俯视/鸟瞰视角), free(自由/观察者视角)。切换视角时会自动将观察相机移动到对应位置",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "view_mode": {"type": "string", "description": "视角模式", "enum": ["third_person", "first_person", "overhead", "free"], "default": "third_person"},
                            "target_actor_id": {"type": "integer", "description": "目标actor ID，如果不指定则自动选择最新生成的车辆或行人", "default": None}
                        },
                        "required": ["view_mode"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "start_recording",
                    "description": "开始录制视频。录制的是当前窗口视角的内容，录制过程中可以自由切换视角",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "fps": {"type": "integer", "description": "帧率，默认30", "default": 30}
                        },
                        "required": []
                    }
                }
            },
             {
                "type": "function",
                "function": {
                    "name": "stop_recording",
                    "description": "停止视频录制并保存视频文件。视频将保存到recordings目录下",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            },
             {
                "type": "function",
                "function": {
                    "name": "generate_sumo_network",
                    "description": "生成 SUMO 路网和车流。当用户提到'路网'、'网格'、'SUMO'时，必须使用此工具。不要将其与 CARLA 车辆生成混淆。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "grid_x": {"type": "integer", "description": "X方向网格数，默认3"},
                            "grid_y": {"type": "integer", "description": "Y方向网格数，默认3"},
                            "duration": {"type": "integer", "description": "仿真时长（秒），默认200"},
                            "rate": {"type": "number", "description": "发车间隔（秒/辆），默认2.0"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "generate_openscenario",
                    "description": "生成 OpenSCENARIO 场景文件。参数：xodr_filename(OpenDRIVE文件名，留空则使用内建直路), scenario_name(场景名称), duration(仿真秒数), vehicle_speed(车辆速度m/s)。示例：'生成一个场景，基于 web_generated.xodr，车以10m/s行驶30秒'",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "xodr_filename": {"type": "string", "description": "OpenDRIVE文件名，默认空字符串（使用内建直路）"},
                            "scenario_name": {"type": "string", "description": "场景名称，默认my_scenario"},
                            "duration": {"type": "number", "description": "仿真时长（秒），默认30"},
                            "vehicle_speed": {"type": "number", "description": "车辆速度（m/s），默认10"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_walker",
                    "description": "控制行人停止或恢复移动。支持 stop(停止指定行人)、resume(恢复指定行人)、stop_all(停止所有行人)、resume_all(恢复所有行人)",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string", "enum": ["stop", "resume", "stop_all", "resume_all"], "description": "操作类型"},
                            "walker_id": {"type": "integer", "description": "行人ID，stop/resume时需要"}
                        },
                        "required": ["action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_vehicle_param",
                    "description": "参数化生成车辆，支持参照物/距离/角度/速度控制。当用户要求精确控制生成位置时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "count": {"type": "integer", "description": "生成数量", "default": 1},
                            "blueprint_filter": {"type": "string", "description": "蓝图过滤，如 vehicle.tesla.model3", "default": "vehicle.*"},
                            "autopilot": {"type": "boolean", "description": "是否开启自动驾驶", "default": True},
                            "reference_id": {"type": "integer", "description": "参照物actor ID，None则使用地图spawn point", "default": None},
                            "relative_distance": {"type": "number", "description": "相对参照物的距离（米）", "default": 10.0},
                            "relative_angle": {"type": "number", "description": "相对参照物的角度（度，0=正前方）", "default": 0.0},
                            "initial_speed": {"type": "number", "description": "初始速度（m/s）", "default": 0.0}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_highway_ramp",
                    "description": "高速-进出匝道场景：在高速公路主路与匝道的汇流/分流点自动布设车辆并开启自动驾驶。推荐地图Town04/Town06。当用户提到'匝道'、'高速进出匝道'、'汇入匝道'、'驶出匝道'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "ramp_type": {"type": "string", "enum": ["on", "off"], "description": "on=匝道汇入(默认), off=主路驶出匝道", "default": "on"},
                            "vehicle_count": {"type": "integer", "description": "总车辆数（主路+匝道），默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名如Town04", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_lane_merge",
                    "description": "城市-车道合并场景：自动寻找车道消失（车道数减少）位置，在消失车道与延续车道布设车辆演示汇流。当用户提到'车道合并'、'车道减少'、'汇流'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_diverge_merge",
                    "description": "城市-分合流路口场景：找到多臂路口，在各进口臂布设车辆，经路口分流/合流。当用户提到'分合流路口'、'分流'、'路口合流'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_side_road",
                    "description": "城市-辅路场景：自动寻找与主路平行的辅路，在主路和辅路上同时布设车辆。当用户提到'辅路'、'辅道'、'侧路'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            }
        ] 

    def process_markdown(self, text):
        """在Python端处理Markdown格式"""
        result = text

        # 处理标题
        result = re.sub(r'^### (.+)$', r'<h3><strong>\1</strong></h3>', result, flags=re.MULTILINE)
        result = re.sub(r'^## (.+)$', r'<h2><strong>\1</strong></h2>', result, flags=re.MULTILINE)
        result = re.sub(r'^# (.+)$', r'<h1><strong>\1</strong></h1>', result, flags=re.MULTILINE)

        # 处理粗体链接 **[text](url)**
        result = re.sub(r'\*\*\[([^\]]+)\]\(([^)]+)\)\*\*', r'<strong><a href="\2" target="_blank">\1</a></strong>',
                        result)

        # 处理普通链接 [text](url)
        result = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2" target="_blank">\1</a>', result)

        # 处理粗体文本 **text**
        result = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', result)

        # 处理换行
        result = result.replace('\n', '<br>')

        return result

    async def call_deepseek_with_tools(self, messages):
        """调用Deepseek API，包含FastMCP工具定义"""
        headers = config.get_deepseek_headers()

        data = {
            "model": "deepseek-chat",
            "messages": messages,
            "tools": self.tools,
            "tool_choice": "auto",
            "max_tokens": 2000,
            "temperature": 0.7
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(config.DEEPSEEK_API_URL, headers=headers, json=data) as response:
                if response.status == 200:
                    return await response.json()
                else:
                    error_text = await response.text()
                    raise Exception(f"Deepseek API调用失败: {response.status} - {error_text}")

    async def execute_fastmcp_tool_call(self, tool_call):
        """执行FastMCP工具调用 - 桥接到FastMCP装饰器函数"""
        function_name = tool_call["function"]["name"]
        arguments = json.loads(tool_call["function"]["arguments"])

        app_logger.info(f"🔧 执行FastMCP工具: {function_name}")
        app_logger.info(f"📝 参数: {arguments}")

        try:
            # 调用实际的工具实现函数（避免FastMCP装饰器问题）
            if function_name == "connect_carla":
                host = arguments.get("host", "localhost")
                port = arguments.get("port", 2000)
                result = await carla_client.connect(host, port)
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_vehicle":
                app_logger.info(f"spawn_vehicle参数详情: {arguments}")
                result = await spawn_vehicle_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1))
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_bicycle":
                app_logger.info(f"spawn_bicycle参数详情: {arguments}")
                result = await spawn_bicycle_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1))
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_motorcycle":
                app_logger.info(f"spawn_motorcycle参数详情: {arguments}")
                result = await spawn_motorcycle_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1))
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_prop":
                app_logger.info(f"spawn_prop参数详情: {arguments}")
                result = await spawn_prop_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1)),
                    target_id=arguments.get("target_id")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_overturned_vehicle":
                app_logger.info(f"spawn_overturned_vehicle参数详情: {arguments}")
                result = await spawn_overturned_vehicle_impl(
                    vehicle_type=arguments.get("vehicle_type", "model3")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_pedestrian":
                app_logger.info(f"spawn_pedestrian参数详情: {arguments}")
                result = await spawn_pedestrian_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1)),
                    speed=arguments.get("speed")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_weather":
                result = await carla_client.set_weather(arguments["weather_type"])
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_synchronous_mode":
                result = await carla_client.set_synchronous_mode(
                    arguments.get("enabled", True),
                    arguments.get("fixed_delta_seconds", 0.05)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "setup_autopilot":
                result = await carla_client.setup_autopilot(
                    arguments.get("enable", True),
                    arguments.get("radius", 0.0)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "switch_view_mode":
                result = await carla_client.switch_view_mode(
                    arguments.get("view_mode", "third_person"),
                    arguments.get("target_id")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "start_recording":
                result = await carla_client.start_recording(
                    arguments.get("filename", "simulation_recording"),
                    arguments.get("duration", 30)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "stop_recording":
                result = await carla_client.stop_recording()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_third_person_view":
                result = await carla_client.set_third_person_view(arguments.get("target_id"))
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_first_person_view":
                result = await carla_client.set_first_person_view(arguments.get("target_id"))
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_overhead_view":
                result = await carla_client.set_overhead_view(arguments.get("target_id"))
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_free_view":
                result = await carla_client.set_free_view()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "start_view_follow":
                result = await carla_client.start_view_follow(
                    arguments.get("view_mode", "third_person"),
                    arguments.get("target_id")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "stop_view_follow":
                result = await carla_client.stop_view_follow()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_sumo_grid_network":
                result = await spawn_sumo_grid_network_impl(
                    grid_size=arguments.get("grid_size", 3),
                    simulation_time=arguments.get("simulation_time", 200),
                    vehicle_spawn_interval=arguments.get("vehicle_spawn_interval", 2)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "generate_openscenario":
                result = await generate_openscenario_impl(
                    xodr_file=arguments.get("xodr_file", "web_generated.xodr"),
                    vehicle_speed=arguments.get("vehicle_speed", 10.0),
                    duration=arguments.get("duration", 30.0)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_highway_ramp":
                result = await scenario_highway_ramp_impl(
                    ramp_type=arguments.get("ramp_type", "on"),
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_lane_merge":
                result = await scenario_lane_merge_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_diverge_merge":
                result = await scenario_diverge_merge_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_side_road":
                result = await scenario_side_road_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "cleanup_scene":
                result = await cleanup_scene_impl()
                return {
                    "success": True,
                    "data": result
                }

            else:
                return {
                    "success": False,
                    "error": f"未知的工具: {function_name}"
                }

        except Exception as e:
            app_logger.error(f"❌ FastMCP工具执行失败: {str(e)}")
            return {
                "success": False,
                "error": str(e)
            }

    # 定义车辆和行人的类型信息
        VEHICLE_TYPES = {
        "model3": "Tesla Model 3",
        "a2": "Audi A2",
        "etron": "Audi e-tron",
        "tt": "Audi TT",
        "grandtourer": "BMW Grand Tourer",
        "i8": "BMW i8",
        "mini": "BMW Mini",
        "impala": "Chevrolet Impala",
        "c3": "Citroen C3",
        "charger_police": "Dodge Charger Police",
        "charger2020": "Dodge Charger 2020",
        "mustang": "Ford Mustang",
        "crown": "Ford Crown",
        "wrangler_rubicon": "Jeep Wrangler Rubicon",
        "mkz_2017": "Lincoln MKZ 2017",
        "mkz_2020": "Lincoln MKZ 2020",
        "benz_coupe": "Mercedes-Benz Coupe",
        "cabrio": "Mercedes-Benz Cabrio",
        "ccc": "Mercedes-Benz CCC",
        "cooper_s": "Mini Cooper S",
        "micra": "Nissan Micra",
        "patrol": "Nissan Patrol",
        "leon": "Seat Leon",
        "t2": "Volkswagen T2",
        "t3": "Volkswagen T3",
        "crossbike": "BH Crossbike",
        "century": "Diamondback Century",
        "omafiets": "Gazelle Omafiets"
    }

    # 车辆中文到英文的映射
    VEHICLE_TYPE_MAP = {
        "特斯拉": "model3",
        "特斯拉model3": "model3",
        "model3": "model3",
        "奥迪": "a2",
        "奥迪a2": "a2",
        "a2": "a2",
        "奥迪etron": "etron",
        "etron": "etron",
        "奥迪tt": "tt",
        "tt": "tt",
        "宝马": "grandtourer",
        "宝马grandtourer": "grandtourer",
        "grandtourer": "grandtourer",
        "宝马i8": "i8",
        "i8": "i8",
        "宝马mini": "mini",
        "mini": "mini",
        "雪佛兰": "impala",
        "雪佛兰impala": "impala",
        "impala": "impala",
        "雪铁龙": "c3",
        "雪铁龙c3": "c3",
        "c3": "c3",
        "道奇": "charger2020",
        "道奇警车": "charger_police",
        "警车": "charger_police",
        "charger_police": "charger_police",
        "道奇charger": "charger2020",
        "charger2020": "charger2020",
        "福特": "mustang",
        "福特野马": "mustang",
        "野马": "mustang",
        "mustang": "mustang",
        "福特crown": "crown",
        "crown": "crown",
        "吉普": "wrangler_rubicon",
        "吉普牧马人": "wrangler_rubicon",
        "牧马人": "wrangler_rubicon",
        "wrangler_rubicon": "wrangler_rubicon",
        "林肯": "mkz_2020",
        "林肯mkz2017": "mkz_2017",
        "mkz_2017": "mkz_2017",
        "林肯mkz2020": "mkz_2020",
        "mkz_2020": "mkz_2020",
        "奔驰": "benz_coupe",
        "奔驰轿跑": "benz_coupe",
        "benz_coupe": "benz_coupe",
        "奔驰敞篷": "cabrio",
        "cabrio": "cabrio",
        "奔驰ccc": "ccc",
        "ccc": "ccc",
        "迷你": "cooper_s",
        "迷你cooper": "cooper_s",
        "cooper_s": "cooper_s",
        "日产": "patrol",
        "日产micra": "micra",
        "micra": "micra",
        "日产patrol": "patrol",
        "patrol": "patrol",
        "西雅特": "leon",
        "西雅特leon": "leon",
        "leon": "leon",
        "大众": "t2",
        "大众t2": "t2",
        "t2": "t2",
        "大众t3": "t3",
        "t3": "t3",
        "自行车": "crossbike",
        "单车": "crossbike",
        "山地自行车": "crossbike",
        "crossbike": "crossbike",
        "公路自行车": "century",
        "century": "century",
        "荷兰自行车": "omafiets",
        "omafiets": "omafiets"
    }
    PEDESTRIAN_TYPES = {
        "pedestrian": "普通行人",
        "elderly": "老年人",
        "child": "儿童",
        "police": "警察",
        "business": "商务人士",
        "jogger": "慢跑者"
    }

    # 中文到英文的映射
    PEDESTRIAN_TYPE_MAP = {
        "普通行人": "pedestrian",
        "行人": "pedestrian",
        "人": "pedestrian",
        "老年人": "elderly",
        "老人": "elderly",
        "儿童": "child",
        "小孩": "child",
        "孩子": "child",
        "警察": "police",
        "警官": "police",
        "商务人士": "business",
        "商人": "business",
        "白领": "business",
        "慢跑者": "jogger",
        "跑步者": "jogger",
        "跑步的人": "jogger"
    }
    def _check_spawn_intent(self, message):
        """检测用户是否有生成车辆、自行车或行人的意图，但缺少必要参数"""
        message = message.lower()

        # 首先排除 SUMO 路网生成相关的指令
        sumo_keywords = ['路网', '网格', 'sumo', '仿真', '交通', '场景']
        for kw in sumo_keywords:
            if kw in message:
                return {
                    'needs_vehicle_type': False,
                    'needs_vehicle_count': False,
                    'needs_pedestrian_type': False,
                    'needs_pedestrian_count': False,
                    'needs_bicycle_type': False,
                    'needs_bicycle_count': False,
                    'is_ambiguous': False
                }

        # 排除视角控制相关的指令
        view_keywords = ['视角', '切换', '人称', '俯视', '鸟瞰', '自由视角', '录制', '录像', '视频']
        if any(kw in message for kw in view_keywords):
            return {
                'needs_vehicle_type': False,
                'needs_vehicle_count': False,
                'needs_pedestrian_type': False,
                'needs_pedestrian_count': False,
                'needs_bicycle_type': False,
                'needs_bicycle_count': False,
                'is_ambiguous': False
            }

        # 排除自动驾驶相关指令（不是生成车辆）
        autopilot_keywords = ['自动驾驶', '车辆运行', '车自己开', '开启自动驾驶', '让车', '让车辆']
        if any(kw in message for kw in autopilot_keywords):
            return {
                'needs_vehicle_type': False,
                'needs_vehicle_count': False,
                'needs_pedestrian_type': False,
                'needs_pedestrian_count': False,
                'needs_bicycle_type': False,
                'needs_bicycle_count': False,
                'is_ambiguous': False
            }

        import re
        has_count = bool(re.search(r'\d+\s*[辆个]', message))

        # ===== 先初始化 result =====
        result = {
            'needs_vehicle_type': False,
            'needs_vehicle_count': False,
            'needs_pedestrian_type': False,
            'needs_pedestrian_count': False,
            'needs_bicycle_type': False,
            'needs_bicycle_count': False,
            'is_ambiguous': False
        }

        # 1️⃣ 仰翻车辆检测（最优先，默认 model3，不提示用户）
        overturned_keywords = ['仰翻', '侧翻', '翻车', '事故车', '翻了', '底朝天']
        has_overturned_request = any(kw in message for kw in overturned_keywords)
        if has_overturned_request:
            return result  # 直接放行，AI 会默认用 model3

        # 2️⃣ 摩托车检测（在普通车辆之前，因为"摩托车"含"车"字）
        motorcycle_keywords = ['摩托车', '生成摩托车', '来辆摩托车', '添加摩托车']
        has_motorcycle_request = any(kw in message for kw in motorcycle_keywords)
        if has_motorcycle_request:
            return result  # 直接放行，AI 会默认用 ninja

        # 3️⃣ 道具检测
        prop_keywords = ['施工锥', '路障', '警示牌', '三角警示牌', '生成道具']
        has_prop_request = any(kw in message for kw in prop_keywords)
        if has_prop_request:
            return result  # 直接放行，AI 会默认用 cone

        # 4️⃣ 自行车检测
        bicycle_keywords = ['自行车', '单车', '生成自行车', '来辆自行车', '添加自行车']
        has_bicycle_request = any(kw in message for kw in bicycle_keywords)
        has_bicycle_type = any(btype in message for btype in ['山地', '公路', '荷兰', 'crossbike', 'century', 'omafiets'])
        if has_bicycle_request and not has_bicycle_type:
            result['needs_bicycle_type'] = True
            result['needs_bicycle_count'] = not has_count
            result['is_ambiguous'] = True
            return result

        # 5️⃣ 车辆检测（最后，避免被"摩托车"等含"车"字的词误触发）
        vehicle_keywords = ['车辆', '汽车', '生成车', '创建车', '来车', '加车', '添加车辆']
        # 排除已处理的类型，避免"摩托车"触发
        if has_motorcycle_request or has_prop_request or has_overturned_request:
            is_vehicle_request = False
        else:
            is_vehicle_request = any(kw in message for kw in vehicle_keywords) or ('车' in message and '摩托车' not in message and '自行车' not in message)
        
        has_vehicle_type = any(vtype in message for vtype in self.VEHICLE_TYPES.keys()) or \
                           any(vname in message for vname in self.VEHICLE_TYPE_MAP.keys())

        # 6️⃣ 行人检测
        pedestrian_keywords = ['行人', '生成行人', '创建行人', '添加行人', '路人']
        person_spawn_verbs = ['生成', '创建', '来', '加', '添加', '放', 'spawn']
        specific_pedestrian_keywords = set(self.PEDESTRIAN_TYPES.keys()) | set(self.PEDESTRIAN_TYPE_MAP.keys()) - {"行人", "人"}
        has_pedestrian_type = any(ptype in message for ptype in specific_pedestrian_keywords)
        is_pedestrian_request = any(kw in message for kw in pedestrian_keywords)
        if not is_pedestrian_request and '人' in message:
            has_spawn_verb = any(verb in message for verb in person_spawn_verbs)
            if has_spawn_verb and '人称' not in message:
                is_pedestrian_request = True

        # 设置结果
        if is_vehicle_request and not has_vehicle_type:
            result['needs_vehicle_type'] = True
            result['needs_vehicle_count'] = not has_count
            result['is_ambiguous'] = True

        if is_pedestrian_request and not has_pedestrian_type:
            result['needs_pedestrian_type'] = True
            result['needs_pedestrian_count'] = not has_count
            result['is_ambiguous'] = True

        return result

    def _generate_spawn_prompt(self, check_result):
        """生成参数询问提示"""
        prompt_parts = []

        if check_result.get('needs_vehicle_type'):
            vehicle_list = "\n".join([f"  • {name} ({key})" for key, name in self.VEHICLE_TYPES.items() if key not in ['crossbike', 'century', 'omafiets']])
            prompt_parts.append(f"🚗 **可用车辆类型：**\n{vehicle_list}")

        if check_result.get('needs_vehicle_count'):
            prompt_parts.append("🚗 **车辆数量：** 支持生成 1-100+ 辆车（取决于地图可用生成点数量）")

        if check_result.get('needs_bicycle_type'):
            prompt_parts.append("🚲 **可用自行车类型：**\n  • 山地自行车 (crossbike)\n  • 公路自行车 (century)\n  • 荷兰自行车 (omafiets)")

        if check_result.get('needs_motorcycle_type') or check_result.get('needs_motorcycle_count'):
                prompt_parts.append('  • "生成3辆川崎忍者摩托车"')
                prompt_parts.append('  • "来5辆雅马哈YZF"')
                prompt_parts.append('  • "生成2辆哈雷"')

        if check_result.get('needs_prop_type') or check_result.get('needs_prop_count'):
                prompt_parts.append('  • "生成3个施工锥"')
                prompt_parts.append('  • "来5个三角警示牌"')
                prompt_parts.append('  • "生成2个路障"')

        if check_result.get('needs_bicycle_count'):
            prompt_parts.append("🚲 **自行车数量：** 支持生成 1-20 辆")

        if check_result.get('needs_pedestrian_type'):
            pedestrian_list = "\n".join([f"  • {name} ({key})" for key, name in self.PEDESTRIAN_TYPES.items()])
            prompt_parts.append(f"🚶 **可用行人类型：**\n{pedestrian_list}")

        if check_result.get('needs_pedestrian_count'):
            prompt_parts.append("🚶 **行人数量：** 支持生成 1-100+ 个行人（取决于地图大小）")

        if prompt_parts:
            prompt_parts.insert(0, "请提供以下信息以完成生成：\n")
            prompt_parts.append("\n💡 **示例指令：**")
            
            if check_result.get('needs_vehicle_type') or check_result.get('needs_vehicle_count'):
                prompt_parts.append('  • "生成5辆特斯拉"')
                prompt_parts.append('  • "来10辆福特野马"')
                prompt_parts.append('  • "生成3辆宝马"')
                prompt_parts.append('  • "来5辆奔驰"')
            
            if check_result.get('needs_bicycle_type') or check_result.get('needs_bicycle_count'):
                prompt_parts.append('  • "生成3辆山地自行车"')
                prompt_parts.append('  • "来5辆公路自行车"')
                prompt_parts.append('  • "生成2辆荷兰自行车"')
            
            if check_result.get('needs_pedestrian_type') or check_result.get('needs_pedestrian_count'):
                prompt_parts.append('  • "生成3个老年人"')
                prompt_parts.append('  • "来5个警察"')
                prompt_parts.append('  • "生成2个儿童"')
                prompt_parts.append('  • "来10个普通行人"')

        return "\n\n".join(prompt_parts)

    def _check_view_switch_intent(self, message):
        """检测用户是否有切换视角的意图，如果有多个行人/车辆则询问选择

        Returns:
            dict: 包含是否需要询问、视角模式、可用目标列表等信息
        """
        import re
        message = message.lower()

        # 视角相关关键词
        view_keywords = ['视角', '人称', '俯视', '鸟瞰', '自由视角', '旁观者']
        has_view_intent = any(kw in message for kw in view_keywords)

        if not has_view_intent:
            return {'needs_target_selection': False}

        # 检测是否指定了特定的视角模式
        view_mode = None
        if '第一人称' in message or '第一视角' in message or 'first_person' in message:
            view_mode = 'first_person'
        elif '第三人称' in message or '第三视角' in message or 'third_person' in message:
            view_mode = 'third_person'
        elif '俯视' in message or '鸟瞰' in message or 'overhead' in message:
            view_mode = 'overhead'
        elif '自由' in message or 'free' in message:
            view_mode = 'free'
        elif '旁观者' in message or 'bystander' in message:
            view_mode = 'bystander'
        else:
            view_mode = 'third_person'
        # 旁观者视角不需要选择目标
        if view_mode == 'bystander':
            return {'needs_target_selection': False, 'view_mode': 'bystander'}

        # 尝试从消息中提取ID（支持 "ID26", "ID 26", "id26", "id 26" 等格式）
        target_id = None
        id_patterns = [
            r'id\s*(\d+)',  # ID 26, id26, ID26
            r'[^\d](\d+)$',  # 以数字结尾
            r'\s(\d+)\s',  # 中间有数字
        ]
        for pattern in id_patterns:
            match = re.search(pattern, message, re.IGNORECASE)
            if match:
                target_id = int(match.group(1))
                break

        # 获取当前所有行人和车辆
        pedestrians = carla_client.get_all_pedestrians()
        vehicles = carla_client.get_all_vehicles()

        all_targets = []
        for p in pedestrians:
            all_targets.append({'id': p['id'], 'type': p['type_name'], 'category': '行人'})
        for v in vehicles:
            all_targets.append({'id': v['id'], 'type': v['type_name'], 'category': '车辆'})

        # 如果提取到了ID，验证该ID是否存在
        if target_id is not None:
            target_exists = any(t['id'] == target_id for t in all_targets)
            if target_exists:
                return {
                    'needs_target_selection': False,
                    'view_mode': view_mode,
                    'target_id': target_id
                }

        # 如果只有一个目标，直接使用
        if len(all_targets) == 1:
            return {
                'needs_target_selection': False,
                'view_mode': view_mode,
                'target_id': all_targets[0]['id']
            }

        # 如果有多个目标，需要询问
        if len(all_targets) > 1:
            return {
                'needs_target_selection': True,
                'view_mode': view_mode,
                'targets': all_targets
            }

        # 没有可用的目标
        return {
            'needs_target_selection': False,
            'view_mode': view_mode,
            'no_targets': True
        }

    def _generate_view_selection_prompt(self, view_mode, targets):
        """生成视角目标选择提示"""
        view_mode_names = {
            'first_person': '第一人称视角',
            'third_person': '第三人称视角',
            'overhead': '俯视视角',
            'free': '自由视角'
        }

        prompt_parts = [f"👁️ 请选择要切换到{view_mode_names.get(view_mode, view_mode)}的目标：\n"]

        for i, target in enumerate(targets, 1):
            prompt_parts.append(f"  {i}. ID: {target['id']} - {target['type']} ({target['category']})")

        prompt_parts.append(f"\n💡 **示例指令：**")
        prompt_parts.append(f'  • "切换到{view_mode_names.get(view_mode, view_mode)} ID {targets[0]["id"]}"')
        prompt_parts.append(f'  • "用ID {targets[0]["id"]} 切换{view_mode_names.get(view_mode, view_mode)}"')

        return "\n".join(prompt_parts)

    async def chat(self, user_message):
        """处理聊天请求 - 使用FastMCP工具的AI对话"""

        # 检查是否有生成意图但缺少参数
        spawn_check = self._check_spawn_intent(user_message)
        if spawn_check['is_ambiguous']:
            prompt = self._generate_spawn_prompt(spawn_check)
            return {
                "message": self.process_markdown(prompt),
                "tool_calls": None,
                "conversation": [{"role": "user", "content": user_message}]
            }

        # 检查是否有视角切换意图
        view_check = self._check_view_switch_intent(user_message)
        if view_check.get('needs_target_selection'):
            # 有多个目标且用户没有指定ID，显示选择列表
            prompt = self._generate_view_selection_prompt(view_check['view_mode'], view_check['targets'])
            return {
                "message": self.process_markdown(prompt),
                "tool_calls": None,
                "conversation": [{"role": "user", "content": user_message}]
            }
        elif view_check.get('target_id'):
            # 用户指定了ID或只有一个目标，直接执行视角切换
            result = await switch_view_impl(
                view_mode=view_check['view_mode'],
                target_actor_id=view_check['target_id']
            )
            return {
                "message": self.process_markdown(result),
                "tool_calls": None,
                "conversation": [{"role": "user", "content": user_message}]
            }

        # 初始消息
        messages = [
    {
        "role": "system",
        "content": """## 🚦 关键路由规则（必须严格遵守）

- 如果用户消息中包含 "路网"、"网格"、"SUMO"、"生成路网" 这些词，**必须**调用 `generate_sumo_network` 工具。
- 如果用户消息中包含 "场景"、"OpenSCENARIO"、"生成场景"，**必须**调用 `generate_openscenario` 工具，**不要**将其理解为车辆生成。
- 如果用户消息中包含 "匝道"、"高速进出匝道"、"汇入匝道"、"驶出匝道"，**必须**调用 `scenario_highway_ramp` 工具。
- 如果用户消息中包含 "车道合并"、"车道减少"、"汇流"，**必须**调用 `scenario_lane_merge` 工具。
- 如果用户消息中包含 "分合流路口"、"分流"、"路口合流"，**必须**调用 `scenario_diverge_merge` 工具。
- 如果用户消息中包含 "辅路"、"辅道"，**必须**调用 `scenario_side_road` 工具。
- 如果用户消息中包含 "连接CARLA"、"CARLA服务器"，调用 `connect_carla` 工具。
- **绝对不要**将"路网"或"网格"理解为 CARLA 车辆生成请求。

你是一个GitHub搜索助手，基于FastMCP框架提供服务。你有以下工具可以使用：

CARLA仿真功能：
5. connect_carla - 连接CARLA服务器（默认localhost:2000）
6. spawn_vehicle - 生成车辆，支持参数：query(车型), count(数量)。支持车型：model3(Tesla), a2/etron/tt(Audi), grandtourer/i8/mini(BMW), impala(Chevrolet), c3(Citroen), charger_police/charger2020(Dodge), mustang/crown(Ford), wrangler_rubicon(Jeep), mkz_2017/mkz_2020(Lincoln), benz_coupe/cabrio/ccc(Mercedes), cooper_s(Mini), micra/patrol(Nissan), leon(Seat), t2/t3(Volkswagen)
7. spawn_pedestrian - 生成行人，支持参数：query(类型), count(数量), speed(速度)。支持类型：pedestrian(普通行人), elderly(老年人), child(儿童), police(警察), business(商务人士), jogger(慢跑者)。速度默认值：普通行人1.4m/s，老年人1.0m/s，慢跑者2.8m/s
8. setup_autopilot - 设置车辆自动驾驶，支持参数：enable(是否启用), radius(范围半径)
9. setup_pedestrian_movement - 设置行人自动移动，支持参数：enable(是否启用), radius(范围半径)
10. set_weather - 设置天气（clear/rain/fog）
11. get_traffic_lights - 查看交通灯状态
12. cleanup_scene - 清理仿真场景
13. switch_view - 切换视角模式，支持 third_person(第三人称跟随), first_person(第一人称), overhead(俯视/鸟瞰), free(自由视角), bystander(旁观者视角)
14. start_recording - 开始视频录制，录制当前窗口视角的内容
15. stop_recording - 停止视频录制
16. generate_openscenario - 基于已有的 OpenDRIVE 文件生成 OpenSCENARIO 场景文件。当用户提到"场景"、"OpenSCENARIO"、"生成场景"、"仿真场景"时使用。参数：xodr_filename(OpenDRIVE文件名), scenario_name(场景名称), duration(仿真时长秒), vehicle_speed(车辆速度m/s)
17. scenario_highway_ramp - 高速-进出匝道场景，参数：ramp_type("on"匝道汇入/"off"驶出匝道), vehicle_count(总车辆数，默认4), map_name(可选，推荐Town04/Town06)。当用户提到"匝道"、"高速进出匝道"时使用
18. scenario_lane_merge - 城市-车道合并场景，参数：vehicle_count(默认4), map_name(可选)。当用户提到"车道合并"、"车道减少"、"汇流"时使用
19. scenario_diverge_merge - 城市-分合流路口场景，参数：vehicle_count(默认4), map_name(可选)。当用户提到"分合流路口"、"分流"时使用
20. scenario_side_road - 城市-辅路场景，参数：vehicle_count(默认4), map_name(可选)。当用户提到"辅路"、"辅道"时使用


CARLA相关：
- 当用户提到"连接"、"服务器"、"CARLA"等明确要求连接时，使用connect_carla
- 当用户提到"车辆"、"生成"、"创建汽车"、"车"等，使用spawn_vehicle，count参数默认为1
- 当用户提到"多辆车"、"生成X辆车"、"几辆车"、指定数量（如5辆、10辆），必须设置count参数为对应数字
- 车辆类型支持中文：特斯拉(model3)、奥迪(a2/etron/tt)、宝马(grandtourer/i8/mini)、雪佛兰(impala)、雪铁龙(c3)、道奇(charger_police/charger2020)、福特(mustang/crown)、吉普(wrangler_rubicon)、林肯(mkz_2017/mkz_2020)、奔驰(benz_coupe/cabrio/ccc)、迷你(cooper_s)、日产(micra/patrol)、西雅特(leon)、大众(t2/t3)
- 当用户使用中文车辆类型（如"生成3辆特斯拉"），你需要将中文类型转换为对应的英文类型：model3、a2、etron、tt、grandtourer、i8、mini、impala、c3、charger_police、charger2020、mustang、crown、wrangler_rubicon、mkz_2017、mkz_2020、benz_coupe、cabrio、ccc、cooper_s、micra、patrol、leon、t2、t3
- 当用户提到"自行车"、"单车"、"生成自行车"、"来辆自行车"等，使用spawn_bicycle，count参数默认为1
- 自行车类型支持中文：山地自行车/crossbike(默认)、公路自行车/century、荷兰自行车/omafiets
- 当用户使用中文自行车类型（如"生成3辆山地自行车"），你需要将中文类型转换为对应的英文类型：crossbike、century、omafiets
- 示例指令：
  - "生成一辆自行车" -> spawn_bicycle(query="crossbike", count=1)
  - "生成5辆山地自行车" -> spawn_bicycle(query="crossbike", count=5)
  - "来3辆公路自行车" -> spawn_bicycle(query="century", count=3)
- 当用户说"生成一辆摩托车"没有指定类型时，默认使用 ninja（川崎忍者）
- 当用户说"生成一辆仰翻的车辆"没有指定类型时，默认使用 model3（特斯拉）
- 当用户说"生成道具"没有指定类型时，默认使用 cone（施工锥）
- 当用户提到"摩托车"、"生成摩托车"、"来辆摩托车"等，使用spawn_motorcycle，count参数默认为1
- 摩托车类型支持中文：川崎忍者/ninja(默认)、雅马哈YZF/yzf、哈雷low_rider
- 示例指令：
  - "生成一辆摩托车" -> spawn_motorcycle(query="ninja", count=1)
  - "生成3辆雅马哈" -> spawn_motorcycle(query="yzf", count=3)

- 当用户提到"施工锥"、"路障"、"警示牌"、"三角警示牌"、"生成道具"等，使用spawn_prop，count参数默认为1
- 道具类型支持中文：施工锥/cone(默认)、路障/barrier、警示牌/warning
- 示例指令：
  - "生成3个施工锥" -> spawn_prop(query="cone", count=3)
  - "来5个三角警示牌" -> spawn_prop(query="warning", count=5)
  - "生成路障" -> spawn_prop(query="barrier", count=1)
- 当用户说"在仰翻车辆后方放警示牌"、"在事故车后面放锥桶"时，需要：
  1. 先确认仰翻车辆的ID（如果刚生成，ID会在返回结果中）
  2. 使用 spawn_prop 并传入 target_id=仰翻车辆ID
- 示例指令：
  - "在ID 32的后方放3个施工锥" -> spawn_prop(query="cone", count=3, target_id=32)
  - "在仰翻车辆后面放三角警示牌" -> 先问用户仰翻车辆的ID，或如果刚生成则直接用该ID
  - "给事故车后方放路障" -> spawn_prop(query="barrier", count=2, target_id=事故车ID)

- 当用户提到"仰翻"、"侧翻"、"翻车"、"事故车"、"翻了的特斯拉"等，使用spawn_overturned_vehicle
- 示例指令：
  - "生成一辆仰翻的特斯拉" -> spawn_overturned_vehicle(vehicle_type="model3")
  - "来一辆侧翻的野马" -> spawn_overturned_vehicle(vehicle_type="mustang")

- 当用户提到"薄雾"、"轻雾"、"雾天（轻）"等，使用set_weather，weather_type设为"light_fog"
- 示例指令：
  - "设置薄雾天气" -> set_weather(weather_type="light_fog")
- 当用户提到"行人"、"生成行人"、"创建行人"、"人"等，直接使用spawn_pedestrian，count参数默认为1
- 当用户提到"多个行人"、"生成X个行人"、"几个行人"、指定数量（如5个、10个），必须设置count参数为对应数字
- 行人类型支持中文：普通行人/行人/人、老年人/老人、儿童/小孩/孩子、警察/警官、商务人士/商人/白领、慢跑者/跑步者/跑步的人
- 当用户使用中文行人类型（如"生成5个老年人"），你需要将中文类型转换为对应的英文类型：elderly、child、police、business、jogger、pedestrian
- 当用户提到"自动驾驶"、"车辆运行"、"车自己开"等，使用setup_autopilot
- 当用户提到"行人移动"、"行人走路"、"行人运行"等，使用setup_pedestrian_movement
- 当用户提到"天气"、"下雨"、"晴天"、"雾天"等，使用set_weather
- 当用户提到"交通灯"、"信号灯"、"红绿灯"等，使用get_traffic_lights
- 当用户提到"清理"、"重置"、"清除场景"等，使用cleanup_scene
- 当用户提到"视角"、"切换视角"、"第三人称"、"第一人称"、"俯视"、"鸟瞰"、"自由视角"、"旁观者"等，使用switch_view
  * third_person: 第三人称跟随视角，相机在目标后方跟随
  * first_person: 第一人称视角，模拟驾驶员或行人视角
  * overhead: 俯视/鸟瞰视角，从上方俯瞰场景
  * free: 自由视角/观察者视角，可以自由观察
  * bystander: 旁观者视角，回到默认观察者位置，不跟随任何目标
- 当用户提到"录制"、"录像"、"视频"、"开始录制"、"录屏"等，使用start_recording
  * 录制的是当前窗口视角的内容，与当前看到的画面一致
  * 录制过程中可以自由切换视角，录制不会中断
  * 可以指定帧率，默认30fps
- 当用户提到"停止录制"、"结束录像"、"保存视频"等，使用stop_recording
- 当用户提到"切换到第三人称视角"、"切换到第一人称"等，但没有指定目标ID时：
  * 如果只有一个行人/车辆，系统会自动选择它
  * 如果有多个行人/车辆，系统会询问用户选择哪个目标
  * 用户可以回复"切换到第三人称视角 ID xxx"来指定目标

重要规则：
- 如果用户已经连接过CARLA服务器，不要再重复调用connect_carla
- 当用户明确要求生成行人或车辆时，直接调用对应的生成工具，不要先调用connect_carla
- 只有当用户明确要求连接服务器时，才调用connect_carla

通用策略：
- 首先判断用户意图是GitHub相关还是CARLA仿真相关
- 搜索时使用英文关键词效果更好
- 必须先连接CARLA服务器才能使用CARLA相关功能
- 不要自动连接CARLA服务器，只在用户明确要求时连接
- 可以根据用户需求调用多个工具获得更全面的结果
- 必须先获取数据，再基于实际数据回答用户问题
- 如果没有找到结果，要明确告知用户

用户指令示例：
- "连接carla服务器" -> connect_carla(host="localhost", port=2000)
- "生成一辆model3" -> spawn_vehicle(query="model3", count=1)
- "生成5辆mustang" -> spawn_vehicle(query="mustang", count=5)
- "生成10辆车" -> spawn_vehicle(query="model3", count=10)
- "给我来3辆奥迪a2" -> spawn_vehicle(query="a2", count=3)
- "创建20辆车" -> spawn_vehicle(query="model3", count=20)
- "生成3辆特斯拉" -> spawn_vehicle(query="model3", count=3)
- "生成5辆宝马" -> spawn_vehicle(query="grandtourer", count=5)
- "生成2辆奔驰" -> spawn_vehicle(query="benz_coupe", count=2)
- "生成4辆福特野马" -> spawn_vehicle(query="mustang", count=4)
- "生成一个行人" -> spawn_pedestrian(query="pedestrian", count=1)
- "生成5个行人" -> spawn_pedestrian(query="pedestrian", count=5)
- "生成3个老年人" -> spawn_pedestrian(query="elderly", count=3)
- "生成10个警察" -> spawn_pedestrian(query="police", count=10)
- "生成一个人" -> spawn_pedestrian(query="pedestrian", count=1)
- "生成5个人" -> spawn_pedestrian(query="pedestrian", count=5)
- "生成3个小孩" -> spawn_pedestrian(query="child", count=3)
- "生成2个商务人士" -> spawn_pedestrian(query="business", count=2)
- "生成4个慢跑者" -> spawn_pedestrian(query="jogger", count=4)
- "生成3个慢跑者，速度3.0" -> spawn_pedestrian(query="jogger", count=3, speed=3.0)
- "开启车辆自动驾驶" -> setup_autopilot(enable=True, radius=0.0)
- "让车辆自己开" -> setup_autopilot(enable=True)
- "开启行人移动" -> setup_pedestrian_movement(enable=True, radius=0.0)
- "让行人走路" -> setup_pedestrian_movement(enable=True)
- "设置雨天" -> set_weather(weather_type="rain")
- "查看交通灯" -> get_traffic_lights()
- "清理场景" -> cleanup_scene()
- "切换到第三人称视角" -> switch_view(view_mode="third_person")
- "切换到第三人称视角 ID 123" -> switch_view(view_mode="third_person", target_actor_id=123)
- "切换到第一人称" -> switch_view(view_mode="first_person")
- "切换到第一人称 ID 456" -> switch_view(view_mode="first_person", target_actor_id=456)
- "切换到俯视视角" -> switch_view(view_mode="overhead")
- "切换到自由视角" -> switch_view(view_mode="free")
- "切换到旁观者视角" -> switch_view(view_mode="bystander")
- "回到默认视角" -> switch_view(view_mode="bystander")
- "开始录制视频" -> start_recording()
- "开始录制60fps视频" -> start_recording(fps=60)
- "停止录制" -> stop_recording()
- "结束录像" -> stop_recording()

重要提示：
- 当用户明确要求生成多辆车时（如"生成5辆车"、"来10辆车"），必须在spawn_vehicle的arguments中包含count参数
- 当用户明确要求生成多个行人时（如"生成5个行人"、"来10个行人"），必须在spawn_pedestrian的arguments中包含count参数
- count参数必须是整数，表示要生成的车辆或行人数量
- 如果不指定count，默认为1
- 当用户要求生成行人时，直接调用spawn_pedestrian，不要先调用connect_carla
- 当用户要求生成车辆时，直接调用spawn_vehicle，不要先调用connect_carla
- 只有当用户明确要求连接服务器时，才调用connect_carla

本助手基于FastMCP框架构建，提供高效、类型安全的工具调用体验。
"""
    },
    {"role": "user", "content": user_message}
]


        # 第一次API调用
        app_logger.info(f"💬 用户消息: {user_message}")
        response = await self.call_deepseek_with_tools(messages)
        assistant_message = response["choices"][0]["message"]

        # 检查是否有工具调用
        tool_calls = assistant_message.get("tool_calls", [])
        messages.append(assistant_message)

        # 执行FastMCP工具调用
        if tool_calls:
            app_logger.info(f"🔧 检测到 {len(tool_calls)} 个FastMCP工具调用")

            for tool_call in tool_calls:
                app_logger.info(f"🔨 执行FastMCP工具: {tool_call['function']['name']}")
                tool_result = await self.execute_fastmcp_tool_call(tool_call)
                app_logger.info(f"✅ FastMCP工具执行完成，结果长度: {len(str(tool_result))}")

                # 添加工具结果到消息历史
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call["id"],
                    "content": json.dumps(tool_result, ensure_ascii=False)
                })

            # 再次调用API获取最终回答
            app_logger.info("🤖 正在生成最终回答...")
            try:
                final_response = await self.call_deepseek_with_tools(messages)
                final_message = final_response["choices"][0]["message"]["content"]
                app_logger.info(f"✅ 最终回答生成成功，长度: {len(final_message)}")

                if not final_message or final_message.strip() == "":
                    app_logger.info("❌ 警告：最终回答为空")
                    final_message = "抱歉，我无法生成回答。请稍后重试。"

                return {
                    "message": self.process_markdown(final_message),
                    "tool_calls": tool_calls,
                    "conversation": messages
                }
            except Exception as e:
                app_logger.error(f"❌ 生成最终回答时出错: {str(e)}")
                return {
                    "message": f"FastMCP工具调用成功，但生成最终回答时出错: {str(e)}",
                    "tool_calls": tool_calls,
                    "conversation": messages
                }
        else:
            return {
                "message": self.process_markdown(assistant_message["content"]),
                "tool_calls": None,
                "conversation": messages
            }


# ============ FastAPI Web界面（AI对话版） ============

app = FastAPI(title="FastMCP GitHub Assistant")

# ===== 新增：文件下载接口 =====
from fastapi.responses import FileResponse
from pathlib import Path

@app.get("/download/{filename}")
async def download_file(filename: str):
    """下载生成的文件"""
    file_path = Path(__file__).parent / "output" / filename
    if file_path.exists():
        return FileResponse(
            path=file_path,
            filename=filename,
            media_type="application/octet-stream"
        )
    return {"error": "文件不存在"}
# ===== 新增结束 =====

def get_web_interface():
    """生成AI对话Web界面HTML"""
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>FastMCP GitHub Assistant - AI智能助手</title>
        <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
        <style>
            * { 
                margin: 0; 
                padding: 0; 
                box-sizing: border-box; 
            }

            body {
                font-family: 'Segoe UI', 'Microsoft YaHei', Tahoma, Geneva, Verdana, sans-serif;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                min-height: 100vh;
                line-height: 1.6;
            }

            .container {
                max-width: 900px;
                margin: 0 auto;
                padding: 20px;
                min-height: 100vh;
                display: flex;
                flex-direction: column;
            }

            .header {
                background: rgba(255, 255, 255, 0.95);
                backdrop-filter: blur(10px);
                padding: 12px 20px;
                border-radius: 15px;
                text-align: center;
                margin-bottom: 15px;
                box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }

            .header h1 {
                color: #2d3748;
                font-size: 1.5em;
                margin: 0;
                font-weight: 700;
                background: linear-gradient(135deg, #667eea, #764ba2);
                -webkit-background-clip: text;
                -webkit-text-fill-color: transparent;
                background-clip: text;
            }

            .chat-container {
                background: rgba(255, 255, 255, 0.95);
                backdrop-filter: blur(10px);
                border-radius: 20px;
                padding: 20px;
                flex: 1;
                display: flex;
                flex-direction: column;
                box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }

            .messages {
                order: 1;
                flex: 1;
                overflow-y: auto;
                overflow-x: hidden;
                padding: 15px;
                margin-bottom: 15px;
                background: rgba(248, 250, 252, 0.5);
                border-radius: 15px;
                border: 1px solid rgba(226, 232, 240, 0.5);
                height: calc(100vh - 280px);
                min-height: 400px;
                max-height: calc(100vh - 280px);
                scroll-behavior: smooth;
            }

            .message {
                margin-bottom: 15px;
                padding: 15px 20px;
                border-radius: 15px;
                max-width: 85%;
                word-wrap: break-word;
                position: relative;
                animation: messageSlide 0.3s ease-out;
            }

            @keyframes messageSlide {
                from {
                    opacity: 0;
                    transform: translateY(10px);
                }
                to {
                    opacity: 1;
                    transform: translateY(0);
                }
            }

            .user-message {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                margin-left: auto;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
                border-bottom-right-radius: 5px;
            }

            .assistant-message {
                background: linear-gradient(135deg, #f8fafc, #e2e8f0);
                color: #2d3748;
                margin-right: auto;
                border-left: 4px solid #667eea;
                box-shadow: 0 4px 15px rgba(0, 0, 0, 0.05);
                border-bottom-left-radius: 5px;
            }

            .tools-used {
                background: rgba(102, 126, 234, 0.05);
                margin-top: 10px;
                border-radius: 10px;
                font-size: 0.9em;
                border: 1px solid rgba(102, 126, 234, 0.2);
                overflow: hidden;
            }

            .tools-header {
                background: rgba(102, 126, 234, 0.1);
                padding: 10px 12px;
                cursor: pointer;
                display: flex;
                align-items: center;
                justify-content: space-between;
                font-weight: 600;
                color: #667eea;
                transition: all 0.3s ease;
            }

            .tools-header:hover {
                background: rgba(102, 126, 234, 0.15);
            }

            .tools-toggle {
                font-size: 0.9em;
                transition: all 0.3s ease;
                font-weight: bold;
            }

            .tools-content {
                padding: 12px;
                display: none;
                border-top: 1px solid rgba(102, 126, 234, 0.1);
            }

            .tools-content.show {
                display: block;
            }

            .input-form {
                order: 2;              /* ← 新增：输入框永远排在消息区下面 */
                position: sticky;      /* ← 新增：吸附在可视区域底部 */
                bottom: 0;             /* ← 新增 */
                z-index: 10;           /* ← 新增：不被消息盖住 */
                display: flex;
                gap: 12px;
                align-items: flex-end;
                background: linear-gradient(135deg, rgba(255, 255, 255, 0.95), rgba(248, 250, 252, 0.9));
                padding: 15px;
                border-radius: 15px;
                border: 1px solid rgba(102, 126, 234, 0.2);
                box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
                backdrop-filter: blur(10px);
            }

            .message-input {
                flex: 1;
                padding: 12px 16px;
                border: 2px solid transparent;
                border-radius: 12px;
                background: white;
                font-size: 0.95em;
                resize: none;
                min-height: 44px;
                max-height: 120px;
                transition: all 0.3s ease;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
                font-family: inherit;
                line-height: 1.4;
            }

            .message-input:focus {
                outline: none;
                border-color: #667eea;
                box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.15), 0 4px 15px rgba(0, 0, 0, 0.15);
                transform: translateY(-1px);
            }

            .message-input::placeholder {
                color: #9ca3af;
                font-style: italic;
            }

            .send-button {
                width: 44px;
                height: 44px;
                background: linear-gradient(135deg, #667eea, #764ba2);
                border: none;
                border-radius: 50%;
                cursor: pointer;
                transition: all 0.3s ease;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
                display: flex;
                align-items: center;
                justify-content: center;
                flex-shrink: 0;
                position: relative;
            }

            .send-button i {
                color: white;
                font-size: 16px;
            }

            .send-button:hover:not(:disabled) {
                transform: translateY(-2px);
                box-shadow: 0 6px 25px rgba(102, 126, 234, 0.4);
                background: linear-gradient(135deg, #5a67d8, #6b46c1);
            }

            .send-button:active:not(:disabled) {
                transform: translateY(0px);
                box-shadow: 0 2px 10px rgba(102, 126, 234, 0.3);
            }

            .send-button:disabled {
                opacity: 0.5;
                cursor: not-allowed;
                transform: none;
                box-shadow: 0 2px 8px rgba(102, 126, 234, 0.2);
                background: linear-gradient(135deg, #9ca3af, #6b7280);
            }

            .loading {
                display: none;
                text-align: center;
                padding: 25px;
                margin: 15px 0;
                background: linear-gradient(135deg, rgba(102, 126, 234, 0.1), rgba(118, 75, 162, 0.1));
                border-radius: 15px;
                border: 1px solid rgba(102, 126, 234, 0.2);
            }

            .loading.show { 
                display: block; 
            }

            .loading-content {
                display: flex;
                flex-direction: column;
                align-items: center;
                gap: 15px;
            }

            .loading-text {
                color: #667eea;
                font-weight: 600;
                font-size: 1.2em;
                display: flex;
                align-items: center;
                gap: 12px;
            }

            .loading-spinner {
                width: 24px;
                height: 24px;
                border: 3px solid rgba(102, 126, 234, 0.2);
                border-top: 3px solid #667eea;
                border-radius: 50%;
                animation: spin 1s linear infinite;
            }

            @keyframes spin {
                from { transform: rotate(0deg); }
                to { transform: rotate(360deg); }
            }

            .example-questions {
                background: linear-gradient(135deg, rgba(248, 250, 252, 0.8), rgba(241, 245, 249, 0.8));
                border-radius: 15px;
                padding: 20px;
                margin-bottom: 15px;
                border: 1px solid rgba(226, 232, 240, 0.5);
                backdrop-filter: blur(5px);
            }

            .welcome-message {
                color: #4a5568;
                margin-bottom: 15px;
                font-size: 1em;
                line-height: 1.5;
                text-align: center;
                padding: 15px;
                background: rgba(255, 255, 255, 0.6);
                border-radius: 12px;
                border-left: 4px solid #667eea;
            }

            .example-questions h3 {
                color: #2d3748;
                margin-bottom: 15px;
                font-size: 1em;
                text-align: center;
                font-weight: 600;
            }

            .examples-grid {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 12px;
            }

            .example-item {
                background: linear-gradient(135deg, rgba(255, 255, 255, 0.9), rgba(248, 250, 252, 0.9));
                border-radius: 10px;
                padding: 12px 16px;
                cursor: pointer;
                transition: all 0.3s ease;
                border-left: 3px solid #667eea;
                font-size: 0.9em;
                box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
                border: 1px solid rgba(226, 232, 240, 0.3);
                text-align: center;
            }

            .example-item:hover {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                transform: translateY(-2px) scale(1.02);
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
            }

            .assistant-message h1 {
                font-size: 1.4em;
                color: #2d3748;
                margin: 15px 0 10px 0;
                font-weight: 700;
            }

            .assistant-message h2 {
                font-size: 1.2em;
                color: #2d3748;
                margin: 12px 0 8px 0;
                font-weight: 600;
            }

            .assistant-message h3 {
                font-size: 1.1em;
                color: #2d3748;
                margin: 10px 0 6px 0;
                font-weight: 600;
            }

            /* 响应式设计 */
            @media (max-width: 768px) {
                .container {
                    padding: 10px;
                }

                .header h1 {
                    font-size: 1.5em;
                }

                .message {
                    max-width: 95%;
                    padding: 12px 15px;
                }

                .examples-grid {
                    grid-template-columns: 1fr;
                    gap: 8px;
                }

                .input-form {
                    flex-direction: column;
                    gap: 12px;
                    padding: 12px;
                }

                .message-input {
                    min-height: 40px;
                }

                .send-button {
                    width: 100%;
                    height: 44px;
                }

                .messages {
                    height: calc(100vh - 320px);
                }
            }

            /* 滚动条美化 */
            .messages::-webkit-scrollbar {
                width: 6px;
            }

            .messages::-webkit-scrollbar-track {
                background: rgba(226, 232, 240, 0.3);
                border-radius: 3px;
            }

            .messages::-webkit-scrollbar-thumb {
                background: linear-gradient(135deg, #667eea, #764ba2);
                border-radius: 3px;
            }

            .messages::-webkit-scrollbar-thumb:hover {
                background: linear-gradient(135deg, #5a67d8, #6b46c1);
            }

            /* ===== 周计划场景快捷按钮面板 ===== */
            .quick-panel {
                margin-top: 15px;
                border-top: 2px dashed #3b82f6;
                padding-top: 10px;
            }
            .quick-title {
                color: #3b82f6;
                font-size: 1.05em;
                font-weight: 700;
                margin-bottom: 8px;
            }
            .week-group { margin-bottom: 10px; }
            .week-label {
                font-size: 0.85em;
                font-weight: 600;
                color: #64748b;
                margin-bottom: 5px;
            }
            .week-label .w1 { color: #10b981; }
            .quick-btns {
                display: flex;
                flex-wrap: wrap;
                gap: 6px;
            }
            .qbtn {
                border: none;
                border-radius: 8px;
                padding: 7px 12px;
                font-size: 0.85em;
                cursor: pointer;
                color: #fff;
                background: linear-gradient(135deg, #3b82f6, #6366f1);
                box-shadow: 0 2px 8px rgba(59, 130, 246, 0.3);
                transition: transform 0.15s ease, box-shadow 0.15s ease;
            }
            .qbtn:hover:not(:disabled) {
                transform: translateY(-2px);
                box-shadow: 0 4px 12px rgba(59, 130, 246, 0.45);
            }
            .qbtn:disabled {
                background: #cbd5e1;
                color: #64748b;
                cursor: not-allowed;
                box-shadow: none;
            }
            .qbtn.running {
                background: linear-gradient(135deg, #f59e0b, #d97706);
            }
            .qbtn-w1 { background: linear-gradient(135deg, #10b981, #059669); box-shadow: 0 2px 8px rgba(16, 185, 129, 0.3); }
            .qbtn-util { background: linear-gradient(135deg, #64748b, #475569); }
            .quick-note { font-size: 0.8em; color: #94a3b8; margin-top: 4px; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🚀 HUTB 模拟器智能助手</h1>
            </div>

            <div class="chat-container">
                <div class="messages" id="messages">
                    <div class="example-questions">
                         <div class="welcome-message">
                            👋 欢迎使用基于FastMCP框架的 HUTB 模拟器智能助手！集成 HUTB 仿真控制。
                            <br><br>
                            🔧 <strong>技术特色</strong>：本助手使用FastMCP装饰器实现工具定义，提供类型安全、自动化的MCP体验！
                        </div>
                        <h3>💡 试试这些问题：</h3>
                        <div class="examples-grid">
                            <div class="example-item" onclick="askExample('连接CARLA仿真服务器')">
                            🔗 连接服务器
                            </div>
                            <div class="example-item" onclick="askExample('设置雨天天气条件')">
                                🌫️ 天气设置（默认雨天）
                            </div>
                            <div class="example-item" onclick="askExample('生成行人')">
                                🚶 生成行人
                            </div>
                            <div class="example-item" onclick="askExample('生成 model3 车辆')">
                                🚗 生成车辆
                            </div>
                        </div>

                        <!-- SUMO 功能（新增） -->
<div style="margin-top: 15px; border-top: 2px dashed #ff6b35; padding-top: 10px;">
    <h3 style="color: #ff6b35;">🚦 SUMO 交通仿真（新增功能）</h3>
    <div class="examples-grid">
        <!-- 原有的路网生成按钮 -->
        <div class="example-item" style="border-left-color: #ff6b35;" onclick="askExample('生成一个3x3网格路网，跑200秒，每2秒发一辆车')">
            🚦 生成默认网格路网
        </div>
        <!-- 新增 OpenSCENARIO 生成按钮 -->
        <div class="example-item" style="border-left-color: #ff6b35;" onclick="askExample('生成一个场景，基于 web_generated.xodr，车以10m/s行驶30秒')">
            🎬 生成 OpenSCENARIO 场景
        </div>
    </div>
    <div style="margin-top: 8px; font-size: 0.85em; color: #666; text-align: center;">
        💡 也支持自然语言自定义参数：<em>"生成4x4网格路网，跑300秒"</em> 或 <em>"生成5x5网格路网，跑500秒，每3秒发一辆车"</em>
    </div>
</div>
<div style="margin-top: 15px; border-top: 2px dashed #10b981; padding-top: 10px;">
        <h3 style="color: #10b981;">🎬 场景与道具（新增功能）</h3>
        <div class="examples-grid">
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆山地自行车')">🚲 生成自行车</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆摩托车')">🏍️ 生成摩托车</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成3个施工锥')">🚧 生成道具</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆仰翻的车辆')">🚓💥 仰翻汽车</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('设置薄雾天气')">🌫️ 薄雾天气</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆警车')">🚓 生成警车</div>
        </div>
    </div>

    <!-- ===== 周计划场景快捷按钮 ===== -->
    <div class="quick-panel">
        <div class="quick-title">⚡ 场景快捷按钮（点击直接执行，无需输入）</div>

        <div class="week-group">
            <div class="week-label"><span class="w1">🛣️ 道路结构场景</span></div>
            <div class="quick-btns">
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_highway_ramp', {ramp_type: 'on'}, this)">🛣️ 匝道汇入</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_highway_ramp', {ramp_type: 'off'}, this)">🛣️ 匝道驶出</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_lane_merge', {}, this)">🔀 车道合并</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_diverge_merge', {}, this)">🚥 分合流路口</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_side_road', {}, this)">🛤️ 辅路</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label">🚦 路口与特殊形态目标</div>
            <div class="quick-btns">
                <button class="qbtn" disabled>🚦 路口+红绿灯</button>
                <button class="qbtn" disabled>🚇 隧道</button>
                <button class="qbtn" disabled>🔄 环岛</button>
                <button class="qbtn" disabled>🧒 儿童姿态</button>
                <button class="qbtn" disabled>🚲 二轮车姿态</button>
                <button class="qbtn" disabled>🚑 特殊任务车辆</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label">👮 特殊群体、光照与危险行为</div>
            <div class="quick-btns">
                <button class="qbtn" disabled>👮 交警/轮椅</button>
                <button class="qbtn" disabled>🌙 弱光</button>
                <button class="qbtn" disabled>☀️ 逆光</button>
                <button class="qbtn" disabled>🚚 侧翻车辆</button>
                <button class="qbtn" disabled>🛑 前车急刹/静止</button>
                <button class="qbtn" disabled>⚠️ 危险切入</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label">⚠️ 复杂危险场景</div>
            <div class="quick-btns">
                <button class="qbtn" disabled>👻 前车消失</button>
                <button class="qbtn" disabled>🏃 路口危险横穿</button>
                <button class="qbtn" disabled>📐 低重叠率目标</button>
                <button class="qbtn" disabled>↩️ 逆行</button>
                <button class="qbtn" disabled>🚗 无保护转弯</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label">常用操作</div>
            <div class="quick-btns">
                <button class="qbtn qbtn-util" onclick="runQuickTool('connect_carla', {}, this)">🔗 连接 CARLA</button>
                <button class="qbtn qbtn-util" onclick="runQuickTool('cleanup_scene', {}, this)">🧹 清理场景</button>
            </div>
        </div>
        <div class="quick-note">💡 场景按钮基于当前地图自动选址，推荐 Town04（高速匝道/隧道/环岛均可）；Town03 地图不可用。</div>
    </div>

</div> 
                </div>
                <div class="loading" id="loading">
                    <div class="loading-content">
                        <div class="loading-text">
                            <div class="loading-spinner"></div>
                            <span>FastMCP工具调用中...</span>
                        </div>
                    </div>
                </div>

                <form class="input-form" onsubmit="return submitForm(event)">
                    <textarea 
                        id="messageInput" 
                        class="message-input" 
                        placeholder="问我任何 HUTB 模拟器相关问题，我会使用 FastMCP 工具来帮你操作..."
                        rows="2"
                        onkeydown="handleKeyPress(event)"
                    ></textarea>
                    <button type="submit" class="send-button" id="sendButton">
                        <i class="fas fa-paper-plane"></i>
                    </button>
                </form>
            </div>
        </div>

        <script>
    function scrollToBottom() {
        var box = document.getElementById('messages');
        if (box) box.scrollTop = box.scrollHeight;
    }
    window.addEventListener('load', scrollToBottom);
    var observer = new MutationObserver(scrollToBottom);
    var msgBox = document.getElementById('messages');
    if (msgBox) observer.observe(msgBox, { childList: true, subtree: true });
    </script>

<script>
function askExample(text) {
    document.getElementById('messageInput').value = text;
    submitMessage();
}

// ===== 周计划快捷按钮：直接执行FastMCP工具（不走AI） =====
async function runQuickTool(tool, args, btn) {
    if (btn) { btn.disabled = true; btn.classList.add('running'); }
    const argStr = Object.entries(args || {}).map(([k, v]) => `${k}=${v}`).join(', ');
    addMessage(`⚡ 快捷执行：${tool}(${argStr})`, 'user');
    try {
        const form = new FormData();
        form.append('tool', tool);
        form.append('args', JSON.stringify(args || {}));
        const response = await fetch('/tool', { method: 'POST', body: form });
        const result = await response.json();
        const toolCalls = result.success
            ? [{ function: { name: tool, arguments: JSON.stringify(args || {}) } }]
            : null;
        addMessage(result.message || (result.success ? '✅ 执行完成' : '❌ 执行失败'), 'assistant', toolCalls);
    } catch (error) {
        addMessage('❌ 请求失败: ' + error, 'assistant');
    } finally {
        if (btn) { btn.disabled = false; btn.classList.remove('running'); }
    }
}

function handleKeyPress(event) {
    if (event.key === 'Enter' && !event.shiftKey) {
        event.preventDefault();
        submitMessage();
    }
}

function submitForm(event) {
    event.preventDefault();
    submitMessage();
    return false;
}

async function submitMessage() {
    const input = document.getElementById('messageInput');
    const message = input.value.trim();
    if (!message) return;

    addMessage(message, 'user');
    input.value = '';
    showLoading(true);

    try {
        const response = await fetch('/chat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'message=' + encodeURIComponent(message)
        });

        if (response.ok) {
            const result = await response.json();
            addMessage(result.message, 'assistant', result.tool_calls);
        } else {
            addMessage('抱歉，发生了错误，请稍后重试。', 'assistant');
        }
    } catch (error) {
        console.error('Error:', error);
        addMessage('网络连接错误，请检查网络后重试。', 'assistant');
    } finally {
        showLoading(false);
    }
}

function addMessage(content, sender, toolCalls) {
    const messages = document.getElementById('messages');
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}-message`;

    let html = `<div>${content}</div>`;

    if (toolCalls && toolCalls.length > 0) {
        const toolsId = 'tools-' + Date.now();
        html += `
            <div class="tools-used">
                <div class="tools-header" onclick="toggleTools('${toolsId}')">
                    <span>🔧 使用的FastMCP工具 (${toolCalls.length}个)</span>
                    <span class="tools-toggle" id="toggle-${toolsId}">▼</span>
                </div>
                <div class="tools-content" id="${toolsId}">`;

        for (let i = 0; i < toolCalls.length; i++) {
            const tool = toolCalls[i];
            const args = JSON.parse(tool.function.arguments);
            let argStr = '';
            for (const k in args) {
                if (argStr) argStr += ', ';
                argStr += `${k}: "${args[k]}"`;
            }
            html += `<div>• <strong>@mcp.tool() ${tool.function.name}</strong>(${argStr})</div>`;
        }

        html += `
                </div>
            </div>`;
    }

    messageDiv.innerHTML = html;
    messages.appendChild(messageDiv);
    messages.scrollTop = messages.scrollHeight;
}

function toggleTools(toolsId) {
    const content = document.getElementById(toolsId);
    const toggle = document.getElementById('toggle-' + toolsId);

    if (content.classList.contains('show')) {
        content.classList.remove('show');
        toggle.classList.remove('expanded');
        toggle.textContent = '▼';
    } else {
        content.classList.add('show');
        toggle.classList.add('expanded');
        toggle.textContent = '▲';
    }
}

function showLoading(show) {
    const loading = document.getElementById('loading');
    const sendButton = document.getElementById('sendButton');

    if (show) {
        loading.classList.add('show');
        sendButton.disabled = true;
    } else {
        loading.classList.remove('show');
        sendButton.disabled = false;
    }
}
</script>
    </body>
    </html>
    """
    return html_content


@app.get("/", response_class=HTMLResponse)
async def index():
    """主页面 - AI对话界面"""
    return get_web_interface()


@app.post("/chat")
async def chat(message: str = Form(...)):
    """处理聊天请求 - 使用FastMCP工具的AI对话"""
    try:
        result = await assistant.chat(message)
        return {
            "success": True,
            "message": result["message"],
            "tool_calls": result["tool_calls"]
        }
    except Exception as e:
        app_logger.error(f"❌ FastMCP聊天处理失败: {str(e)}")
        return {
            "success": False,
            "message": f"抱歉，处理您的请求时出现错误: {str(e)}",
            "tool_calls": None
        }


@app.post("/tool")
async def run_tool(tool: str = Form(...), args: str = Form("{}")):
    """快捷按钮接口 - 直接执行FastMCP工具（不经过AI），供网页快捷按钮调用"""
    try:
        arguments = json.loads(args) if args else {}
    except Exception:
        arguments = {}
    try:
        # 除连接工具外，未连接时先自动连接CARLA
        if tool != "connect_carla" and carla_client.world is None:
            connected = await carla_client.connect('localhost', 2000)
            if not connected:
                return {"success": False, "message": "❌ 无法连接到CARLA服务器(localhost:2000)，请先启动仿真器", "tool": tool}

        tool_call = {
            "id": "quick-btn",
            "type": "function",
            "function": {"name": tool, "arguments": json.dumps(arguments)}
        }
        result = await assistant.execute_fastmcp_tool_call(tool_call)
        if result.get("success"):
            data = result.get("data")
            if isinstance(data, str):
                message = data
            else:
                message = "✅ 操作成功" if data else "❌ 操作失败"
            return {"success": True, "message": message, "tool": tool}
        return {"success": False, "message": result.get("error", "执行失败"), "tool": tool}
    except Exception as e:
        app_logger.error(f"❌ 快捷工具执行失败: {str(e)}")
        return {"success": False, "message": f"执行出错: {str(e)}", "tool": tool}


# 创建全局AI助手实例
assistant = FastMCPGitHubAssistant()

def main():
    """主函数 - 支持 Web界面、标准MCP、SSE-MCP 三种启动模式"""
    import sys
    import socket

    # 1. 提取公共逻辑：无论进入哪个模式，都先进行一次环境校验
    if not config.validate():
        print("[ERROR] 配置验证失败，请检查环境变量设置")
        print("[INFO] 请确保 .env 文件包含以下必要配置：")
        print("   - GITHUB_TOKEN=your_github_token")
        print("   - DEEPSEEK_API_KEY=your_deepseek_api_key")
        return
    print("[OK] 环境配置验证通过")

    # 2. 根据命令行参数进行路由分发
    if len(sys.argv) > 1 and sys.argv[1] == "mcp":
        print("[MCP] 启动 FastMCP AI助手 MCP/stdio 服务器...")
        mcp.run()

    elif len(sys.argv) > 1 and sys.argv[1] == "sse":
        print("[MCP] 启动 FastMCP AI助手 SSE 服务端 (OpenClaw专用)...")
        # 监听 0.0.0.0 允许 Docker 跨环境访问，使用 3001 端口与 Web 端物理隔离
        mcp.run(transport="sse", host="0.0.0.0", port=3001)

    else:
        print("[WEB] 启动 FastMCP AI助手 Web 对话界面...")
        host_ip = socket.gethostbyname(socket.gethostname())
        print(f"[INFO] 访问地址: http://{host_ip}:3000")
        uvicorn.run(app='main_ai:app', host=host_ip, port=3000, reload=True)

@mcp.tool()
async def generate_sumo_network(
    grid_x: int = 3,
    grid_y: int = 3,
    duration: int = 200,
    rate: float = 2.0
) -> str:
    """生成 SUMO 路网和车流。"""
    return await generate_sumo_network_impl(grid_x, grid_y, duration, rate)

async def generate_sumo_network_impl(
    grid_x: int = 3,
    grid_y: int = 3,
    duration: int = 200,
    rate: float = 2.0
) -> str:
    """SUMO 路网生成实现函数"""
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        return "❌ 错误：未设置 SUMO_HOME 环境变量。请在终端中设置：`set SUMO_HOME=D:\\mcp\\sumo\\sumo_install\\sumo-win64-1.27.0\\sumo-1.27.0`"

 # 创建输出目录
    output_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(output_dir, exist_ok=True)

    bin_dir = os.path.join(sumo_home, "bin")
    tools_dir = os.path.join(sumo_home, "tools")

    prefix = os.path.join(output_dir, "web_generated")
    net_file = f"{prefix}.net.xml"
    trips_file = f"{prefix}.trips.xml"
    rou_file = f"{prefix}.rou.xml"
    cfg_file = f"{prefix}.sumocfg"
    xodr_file = f"{prefix}.xodr"

    try:
        # 1. 生成路网
        subprocess.run([
            os.path.join(bin_dir, "netgenerate"),
            "--grid",
            f"--grid-x-number={grid_x}",
            f"--grid-y-number={grid_y}",
            f"--grid-x-length=500",
            f"--grid-y-length=500",
            f"--output-file={net_file}"
        ], check=True, capture_output=True, text=True)

        # 2. 生成出行
        subprocess.run([
            "python",
            os.path.join(tools_dir, "randomTrips.py"),
            f"-n={net_file}",
            f"-e={duration}",
            "-l",
            f"-p={rate}",
            f"-o={trips_file}"
        ], check=True, capture_output=True, text=True)

        # 3. 生成路由
        subprocess.run([
            os.path.join(bin_dir, "duarouter"),
            f"-n={net_file}",
            f"-t={trips_file}",
            f"-o={rou_file}",
            "--ignore-errors"
        ], check=True, capture_output=True, text=True)

        # 4. 创建配置文件
        with open(cfg_file, "w", encoding="utf-8") as f:
            f.write(f'''<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{net_file}"/>
        <route-files value="{rou_file}"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{duration}"/>
    </time>
</configuration>
''')

 # 4.5 转换为 OpenDRIVE 格式
        xodr_file = f"{prefix}.xodr"
        xodr_msg = ""
        try:
            subprocess.run([
                os.path.join(bin_dir, "netconvert"),
                f"--sumo-net-file={net_file}",
                f"--opendrive-output={xodr_file}"
            ], check=True, capture_output=True, text=True)
            xodr_msg = f"- OpenDRIVE: {xodr_file}"
        except subprocess.CalledProcessError as e:
            xodr_msg = f"- OpenDRIVE 转换失败：{e.stderr}"

 # 5. 返回结果（修改返回信息，添加 xodr_msg）
        download_url = f"/download/{os.path.basename(xodr_file)}"
        return f"""✅ SUMO 路网和车流生成成功！

📁 生成的文件：
- 路网: {net_file}
- 出行: {trips_file}
- 路由: {rou_file}
- 配置: {cfg_file}
{xodr_msg}

📊 参数：
- 网格: {grid_x}x{grid_y}
- 仿真时长: {duration} 秒
- 发车间隔: {rate} 秒/辆

📥 下载链接：
- [点击下载 OpenDRIVE 文件]({download_url})

▶️ 在终端中运行以下命令查看：
cd D:\\mcp\\sumo
sumo-gui -c {cfg_file}
"""

    except subprocess.CalledProcessError as e:
        return f"❌ 生成失败：{e.stderr}"
    
# ============ OpenSCENARIO 场景生成功能（新增） ============

async def generate_openscenario_impl(
    xodr_filename: str = "",
    scenario_name: str = "my_scenario",
    duration: float = 30.0,
    vehicle_speed: float = 10.0
) -> str:
    """
    生成 OpenSCENARIO 场景文件（支持外部 .xodr 或内建直路）。
    """
    import xml.etree.ElementTree as ET
    import os
    from datetime import datetime

    output_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(output_dir, exist_ok=True)

    try:
        root = ET.Element("OpenSCENARIO", {
            "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
            "xsi:noNamespaceSchemaLocation": "http://www.asam.net/xml/OpenSCENARIO/1.0.0/OpenSCENARIO.xsd"
        })

        header = ET.SubElement(root, "FileHeader", {
            "revMajor": "1",
            "revMinor": "0",
            "date": datetime.now().isoformat(),
            "description": f"Scenario: {scenario_name}",
            "author": "MCP Assistant"
        })

        # 路网
        road_network = ET.SubElement(root, "RoadNetwork")
        if xodr_filename:
            # 使用外部 OpenDRIVE 文件
            xodr_path = os.path.join(output_dir, xodr_filename)
            if not os.path.exists(xodr_path):
                return f"❌ 错误：找不到 OpenDRIVE 文件 {xodr_filename}，请先生成路网。"
            ET.SubElement(road_network, "LogicFile", {"filepath": xodr_path})
        # 否则不添加 LogicFile，CARLA 会使用默认地图

        # 实体
        entities = ET.SubElement(root, "Entities")
        obj = ET.SubElement(entities, "ScenarioObject", {"name": "ego_vehicle"})
        vehicle = ET.SubElement(obj, "Vehicle", {"name": "ego_vehicle", "vehicleCategory": "car"})
        ET.SubElement(vehicle, "Performance", {"maxSpeed": "20", "maxAcceleration": "5", "maxDeceleration": "5"})

        # 故事板
        storyboard = ET.SubElement(root, "Storyboard")

        # 初始化：车辆放在原点
        init = ET.SubElement(storyboard, "Init")
        private = ET.SubElement(init, "Private", {"entityRef": "ego_vehicle"})
        action = ET.SubElement(private, "Action")
        teleport = ET.SubElement(action, "TeleportAction")
        pos = ET.SubElement(teleport, "Position")
        ET.SubElement(pos, "WorldPosition", {"x": "0.0", "y": "0.0", "z": "0.0", "h": "0.0"})

        # 故事：匀速行驶
        story = ET.SubElement(storyboard, "Story", {"name": "drive_story"})
        act = ET.SubElement(story, "Act", {"name": "drive_act"})
        maneuver = ET.SubElement(act, "Maneuver", {"name": "drive_maneuver"})
        event = ET.SubElement(maneuver, "Event", {"name": "speed_event", "priority": "overwrite"})

        action = ET.SubElement(event, "Action")
        speed_action = ET.SubElement(action, "SpeedAction")
        ET.SubElement(speed_action, "SpeedActionDynamics", {"dynamicsShape": "step", "value": "0.0"})
        target = ET.SubElement(speed_action, "SpeedActionTarget")
        ET.SubElement(target, "AbsoluteSpeed", {"value": str(vehicle_speed)})

        # 开始触发
        start = ET.SubElement(event, "StartTrigger")
        cond_group = ET.SubElement(start, "ConditionGroup")
        cond = ET.SubElement(cond_group, "Condition", {"rule": "greaterThan", "edge": "rising"})
        by_val = ET.SubElement(cond, "ByValueCondition")
        ET.SubElement(by_val, "SimulationTimeCondition", {"value": "0.0"})

        # 结束触发
        stop = ET.SubElement(event, "StopTrigger")
        stop_group = ET.SubElement(stop, "ConditionGroup")
        stop_cond = ET.SubElement(stop_group, "Condition", {"rule": "greaterThan", "edge": "rising"})
        stop_by_val = ET.SubElement(stop_cond, "ByValueCondition")
        ET.SubElement(stop_by_val, "SimulationTimeCondition", {"value": str(duration)})

        tree = ET.ElementTree(root)
        output_file = os.path.join(output_dir, f"{scenario_name}.xosc")
        tree.write(output_file, encoding="UTF-8", xml_declaration=True)

        return f"""✅ OpenSCENARIO 场景文件生成成功！

📁 文件路径: {output_file}
📊 文件大小: {os.path.getsize(output_file)} 字节

📥 下载链接：
<a href="/download/{scenario_name}.xosc" target="_blank">点击下载 {scenario_name}.xosc</a>

▶️ 可直接在 CARLA 的 OpenSCENARIO 播放器中运行。
"""
    except Exception as e:
        return f"❌ 生成失败: {str(e)}"
    # 2. 工具函数（有 @mcp.tool() 装饰器）
@mcp.tool()
async def generate_openscenario(
    xodr_filename: str = "web_generated.xodr",
    scenario_name: str = "my_scenario",
    duration: float = 30.0,
    vehicle_speed: float = 10.0
) -> str:
    """生成 OpenSCENARIO 场景文件。"""
    return await generate_openscenario_impl(xodr_filename, scenario_name, duration, vehicle_speed)

# ============ 修复5: SpawnParameters 参数面板类 ============
class SpawnParameters:
    """生成参数面板"""
    def __init__(self):
        self.actor_type = "vehicle"
        self.blueprint_filter = "vehicle.*"
        self.reference_actor_id = None
        self.relative_distance = 10.0
        self.relative_angle = 0.0
        self.lane_type = "Driving"
        self.initial_speed = 0.0
        self.autopilot = False
        self.count = 1

# ============ 修复8: MCP Tool - 控制行人停止/恢复 ============
@mcp.tool()
async def control_walker(action: str, walker_id: int = None) -> str:
    """控制行人停止或恢复移动
    
    Args:
        action: "stop" | "resume" | "stop_all" | "resume_all"
        walker_id: 行人ID（stop/resume需要）
    """
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    if action == "stop":
        if walker_id is None:
            return "❌ stop需要指定walker_id"
        carla_client.stop_walker(walker_id)
        return f"✅ 行人 {walker_id} 已停止"
    elif action == "resume":
        if walker_id is None:
            return "❌ resume需要指定walker_id"
        if carla_client.resume_walker(walker_id):
            return f"✅ 行人 {walker_id} 已恢复移动"
        return f"❌ 行人 {walker_id} 恢复失败"
    elif action == "stop_all":
        count = carla_client.stop_all_walkers()
        return f"✅ 已停止 {count} 个行人"
    elif action == "resume_all":
        count = carla_client.resume_all_walkers()
        return f"✅ 已恢复 {count} 个行人"
    return f"❌ 未知操作: {action}"

# ============ 修复5+6+7: 底层实现函数 ============
async def spawn_vehicle_param_impl(
    count: int = 1,
    blueprint_filter: str = "vehicle.*",
    autopilot: bool = True,
    reference_id: int = None,
    relative_distance: float = 10.0,
    relative_angle: float = 0.0,
    initial_speed: float = 0.0
) -> str:
    """参数化生成车辆的底层实现"""
    try:
        if carla_client.world is None:
            return "❌ 未连接到CARLA服务器"
        
        params = SpawnParameters()
        params.actor_type = "vehicle"
        params.blueprint_filter = blueprint_filter
        params.reference_actor_id = reference_id
        params.relative_distance = relative_distance
        params.relative_angle = relative_angle
        params.autopilot = autopilot
        params.initial_speed = initial_speed
        params.count = count
        params.lane_type = "Driving"
        
        if reference_id is None:
            result = await carla_client.batch_spawn_vehicles_with_id(count, blueprint_filter, autopilot)
            lines = [f"✅ 生成完成：成功{result['success']}辆，失败{result['failed']}辆"]
            for v in result["vehicles"]:
                lines.append(f"  [{v['index']}] ID={v['id']} {v['type_id']} @({v['location']['x']},{v['location']['y']})")
            return "\n".join(lines)
        else:
            spawned = await carla_client.spawn_vehicles_with_params(params)
            return f"✅ 生成完成：成功{len(spawned)}辆，ID={[a.id for a in spawned]}"
    except Exception as e:
        app_logger.error(f"❌ spawn_vehicle_param_impl 异常: {e}")
        return f"❌ 生成失败: {str(e)}"


async def control_walker_impl(action: str, walker_id: int = None) -> str:
    """控制行人停止/恢复的底层实现"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    if action == "stop":
        if walker_id is None:
            return "❌ stop需要指定walker_id"
        carla_client.stop_walker(walker_id)
        return f"✅ 行人 {walker_id} 已停止"
    elif action == "resume":
        if walker_id is None:
            return "❌ resume需要指定walker_id"
        if carla_client.resume_walker(walker_id):
            return f"✅ 行人 {walker_id} 已恢复移动"
        return f"❌ 行人 {walker_id} 恢复失败"
    elif action == "stop_all":
        count = carla_client.stop_all_walkers()
        return f"✅ 已停止 {count} 个行人"
    elif action == "resume_all":
        count = carla_client.resume_all_walkers()
        return f"✅ 已恢复 {count} 个行人"
    return f"❌ 未知操作: {action}"

# ============ 修复5+6+7: MCP Tool - 参数化生成车辆 ============
@mcp.tool()
async def spawn_vehicle_param(
    count: int = 1,
    blueprint_filter: str = "vehicle.*",
    autopilot: bool = True,
    reference_id: int = None,
    relative_distance: float = 10.0,
    relative_angle: float = 0.0,
    initial_speed: float = 0.0
) -> str:
    """参数化生成车辆，支持参照物/距离/角度/速度控制"""
    try:
        if carla_client.world is None:
            return "❌ 未连接到CARLA服务器"
        
        params = SpawnParameters()
        params.actor_type = "vehicle"
        params.blueprint_filter = blueprint_filter
        params.reference_actor_id = reference_id
        params.relative_distance = relative_distance
        params.relative_angle = relative_angle
        params.autopilot = autopilot
        params.initial_speed = initial_speed
        params.count = count
        params.lane_type = "Driving"
        
        if reference_id is None:
            result = await carla_client.batch_spawn_vehicles_with_id(count, blueprint_filter, autopilot)
            lines = [f"✅ 生成完成：成功{result['success']}辆，失败{result['failed']}辆"]
            for v in result["vehicles"]:
                lines.append(f"  [{v['index']}] ID={v['id']} {v['type_id']} @({v['location']['x']},{v['location']['y']})")
            return "\n".join(lines)
        else:
            spawned = await carla_client.spawn_vehicles_with_params(params)
            return f"✅ 生成完成：成功{len(spawned)}辆，ID={[a.id for a in spawned]}"
    except Exception as e:
        app_logger.error(f"❌ spawn_vehicle_param 异常: {e}")
        return f"❌ 生成失败: {str(e)}"
#!/usr/bin/env python3
"""
FastMCP HUTB Assistant - 使用FastMCP框架的模拟器接口
集成Deepseek AI模型，支持自然语言使用 HUTB 模拟器
使用 FastMCP 装饰器方式实现 MCP 工具调用机制
"""

import socket
import sys
import json
import re
from pathlib import Path
from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse
import uvicorn
import aiohttp
from typing import Optional
import carla
import subprocess
import os
import xml.etree.ElementTree as ET
import random
import math
from pathlib import Path
from fastapi.responses import FileResponse
# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir/ "src"))

from fastmcp import FastMCP
from src.github_client import GitHubClient
from src.config import config               
from src.utils.logger import app_logger

# 创建FastMCP实例
mcp = FastMCP("AI智能助手")


class CarlaClient:
    """CARLA客户端封装类"""

    def __init__(self):
        self.client = None
        self.world = None
        self.actors = []
        self.tick_task = None
        self.is_ticking = False
        # 视频录制相关
        self.is_recording = False
        self.recording_task = None
        self.recording_output_path = None
        self.recording_fps = 30
        self.recording_frame_count = 0
        self.video_writer = None
        self.camera_sensor = None
        self.image_queue = None
        # 视角控制相关
        self.current_view_mode = "spectator"  # spectator, third_person, first_person, overhead, bystander
        self.view_target = None  # 当前视角跟随的目标
        self.view_follow_task = None  # 视角跟随任务
        self.is_view_following = False  # 是否正在跟随视角
        # 修复3+8: 行人状态管理
        self.walker_controllers = {}   # {walker_id: controller_actor}
        self.walker_goals = {}         # {walker_id: {'last_loc':..., 'stuck':0, 'target':...}}
        self.walker_check_interval = 0
        # 场景后台任务（急刹/切入/逆行动态控制等）
        self.scenario_tasks = []       # [asyncio.Task, ...]
        self._tm_used = False          # 本进程一旦用过自动驾驶(TM)，运行时切图必崩，标记后拒绝切图

    async def connect(self, host='localhost', port=2000):
        """连接CARLA服务器"""
        try:
            self.host = host
            self.port = port
            self.client = carla.Client(host, port)
            self.client.set_timeout(10)
            self.world = self.client.get_world()
            if self.world is None:
                app_logger.error("❌ CARLA返回了空的world对象")
                return False
            app_logger.info("✅ CARLA服务器连接成功")
            return True
        except Exception as e:
            app_logger.error(f"❌ 连接CARLA失败: {str(e)}")
            return False
        
    async def load_world(self, map_name='Town05'):
        """加载指定地图"""
        try:
            if self.client is None:
                app_logger.error("❌ 未连接到CARLA服务器")
                return False
            # 已是目标地图则跳过（重复加载同一地图也会重建世界，耗时且可能超时）
            current = self.world.get_map().name if self.world is not None else ''
            if map_name in current:
                app_logger.info(f"✅ 地图已是 {map_name}，无需重复加载")
                return True
            if 'Town03' in map_name:
                app_logger.error("❌ Town03地图包在本环境加载即崩溃，已禁用")
                return False
            if getattr(self, '_tm_used', False):
                app_logger.error("❌ 本会话已使用过自动驾驶(Traffic Manager)，运行时切图会崩掉服务进程。请用目标地图重新启动CARLA后再连接")
                return False
            # 大地图加载常超过10s，临时调高超时时间
            self.client.set_timeout(90)
            self.world = self.client.load_world(map_name)
            self.client.set_timeout(10)
            app_logger.info(f"✅ 地图加载成功: {map_name}")
            return True
        except Exception as e:
            app_logger.error(f"❌ 加载地图失败: {str(e)}")
            return False

    async def set_synchronous_mode(self, enabled=True, fixed_delta_seconds=0.02):
        """设置同步模式 - 参考tuto_G_pedestrian_navigation.py"""
        try:
            if self.world is None:
                app_logger.error("❌ 未连接到CARLA服务器")
                return False
            
            settings = self.world.get_settings()
            settings.synchronous_mode = enabled
            if enabled:
                settings.fixed_delta_seconds = fixed_delta_seconds
            else:
                settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            
            if enabled:
                app_logger.info(f"✅ 同步模式已启用，固定时间步长: {fixed_delta_seconds}s")
            else:
                app_logger.info("✅ 同步模式已禁用")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置同步模式失败: {str(e)}")
            return False

    async def start_tick_loop(self):
        """启动后台tick循环，确保世界持续运行，并同步更新视角"""
        import asyncio
        if self.is_ticking:
            app_logger.info("⚠️ tick循环已在运行")
            return
        
        # 先启用同步模式
        await self.set_synchronous_mode(True, 0.05)
        
        self.is_ticking = True
        app_logger.info("🔄 启动后台tick循环")
        
        async def tick_loop():
            while self.is_ticking and self.world:
                try:
                    self.world.tick()
                    self.check_and_fix_stuck_walkers()
                    
                    # 🔑 关键修复：在每次tick后立即更新视角跟随
                    if self.is_view_following and self.view_target and self.view_target.is_alive:
                        if self.current_view_mode == "third_person":
                            self._update_third_person_view(self.view_target)
                        elif self.current_view_mode == "first_person":
                            self._update_first_person_view(self.view_target)
                        elif self.current_view_mode == "overhead":
                            self._update_overhead_view(self.view_target)
                    
                    await asyncio.sleep(0.05)
                except Exception as e:
                    app_logger.warning(f"⚠️ tick时出错: {e}")
                    await asyncio.sleep(0.1)
        
        self.tick_task = asyncio.create_task(tick_loop())

    async def stop_tick_loop(self):
        """停止后台tick循环"""
        self.is_ticking = False
        if self.tick_task:
            self.tick_task.cancel()
            try:
                await self.tick_task
            except asyncio.CancelledError:
                pass
            self.tick_task = None
        
        # 禁用同步模式
        await self.set_synchronous_mode(False)
        
        app_logger.info("🛑 停止后台tick循环")

    async def spawn_vehicles(self, vehicle_type='model3', count=1, autopilot=False):
        """生成多辆车辆，确保在车道内，返回详细ID列表"""
        count = int(count)  # ← 新增：强制转int，防止LLM传字符串
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        vehicle_blueprints = {
            'model3': 'vehicle.tesla.model3', 'a2': 'vehicle.audi.a2',
            'etron': 'vehicle.audi.etron', 'tt': 'vehicle.audi.tt',
            'grandtourer': 'vehicle.bmw.grandtourer', 'i8': 'vehicle.bmw.i8',
            'mini': 'vehicle.bmw.mini', 'impala': 'vehicle.chevrolet.impala',
            'c3': 'vehicle.citroen.c3', 'charger_police': 'vehicle.dodge.charger_police',
            'charger2020': 'vehicle.dodge.charger2020', 'mustang': 'vehicle.ford.mustang',
            'crown': 'vehicle.ford.crown', 'wrangler_rubicon': 'vehicle.jeep.wrangler_rubicon',
            'mkz_2017': 'vehicle.lincoln.mkz_2017', 'mkz_2020': 'vehicle.lincoln.mkz_2020',
            'benz_coupe': 'vehicle.mercedes.benz_coupe', 'cabrio': 'vehicle.mercedes.cabrio',
            'ccc': 'vehicle.mercedes.ccc', 'cooper_s': 'vehicle.mini.cooper_s',
            'micra': 'vehicle.nissan.micra', 'patrol': 'vehicle.nissan.patrol',
            'leon': 'vehicle.seat.leon', 't2': 'vehicle.volkswagen.t2',
            't3': 'vehicle.volkswagen.t3',
        }
        
        blueprint_path = vehicle_blueprints.get(vehicle_type.lower(), f'vehicle.tesla.{vehicle_type}')
        
        # 获取有效生成点
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        if not valid_points:
            app_logger.warning("⚠️ 没有可用的车辆生成点")
            return []
        
        blueprint_library = self.world.get_blueprint_library()
        spawned_vehicles = []
        
        # 遍历所有可用点，直到生成够 count 辆
        for i, transform in enumerate(valid_points):
            if len(spawned_vehicles) >= count:
                break
            
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if bp.id.startswith('vehicle.')]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    continue
                
                if blueprint.has_attribute('color'):
                    r, g, b = random.randint(0,255), random.randint(0,255), random.randint(0,255)
                    blueprint.set_attribute('color', f"{r},{g},{b}")
                
                if blueprint.has_attribute('role_name'):
                    blueprint.set_attribute('role_name', 'autopilot')
                
                # 小偏移避免位置被占（±0.5米，自动驾驶启动后会自动修正到车道中心）
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_loc = carla.Location(
                    x=transform.location.x + offset_x,
                    y=transform.location.y + offset_y,
                    z=transform.location.z + 0.5
                )
                new_transform = carla.Transform(new_loc, transform.rotation)
                vehicle = self.world.try_spawn_actor(blueprint, new_transform)
                
                if vehicle:
                    if autopilot:
                        vehicle.set_autopilot(True)  # noqa
                        self._tm_used = True
                    self.actors.append(vehicle)
                    spawned_vehicles.append(vehicle)
                    app_logger.info(f"🚗 [第{len(spawned_vehicles)}辆] ID={vehicle.id} | {blueprint.id} | 位置=({transform.location.x:.1f}, {transform.location.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 位置 ({transform.location.x:.1f}, {transform.location.y:.1f}) 被占用，尝试下一个点")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成出错: {e}")
                continue
        
        # 启动后台tick循环（视角跟随和自动驾驶都依赖它）
        if spawned_vehicles and not self.is_ticking:
            await self.start_tick_loop()
            app_logger.info("🔄 已启动后台tick循环")
        
        app_logger.info(f"✅ 共生成 {len(spawned_vehicles)} 辆车，ID列表: {[v.id for v in spawned_vehicles]}")
        return spawned_vehicles

    async def spawn_bicycles(self, bicycle_type='crossbike', count=1):
        count = int(count)  # ← 新增
        """生成多辆自行车，返回详细ID列表
        
        支持的自行车类型:
        - crossbike: BH Crossbike
        - century: Diamondback Century
        - omafiets: Gazelle Omafiets
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        bicycle_blueprints = {
            'crossbike': 'vehicle.bh.crossbike',
            'century': 'vehicle.diamondback.century',
            'omafiets': 'vehicle.gazelle.omafiets',
        }
        
        blueprint_path = bicycle_blueprints.get(bicycle_type.lower(), f'vehicle.bh.{bicycle_type}')
        
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        if not valid_points:
            app_logger.warning("⚠️ 没有可用的自行车生成点")
            return []
        
        blueprint_library = self.world.get_blueprint_library()
        spawned_bicycles = []
        
        for i, transform in enumerate(valid_points):
            if len(spawned_bicycles) >= count:
                break
            
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    # 回退到任意自行车蓝图
                    blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                                  if 'crossbike' in bp.id or 'diamondback' in bp.id or 'gazelle' in bp.id or 'bike' in bp.id]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    continue
                
                # 自行车通常没有颜色属性，跳过颜色设置
                # 小偏移避免位置被占
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_loc = carla.Location(
                    x=transform.location.x + offset_x,
                    y=transform.location.y + offset_y,
                    z=transform.location.z + 0.5
                )
                new_transform = carla.Transform(new_loc, transform.rotation)
                bicycle = self.world.try_spawn_actor(blueprint, new_transform)
                
                if bicycle:
                    self.actors.append(bicycle)
                    spawned_bicycles.append(bicycle)
                    app_logger.info(f"🚲 [第{len(spawned_bicycles)}辆] ID={bicycle.id} | {blueprint.id} | 位置=({transform.location.x:.1f}, {transform.location.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 位置 ({transform.location.x:.1f}, {transform.location.y:.1f}) 被占用，尝试下一个点")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成自行车出错: {e}")
                continue
        
        # 启动后台tick循环
        if spawned_bicycles and not self.is_ticking:
            await self.start_tick_loop()
            app_logger.info("🔄 已启动后台tick循环")
        
        app_logger.info(f"✅ 共生成 {len(spawned_bicycles)} 辆自行车，ID列表: {[b.id for b in spawned_bicycles]}")
        return spawned_bicycles     

    async def spawn_motorcycles(self, motorcycle_type='ninja', count=1):
        count = int(count)  # ← 新增
        """生成多辆摩托车，返回详细ID列表
        
        支持的摩托车类型:
        - ninja: Kawasaki Ninja
        - yzf: Yamaha YZF
        - low_rider: Harley-Davidson Low Rider
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        motorcycle_blueprints = {
            'ninja': 'vehicle.kawasaki.ninja',
            'yzf': 'vehicle.yamaha.yzf',
            'low_rider': 'vehicle.harley-davidson.low_rider',
        }
        
        blueprint_path = motorcycle_blueprints.get(motorcycle_type.lower(), f'vehicle.kawasaki.{motorcycle_type}')
        
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        if not valid_points:
            app_logger.warning("⚠️ 没有可用的摩托车生成点")
            return []
        
        blueprint_library = self.world.get_blueprint_library()
        spawned_motorcycles = []
        
        for i, transform in enumerate(valid_points):
            if len(spawned_motorcycles) >= count:
                break
            
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                                  if 'ninja' in bp.id or 'yzf' in bp.id or 'low_rider' in bp.id or 'harley' in bp.id]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    continue
                
                offset_x = random.uniform(-0.5, 0.5)
                offset_y = random.uniform(-0.5, 0.5)
                new_loc = carla.Location(
                    x=transform.location.x + offset_x,
                    y=transform.location.y + offset_y,
                    z=transform.location.z + 0.5
                )
                new_transform = carla.Transform(new_loc, transform.rotation)
                motorcycle = self.world.try_spawn_actor(blueprint, new_transform)
                
                if motorcycle:
                    self.actors.append(motorcycle)
                    spawned_motorcycles.append(motorcycle)
                    app_logger.info(f"🏍️ [第{len(spawned_motorcycles)}辆] ID={motorcycle.id} | {blueprint.id} | 位置=({transform.location.x:.1f}, {transform.location.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 位置 ({transform.location.x:.1f}, {transform.location.y:.1f}) 被占用，尝试下一个点")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成摩托车出错: {e}")
                continue
        
        if spawned_motorcycles and not self.is_ticking:
            await self.start_tick_loop()
            app_logger.info("🔄 已启动后台tick循环")
        
        app_logger.info(f"✅ 共生成 {len(spawned_motorcycles)} 辆摩托车，ID列表: {[m.id for m in spawned_motorcycles]}")
        return spawned_motorcycles

    async def spawn_props(self, prop_type='cone', count=1, location=None):
        count = int(count)  # ← 新增
        """生成静态道具（施工警示牌、三角警示牌、路障等）
        
        支持的道具类型:
        - cone: 施工锥/路锥
        - barrier: 路障/护栏
        - warning: 三角警示牌/交通警示牌
        - construction: 施工警示牌（如果有）
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return []
        
        prop_blueprints = {
            'cone': 'static.prop.constructioncone',
            'barrier': 'static.prop.barrier',
            'warning': 'static.prop.trafficwarning',
            'construction': 'static.prop.constructioncone',  # 回退到施工锥
        }
        
        blueprint_path = prop_blueprints.get(prop_type.lower(), f'static.prop.{prop_type}')
        blueprint_library = self.world.get_blueprint_library()
        spawned_props = []
        
        for i in range(count):
            try:
                blueprint = blueprint_library.find(blueprint_path)
                if blueprint is None:
                    # 回退到任意静态道具
                    blueprints = [bp for bp in blueprint_library.filter('static.prop.*') 
                                  if 'cone' in bp.id or 'barrier' in bp.id or 'warning' in bp.id]
                    blueprint = blueprints[i % len(blueprints)] if blueprints else None
                
                if blueprint is None:
                    app_logger.warning(f"⚠️ 找不到道具蓝图: {blueprint_path}")
                    continue
                
                # 如果指定了位置，使用指定位置；否则在车辆附近随机放置
                if location:
                    spawn_loc = carla.Location(
                        x=location.x + random.uniform(-2.0, 2.0),
                        y=location.y + random.uniform(-2.0, 2.0),
                        z=location.z + 0.1
                    )
                else:
                    # 默认在地图原点附近
                    spawn_loc = carla.Location(
                        x=random.uniform(-50, 50),
                        y=random.uniform(-50, 50),
                        z=0.1
                    )
                
                spawn_transform = carla.Transform(spawn_loc, carla.Rotation())
                prop = self.world.try_spawn_actor(blueprint, spawn_transform)
                
                if prop:
                    self.actors.append(prop)
                    spawned_props.append(prop)
                    app_logger.info(f"🚧 [第{len(spawned_props)}个] ID={prop.id} | {blueprint.id} | 位置=({spawn_loc.x:.1f}, {spawn_loc.y:.1f})")
                else:
                    app_logger.warning(f"⚠️ 道具生成失败，位置可能被占用")
                    
            except Exception as e:
                app_logger.error(f"❌ 生成道具出错: {e}")
                continue
        
        app_logger.info(f"✅ 共生成 {len(spawned_props)} 个道具，ID列表: {[p.id for p in spawned_props]}")
        return spawned_props

    async def spawn_overturned_vehicle(self, vehicle_type='model3', location=None):
        """生成仰翻的车辆（多重容错：遍历所有点 → 随机位置 → 手动空位）"""
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return None
        
        try:
            vehicle_type = str(vehicle_type).lower()
            
            vehicle_blueprints = {
                'model3': 'vehicle.tesla.model3',
                'mustang': 'vehicle.ford.mustang',
                'a2': 'vehicle.audi.a2',
                'charger_police': 'vehicle.dodge.charger_police',
            }
            
            blueprint_path = vehicle_blueprints.get(vehicle_type, 'vehicle.tesla.model3')
            blueprint_library = self.world.get_blueprint_library()
            blueprint = blueprint_library.find(blueprint_path)
            
            if blueprint is None:
                app_logger.error(f"❌ 找不到蓝图: {blueprint_path}")
                return None
            
            # 收集所有可能的生成位置
            spawn_locations = []
            
            # 1. 使用传入的位置
            if location:
                spawn_locations.append(location)
            
            # 2. 使用地图车辆生成点
            map_spawn_points = self.world.get_map().get_spawn_points()
            if map_spawn_points:
                random.shuffle(map_spawn_points)
                for sp in map_spawn_points[:20]:  # 取前20个随机点
                    spawn_locations.append(sp.location)
            
            # 3. 使用导航随机位置
            for _ in range(10):
                nav_loc = self.world.get_random_location_from_navigation()
                if nav_loc:
                    spawn_locations.append(nav_loc)
            
            # 4. 手动指定几个空位（地图中心附近）
            for offset in [(0,0), (10,0), (-10,0), (0,10), (0,-10), (20,20), (-20,-20)]:
                spawn_locations.append(carla.Location(x=offset[0], y=offset[1], z=0.5))
            
            if not spawn_locations:
                app_logger.error("❌ 没有任何可用生成位置")
                return None
            
            # 目标翻转姿态
            overturned_rotation = carla.Rotation(pitch=0, yaw=random.uniform(0, 360), roll=180)
            
            # 遍历所有位置尝试生成
            for idx, spawn_loc in enumerate(spawn_locations):
                try:
                    # 确保z轴有足够高度
                    spawn_loc.z = max(spawn_loc.z, 0.5)
                    
                    # 方案A：直接仰翻 spawn
                    spawn_transform = carla.Transform(spawn_loc, overturned_rotation)
                    vehicle = self.world.try_spawn_actor(blueprint, spawn_transform)
                    
                    if vehicle:
                        vehicle.set_simulate_physics(False)
                        self.actors.append(vehicle)
                        app_logger.info(f"🚗💥 仰翻车辆生成成功: {vehicle_type} (ID: {vehicle.id}, 尝试{idx+1}次)")
                        return vehicle
                    
                    # 方案B：正常 spawn 后翻转
                    normal_transform = carla.Transform(spawn_loc, carla.Rotation())
                    vehicle = self.world.try_spawn_actor(blueprint, normal_transform)
                    
                    if vehicle:
                        vehicle.set_simulate_physics(False)
                        final_loc = vehicle.get_location()
                        vehicle.set_transform(carla.Transform(final_loc, overturned_rotation))
                        self.actors.append(vehicle)
                        app_logger.info(f"🚗💥 仰翻车辆生成成功(翻转): {vehicle_type} (ID: {vehicle.id}, 尝试{idx+1}次)")
                        return vehicle
                        
                except Exception as e:
                    app_logger.warning(f"⚠️ 位置{idx+1} ({spawn_loc.x:.1f}, {spawn_loc.y:.1f}) 失败: {e}")
                    continue
            
            app_logger.error(f"❌ 仰翻车辆生成彻底失败，已尝试{len(spawn_locations)}个位置")
            return None
            
        except Exception as e:
            app_logger.error(f"❌ 生成仰翻车辆异常: {e}")
            import traceback
            app_logger.error(traceback.format_exc())
            return None

    async def spawn_vehicle(self, vehicle_type='model3'):
        """生成单辆车辆（兼容旧接口）"""
        vehicles = await self.spawn_vehicles(vehicle_type, count=1)
        return vehicles[0] if vehicles else None

    async def set_weather(self, weather_type='clear'):
        """设置天气"""
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return False
        
        weather_presets = {
            'clear': carla.WeatherParameters(
                cloudiness=0, precipitation=0, precipitation_deposits=0,
                wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=75,
                fog_density=0, fog_distance=0, wetness=0
            ),
            'rain': carla.WeatherParameters(
                cloudiness=100, precipitation=80, precipitation_deposits=50,
                wind_intensity=30, sun_azimuth_angle=0, sun_altitude_angle=15,
                fog_density=10, fog_distance=100, wetness=60
            ),
            'fog': carla.WeatherParameters(
                cloudiness=80, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=30,
                fog_density=90, fog_distance=50, wetness=20
            ),
            'light_fog': carla.WeatherParameters(  # ← 新增：薄雾
                cloudiness=60, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=45,
                fog_density=25, fog_distance=80, wetness=10, fog_falloff=2.0
            ),
            'snow': carla.WeatherParameters(
                cloudiness=80, precipitation=60, precipitation_deposits=80,
                wind_intensity=20, sun_azimuth_angle=0, sun_altitude_angle=10,
                fog_density=20, fog_distance=200, wetness=30
            ),
            'night': carla.WeatherParameters(
                cloudiness=20, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
                fog_density=0, fog_distance=0, wetness=0
            ),
            'dawn': carla.WeatherParameters(  # 清晨弱光
                cloudiness=40, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=90, sun_altitude_angle=8,
                fog_density=5, fog_distance=100, wetness=0
            ),
            'dusk': carla.WeatherParameters(  # 黄昏弱光
                cloudiness=50, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=270, sun_altitude_angle=6,
                fog_density=5, fog_distance=100, wetness=0
            ),
            'overcast': carla.WeatherParameters(  # 阴天弱光
                cloudiness=95, precipitation=0, precipitation_deposits=0,
                wind_intensity=15, sun_azimuth_angle=0, sun_altitude_angle=35,
                fog_density=0, fog_distance=0, wetness=0
            ),
        }
        if weather_type in weather_presets:
            self.world.set_weather(weather_presets[weather_type])
            app_logger.info(f"✅ 天气已设置为 {weather_type}")
            return True
        return False

    async def get_traffic_lights(self):
        """获取交通灯状态"""
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return []
        
        lights = [light for light in self.world.get_actors() if 'traffic_light' in light.type_id]
        return lights[:5]  # 只返回前5个

    async def spawn_pedestrians(self, pedestrian_type='pedestrian', count=1, speed=None):
        """生成多个行人，返回生成的行人列表和最后一个行人
        
        支持的行人类型:
        - pedestrian: 普通行人
        - elderly: 老年人
        - child: 儿童
        - police: 警察
        - business: 商务人士
        - jogger: 慢跑者
        
        参数:
        - speed: 行人移动速度（m/s），默认为1.4（正常步行速度），慢跑者默认为2.8
        
        数据量支持: 取决于地图大小，通常支持10-100+个行人
        """
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return []
        
        try:
            # 行人类型到蓝图编号的映射
            pedestrian_blueprint_map = {
                'police': ['0030', '0032'],
                'child': ['0009', '0010', '0011', '0012', '0013', '0014', '0048', '0049'],
                'elderly': ['0020', '0021', '0022', '0023', '0024', '0025'],
                'business': ['0027', '0028', '0029'],
                'pedestrian': ['0001', '0002', '0003', '0004', '0005', '0006', '0007', '0008', 
                               '0015', '0016', '0017', '0018', '0019', '0026', '0031', '0033', 
                               '0034', '0035', '0036', '0037', '0038', '0039', '0040', '0041', 
                               '0042', '0043', '0044', '0045', '0046', '0047'],
                'jogger': ['0001', '0002', '0003', '0004', '0005', '0006', '0007', '0008', 
                           '0015', '0016', '0017', '0018', '0019', '0026', '0031', '0033', 
                           '0034', '0035', '0036', '0037', '0038', '0039', '0040', '0041', 
                           '0042', '0043', '0044', '0045', '0046', '0047']
            }
            
            # 根据行人类型设置默认速度
            if speed is None:
                if pedestrian_type == 'jogger':
                    speed = 2.8  # 慢跑者默认速度
                elif pedestrian_type == 'elderly':
                    speed = 1.0  # 老年人默认速度较慢
                else:
                    speed = 1.4  # 正常步行速度
            
            # 获取当前行人类型对应的蓝图编号列表
            blueprint_numbers = pedestrian_blueprint_map.get(pedestrian_type, pedestrian_blueprint_map['pedestrian'])
            
            # 检查地图是否支持行人导航
            spawn_location = self.world.get_random_location_from_navigation()
            if spawn_location:
                app_logger.info(f"✅ 地图支持行人导航，测试位置: {spawn_location}")
            else:
                app_logger.warning("⚠️ 警告: 地图可能不支持行人导航，get_random_location_from_navigation()返回None")
                app_logger.warning("⚠️ 建议: 尝试加载Town05地图（client.load_world('Town05')）")
            
            # 获取控制器蓝图
            controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            if not controller_bp:
                app_logger.error("❌ 无法找到行人控制器蓝图")
                return []
            
            spawned_pedestrians = []
            
            for i in range(count):
                try:
                    # 从指定类型的蓝图编号中随机选择一个
                    blueprint_number = random.choice(blueprint_numbers)
                    blueprint_id = f'walker.pedestrian.{blueprint_number}'
                    
                    # 查找指定的行人蓝图
                    blueprint_library = self.world.get_blueprint_library()
                    pedestrian_bp = blueprint_library.find(blueprint_id)
                    
                    if not pedestrian_bp:
                        app_logger.error(f"❌ 无法找到行人蓝图: {blueprint_id}")
                        continue
                    
                    app_logger.info(f"📋 尝试生成行人，蓝图: {pedestrian_bp.id}")
                    
                    # 如果是老年人，随机设置轮椅
                    if pedestrian_type == 'elderly':
                        if pedestrian_bp.has_attribute('can_use_wheelchair'):
                            if random.random() < 0.3:  # 30%的概率使用轮椅
                                pedestrian_bp.set_attribute('use_wheelchair', 'True')
                                app_logger.info(f"♿ 为老年人设置轮椅")
                    
                    # 尝试多个位置生成行人
                    spawn_success = False
                    for attempt in range(3):  # 尝试3次
                        try:
                            # 随机生成位置
                            spawn_location = self.world.get_random_location_from_navigation()
                            if not spawn_location:
                                # 如果无法获取随机位置，使用默认位置
                                spawn_location = carla.Location(x=-134 + i*2, y=78.1, z=1.18)
                            
                            spawn_transform = carla.Transform(spawn_location)
                            app_logger.info(f"📍 尝试在位置生成: {spawn_location}")
                            
                            # 生成行人
                            pedestrian = self.world.try_spawn_actor(pedestrian_bp, spawn_transform)
                            if pedestrian:
                                app_logger.info(f"✅ 行人生成成功: {pedestrian.id}")
                                
                                # 为行人设置AI控制器 - 参考tuto_G_pedestrian_navigation.py
                                try:
                                    # 使用行人的变换作为控制器的生成位置
                                    controller = self.world.spawn_actor(controller_bp, pedestrian.get_transform(), pedestrian)
                                    if controller:
                                        app_logger.info(f"✅ 控制器生成成功: {controller.id}")
                                        # 启动控制器并给它一个随机位置
                                        controller.start()
                                        target_location = self.world.get_random_location_from_navigation()
                                        if target_location is None:
                                            # 如果导航网格返回None，用行人当前位置附近
                                            current_loc = pedestrian.get_location()
                                            target_location = carla.Location(
                                                x=current_loc.x + random.uniform(-20, 20),
                                                y=current_loc.y + random.uniform(-20, 20),
                                                z=current_loc.z
                                            )
                                        controller.go_to_location(target_location)
                                        controller.set_max_speed(speed)
                                        app_logger.info(f"🎯 为行人设置随机目标位置，速度: {speed} m/s")
                                        
                                        # 修复3+8: 保存控制器引用
                                        self.walker_controllers[pedestrian.id] = controller
                                        self.walker_goals[pedestrian.id] = {
                                            'last_location': pedestrian.get_location(),
                                            'stuck_count': 0,
                                            'target': target_location
                                        }

                                        # 存储控制器和行人的关联关系
                                        self.actors.append(pedestrian)
                                        self.actors.append(controller)
                                        spawned_pedestrians.append(pedestrian)
                                        app_logger.info(f"🚶 生成第{i+1}个行人: {pedestrian_bp.id} (ID: {pedestrian.id})")
                                        
                                        # 将世界移动几帧，让行人生成 - 参考tuto_G_pedestrian_navigation.py
                                        for frame in range(0, 5):
                                            try:
                                                self.world.tick()
                                            except Exception as tick_error:
                                                app_logger.warning(f"⚠️ 推进世界时出错: {tick_error}")
                                                continue
                                        
                                        spawn_success = True
                                        break
                                    else:
                                        # 如果控制器生成失败，销毁行人
                                        if pedestrian.is_alive:
                                            pedestrian.destroy()
                                        app_logger.error(f"❌ 为第{i+1}个行人创建控制器失败")
                                except Exception as ctrl_error:
                                    # 如果控制器生成失败，销毁行人
                                    if pedestrian.is_alive:
                                        pedestrian.destroy()
                                    app_logger.error(f"❌ 控制器生成异常: {ctrl_error}")
                            else:
                                app_logger.warning(f"⚠️ 尝试 {attempt+1}/3: 生成行人失败，位置可能被占用")
                                
                        except Exception as loc_error:
                            app_logger.error(f"❌ 位置生成时出错: {loc_error}")
                            continue
                    
                    if not spawn_success:
                        app_logger.error(f"❌ 生成第{i+1}个行人失败，已尝试3个位置")
                        
                except Exception as e:
                    app_logger.error(f"❌ 生成第{i+1}个行人时出错: {str(e)}")
                    continue
            
            app_logger.info(f"✅ 共生成{len(spawned_pedestrians)}个行人")
            
            # 启动后台tick循环，确保行人持续移动
            if spawned_pedestrians and not self.is_ticking:
                await self.start_tick_loop()
                app_logger.info("🔄 已启动后台tick循环，行人将开始移动")
            
            # 再次确保所有行人控制器都有目标位置
            if spawned_pedestrians:
                import asyncio
                await asyncio.sleep(0.5)  # 等待一小段时间让控制器初始化
                for pedestrian in spawned_pedestrians:
                    try:
                        # 获取行人的控制器
                        controller = pedestrian.get_control()
                        if controller:
                            # 重新设置随机目标位置
                            target_location = self.world.get_random_location_from_navigation()
                            if target_location:
                                # 通过walker的controller来设置目标
                                walker_controller = None
                                for actor in self.world.get_actors():
                                    if 'controller.ai.walker' in actor.type_id:
                                        # 检查这个控制器是否附着到当前行人
                                        try:
                                            if hasattr(actor, 'parent') and actor.parent == pedestrian:
                                                walker_controller = actor
                                                break
                                        except:
                                            pass
                                
                                if walker_controller:
                                    walker_controller.go_to_location(target_location)
                                    walker_controller.set_max_speed(speed)
                                    app_logger.info(f"🚶 为行人 {pedestrian.id} 重新设置目标位置，速度: {speed} m/s")
                    except Exception as e:
                        app_logger.warning(f"⚠️ 为行人设置目标时出错: {e}")
                        continue
            
            return spawned_pedestrians
            
        except Exception as e:
            app_logger.error(f"❌ 生成行人失败: {str(e)}")
            return []

    async def spawn_pedestrian(self, pedestrian_type='walker', speed=None):
        """生成单个行人（兼容旧接口）"""
        pedestrians = await self.spawn_pedestrians(pedestrian_type, count=1, speed=speed)
        return pedestrians[0] if pedestrians else None

    def set_spectator_view(self, target_actor):
        """将视角对准目标actor"""
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False
        
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            
            # 设置相机位置在目标actor前方5米，上方2米处
            # 这样可以从正面看到行人
            camera_location = carla.Location(
                x=target_transform.location.x + 5.0,  # 前方5米
                y=target_transform.location.y,
                z=target_transform.location.z + 2.0
            )
            
            # 计算相机朝向，指向行人
            # yaw=180.0 让相机朝向行人方向
            camera_rotation = carla.Rotation(
                pitch=-15.0,  # 略微向下看
                yaw=180.0,    # 朝向行人
                roll=0.0
            )
            
            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  视角已对准actor {target_actor.id}")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置视角失败: {str(e)}")
            return False

    async def setup_autopilot(self, enable=True, radius=0.0):
        """设置车辆自动驾驶模式"""
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器")
            return False
        
        try:
            vehicles = [actor for actor in self.world.get_actors() if 'vehicle' in actor.type_id]
            
            enabled_count = 0
            for vehicle in vehicles:
                if radius > 0:
                    spectator = self.world.get_spectator()
                    if spectator.get_location().distance(vehicle.get_location()) > radius:
                        continue
                
                # 直接启用自动驾驶，CARLA 会自动走默认 Traffic Manager
                vehicle.set_autopilot(enable)
                enabled_count += 1
                app_logger.info(f"🚗 车辆 {vehicle.id} 自动驾驶已{'启用' if enable else '禁用'}")
            
            app_logger.info(f"✅ {'启用' if enable else '禁用'}了 {enabled_count} 辆车的自动驾驶")
            return True
            
        except Exception as e:
            app_logger.error(f"❌ 设置自动驾驶失败: {str(e)}")
            return False

    async def setup_pedestrian_movement(self, enable=True, radius=0.0):
        """设置行人自动移动
        
        Args:
            enable: 是否启用行人移动
            radius: 移动范围半径（米），0表示全图
        """
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return False
        
        try:
            # 获取所有行人控制器 - 参考官方文档
            controllers = [actor for actor in self.world.get_actors() if 'controller.ai.walker' in actor.type_id]
            
            updated_count = 0
            for controller in controllers:
                try:
                    if enable:
                        # 启用控制器并设置随机目标位置
                        controller.start()
                        target_location = self.world.get_random_location_from_navigation()
                        if target_location:
                            controller.go_to_location(target_location)
                            app_logger.info(f"🚶 行人控制器 {controller.id} 已启用并设置目标: {target_location}")
                            updated_count += 1
                        else:
                            app_logger.warning(f"⚠️ 无法获取随机目标位置")
                    else:
                        # 禁用控制器
                        controller.stop()
                        app_logger.info(f"🚶 行人控制器 {controller.id} 已禁用")
                        updated_count += 1
                except Exception as ctrl_error:
                    app_logger.error(f"❌ 操作控制器 {controller.id} 时出错: {ctrl_error}")
                    continue
            
            app_logger.info(f"✅ {'启用' if enable else '禁用'}了 {updated_count} 个行人控制器")
            return True
            
        except Exception as e:
            app_logger.error(f"❌ 设置行人移动失败: {str(e)}")
            return False

     # ============ 修复6: 获取车道内有效生成点 ============
    def get_valid_vehicle_spawn_points(self, safe_mode=True):
        """获取有效的车辆生成点（仅在Driving车道内）"""
        carla_map = self.world.get_map()
        all_spawn_points = carla_map.get_spawn_points()
        valid_points = []
        for sp in all_spawn_points:
            waypoint = carla_map.get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            if waypoint is None or waypoint.lane_type != carla.LaneType.Driving:
                continue
            if safe_mode:
                nearby = [a for a in self.world.get_actors().filter("vehicle.*")
                          if a.get_location().distance(sp.location) < 5.0]
                if nearby:
                    continue
            valid_points.append(waypoint.transform)
        return valid_points

     # ============ 修复7: 批量生成车辆并返回详细ID ============
    async def batch_spawn_vehicles_with_id(self, count=10, blueprint_filter="vehicle.*", autopilot=True):
        """批量生成车辆，返回带ID的详细列表"""
        result = {"total": count, "success": 0, "failed": 0, "vehicles": []}
        blueprints = [bp for bp in self.world.get_blueprint_library().filter(blueprint_filter)
                      if bp.id.startswith("vehicle.")]
        valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
        
        for i in range(min(count, len(valid_points))):
            blueprint = random.choice(blueprints)
            color = "default"
            if blueprint.has_attribute("color"):
                color = f"{random.randint(0,255)},{random.randint(0,255)},{random.randint(0,255)}"
                blueprint.set_attribute("color", color)
            transform = valid_points[i]
            try:
                vehicle = self.world.try_spawn_actor(blueprint, transform)
                if vehicle:
                    if autopilot:
                        vehicle.set_autopilot(True)  # noqa
                        self._tm_used = True
                    self.actors.append(vehicle)
                    info = {
                        "index": i + 1,
                        "id": vehicle.id,
                        "type_id": vehicle.type_id,
                        "blueprint": blueprint.id,
                        "color": color,
                        "location": {
                            "x": round(transform.location.x, 2),
                            "y": round(transform.location.y, 2),
                            "z": round(transform.location.z, 2)
                        },
                        "autopilot": autopilot
                    }
                    result["vehicles"].append(info)
                    result["success"] += 1
            except Exception as e:
                result["failed"] += 1
        
        app_logger.info("=" * 50)
        app_logger.info(f"[修复7] 批量生成: 成功={result['success']}, 失败={result['failed']}")
        for v in result["vehicles"]:
            app_logger.info(f"  [{v['index']}] ID={v['id']:>4} | {v['type_id']:<35} | ({v['location']['x']}, {v['location']['y']})")
        app_logger.info("=" * 50)
        return result

        # ============ 修复5: 参数化生成actor ============
    async def spawn_vehicles_with_params(self, params):
        """根据参数面板生成actor"""
        blueprint_library = self.world.get_blueprint_library()
        carla_map = self.world.get_map()
        spawned = []
        
        if params.actor_type == "vehicle":
            blueprints = [bp for bp in blueprint_library.filter(params.blueprint_filter) if bp.id.startswith("vehicle.")]
        else:
            blueprints = blueprint_library.filter("walker.pedestrian.*")
        
        if not blueprints:
            app_logger.warning("⚠️ [参数化] 未找到匹配蓝图")
            return spawned
        
        spawn_locations = []
        if params.reference_actor_id is not None:
            ref_actor = self.world.get_actor(params.reference_actor_id)
            if ref_actor and ref_actor.is_alive:
                ref_loc = ref_actor.get_transform().location
                app_logger.info(f"🔍 [参数化] 参照物位置: ({ref_loc.x:.1f}, {ref_loc.y:.1f})")
                for i in range(params.count):
                    angle_rad = math.radians(params.relative_angle + i * (360 / max(params.count, 1)))
                    dist = params.relative_distance
                    sx = ref_loc.x + dist * math.cos(angle_rad) + random.uniform(-3.0, 3.0)
                    sy = ref_loc.y + dist * math.sin(angle_rad) + random.uniform(-3.0, 3.0)
                    spawn_loc = carla.Location(x=sx, y=sy, z=ref_loc.z + 0.5)
                    
                    if params.lane_type == "Driving":
                        wp = carla_map.get_waypoint(spawn_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                    elif params.lane_type == "Sidewalk":
                        wp = carla_map.get_waypoint(spawn_loc, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
                    else:
                        wp = carla_map.get_waypoint(spawn_loc, project_to_road=True)
                    
                    if wp:
                        # 在waypoint位置基础上再加偏移，避免被占用
                        final_loc = carla.Location(
                            x=wp.transform.location.x + random.uniform(-2.0, 2.0),
                            y=wp.transform.location.y + random.uniform(-2.0, 2.0),
                            z=wp.transform.location.z + 0.5
                        )
                        spawn_locations.append(final_loc)
                        app_logger.info(f"🔍 [参数化] 位置{i+1}: waypoint+偏移=({final_loc.x:.1f}, {final_loc.y:.1f})")
                    else:
                        spawn_locations.append(spawn_loc)
                        app_logger.info(f"🔍 [参数化] 位置{i+1}: 原始位置=({spawn_loc.x:.1f}, {spawn_loc.y:.1f})")
            else:
                app_logger.warning(f"⚠️ [参数化] 参照物ID={params.reference_actor_id} 不存在，回退到地图生成点")
                valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
                for i in range(min(params.count, len(valid_points))):
                    spawn_locations.append(valid_points[i].location)
        else:
            valid_points = self.get_valid_vehicle_spawn_points(safe_mode=True)
            for i in range(min(params.count, len(valid_points))):
                spawn_locations.append(valid_points[i].location)
        
        app_logger.info(f"🔍 [参数化] 请求{params.count}辆, 实际位置数{len(spawn_locations)}")
        
        for i, loc in enumerate(spawn_locations):
            blueprint = random.choice(blueprints)
            if params.actor_type == "vehicle" and blueprint.has_attribute("color"):
                r, g, b = random.randint(0,255), random.randint(0,255), random.randint(0,255)
                blueprint.set_attribute('color', f"{r},{g},{b}")
            
            transform = carla.Transform(loc, carla.Rotation())
            try:
                actor = self.world.try_spawn_actor(blueprint, transform)
                if actor:
                    if params.initial_speed > 0:
                        actor.set_target_velocity(carla.Vector3D(x=params.initial_speed, y=0, z=0))
                    if params.actor_type == "vehicle" and params.autopilot:
                        actor.set_autopilot(True)  # noqa
                        self._tm_used = True
                    self.actors.append(actor)
                    spawned.append(actor)
                    app_logger.info(f"🚗 [参数化] 第{i+1}辆成功: ID={actor.id}")
                else:
                    app_logger.warning(f"⚠️ [参数化] 第{i+1}辆失败: 位置被占用 ({loc.x:.1f}, {loc.y:.1f})")
            except Exception as e:
                app_logger.warning(f"❌ [参数化] 第{i+1}辆异常: {e}")
        
        app_logger.info(f"🔍 [参数化] 完成: 请求{params.count}辆, 成功{len(spawned)}辆")
        return spawned

    # ============================================================
    # 第1周场景任务: 高速进出匝道 / 城市车道合并 / 分合流路口 / 辅路
    # ============================================================
    def _get_driving_waypoints(self, distance=4.0):
        """获取地图上所有 Driving 车道 waypoint"""
        carla_map = self.world.get_map()
        return [wp for wp in carla_map.generate_waypoints(distance)
                if wp.lane_type == carla.LaneType.Driving]

    @staticmethod
    def _group_waypoints_by_road(waypoints):
        """按 (road_id, section_id) 分组"""
        groups = {}
        for wp in waypoints:
            groups.setdefault((wp.road_id, wp.section_id), []).append(wp)
        return groups

    @staticmethod
    def _road_length(waypoints):
        """用 s 值估计道路长度"""
        s_values = [wp.s for wp in waypoints]
        return max(s_values) - min(s_values) if s_values else 0.0

    def _follow_lane_all(self, start_wp, step=10.0, max_hops=25, max_visited=300):
        """沿车道向前多分支跟随（路口/分流处自动分叉），
        返回 {road_id: 首个到达该 road 的 waypoint}"""
        reached = {}
        frontier = [start_wp]
        visited = set()
        for _ in range(max_hops):
            new_frontier = []
            for wp in frontier:
                key = (wp.road_id, wp.section_id, round(wp.s, 0), wp.lane_id)
                if key in visited:
                    continue
                visited.add(key)
                reached.setdefault(wp.road_id, wp)
                try:
                    new_frontier.extend(wp.next(step))
                except Exception:
                    pass
            frontier = [w for w in new_frontier
                        if (w.road_id, w.section_id, round(w.s, 0), w.lane_id) not in visited]
            if not frontier or len(visited) >= max_visited:
                break
        return reached

    def _lane_terminal(self, start_wp, step=4.0, max_hops=80):
        """沿同一 road 同一 lane_id 跟随直到无法继续，返回该车道终点 waypoint"""
        current = start_wp
        visited = set()
        for _ in range(max_hops):
            key = (current.road_id, current.section_id, round(current.s, 0), current.lane_id)
            if key in visited:
                break
            visited.add(key)
            try:
                nxt = current.next(step)
            except Exception:
                return current
            same = [w for w in nxt if w.road_id == current.road_id and w.lane_id == current.lane_id]
            if not same:
                return current
            current = same[0]
        return current

    def _spawn_vehicle_on_waypoint(self, waypoint, autopilot=True, tag=""):
        """在指定 waypoint 生成一辆车（位置被占用则向后找空位），返回 actor 或 None"""
        blueprint_library = self.world.get_blueprint_library()
        blueprints = [bp for bp in blueprint_library.filter("vehicle.*") if bp.id.startswith("vehicle.")]
        if not blueprints:
            return None
        candidates = [waypoint]
        candidates.extend(waypoint.previous(8.0) or [])
        candidates.extend(waypoint.previous(16.0) or [])
        candidates.extend(waypoint.previous(24.0) or [])
        for wp in candidates:
            loc = wp.transform.location
            if any(a.get_location().distance(loc) < 6.0 for a in self.world.get_actors().filter("vehicle.*")):
                continue
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                blueprint.set_attribute("color", f"{random.randint(0,255)},{random.randint(0,255)},{random.randint(0,255)}")
            try:
                actor = self.world.try_spawn_actor(blueprint, wp.transform)
            except Exception as e:
                app_logger.warning(f"⚠️ [场景{tag}] 生成异常: {e}")
                continue
            if actor:
                if autopilot:
                    actor.set_autopilot(True)  # noqa
                    self._tm_used = True
                self.actors.append(actor)
                return actor
        return None

    def _set_spectator_overhead(self, location, height=45.0):
        """把观察者相机设置为某位置的俯视视角"""
        try:
            spectator = self.world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=location.x, y=location.y, z=location.z + height),
                carla.Rotation(pitch=-90.0)))
        except Exception as e:
            app_logger.warning(f"⚠️ 设置俯视视角失败: {e}")

    async def _ensure_map(self, map_name):
        """切换到指定地图（仅在本进程尚未使用过Traffic Manager时安全）。

        实测结论（CARLA 0.9.16 Windows）：
        - 一旦本进程使用过自动驾驶(TM)，任何后续切图都会触发原生断言
          Simulator.h:124 `_episode != nullptr` 直接崩掉 web 进程，无法规避；
        - Town03 地图包在本环境加载即导致 CARLA 服务器崩溃， permanently 禁用。
        因此：用过TM后拒绝切图并给出明确指引；未用过TM时可在干净会话内切图。
        """
        if not map_name:
            return True
        try:
            current = self.world.get_map().name
            if map_name in current:
                return True
            if 'Town03' in map_name:
                app_logger.error(f"❌ [场景] Town03地图包在本环境加载即崩溃，已禁用: {map_name}")
                return False
            if getattr(self, '_tm_used', False):
                app_logger.error(
                    f"❌ [场景] 本会话已使用过自动驾驶(Traffic Manager)，运行时切图会崩掉服务进程。"
                    f"请改用启动参数加载 {map_name} 启动CARLA，并重启网页服务后再试")
                return False
            app_logger.info(f"🗺️ [场景] 干净会话内加载地图 {map_name} (当前: {current})")
            self.client.set_timeout(90)
            self.world = self.client.load_world(map_name)
            self.client.set_timeout(10)
            app_logger.info(f"✅ 地图加载成功: {map_name}")
            return True
        except Exception as e:
            app_logger.error(f"❌ [场景] 加载地图失败: {e}")
            return False

    async def scenario_highway_ramp(self, ramp_type='on', vehicle_count=4, map_name=None):
        """高速-进出匝道场景。
        ramp_type='on': 匝道车辆汇入主路；'off': 主路车辆驶出匝道。
        自动寻找多车道高速主路与单车道匝道的连接点布设车辆。
        """
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        recommended = "Town04"
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(4.0)
        groups = self._group_waypoints_by_road(waypoints)
        main_road_ids = set(k[0] for k, v in groups.items() if len({w.lane_id for w in v}) >= 2)
        if not main_road_ids:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到多车道道路（高速），建议使用 {recommended}"}

        # 单车道匝道候选（长度 20~500m）
        ramp_candidates = []
        for (road_id, section_id), wps in groups.items():
            if len({w.lane_id for w in wps}) != 1:
                continue
            length = self._road_length(wps)
            if 20.0 <= length <= 500.0:
                ramp_candidates.append(sorted(wps, key=lambda w: w.s))
        if not ramp_candidates:
            return {"success": False, "error": f"当前地图 {current_map} 未找到匝道结构，建议使用 {recommended}"}

        scene = None
        if ramp_type == 'on':
            # 从匝道入口前向跟随，能找到主路 → 汇入型匝道
            for wps in ramp_candidates:
                reached = self._follow_lane_all(wps[0], step=10.0, max_hops=30)
                main_hits = [rid for rid in reached if rid in main_road_ids]
                if main_hits:
                    scene = {"ramp_wps": wps, "point": reached[main_hits[0]], "reached": reached}
                    break
        else:
            # 从主路各车道前向跟随（多分支），能找到匝道 → 驶出型匝道
            for (road_id, section_id), wps in groups.items():
                if road_id not in main_road_ids:
                    continue
                for wp in wps[::max(1, len(wps) // 6)]:
                    reached = self._follow_lane_all(wp, step=10.0, max_hops=25)
                    ramp_hits = [rid for rid in reached
                                 if rid not in main_road_ids
                                 and any(rid == c[0].road_id for c in ramp_candidates)]
                    if ramp_hits:
                        ramp_first = reached[ramp_hits[0]]
                        ramp_wps = next(c for c in ramp_candidates if c[0].road_id == ramp_hits[0])
                        scene = {"ramp_wps": ramp_wps, "point": wp, "fork_wp": ramp_first}
                        break
                if scene:
                    break

        if not scene:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到{'汇入' if ramp_type == 'on' else '驶出'}型匝道，建议换用 {recommended}"}

        spawned = []
        desc = []
        # 匝道侧车辆
        ramp_count = max(1, vehicle_count // 3)
        ramp_wps = scene["ramp_wps"]
        step_idx = max(1, len(ramp_wps) // (ramp_count + 1))
        for i in range(ramp_count):
            wp = ramp_wps[min((i + 1) * step_idx, len(ramp_wps) - 1)]
            actor = self._spawn_vehicle_on_waypoint(wp, autopilot=True, tag="匝道")
            if actor:
                spawned.append(actor)
                desc.append(f"匝道车 ID={actor.id} @road{wp.road_id} s={wp.s:.0f}")
        # 主路车辆：在汇流/分流点之前的主路车道上
        main_count = max(1, vehicle_count - ramp_count)
        point = scene["point"]
        lane_offsets = [0.0, 8.0, 16.0, 24.0]
        for i in range(main_count):
            back = lane_offsets[i % len(lane_offsets)]
            prev = point.previous(back) if back > 0 else [point]
            if not prev:
                continue
            wp = prev[0]
            actor = self._spawn_vehicle_on_waypoint(wp, autopilot=True, tag="主路")
            if actor:
                spawned.append(actor)
                desc.append(f"主路车 ID={actor.id} @road{wp.road_id} lane={wp.lane_id} s={wp.s:.0f}")

        self._set_spectator_overhead(point.transform.location, height=50.0)
        ramp_desc = "汇入(on-ramp)" if ramp_type == 'on' else "驶出(off-ramp)"
        return {
            "success": bool(spawned),
            "scenario": f"高速-进出匝道({ramp_desc})",
            "map": current_map,
            "merge_point": {"x": round(point.transform.location.x, 1),
                            "y": round(point.transform.location.y, 1)},
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_lane_merge(self, vehicle_count=4, map_name=None):
        """城市-车道合并场景：找到车道数减少（车道消失）的位置，
        在消失车道与延续车道上布设车辆，演示汇流。"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(4.0)
        groups = self._group_waypoints_by_road(waypoints)

        # 找"车道消失点"：同路段中某车道的终点 s 明显短于其他车道
        # （该车道提前结束=汇入邻道，其他车道可走到路段末尾）
        drop_point = None
        drop_lane_id = None
        cont_lane_wp = None
        for (road_id, section_id), wps in groups.items():
            if len({w.lane_id for w in wps}) < 2:
                continue
            if self._road_length(wps) < 40.0:
                continue
            by_lane = {}
            for w in wps:
                by_lane.setdefault(w.lane_id, []).append(w)
            road_max_s = max(w.s for w in wps)
            terminals = {}
            for lane_id, lane_wps in by_lane.items():
                start = min(lane_wps, key=lambda w: w.s)
                terminals[lane_id] = self._lane_terminal(start)
            for lane_id, term in terminals.items():
                if term.s >= road_max_s - 8.0:
                    continue  # 能走到路段末尾，不是消失车道
                # 存在另一条能走到接近路段末尾的车道 → 确认车道消失
                for other_id, other_term in terminals.items():
                    if other_id != lane_id and other_term.s >= road_max_s - 8.0:
                        drop_point, drop_lane_id = term, lane_id
                        cont_lane_wp = min(by_lane[other_id], key=lambda w: abs(w.s - term.s))
                        break
                if drop_point:
                    break
            if drop_point:
                break

        if not drop_point:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到车道消失（合并）结构，建议换用 Town04/Town05/Town10"}

        spawned = []
        desc = []
        # 消失车道上的车（将被挤入邻道）
        vanish_count = max(1, vehicle_count // 2)
        vanish_prev = drop_point.previous(30.0) or drop_point.previous(20.0) or [drop_point]
        for i in range(vanish_count):
            base = vanish_prev[min(i, len(vanish_prev) - 1)]
            actor = self._spawn_vehicle_on_waypoint(base, autopilot=True, tag="消失车道")
            if actor:
                spawned.append(actor)
                desc.append(f"消失车道车 ID={actor.id} lane={drop_lane_id} s={base.s:.0f}")
        # 延续车道上的车
        cont_count = max(1, vehicle_count - vanish_count)
        if cont_lane_wp is not None:
            for back in (15.0, 25.0, 35.0)[:cont_count]:
                prev = cont_lane_wp.previous(back)
                if not prev:
                    continue
                actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="延续车道")
                if actor:
                    spawned.append(actor)
                    desc.append(f"延续车道车 ID={actor.id} lane={cont_lane_wp.lane_id} s={prev[0].s:.0f}")

        self._set_spectator_overhead(drop_point.transform.location, height=40.0)
        return {
            "success": bool(spawned),
            "scenario": "城市-车道合并",
            "map": current_map,
            "merge_point": {"x": round(drop_point.transform.location.x, 1),
                            "y": round(drop_point.transform.location.y, 1),
                            "road_id": drop_point.road_id,
                            "dropped_lane": drop_lane_id},
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_diverge_merge(self, vehicle_count=4, map_name=None):
        """城市-分合流路口场景：找到有多个进出口臂的路口，
        在各进口臂布设车辆，经路口分流后从不同出口驶出/合流。"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        carla_map = self.world.get_map()
        waypoints = self._get_driving_waypoints(6.0)
        junction_wp = next((w for w in waypoints if w.is_junction), None)
        if not junction_wp:
            return {"success": False, "error": f"当前地图 {current_map} 未找到路口，建议换用 Town05/Town10"}

        # 找一个 >=3 臂的路口
        junction = None
        arms = []
        visited_junctions = set()
        for w in waypoints:
            if not w.is_junction:
                continue
            j = w.get_junction()
            if j.id in visited_junctions:
                continue
            visited_junctions.add(j.id)
            try:
                pairs = j.get_waypoints(carla.LaneType.Driving)
            except Exception:
                continue
            entries = {}
            for entry_wp, exit_wp in pairs:
                entries.setdefault(entry_wp.road_id, entry_wp)
            if len(entries) >= 3:
                junction, arms = j, list(entries.values())
                break
        if not junction:
            junction = junction_wp.get_junction()
            pairs = junction.get_waypoints(carla.LaneType.Driving)
            arms = []
            for entry_wp, exit_wp in pairs:
                if all(entry_wp.road_id != a.road_id for a in arms):
                    arms.append(entry_wp)

        spawned = []
        desc = []
        center = junction.bounding_box.location
        per_arm = max(1, vehicle_count // max(1, len(arms)))
        for arm in arms[:4]:
            for i in range(per_arm):
                back = 25.0 + i * 12.0
                prev = arm.previous(back)
                if not prev:
                    prev = [arm]
                actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="路口")
                if actor:
                    spawned.append(actor)
                    desc.append(f"进口臂车 ID={actor.id} road{arm.road_id}")
        self._set_spectator_overhead(center, height=55.0)
        return {
            "success": bool(spawned),
            "scenario": "城市-分合流路口",
            "map": current_map,
            "junction_id": junction.id,
            "arms": len(arms),
            "center": {"x": round(center.x, 1), "y": round(center.y, 1)},
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_side_road(self, vehicle_count=4, map_name=None):
        """城市-辅路场景：找到与主路平行且近距离的道路（辅路），
        在主路和辅路上同时布设车辆。"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)

        # 找一对平行且间距 8~45m 的不同道路
        pair = None
        stride = max(1, len(waypoints) // 120)
        for w1 in waypoints[::stride]:
            if self._road_length(self._group_waypoints_by_road(waypoints).get((w1.road_id, w1.section_id), [])) < 60.0:
                continue
            best = None
            best_d = 999.0
            for w2 in waypoints:
                if w2.road_id == w1.road_id:
                    continue
                d = w2.transform.location.distance(w1.transform.location)
                if not (8.0 <= d <= 45.0) or d >= best_d:
                    continue
                yaw1 = w1.transform.rotation.yaw % 360
                yaw2 = w2.transform.rotation.yaw % 360
                diff = abs(yaw1 - yaw2) % 360
                diff = min(diff, 360 - diff)
                if diff < 25.0:
                    best, best_d = w2, d
            if best:
                pair = (w1, best, best_d)
                break

        if not pair:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到与主路平行的辅路结构，建议换用 Town05/Town10"}

        main_wp, side_wp, side_dist = pair
        spawned = []
        desc = []

        def _spread(wp, count, tag):
            actors = []
            lane_wps = [w for w in waypoints
                        if w.road_id == wp.road_id and w.lane_id == wp.lane_id]
            lane_wps.sort(key=lambda w: w.s)
            if not lane_wps:
                lane_wps = [wp]
            step_idx = max(1, len(lane_wps) // (count + 1))
            for i in range(count):
                idx = min((i + 1) * step_idx, len(lane_wps) - 1)
                actor = self._spawn_vehicle_on_waypoint(lane_wps[idx], autopilot=True, tag=tag)
                if actor:
                    actors.append(actor)
                    desc.append(f"{tag}车 ID={actor.id} road{wp.road_id} s={lane_wps[idx].s:.0f}")
            return actors

        main_count = max(1, (vehicle_count + 1) // 2)
        side_count = max(1, vehicle_count - main_count)
        spawned.extend(_spread(main_wp, main_count, "主路"))
        spawned.extend(_spread(side_wp, side_count, "辅路"))

        mid = carla.Location(x=(main_wp.transform.location.x + side_wp.transform.location.x) / 2,
                             y=(main_wp.transform.location.y + side_wp.transform.location.y) / 2,
                             z=main_wp.transform.location.z)
        self._set_spectator_overhead(mid, height=50.0)
        return {
            "success": bool(spawned),
            "scenario": "城市-辅路",
            "map": current_map,
            "main_road_id": main_wp.road_id,
            "side_road_id": side_wp.road_id,
            "parallel_distance_m": round(side_dist, 1),
            "spawned_count": len(spawned),
            "details": desc
        }
    # ============================================================
    # 第2周场景任务: 路口红绿灯/隧道/环岛/儿童姿态/二轮车/特殊车辆
    # ============================================================
    def _find_junctions_with_arms(self, min_arms=2):
        """枚举所有路口及进口臂，返回 [{junction, arms, center}]"""
        carla_map = self.world.get_map()
        waypoints = self._get_driving_waypoints(8.0)
        result = []
        visited = set()
        for w in waypoints:
            if not w.is_junction:
                continue
            j = w.get_junction()
            if j.id in visited:
                continue
            visited.add(j.id)
            try:
                pairs = j.get_waypoints(carla.LaneType.Driving)
            except Exception:
                continue
            entries = []
            seen_roads = set()
            for entry_wp, exit_wp in pairs:
                if entry_wp.road_id in seen_roads:
                    continue
                seen_roads.add(entry_wp.road_id)
                entries.append(entry_wp)
            if len(entries) >= min_arms:
                result.append({"junction": j, "arms": entries,
                               "center": j.bounding_box.location, "arm_count": len(entries)})
        return result

    async def scenario_junction_light(self, junction_shape='any', vehicle_count=4, map_name=None):
        """城市-路口（十字、T型、Y型）及红绿灯场景"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        lights = list(self.world.get_actors().filter("traffic.traffic_light*"))
        junctions = self._find_junctions_with_arms(3)

        def _shape_of(arm_count):
            if arm_count >= 4:
                return "十字"
            if arm_count == 3:
                return "T型/Y型"
            return "支路"

        want = {'cross': 4, 't': 3, 'y': 3}.get(junction_shape.lower(), None)
        picked = None
        picked_lights = []
        for info in junctions:
            if want and (info["arm_count"] >= 4) != (want >= 4) and want != 3:
                continue
            if want == 3 and info["arm_count"] != 3:
                continue
            if want == 4 and info["arm_count"] < 4:
                continue
            near = [l for l in lights
                    if l.get_location().distance(info["center"]) < 40.0]
            if near:
                picked, picked_lights = info, near
                break
        if not picked:
            # 退而求其次：任意多臂路口
            for info in junctions:
                near = [l for l in lights
                        if l.get_location().distance(info["center"]) < 40.0]
                if near:
                    picked, picked_lights = info, near
                    break
        if not picked and junctions:
            picked = junctions[0]
        if not picked:
            return {"success": False, "error": f"当前地图 {current_map} 未找到合适路口"}

        spawned = []
        desc = []
        arms = picked["arms"]
        per_arm = max(1, vehicle_count // len(arms))
        for arm in arms[:4]:
            for i in range(per_arm):
                prev = arm.previous(20.0 + i * 10.0)
                if not prev:
                    continue
                actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="路口")
                if actor:
                    spawned.append(actor)
                    desc.append(f"进口臂车 ID={actor.id} road{arm.road_id}")
        self._set_spectator_overhead(picked["center"], height=55.0)

        light_desc = []
        for l in picked_lights[:4]:
            state = {carla.TrafficLightState.Green: "绿", carla.TrafficLightState.Red: "红",
                     carla.TrafficLightState.Yellow: "黄"}.get(l.state, str(l.state))
            light_desc.append(f"灯ID={l.id}:{state}")
        return {
            "success": bool(spawned),
            "scenario": f"城市-路口({_shape_of(picked['arm_count'])})及红绿灯",
            "map": current_map,
            "junction_id": picked["junction"].id,
            "shape": _shape_of(picked["arm_count"]),
            "arms": picked["arm_count"],
            "traffic_lights": light_desc if light_desc else ["该路口无信号灯（无保护）"],
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_tunnel(self, vehicle_count=4, map_name=None):
        """隧道场景：寻找上方被桥梁/建筑覆盖的下穿道路（隧道/地下道/桥下通道）

        判定：车道点水平半径8m内存在高出4m以上的其他车道 → 该点被覆盖。
        已安装的地图均无地质下沉道路（扫描验证），下穿通道是唯一可行的真隧道结构。
        """
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = [wp for wp in self.world.get_map().generate_waypoints(3.0)
                     if wp.lane_type == carla.LaneType.Driving]
        if not waypoints:
            return {"success": False, "error": "地图无Driving车道"}
        # 空间网格索引（10m格）
        CELL = 10.0
        grid = {}
        for wp in waypoints:
            key = (int(wp.transform.location.x // CELL), int(wp.transform.location.y // CELL))
            grid.setdefault(key, []).append(wp.transform.location.z)
        covered = []
        for wp in waypoints:
            loc = wp.transform.location
            hit = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for z in grid.get((int(loc.x // CELL) + dx, int(loc.y // CELL) + dy), ()):
                        if z - loc.z > 4.0:
                            hit = True
                            break
                    if hit:
                        break
                if hit:
                    break
            if hit:
                covered.append(wp)
        # 按道路分组，取覆盖段最长的路
        groups = self._group_waypoints_by_road(covered)
        tunnel_roads = [sorted(wps, key=lambda w: w.s) for wps in groups.values()
                        if len(wps) >= 4 and self._road_length(wps) > 20.0]
        if not tunnel_roads:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未检测到隧道/下穿通道，建议用 Town04（有立交桥）启动CARLA"}
        tunnel_roads.sort(key=len, reverse=True)
        wps = tunnel_roads[0]
        cover_z = max(loc_z for loc_z in
                      [max(grid.get((int(wps[len(wps)//2].transform.location.x // CELL) + dx,
                                     int(wps[len(wps)//2].transform.location.y // CELL) + dy), (0,)))
                       for dx in (-1, 0, 1) for dy in (-1, 0, 1)])

        spawned = []
        desc = []
        entry_idx = [0, len(wps) - 1]
        per_side = max(1, vehicle_count // 2)
        for side, idx in enumerate(entry_idx):
            for i in range(per_side):
                wp = wps[min(idx + (i * 3 if side == 0 else -(i * 3)), len(wps) - 1)]
                actor = self._spawn_vehicle_on_waypoint(wp, autopilot=True, tag="隧道")
                if actor:
                    try:
                        actor.set_light_state(carla.VehicleLightState(
                            carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position))
                    except Exception:
                        pass
                    spawned.append(actor)
                    desc.append(f"隧道车 ID={actor.id} s={wp.s:.0f} z={wp.transform.location.z:.1f}")
        mid_wp = wps[len(wps) // 2]
        self._set_spectator_overhead(mid_wp.transform.location, height=30.0)
        return {
            "success": bool(spawned),
            "scenario": "隧道(下穿通道)",
            "map": current_map,
            "road_id": wps[0].road_id,
            "cover_z": round(cover_z, 1),
            "road_z": round(mid_wp.transform.location.z, 1),
            "covered_wp_count": len(wps),
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_roundabout(self, vehicle_count=5, map_name=None):
        """环岛场景：寻找车辆绕一圈能回到该路口的环形路口"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        junctions = self._find_junctions_with_arms(3)
        picked = None
        for info in junctions:
            if info["arm_count"] > 6:
                continue
            arm_road_ids = {a.road_id for a in info["arms"]}
            reached = self._follow_lane_all(info["arms"][0], step=10.0, max_hops=35)
            # 从一条臂出发能到达另外>=2条臂 → 绕圈回环 = 环岛
            if len([r for r in reached if r in arm_road_ids]) >= 3:
                picked = info
                break
        if not picked:
            return {"success": False,
                    "error": f"当前地图 {current_map} 未找到环岛，建议用 Town05 启动CARLA"}

        spawned = []
        desc = []
        arms = picked["arms"]
        per_arm = max(1, vehicle_count // len(arms))
        for arm in arms:
            for i in range(per_arm):
                prev = arm.previous(20.0 + i * 12.0)
                if not prev:
                    continue
                actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="环岛")
                if actor:
                    spawned.append(actor)
                    desc.append(f"环岛进口车 ID={actor.id} road{arm.road_id}")
        self._set_spectator_overhead(picked["center"], height=50.0)
        return {
            "success": bool(spawned),
            "scenario": "环岛",
            "map": current_map,
            "junction_id": picked["junction"].id,
            "arms": picked["arm_count"],
            "spawned_count": len(spawned),
            "details": desc
        }

    async def spawn_pedestrian_pose(self, pedestrian_type='child', pose='stand', count=1):
        """儿童/成人姿态场景：站立/行走/蹲下/躺下/打伞
        蹲下/躺下/打伞无动画，仅能set_transform硬摆姿态"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}

        blueprint_map = {
            'child': ['0009', '0010', '0011', '0012', '0013', '0014', '0048', '0049'],
            'pedestrian': ['0001', '0002', '0003', '0004', '0005', '0006'],
            'elderly': ['0020', '0021', '0022'],
            'police': ['0030', '0032'],
        }
        numbers = blueprint_map.get(pedestrian_type.lower(), blueprint_map['child'])
        blueprint_library = self.world.get_blueprint_library()
        controller_bp = blueprint_library.find('controller.ai.walker')

        spawned = []
        desc = []
        for i in range(int(count)):
            bp = None
            for num in random.sample(numbers, len(numbers)):
                bp = blueprint_library.find(f'walker.pedestrian.{num}')
                if bp:
                    break
            if not bp:
                return {"success": False, "error": f"找不到{pedestrian_type}行人蓝图"}
            # 多次尝试随机导航点，单次落位失败不直接放弃
            walker = None
            for _attempt in range(10):
                loc = self.world.get_random_location_from_navigation()
                if not loc:
                    continue
                try:
                    walker = self.world.try_spawn_actor(bp, carla.Transform(loc))
                except Exception:
                    walker = None
                if walker:
                    break
            if not walker:
                continue
            self.actors.append(walker)
            spawned.append(walker)

            if pose == 'walk' and controller_bp:
                ctrl = self.world.spawn_actor(controller_bp, walker.get_transform(), walker)
                if ctrl:
                    ctrl.start()
                    target = self.world.get_random_location_from_navigation()
                    if target:
                        ctrl.go_to_location(target)
                    ctrl.set_max_speed(1.2 if pedestrian_type == 'child' else 1.4)
                    self.walker_controllers[walker.id] = ctrl
                    self.actors.append(ctrl)
                    desc.append(f"行走 ID={walker.id} ({bp.id})")
            elif pose == 'crouch':
                walker.set_simulate_physics(False)
                t = walker.get_transform()
                t.location.z -= 0.35
                t.rotation.pitch = 15
                walker.set_transform(t)
                desc.append(f"蹲下(硬摆) ID={walker.id} ({bp.id})")
            elif pose == 'lie':
                walker.set_simulate_physics(False)
                t = walker.get_transform()
                t.location.z -= 0.55
                t.rotation.roll = 90
                walker.set_transform(t)
                desc.append(f"躺下(硬摆) ID={walker.id} ({bp.id})")
            elif pose == 'umbrella':
                # 无伞道具，站立姿态近似
                desc.append(f"打伞(近似站立，无伞道具) ID={walker.id} ({bp.id})")
            else:
                desc.append(f"站立 ID={walker.id} ({bp.id})")
        note = "蹲下/躺下/打伞为set_transform硬摆姿态（无对应动画）" if pose in ('crouch', 'lie', 'umbrella') else ""
        if not spawned:
            return {"success": False, "scenario": f"{pedestrian_type}-{pose}",
                    "error": "多次尝试均无法落位行人（导航网格随机点不可达）"}
        return {
            "success": True,
            "scenario": f"{pedestrian_type}-{'站立' if pose=='stand' else '行走' if pose=='walk' else '蹲下' if pose=='crouch' else '躺下' if pose=='lie' else '打伞'}",
            "map": self.world.get_map().name.split('/')[-1],
            "spawned_count": len(spawned),
            "note": note,
            "details": desc
        }

    async def scenario_two_wheeler(self, vehicle_type='bicycle', state='stand', count=2):
        """自行车/摩托车-站立、行进、倒地场景"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        current_map = self.world.get_map().name.split('/')[-1]
        blueprint_library = self.world.get_blueprint_library()
        bp_ids = {
            'bicycle': ['vehicle.bh.crossbike', 'vehicle.diamondback.century', 'vehicle.gazelle.omafiets'],
            'motorcycle': ['vehicle.kawasaki.ninja', 'vehicle.yamaha.yzf', 'vehicle.harley-davidson.low_rider'],
        }.get(vehicle_type.lower(), ['vehicle.bh.crossbike'])
        blueprints = [blueprint_library.find(b) for b in bp_ids]
        blueprints = [b for b in blueprints if b]

        waypoints = self._get_driving_waypoints(6.0)
        random.shuffle(waypoints)
        spawned = []
        desc = []
        for i in range(int(count)):
            bp = random.choice(blueprints)
            actor = None
            for wp in waypoints[:40]:
                if any(a.get_location().distance(wp.transform.location) < 4.0
                       for a in self.world.get_actors().filter("vehicle.*")):
                    continue
                try:
                    actor = self.world.try_spawn_actor(bp, wp.transform)
                except Exception:
                    actor = None
                if actor:
                    break
            if not actor:
                continue
            self.actors.append(actor)
            spawned.append(actor)
            if state == 'move':
                actor.set_autopilot(True)  # noqa
                self._tm_used = True
                desc.append(f"行进 ID={actor.id} ({bp.id})")
            elif state == 'fallen':
                actor.set_simulate_physics(False)
                t = actor.get_transform()
                t.rotation.roll = random.choice([75, -75])
                actor.set_transform(t)
                desc.append(f"倒地 ID={actor.id} ({bp.id})")
            else:
                desc.append(f"站立 ID={actor.id} ({bp.id})")
        return {
            "success": bool(spawned),
            "scenario": f"{vehicle_type}-{'站立' if state=='stand' else '行进' if state=='move' else '倒地'}",
            "map": current_map,
            "spawned_count": len(spawned),
            "details": desc
        }

    async def spawn_special_vehicle(self, vehicle_type='ambulance', moving=True, count=1):
        """特殊任务车辆：救护车/警车（自动配送物流车无蓝图，无法完成）"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        blueprint_library = self.world.get_blueprint_library()
        candidates = {
            'ambulance': ['vehicle.ford.ambulance'],
            'police': ['vehicle.dodge.charger_police', 'vehicle.dodge.charger_police_2020'],
        }.get(vehicle_type.lower())
        if not candidates:
            return {"success": False,
                    "error": f"不支持的特殊车辆: {vehicle_type}（救护车ambulance/警车police；自动配送物流车无蓝图，无法完成）"}
        blueprint = None
        for c in candidates:
            blueprint = blueprint_library.find(c)
            if blueprint:
                break
        if not blueprint:
            return {"success": False, "error": f"找不到蓝图: {candidates}"}

        spawned = []
        waypoints = self._get_driving_waypoints(8.0)
        random.shuffle(waypoints)
        for i in range(int(count)):
            actor = None
            for wp in waypoints[:30]:
                if any(a.get_location().distance(wp.transform.location) < 6.0
                       for a in self.world.get_actors().filter("vehicle.*")):
                    continue
                try:
                    actor = self.world.try_spawn_actor(blueprint, wp.transform)
                except Exception:
                    actor = None
                if actor:
                    break
            if actor:
                if moving:
                    actor.set_autopilot(True)  # noqa
                    self._tm_used = True
                self.actors.append(actor)
                spawned.append(actor)
        note = "自动配送物流车无蓝图（无法完成项）" if vehicle_type == 'ambulance' else ""
        if not spawned:
            return {"success": False, "scenario": f"特殊任务车辆-{vehicle_type}",
                    "error": "未找到可落位的车道点（道路可能被占用）"}
        return {
            "success": True,
            "scenario": f"特殊任务车辆-{vehicle_type}({'行进' if moving else '静止'})",
            "map": self.world.get_map().name.split('/')[-1],
            "spawned_count": len(spawned),
            "note": note,
            "details": [f"ID={a.id} ({blueprint.id})" for a in spawned]
        }
    # ============================================================
    # 第3周场景任务: 交警轮椅/弱光/逆光/侧翻/前车急刹/危险切入
    # ============================================================
    async def _control_task(self, actor, control_fn, duration=3.0, interval=0.1, end_autopilot=False):
        """后台任务：在duration秒内每interval秒对actor执行control_fn(control)"""
        import asyncio
        async def _run():
            try:
                if actor and actor.is_alive:
                    actor.set_autopilot(False)
                elapsed = 0.0
                while elapsed < duration and actor and actor.is_alive:
                    control = carla.VehicleControl()
                    control_fn(control)
                    actor.apply_control(control)
                    await asyncio.sleep(interval)
                    elapsed += interval
                if end_autopilot and actor and actor.is_alive:
                    actor.set_autopilot(True)  # noqa
                    self._tm_used = True
            except asyncio.CancelledError:
                pass
            except Exception as e:
                app_logger.warning(f"⚠️ 控制任务异常: {e}")
        task = asyncio.get_event_loop().create_task(_run())
        self.scenario_tasks.append(task)
        return task

    async def scenario_officer(self, element='traffic_police', with_companion=False):
        """特殊群体场景：交警/轮椅（婴儿车无蓝图，返回无法完成说明）"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        blueprint_library = self.world.get_blueprint_library()

        if element == 'stroller':
            return {"success": False,
                    "error": "婴儿车无蓝图，无法完成（交警、轮椅可行）。可用普通行人+静止姿态近似伴行场景"}
        if element == 'wheelchair':
            numbers = ['0020', '0021', '0022', '0023', '0024', '0025']
            use_chair = False
            bp = None
            for num in numbers:
                bp = blueprint_library.find(f'walker.pedestrian.{num}')
                if bp:
                    if bp.has_attribute('can_use_wheelchair'):
                        bp.set_attribute('use_wheelchair', 'True')
                        use_chair = True
                    break
            if not bp:
                return {"success": False, "error": "找不到可用的轮椅行人蓝图"}
            # 多次尝试随机导航点（单次落位失败不直接放弃）
            walker = None
            loc = None
            for _attempt in range(10):
                loc = self.world.get_random_location_from_navigation()
                if not loc:
                    continue
                walker = self.world.try_spawn_actor(bp, carla.Transform(loc))
                if walker:
                    break
            if not walker:
                return {"success": False, "error": "轮椅行人生成失败（10次尝试均无法落位）"}
            self.actors.append(walker)
            desc = [f"轮椅行人 ID={walker.id} ({bp.id}) use_wheelchair={use_chair}"]
            if with_companion:
                comp = self._spawn_companion_walker(loc)
                if comp:
                    desc.append(f"伴行成人 ID={comp.id}")
            return {"success": True, "scenario": "特殊群体-坐轮椅的人",
                    "map": self.world.get_map().name.split('/')[-1],
                    "spawned_count": len(desc), "details": desc}

        # traffic_police: 交警站在车道中央（0030/0032）
        bp = blueprint_library.find('walker.pedestrian.0030') or blueprint_library.find('walker.pedestrian.0032')
        app_logger.info("👮 [officer] 交警蓝图: %s", bp and bp.id)
        if not bp:
            return {"success": False, "error": "找不到交警蓝图 walker.pedestrian.0030/0032"}
        waypoints = self._get_driving_waypoints(8.0)
        candidates = [w for w in waypoints if not w.is_junction] or waypoints
        # 多次尝试不同道路点，单次被占用不直接失败
        walker = None
        road_wp = None
        for wp in random.sample(candidates, min(10, len(candidates))):
            walker = self.world.try_spawn_actor(bp, wp.transform)
            if walker:
                road_wp = wp
                break
        app_logger.info("👮 [officer] 交警spawn结果: %s", walker and walker.id)
        if not walker:
            return {"success": False, "error": "交警生成失败（位置被占用）"}
        walker.set_simulate_physics(False)
        app_logger.info("👮 [officer] physics off 完成")
        self.actors.append(walker)
        self._set_spectator_overhead(road_wp.transform.location, height=25.0)
        app_logger.info("👮 [officer] spectator 完成")
        desc = [f"交警 ID={walker.id} @road{road_wp.road_id}"]
        if with_companion:
            comp = self._spawn_companion_walker(road_wp.transform.location)
            app_logger.info("👮 [officer] 伴行者: %s", comp and comp.id)
            if comp:
                desc.append(f"伴行成人 ID={comp.id}")
        return {"success": True, "scenario": "特殊群体-交警",
                "map": self.world.get_map().name.split('/')[-1],
                "spawned_count": len(desc), "details": desc}

    def _spawn_companion_walker(self, near_loc, speed=1.2):
        """在指定位置附近生成一个行走的成人（伴行）"""
        blueprint_library = self.world.get_blueprint_library()
        bp = None
        for num in ['0001', '0002', '0003', '0015', '0016']:
            bp = blueprint_library.find(f'walker.pedestrian.{num}')
            if bp:
                break
        controller_bp = blueprint_library.find('controller.ai.walker')
        if not bp or not controller_bp:
            return None
        # 伴行者必须出生在导航网格上：直接生成在车道点(非网格点)会让
        # go_to_location 的寻路陷入无限递归导致栈溢出崩溃（实测）
        best_loc, best_d = None, 1e12
        for _ in range(15):
            loc = self.world.get_random_location_from_navigation()
            if not loc:
                continue
            d = loc.distance(near_loc)
            if d < best_d:
                best_loc, best_d = loc, d
            if d < 25.0:
                break
        if not best_loc:
            return None
        walker = self.world.try_spawn_actor(bp, carla.Transform(best_loc))
        if not walker:
            return None
        self.actors.append(walker)
        ctrl = self.world.spawn_actor(controller_bp, walker.get_transform(), walker)
        if ctrl:
            ctrl.start()
            target = self.world.get_random_location_from_navigation()
            if target:
                ctrl.go_to_location(target)
            ctrl.set_max_speed(speed)
            self.walker_controllers[walker.id] = ctrl
            self.actors.append(ctrl)
        return walker

    async def set_lighting(self, condition='night'):
        """弱光条件：清晨/黄昏/阴天/夜晚（在set_weather基础上扩展光照）"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        presets = {
            'dawn': carla.WeatherParameters(  # 清晨
                cloudiness=40, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=90, sun_altitude_angle=8,
                fog_density=5, fog_distance=100, wetness=0),
            'dusk': carla.WeatherParameters(  # 黄昏
                cloudiness=50, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=270, sun_altitude_angle=6,
                fog_density=5, fog_distance=100, wetness=0),
            'overcast': carla.WeatherParameters(  # 阴天
                cloudiness=95, precipitation=0, precipitation_deposits=0,
                wind_intensity=15, sun_azimuth_angle=0, sun_altitude_angle=35,
                fog_density=0, fog_distance=0, wetness=0),
            'night': carla.WeatherParameters(
                cloudiness=20, precipitation=0, precipitation_deposits=0,
                wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
                fog_density=0, fog_distance=0, wetness=0),
        }
        if condition not in presets:
            return {"success": False, "error": f"不支持的弱光条件: {condition}（dawn清晨/dusk黄昏/overcast阴天/night夜晚）"}
        self.world.set_weather(presets[condition])
        names = {'dawn': '清晨', 'dusk': '黄昏', 'overcast': '阴天', 'night': '夜晚'}
        return {"success": True, "scenario": f"弱光-{names[condition]}",
                "details": [f"太阳高度角={presets[condition].sun_altitude_angle}° 云量={presets[condition].cloudiness}%"]}

    async def scenario_backlight(self, map_name=None):
        """逆光场景：低角度太阳正对来车方向 + 对向车辆开大灯"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(8.0)
        straight = [w for w in waypoints if not w.is_junction and self._road_length(
            self._group_waypoints_by_road(waypoints).get((w.road_id, w.section_id), [w])) > 80.0]
        if not straight:
            return {"success": False, "error": "未找到适合演示的长直道路"}
        wp = random.choice(straight)
        yaw = wp.transform.rotation.yaw

        # 太阳压低到正对道路方向（逆光）
        self.world.set_weather(carla.WeatherParameters(
            cloudiness=0, precipitation=0, precipitation_deposits=0,
            wind_intensity=5, sun_azimuth_angle=yaw, sun_altitude_angle=7,
            fog_density=0, fog_distance=0, wetness=0))

        spawned = []
        # 同向viewer车 + 对向开灯车
        viewer = self._spawn_vehicle_on_waypoint(wp, autopilot=True, tag="逆光")
        if viewer:
            spawned.append(viewer)
        try:
            opp_wp = wp.get_left_lane()
            if not opp_wp or opp_wp.lane_type != carla.LaneType.Driving:
                opp_wp = wp.get_right_lane()
        except Exception:
            opp_wp = None
        oncoming = None
        if opp_wp and opp_wp.lane_type == carla.LaneType.Driving:
            t = opp_wp.transform
            t.rotation.yaw = (t.rotation.yaw + 180) % 360
            try:
                oncoming = self.world.try_spawn_actor(
                    random.choice([b for b in self.world.get_blueprint_library().filter("vehicle.*")
                                   if b.id.startswith("vehicle.")]), t)
            except Exception:
                oncoming = None
            if oncoming:
                try:
                    oncoming.set_light_state(carla.VehicleLightState(
                        carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position))
                except Exception:
                    pass
                fwd = t.rotation.get_forward_vector()
                oncoming.set_target_velocity(carla.Vector3D(fwd.x * 8, fwd.y * 8, 0))
                self.actors.append(oncoming)
                spawned.append(oncoming)
        # 视角：viewer车后上方看向太阳方向
        v_t = viewer.get_transform() if viewer else wp.transform
        fwd = v_t.rotation.get_forward_vector()
        self._set_spectator_overhead(wp.transform.location, height=12.0)
        if not spawned:
            return {"success": False, "scenario": "逆光（太阳）",
                    "error": "逆光车辆生成失败（道路可能被占用），太阳角度已设置",
                    "sun": {"azimuth": round(yaw, 1), "altitude": 7}}
        return {
            "success": True,
            "scenario": "逆光（太阳）",
            "map": current_map,
            "sun": {"azimuth": round(yaw, 1), "altitude": 7},
            "spawned_count": len(spawned),
            "details": [f"ID={a.id} {a.type_id}" for a in spawned] +
                       (["对向车已开大灯"] if oncoming else ["无对向车道，仅低角度太阳"])
        }

    async def spawn_rollover_vehicle(self, vehicle_type='car', rollover='side', map_name=None):
        """侧翻/仰翻车辆（汽车、货车近似）"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        blueprint_library = self.world.get_blueprint_library()
        bp_map = {
            'car': ['vehicle.tesla.model3', 'vehicle.audi.a2', 'vehicle.ford.mustang'],
            'van': ['vehicle.mercedes.sprinter'],  # 货车近似（CARLA无大型货车蓝图）
            'truck': ['vehicle.mercedes.sprinter'],
        }
        candidates = bp_map.get(vehicle_type.lower())
        if not candidates:
            return {"success": False, "error": f"不支持的类型: {vehicle_type}（car汽车/van货车近似）"}
        blueprint = None
        for c in candidates:
            blueprint = blueprint_library.find(c)
            if blueprint:
                break
        if not blueprint:
            return {"success": False, "error": f"找不到蓝图: {candidates}"}

        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        roll = 180 if rollover == 'upside' else random.choice([85, -85])
        vehicle = None
        for sp in spawn_points[:25]:
            loc = sp.location
            loc.z = max(loc.z, 0.5)
            try:
                vehicle = self.world.try_spawn_actor(blueprint, carla.Transform(
                    loc, carla.Rotation(pitch=0, yaw=random.uniform(0, 360), roll=roll)))
            except Exception:
                vehicle = None
            if vehicle:
                break
            try:
                vehicle = self.world.try_spawn_actor(blueprint, carla.Transform(loc, carla.Rotation()))
            except Exception:
                vehicle = None
            if vehicle:
                vehicle.set_simulate_physics(False)
                t = vehicle.get_transform()
                t.rotation.roll = roll
                vehicle.set_transform(t)
                break
        if not vehicle:
            return {"success": False, "error": "翻倒车生成失败（未找到空位）"}
        vehicle.set_simulate_physics(False)
        self.actors.append(vehicle)
        note = "CARLA无大型货车蓝图，货车用奔驰Sprinter厢式车近似" if vehicle_type.lower() in ('van', 'truck') else ""
        return {
            "success": True,
            "scenario": f"翻车车辆-{vehicle_type}({'仰翻' if rollover=='upside' else '侧翻'})",
            "map": self.world.get_map().name.split('/')[-1],
            "spawned_count": 1,
            "details": [f"ID={vehicle.id} ({blueprint.id}) roll={roll}°"],
            "note": note
        }

    async def scenario_lead_vehicle(self, mode='stationary', distance=25.0, map_name=None):
        """前车急刹/静止场景：同车道前车 + 后车自动驾驶"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)
        straight = [w for w in waypoints if not w.is_junction]
        wp = None
        for cand in random.sample(straight, min(30, len(straight))):
            if cand.previous(distance) and cand.previous(distance + 10.0):
                wp = cand
                break
        if not wp:
            return {"success": False, "error": "未找到适合的长直车道"}

        lead = self._spawn_vehicle_on_waypoint(wp, autopilot=(mode == 'brake'), tag="前车")
        follower = None
        prev = wp.previous(distance)
        if prev:
            follower = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="后车")
        if not lead:
            return {"success": False, "error": "前车生成失败"}

        if mode == 'brake':
            import asyncio
            async def _brake_later():
                await asyncio.sleep(4.0)
                if lead.is_alive:
                    await self._control_task(
                        lead, lambda c: setattr(c, 'brake', 1.0) or setattr(c, 'throttle', 0.0),
                        duration=4.0)
            self.scenario_tasks.append(asyncio.get_event_loop().create_task(_brake_later()))

        self._set_spectator_overhead(wp.transform.location, height=30.0)
        return {
            "success": True,
            "scenario": f"前车{'急刹' if mode == 'brake' else '静止'}",
            "map": current_map,
            "spawned_count": 1 + (1 if follower else 0),
            "distance_m": distance,
            "details": [f"前车 ID={lead.id}"] + ([f"后车 ID={follower.id}"] if follower else []) +
                       (["4秒后急刹4秒"] if mode == 'brake' else [])
        }

    async def scenario_cut_in(self, direction='left', map_name=None):
        """危险切入场景：邻道车突然变道到本车前方"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)
        # 找多车道直路
        groups = self._group_waypoints_by_road(waypoints)
        multi = [k for k, v in groups.items()
                 if len({w.lane_id for w in v}) >= 2 and self._road_length(v) > 60.0 and not v[0].is_junction]
        if not multi:
            return {"success": False, "error": "未找到多车道直路"}
        key = random.choice(multi)
        wps = sorted(groups[key], key=lambda w: w.s)
        lane_ids = sorted({w.lane_id for w in wps})
        ego_wp = random.choice([w for w in wps if w.lane_id == lane_ids[0]])
        # 相邻车道（左侧优先）
        adj_wp = None
        try:
            cand = ego_wp.get_left_lane() if direction == 'left' else ego_wp.get_right_lane()
            if cand and cand.lane_type == carla.LaneType.Driving:
                adj_wp = cand
        except Exception:
            pass
        if not adj_wp:
            try:
                cand = ego_wp.get_right_lane() if direction == 'left' else ego_wp.get_left_lane()
                if cand and cand.lane_type == carla.LaneType.Driving:
                    adj_wp = cand
                    direction = 'right' if direction == 'left' else 'left'
            except Exception:
                pass
        if not adj_wp:
            return {"success": False, "error": "未找到相邻车道"}

        ego = self._spawn_vehicle_on_waypoint(ego_wp, autopilot=True, tag="本车")
        cutter = None
        ahead = adj_wp.next(12.0)
        if ahead:
            cutter = self._spawn_vehicle_on_waypoint(ahead[0], autopilot=True, tag="切入车")
        if not ego or not cutter:
            return {"success": False, "error": "场景车辆生成失败"}

        import asyncio
        steer = -0.5 if direction == 'left' else 0.5
        async def _cut_in_later():
            await asyncio.sleep(3.0)
            if cutter.is_alive:
                # 先手动变道
                await self._control_task(
                    cutter, lambda c: (setattr(c, 'steer', steer), setattr(c, 'throttle', 0.45)),
                    duration=1.6)
        self.scenario_tasks.append(asyncio.get_event_loop().create_task(_cut_in_later()))

        self._set_spectator_overhead(ego_wp.transform.location, height=30.0)
        return {
            "success": True,
            "scenario": f"危险切入({direction})",
            "map": current_map,
            "spawned_count": 2,
            "details": [f"本车 ID={ego.id} lane={ego_wp.lane_id}",
                        f"切入车 ID={cutter.id} lane={adj_wp.lane_id}", "3秒后向本车道切入"]
        }
    # ============================================================
    # 第4周场景任务: 前车消失/危险横穿/低重叠/逆行/无保护转弯
    # ============================================================
    async def scenario_lead_disappear(self, map_name=None):
        """前车消失场景：前车切入邻道，露出前方静止障碍物"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)
        straight = [w for w in waypoints if not w.is_junction]
        wp = None
        for cand in random.sample(straight, min(30, len(straight))):
            if cand.previous(35.0) and cand.previous(50.0):
                wp = cand
                break
        if not wp:
            return {"success": False, "error": "未找到适合的长直车道"}

        # 尝试多个候选点，直到障碍与前车都成功落位
        obstacle = None
        lead = None
        wp_picked = None
        for cand in random.sample(straight, min(15, len(straight))):
            if not (cand.previous(35.0) and cand.previous(50.0)):
                continue
            obstacle = self._spawn_vehicle_on_waypoint(cand, autopilot=False, tag="障碍")
            if not obstacle:
                continue
            prev = cand.previous(45.0)
            lead = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="前车") if prev else None
            if lead:
                wp_picked = cand
                break
            # 前车落位失败：回收障碍，换下一个候选点
            try:
                obstacle.destroy()
            except Exception:
                pass
            if obstacle in self.actors:
                self.actors.remove(obstacle)
            obstacle = None
        if not obstacle or not lead:
            return {"success": False, "error": "场景车辆生成失败（多次尝试均被占用）"}
        wp = wp_picked

        import asyncio
        async def _lead_out():
            await asyncio.sleep(4.0)
            if lead.is_alive:
                await self._control_task(
                    lead, lambda c: (setattr(c, 'steer', 0.5), setattr(c, 'throttle', 0.5)),
                    duration=1.8)
        self.scenario_tasks.append(asyncio.get_event_loop().create_task(_lead_out()))

        self._set_spectator_overhead(wp.transform.location, height=35.0)
        return {
            "success": True,
            "scenario": "前车消失（前车切出，前方有目标物）",
            "map": current_map,
            "spawned_count": 2,
            "details": [f"前方障碍 ID={obstacle.id}（静止）", f"前车 ID={lead.id}", "4秒后前车切出，露出障碍"]
        }

    async def scenario_crossing_hazard(self, crosser='pedestrian', map_name=None):
        """路口-危险横穿场景：人/机动车/二轮车横穿本车方向"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        junctions = self._find_junctions_with_arms(3)
        if not junctions:
            return {"success": False, "error": "未找到路口"}
        picked = random.choice(junctions[:5])
        arms = picked["arms"]
        ego_arm = arms[0]
        cross_arm = arms[1 % len(arms)]

        spawned = []
        desc = []
        ego = None
        prev = ego_arm.previous(30.0)
        if prev:
            ego = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="本车")
        if ego:
            spawned.append(ego)
            desc.append(f"本车 ID={ego.id} road{ego_arm.road_id}")

        if crosser == 'pedestrian':
            target = None
            for a in arms:
                if a.road_id != cross_arm.road_id and a.road_id != ego_arm.road_id:
                    target = a
                    break
            loc = cross_arm.transform.location
            walker = self._spawn_companion_walker(loc, speed=1.6)
            if walker and target:
                ctrl = self.walker_controllers.get(walker.id)
                if ctrl:
                    # 目标点必须落在导航网格上，直接传车道点会触发寻路栈溢出崩溃
                    tgt = target.transform.location
                    safe_tgt = None
                    for _ in range(20):
                        cand = self.world.get_random_location_from_navigation()
                        if cand and cand.distance(tgt) < 40.0:
                            safe_tgt = cand
                            break
                    if safe_tgt:
                        ctrl.go_to_location(safe_tgt)
                        desc.append(f"横穿行人 ID={walker.id}（走向对向臂）")
                    else:
                        desc.append(f"横穿行人 ID={walker.id}（附近无导航点，原地等待）")
                spawned.append(walker)
                if not any("横穿行人" in d for d in desc):
                    desc.append(f"横穿行人 ID={walker.id}")
        else:
            bp_filter = "vehicle.bh.crossbike" if crosser == 'bicycle' else "vehicle.*"
            bps = [b for b in self.world.get_blueprint_library().filter(bp_filter)
                   if b.id.startswith("vehicle.")]
            cross_v = None
            prev2 = cross_arm.previous(15.0)
            if prev2 and bps:
                try:
                    cross_v = self.world.try_spawn_actor(random.choice(bps), prev2[0].transform)
                except Exception:
                    cross_v = None
            if cross_v:
                cross_v.set_autopilot(True)  # noqa
                self._tm_used = True
                self.actors.append(cross_v)
                spawned.append(cross_v)
                desc.append(f"横穿{'自行车' if crosser=='bicycle' else '机动车'} ID={cross_v.id}")

        self._set_spectator_overhead(picked["center"], height=40.0)
        if not spawned:
            return {"success": False, "scenario": f"路口-危险横穿({crosser})",
                    "error": "横穿目标与本车均生成失败（路口周边被占用）"}
        return {
            "success": True,
            "scenario": f"路口-危险横穿({crosser})",
            "map": current_map,
            "junction_id": picked["junction"].id,
            "arms": picked["arm_count"],
            "spawned_count": len(spawned),
            "details": desc
        }

    async def scenario_low_overlap(self, offset_ratio=0.35, map_name=None):
        """前方低重叠率行驶目标：目标车贴车道线行驶，部分侵入本车道"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)
        groups = self._group_waypoints_by_road(waypoints)
        multi = [k for k, v in groups.items()
                 if len({w.lane_id for w in v}) >= 2 and self._road_length(v) > 60.0 and not v[0].is_junction]
        if not multi:
            return {"success": False, "error": "未找到多车道直路"}
        key = random.choice(multi)
        wps = sorted(groups[key], key=lambda w: w.s)
        lane_ids = sorted({w.lane_id for w in wps})
        follower_wp = random.choice([w for w in wps if w.lane_id == lane_ids[0]])
        target_wp = None
        for w in wps:
            if w.lane_id == lane_ids[-1] and abs(w.s - follower_wp.s) < 10.0:
                target_wp = w
                break
        if not target_wp:
            target_wp = next((w for w in wps if w.lane_id == lane_ids[-1]), None)
        if not target_wp:
            return {"success": False, "error": "未找到相邻目标车道"}

        # 目标车横向偏移（朝本车道一侧贴线）
        t = target_wp.transform
        lane_width = target_wp.lane_width
        shift = lane_width * (0.5 - offset_ratio)  # 向左侧车道线偏移
        left_vec = t.rotation.get_left_vector() if hasattr(t.rotation, 'get_left_vector') else None
        if left_vec is None:
            # 用yaw手工算左向量
            import math
            rad = math.radians(t.rotation.yaw + 90)
            left_vec = carla.Vector3D(math.cos(rad), math.sin(rad), 0)
        new_loc = carla.Location(
            x=t.location.x + left_vec.x * shift,
            y=t.location.y + left_vec.y * shift,
            z=t.location.z + 0.5)
        bps = [b for b in self.world.get_blueprint_library().filter("vehicle.*")
               if b.id.startswith("vehicle.")]
        target = None
        try:
            target = self.world.try_spawn_actor(random.choice(bps),
                                                carla.Transform(new_loc, t.rotation))
        except Exception:
            target = None
        if not target:
            return {"success": False, "error": "目标车生成失败"}

        fwd = t.rotation.get_forward_vector()
        target.set_target_velocity(carla.Vector3D(fwd.x * 6, fwd.y * 6, 0))
        self.actors.append(target)
        # 保持匀速的守护任务
        import asyncio
        async def _keep_speed():
            while target.is_alive:
                try:
                    v = target.get_velocity()
                    if abs(v.x) + abs(v.y) < 4.0:
                        target.set_target_velocity(carla.Vector3D(fwd.x * 6, fwd.y * 6, 0))
                except Exception:
                    break
                await asyncio.sleep(1.0)
        try:
            self.scenario_tasks.append(asyncio.get_event_loop().create_task(_keep_speed()))
        except Exception:
            pass

        follower = None
        prev = follower_wp.previous(18.0)
        if prev:
            follower = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="本车")

        self._set_spectator_overhead(target_wp.transform.location, height=25.0)
        return {
            "success": True,
            "scenario": "前方低重叠率行驶目标",
            "map": current_map,
            "spawned_count": 1 + (1 if follower else 0),
            "lateral_shift_m": round(shift, 2),
            "overlap_note": f"目标车横向偏移{shift:.2f}m，侵入本车道约{offset_ratio*100:.0f}%",
            "details": [f"目标车 ID={target.id} lane={target_wp.lane_id}"] +
                       ([f"本车 ID={follower.id} lane={follower_wp.lane_id}"] if follower else [])
        }

    async def scenario_wrong_way(self, speed=8.0, map_name=None):
        """逆行场景：对向车道出现逆行车辆"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        waypoints = self._get_driving_waypoints(6.0)
        groups = self._group_waypoints_by_road(waypoints)
        # 双向道路：同时存在正负lane_id
        two_way = [k for k, v in groups.items()
                   if any(w.lane_id > 0 for w in v) and any(w.lane_id < 0 for w in v)
                   and self._road_length(v) > 60.0 and not v[0].is_junction]
        if not two_way:
            return {"success": False, "error": "未找到双向道路"}
        random.shuffle(two_way)
        bps = [b for b in self.world.get_blueprint_library().filter("vehicle.*")
               if b.id.startswith("vehicle.")]
        ref = None
        wrong = None
        wrong_wp = None
        ref_wp = None
        t = None
        # 多试几条双向路，直到逆行车辆成功落位
        for key in two_way[:8]:
            wps = groups[key]
            lane_pos = [w for w in wps if w.lane_id > 0]
            lane_neg = [w for w in wps if w.lane_id < 0]
            if not lane_pos or not lane_neg:
                continue
            ref_wp_c = random.choice(lane_neg)
            wrong_wp_c = min(lane_pos, key=lambda w: abs(w.s - ref_wp_c.s))
            t_c = wrong_wp_c.transform
            t_c.rotation.yaw = (t_c.rotation.yaw + 180) % 360
            wrong_c = None
            for _bp in random.sample(bps, min(4, len(bps))):
                try:
                    wrong_c = self.world.try_spawn_actor(_bp, t_c)
                except Exception:
                    wrong_c = None
                if wrong_c:
                    break
            if wrong_c:
                wrong = wrong_c
                wrong_wp = wrong_wp_c
                ref_wp = ref_wp_c
                t = t_c
                break
        if not wrong:
            return {"success": False, "error": "逆行车辆生成失败（多条道路尝试均被占用）"}
        # 正向参考车（自动驾驶，非必需）
        ref = self._spawn_vehicle_on_waypoint(ref_wp, autopilot=True, tag="正向车")
        fwd = t.rotation.get_forward_vector()
        wrong.set_target_velocity(carla.Vector3D(fwd.x * speed, fwd.y * speed, 0))
        self.actors.append(wrong)

        import asyncio
        async def _keep_wrong_speed():
            while wrong.is_alive:
                try:
                    v = wrong.get_velocity()
                    if abs(v.x) + abs(v.y) < speed * 0.6:
                        wrong.set_target_velocity(carla.Vector3D(fwd.x * speed, fwd.y * speed, 0))
                except Exception:
                    break
                await asyncio.sleep(1.0)
        self.scenario_tasks.append(asyncio.get_event_loop().create_task(_keep_wrong_speed()))

        self._set_spectator_overhead(ref_wp.transform.location, height=30.0)
        return {
            "success": True,
            "scenario": "逆行",
            "map": current_map,
            "spawned_count": (1 if ref else 0) + 1,
            "details": ([f"正向车 ID={ref.id} lane={ref_wp.lane_id}"] if ref else []) +
                       [f"逆行车辆 ID={wrong.id} lane={wrong_wp.lane_id}（朝车道反方向行驶）"]
        }

    async def scenario_unprotected_turn(self, map_name=None):
        """路口无保护通行场景：无信号灯路口多方向来车交汇"""
        if self.world is None:
            return {"success": False, "error": "未连接到CARLA服务器"}
        if not await self._ensure_map(map_name):
            return {"success": False, "error": f"地图加载失败: {map_name}"}

        current_map = self.world.get_map().name.split('/')[-1]
        lights = list(self.world.get_actors().filter("traffic.traffic_light*"))
        junctions = self._find_junctions_with_arms(3)
        picked = None
        for info in junctions:
            if all(l.get_location().distance(info["center"]) > 45.0 for l in lights):
                picked = info
                break
        if not picked:
            picked = junctions[0] if junctions else None
        if not picked:
            return {"success": False, "error": "未找到路口"}

        spawned = []
        desc = []
        arms = picked["arms"]
        for arm in arms[:3]:
            prev = arm.previous(25.0)
            if not prev:
                continue
            actor = self._spawn_vehicle_on_waypoint(prev[0], autopilot=True, tag="无保护")
            if actor:
                spawned.append(actor)
                desc.append(f"来向车 ID={actor.id} road{arm.road_id}")
        self._set_spectator_overhead(picked["center"], height=50.0)
        if not spawned:
            return {"success": False, "scenario": "路口无保护通行",
                    "error": "无保护路口来向车生成失败（路口周边被占用）"}
        return {
            "success": True,
            "scenario": "路口无保护通行",
            "map": current_map,
            "junction_id": picked["junction"].id,
            "arms": picked["arm_count"],
            "protected": False,
            "spawned_count": len(spawned),
            "details": desc
        }



        # ============ 修复8: 行人停止/恢复移动 ============
    def stop_walker(self, walker_id):
        controller = self.walker_controllers.get(walker_id)
        if controller and controller.is_alive:
            controller.stop()
            app_logger.info(f"[修复8] 行人 {walker_id} 已停止")
            return True
        return False

    def resume_walker(self, walker_id, new_target=None):
        walker = self.world.get_actor(walker_id)
        controller = self.walker_controllers.get(walker_id)
        if not walker or not walker.is_alive:
            app_logger.warning(f"[修复8] 行人 {walker_id} 不存在")
            return False
        if not controller or not controller.is_alive:
            controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            controller = self.world.spawn_actor(controller_bp, carla.Transform(), walker)
            self.walker_controllers[walker_id] = controller
        if new_target is None:
            new_target = self.world.get_random_location_from_navigation()
        if not new_target:
            current_loc = walker.get_location()
            new_target = carla.Location(
                x=current_loc.x + random.uniform(-20, 20),
                y=current_loc.y + random.uniform(-20, 20),
                z=current_loc.z
            )
        controller.go_to_location(new_target)
        controller.set_max_speed(random.uniform(1.0, 2.0))
        controller.start()
        if walker_id in self.walker_goals:
            self.walker_goals[walker_id]['target'] = new_target
            self.walker_goals[walker_id]['stuck_count'] = 0
            self.walker_goals[walker_id]['last_location'] = walker.get_location()
        app_logger.info(f"[修复8] 行人 {walker_id} 已恢复移动")
        return True

    def stop_all_walkers(self):
        count = 0
        for walker_id, controller in self.walker_controllers.items():
            if controller and controller.is_alive:
                controller.stop()
                count += 1
        app_logger.info(f"[修复8] 共停止 {count} 个行人")
        return count

    def resume_all_walkers(self):
        count = 0
        for walker_id in list(self.walker_controllers.keys()):
            if self.resume_walker(walker_id):
                count += 1
        app_logger.info(f"[修复8] 共恢复 {count} 个行人")
        return count

    async def cleanup(self):
        """清理环境"""
        # 停止后台tick循环
        await self.stop_tick_loop()

        # 停止视角跟随
        await self.stop_view_follow()

        # 停止视频录制
        await self.stop_recording()

        # 先关闭所有车辆自动驾驶，避免Traffic Manager与destroy竞争导致原生崩溃
        for actor in list(self.actors):
            try:
                if actor.is_alive and 'vehicle' in actor.type_id:
                    actor.set_autopilot(False)
            except Exception:
                pass
        # 等待Traffic Manager注销车辆（立即销毁会与TM竞争导致原生崩溃）
        import asyncio
        await asyncio.sleep(1.0)

        # 取消场景后台任务
        for task in self.scenario_tasks:
            try:
                task.cancel()
            except Exception:
                pass
        self.scenario_tasks = []

        for actor in self.actors:
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception as e:
                app_logger.warning(f"⚠️ 销毁actor {getattr(actor, 'id', '?')} 失败: {e}")
        self.actors = []
        app_logger.info("🧹 清理所有CARLA actor")

        # ============ 修复3: 行人卡住检测与自动修复 ============
    def check_and_fix_stuck_walkers(self):
        """每帧检测行人是否卡住，若卡住则重新设置目标"""
        self.walker_check_interval += 1
        if self.walker_check_interval < 30:
            return
        self.walker_check_interval = 0
        
        for walker_id, info in list(self.walker_goals.items()):
            walker = self.world.get_actor(walker_id)
            if walker is None or not walker.is_alive:
                del self.walker_goals[walker_id]
                if walker_id in self.walker_controllers:
                    del self.walker_controllers[walker_id]
                continue
            
            current_loc = walker.get_location()
            last_loc = info['last_location']
            distance = current_loc.distance(last_loc)
            
            if distance < 0.15:
                info['stuck_count'] += 1
            else:
                info['stuck_count'] = 0
                info['last_location'] = current_loc
            
            if info['stuck_count'] >= 3:
                controller = self.walker_controllers.get(walker_id)
                if controller and controller.is_alive:
                    new_target = self.world.get_random_location_from_navigation()
                    if new_target:
                        controller.go_to_location(new_target)
                        info['target'] = new_target
                        info['stuck_count'] = 0
                        info['last_location'] = current_loc
                        app_logger.info(f"[修复3] 行人 {walker_id} 原地踏步，已重新设置目标")
                    else:
                        fallback = carla.Location(
                            x=current_loc.x + random.uniform(-15, 15),
                            y=current_loc.y + random.uniform(-15, 15),
                            z=current_loc.z
                        )
                        controller.go_to_location(fallback)
                        info['target'] = fallback
                        info['stuck_count'] = 0
                        info['last_location'] = current_loc
                        app_logger.info(f"[修复3] 行人 {walker_id} 原地踏步，已设置备用目标")

    # ============ 视角控制功能 ============

    def set_third_person_view(self, target_actor, distance=5.0, height=2.0, offset_angle=0):
        """设置第三人称视角（跟随视角）

        Args:
            target_actor: 目标actor（车辆或行人）
            distance: 相机与目标的距离（米）
            height: 相机高度（米）
            offset_angle: 水平偏移角度（度）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            # 计算相机位置（在目标后方指定距离和高度）
            yaw_rad = math.radians(target_transform.rotation.yaw + offset_angle + 180)  # +180 表示在目标后方
            camera_x = target_location.x + distance * math.cos(yaw_rad)
            camera_y = target_location.y + distance * math.sin(yaw_rad)
            camera_z = target_location.z + height

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)

            # 计算相机朝向，指向目标
            camera_rotation = carla.Rotation(
                pitch=-15.0,  # 略微向下看
                yaw=target_transform.rotation.yaw + offset_angle,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  第三人称视角已设置 - 目标: {target_actor.id}, 距离: {distance}m, 高度: {height}m")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置第三人称视角失败: {str(e)}")
            return False

    def set_first_person_view(self, target_actor, offset_x=0.3, offset_y=0.0, offset_z=1.2):
        """设置第一人称视角（驾驶员/行人视角）

        Args:
            target_actor: 目标actor（车辆或行人）
            offset_x: 前后偏移（米），默认0.3米（稍微向前）
            offset_y: 左右偏移（米）
            offset_z: 高度偏移（米），默认1.2米（眼睛高度）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            # 计算相机位置（在目标位置，考虑旋转）
            yaw_rad = math.radians(target_transform.rotation.yaw)
            # 相机位置：在目标前方offset_x处（行人/车辆朝向的方向）
            camera_x = target_location.x + offset_x * math.cos(yaw_rad) - offset_y * math.sin(yaw_rad)
            camera_y = target_location.y + offset_x * math.sin(yaw_rad) + offset_y * math.cos(yaw_rad)
            # 高度：目标位置高度 + 眼睛高度偏移
            camera_z = target_location.z + offset_z

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)

            # 相机朝向与目标相同
            camera_rotation = carla.Rotation(
                pitch=0.0,  # 平视
                yaw=target_transform.rotation.yaw,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  第一人称视角已设置 - 目标: {target_actor.id}, 高度: {camera_z:.2f}m")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置第一人称视角失败: {str(e)}")
            return False

    def set_overhead_view(self, target_actor=None, height=30.0):
        """设置俯视视角（鸟瞰视角）

        Args:
            target_actor: 目标actor，如果为None则使用地图中心
            height: 相机高度（米）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            self.view_target = target_actor  # 修复4: 保存追踪目标

            if target_actor:
                target_location = target_actor.get_transform().location
            else:
                # 使用地图中心或默认位置
                target_location = carla.Location(x=0, y=0, z=0)

            camera_location = carla.Location(
                x=target_location.x,
                y=target_location.y,
                z=target_location.z + height
            )

            camera_rotation = carla.Rotation(
                pitch=-90.0,  # 垂直向下看
                yaw=0.0,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  俯视视角已设置 - 高度: {height}m" + 
                          (f"，追踪目标ID={target_actor.id}" if target_actor else ""))
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置俯视视角失败: {str(e)}")
            return False

    def set_free_view(self, location=None, rotation=None):
        """设置自由视角（观察者视角）

        Args:
            location: 相机位置，如果为None则使用默认位置
            rotation: 相机旋转，如果为None则使用默认旋转

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()

            if location is None:
                location = carla.Location(x=0, y=0, z=50)
            if rotation is None:
                rotation = carla.Rotation(pitch=-45, yaw=0, roll=0)

            camera_transform = carla.Transform(location, rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  自由视角已设置 - 位置: ({location.x}, {location.y}, {location.z})")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置自由视角失败: {str(e)}")
            return False

    def rotate_view_around_target(self, target_actor, angle_degrees, distance=5.0, height=2.0):
        """围绕目标旋转视角

        Args:
            target_actor: 目标actor
            angle_degrees: 旋转角度（度）
            distance: 相机与目标的距离（米）
            height: 相机高度（米）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            spectator = self.world.get_spectator()
            target_location = target_actor.get_transform().location


            angle_rad = math.radians(angle_degrees)
            camera_x = target_location.x + distance * math.cos(angle_rad)
            camera_y = target_location.y + distance * math.sin(angle_rad)
            camera_z = target_location.z + height

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)

            # 计算朝向目标的旋转
            yaw = angle_degrees + 180  # 朝向中心
            camera_rotation = carla.Rotation(pitch=-15, yaw=yaw, roll=0)

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
            app_logger.info(f"👁️  视角已旋转到 {angle_degrees}°")
            return True
        except Exception as e:
            app_logger.error(f"❌ 旋转视角失败: {str(e)}")
            return False

    async def set_bystander_view(self):
        """设置旁观者视角（默认观察者视角，不跟随任何目标）

        Returns:
            bool: 是否设置成功
        """
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法设置视角")
            return False

        try:
            # 停止之前的视角跟随
            await self.stop_view_follow()

            spectator = self.world.get_spectator()

            # 获取地图的推荐观察者位置
            spawn_points = self.world.get_map().get_spawn_points()
            if spawn_points:
                # 使用第一个生成点作为参考，在其上方设置观察者
                ref_point = spawn_points[0].location
                location = carla.Location(x=ref_point.x, y=ref_point.y, z=ref_point.z + 50)
            else:
                location = carla.Location(x=0, y=0, z=50)

            rotation = carla.Rotation(pitch=-45, yaw=0, roll=0)
            camera_transform = carla.Transform(location, rotation)
            spectator.set_transform(camera_transform)

            # 清除当前视角目标
            self.view_target = None
            self.current_view_mode = "bystander"

            app_logger.info(f"👁️  旁观者视角已设置 - 位置: ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})")
            return True
        except Exception as e:
            app_logger.error(f"❌ 设置旁观者视角失败: {str(e)}")
            return False

    def start_view_follow(self, view_mode, target_actor):
        """启用视角跟随（实际更新逻辑已合并到tick_loop中）
        
        Args:
            view_mode: 视角模式 - third_person, first_person, overhead
            target_actor: 要跟随的目标actor
        """
        self.is_view_following = True
        self.view_target = target_actor
        self.current_view_mode = view_mode

        app_logger.info(f"🎯 视角跟随已启用 - 模式: {view_mode}, 目标: {target_actor.id}")

    async def stop_view_follow(self):
        """停止视角跟随"""
        if self.is_view_following:
            self.is_view_following = False
            app_logger.info("🛑 停止视角跟随")

        if self.view_follow_task and not self.view_follow_task.done():
            self.view_follow_task.cancel()
            try:
                await self.view_follow_task
            except asyncio.CancelledError:
                pass
            self.view_follow_task = None

    def _update_third_person_view(self, target_actor, distance=5.0, height=2.0):
        """更新第三人称视角位置（用于跟随）"""
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            yaw_rad = math.radians(target_transform.rotation.yaw + 180)
            camera_x = target_location.x + distance * math.cos(yaw_rad)
            camera_y = target_location.y + distance * math.sin(yaw_rad)
            camera_z = target_location.z + height

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
            camera_rotation = carla.Rotation(
                pitch=-15.0,
                yaw=target_transform.rotation.yaw,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        except Exception as e:
            app_logger.warning(f"⚠️ 更新第三人称视角出错: {e}")

    def _update_first_person_view(self, target_actor, offset_x=0.3, offset_y=0.0, offset_z=1.2):
        """更新第一人称视角位置（用于跟随）"""
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            yaw_rad = math.radians(target_transform.rotation.yaw)
            camera_x = target_location.x + offset_x * math.cos(yaw_rad) - offset_y * math.sin(yaw_rad)
            camera_y = target_location.y + offset_x * math.sin(yaw_rad) + offset_y * math.cos(yaw_rad)
            camera_z = target_location.z + offset_z

            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
            camera_rotation = carla.Rotation(
                pitch=0.0,
                yaw=target_transform.rotation.yaw,
                roll=0.0
            )

            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        except Exception as e:
            app_logger.warning(f"⚠️ 更新第一人称视角出错: {e}")


    def get_all_pedestrians(self):
        """获取当前世界中所有行人列表"""
        if self.world is None:
            return []

        pedestrians = []
        try:
            for actor in self.world.get_actors():
                if 'walker' in actor.type_id and 'controller' not in actor.type_id:
                    type_name = self._get_pedestrian_type_name(actor.type_id)
                    pedestrians.append({
                        'id': actor.id,
                        'type_id': actor.type_id,
                        'type_name': type_name
                    })
        except Exception as e:
            app_logger.error(f"❌ 获取行人列表失败: {str(e)}")

        return pedestrians

    def _update_overhead_view(self, target_actor, height=30.0):
        """更新俯视视角位置（用于跟随目标）"""
        try:
            spectator = self.world.get_spectator()
            target_location = target_actor.get_transform().location
            camera_location = carla.Location(
                x=target_location.x, y=target_location.y, z=target_location.z + height
            )
            camera_rotation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        except Exception as e:
            app_logger.warning(f"⚠️ 更新俯视视角出错: {e}")

    def get_all_pedestrians(self):
        """获取当前世界中所有行人列表

        Returns:
            list: 行人信息列表，每个元素包含 (id, type_id, type_name)
        """
        if self.world is None:
            return []

        pedestrians = []
        try:
            for actor in self.world.get_actors():
                if 'walker' in actor.type_id and 'controller' not in actor.type_id:
                    # 提取行人类型名称
                    type_name = self._get_pedestrian_type_name(actor.type_id)
                    pedestrians.append({
                        'id': actor.id,
                        'type_id': actor.type_id,
                        'type_name': type_name
                    })
        except Exception as e:
            app_logger.error(f"❌ 获取行人列表失败: {str(e)}")

        return pedestrians

    def _get_pedestrian_type_name(self, type_id):
        """根据type_id获取行人类型中文名称"""
        # 从蓝图ID中提取编号
        import re
        match = re.search(r'walker\.pedestrian\.(\d+)', type_id)
        if match:
            blueprint_number = match.group(1)
            # 根据编号判断类型
            if blueprint_number in ['0030', '0032']:
                return "警察"
            elif blueprint_number in ['0009', '0010', '0011', '0012', '0013', '0014', '0048', '0049']:
                return "儿童"
            elif blueprint_number in ['0020', '0021', '0022', '0023', '0024', '0025']:
                return "老年人"
            elif blueprint_number in ['0027', '0028', '0029']:
                return "商务人士"
            else:
                return "普通行人"
        return "未知类型"

    def get_all_vehicles(self):
        """获取当前世界中所有车辆列表

        Returns:
            list: 车辆信息列表，每个元素包含 (id, type_id, type_name)
        """
        if self.world is None:
            return []

        vehicles = []
        try:
            for actor in self.world.get_actors():
                if 'vehicle' in actor.type_id:
                    # 提取车辆类型名称
                    type_name = self._get_vehicle_type_name(actor.type_id)
                    vehicles.append({
                        'id': actor.id,
                        'type_id': actor.type_id,
                        'type_name': type_name
                    })
        except Exception as e:
            app_logger.error(f"❌ 获取车辆列表失败: {str(e)}")

        return vehicles

    def _get_vehicle_type_name(self, type_id):
        """根据type_id获取车辆类型中文名称"""
        # 车辆类型映射表
        vehicle_types = {
            'model3': '特斯拉 Model 3',
            'a2': '奥迪 A2',
            'etron': '奥迪 e-tron',
            'tt': '奥迪 TT',
            'grandtourer': '宝马 Grand Tourer',
            'i8': '宝马 i8',
            'mini': '宝马 Mini',
            'impala': '雪佛兰 Impala',
            'c3': '雪铁龙 C3',
            'charger_police': '道奇 Charger Police',
            'charger2020': '道奇 Charger 2020',
            'mustang': '福特 Mustang',
            'crown': '福特 Crown',
            'wrangler_rubicon': '吉普 Wrangler Rubicon',
            'mkz_2017': '林肯 MKZ 2017',
            'mkz_2020': '林肯 MKZ 2020',
            'benz_coupe': '奔驰 Coupe',
            'cabrio': '奔驰 Cabrio',
            'ccc': '奔驰 CCC',
            'cooper_s': 'Mini Cooper S',
            'micra': '日产 Micra',
            'patrol': '日产 Patrol',
            'leon': '西雅特 Leon',
            't2': '大众 T2',
            't3': '大众 T3',
        }
        # 从type_id中提取车辆型号
        for key, name in vehicle_types.items():
            if key in type_id.lower():
                return name
        return "未知车辆"

    # ============ 视频录制功能 ============

    async def start_recording(self, fps=30, output_path=None):
        """开始视频录制 - 从当前窗口视角录制

        Args:
            fps: 帧率
            output_path: 输出文件路径，如果为None则自动生成

        Returns:
            bool: 是否成功开始录制
        """
        import os
        import datetime

        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，无法开始录制")
            return False

        if self.is_recording:
            app_logger.warning("⚠️ 已经在录制中，请先停止当前录制")
            return False

        try:
            # 设置输出路径
            if output_path is None:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = "recordings"
                os.makedirs(output_dir, exist_ok=True)
                output_path = os.path.join(output_dir, f"carla_recording_{timestamp}.mp4")

            self.recording_output_path = output_path
            self.recording_fps = fps
            self.recording_frame_count = 0
            self.is_recording = True

            # 创建相机传感器（使用spectator视角）
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1920')
            camera_bp.set_attribute('image_size_y', '1080')
            camera_bp.set_attribute('fov', '110')

            # 初始位置在spectator位置
            spectator = self.world.get_spectator()
            camera_transform = spectator.get_transform()
            self.camera_sensor = self.world.spawn_actor(camera_bp, camera_transform)

            # 创建图像队列
            import queue
            self.image_queue = queue.Queue()
            self.camera_sensor.listen(self.image_queue.put)

            # 启动录制任务
            import asyncio
            self.recording_task = asyncio.create_task(self._recording_loop())

            app_logger.info(f"🎥 开始录制 - 输出: {output_path}, 帧率: {fps}fps, 分辨率: 1920x1080")
            return True

        except Exception as e:
            app_logger.error(f"❌ 开始录制失败: {str(e)}")
            return False

    async def _recording_loop(self):
        """录制循环 - 持续捕获帧并写入视频"""
        import asyncio

        frame_interval = 1.0 / self.recording_fps

        while self.is_recording:
            try:
                # 更新相机位置到当前spectator位置
                if self.world and self.camera_sensor:
                    spectator = self.world.get_spectator()
                    camera_transform = spectator.get_transform()
                    self.camera_sensor.set_transform(camera_transform)

                # 获取图像
                if self.image_queue and not self.image_queue.empty():
                    image = self.image_queue.get()
                    # 转换为numpy数组
                    import numpy as np
                    import cv2
                    array = np.frombuffer(image.raw_data, dtype=np.uint8)
                    array = array.reshape((image.height, image.width, 4))
                    array = array[:, :, :3]
                    img_rgb = array[:, :, ::-1]
                    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

                    # 写入视频文件
                    if self.video_writer is None:
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        self.video_writer = cv2.VideoWriter(
                            self.recording_output_path,
                            fourcc,
                            self.recording_fps,
                            (image.width, image.height)
                        )
                        app_logger.info(f"📹 视频写入器已创建")

                    self.video_writer.write(img_bgr)
                    self.recording_frame_count += 1

                await asyncio.sleep(frame_interval)

            except Exception as e:
                app_logger.warning(f"⚠️ 录制帧捕获出错: {e}")
                await asyncio.sleep(frame_interval)

    async def stop_recording(self):
        """停止视频录制

        Returns:
            str: 操作结果信息
        """
        import asyncio

        if not self.is_recording:
            return "未在录制中"

        try:
            self.is_recording = False

            # 等待录制任务结束
            if self.recording_task:
                try:
                    await asyncio.wait_for(self.recording_task, timeout=2.0)
                except asyncio.TimeoutError:
                    self.recording_task.cancel()

            # 释放视频写入器
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
                app_logger.info(f"📹 视频写入器已释放")

            # 停止相机监听
            if self.camera_sensor:
                self.camera_sensor.stop()

            # 清理相机传感器
            if self.camera_sensor:
                if self.camera_sensor.is_alive:
                    self.camera_sensor.destroy()
                self.camera_sensor = None

            self.image_queue = None

            result = f"✅ 录制已停止，共录制 {self.recording_frame_count} 帧，已保存至: {self.recording_output_path}"
            app_logger.info(result)

            self.recording_frame_count = 0
            return result

        except Exception as e:
            app_logger.error(f"❌ 停止录制失败: {str(e)}")
            return f"停止录制失败: {str(e)}"

    async def switch_view_mode(self, view_mode, target_actor_id=None):
        """切换视角模式

        Args:
            view_mode: 视角模式 - third_person, first_person, overhead, free, bystander
            target_actor_id: 目标actor ID

        Returns:
            str: 操作结果信息
        """
        import asyncio

        if self.world is None:
            return "❌ 未连接到CARLA服务器"

        # 旁观者视角不需要目标
        if view_mode == "bystander":
            await self.set_bystander_view()
            return "✅ 已切换到旁观者视角"

        # 确定目标actor
        target_actor = None
        if target_actor_id:
            target_actor = self.world.get_actor(target_actor_id)
        elif self.view_target:
            target_actor = self.view_target
        elif self.actors:
            for actor in reversed(self.actors):
                if 'vehicle' in actor.type_id or 'walker' in actor.type_id:
                    target_actor = actor
                    break

        if target_actor:
            self.view_target = target_actor

        # 设置视角
        if view_mode == "third_person":
            if target_actor:
                # 先停止之前的视角跟随
                await self.stop_view_follow()
                # 先设置一次视角
                self.set_third_person_view(target_actor)
                self.current_view_mode = "third_person"
                # 启动视角跟随（普通方法，直接调用）
                self.start_view_follow("third_person", target_actor)
                result = f"✅ 已切换到第三人称视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "❌ 第三人称视角需要指定目标"

        elif view_mode == "first_person":
            if target_actor:
                # 先停止之前的视角跟随
                await self.stop_view_follow()
                # 先设置一次视角
                self.set_first_person_view(target_actor)
                self.current_view_mode = "first_person"
                # 启动视角跟随（普通方法，直接调用）
                self.start_view_follow("first_person", target_actor)
                result = f"✅ 已切换到第一人称视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "❌ 第一人称视角需要指定目标"

        elif view_mode == "overhead":
            # 停止之前的跟随
            await self.stop_view_follow()
            self.set_overhead_view(target_actor)
            self.current_view_mode = "overhead"
            # 如果有目标，启动跟随
            if target_actor:
                self.start_view_follow("overhead", target_actor)
                result = f"✅ 已切换到俯视视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "✅ 已切换到俯视视角"

        elif view_mode == "free":
            # 停止之前的跟随
            await self.stop_view_follow()
            self.set_free_view()
            self.current_view_mode = "free"
            result = "✅ 已切换到自由视角"

        else:
            result = f"❌ 未知的视角模式: {view_mode}"

        return result


# 全局客户端实例
carla_client = CarlaClient()

async def connect_carla_impl(host: str = 'localhost', port: int = 2000) -> str:
    success = await carla_client.connect(host, port)
    # 双重保险：确认 world 真的获取到了
    if success and carla_client.world is None:
        try:
            carla_client.world = carla_client.client.get_world()
        except:
            pass
    if carla_client.world is None:
        return "❌ CARLA连接异常：无法获取world对象，请确认服务器已启动"
    return "✅ CARLA服务器连接成功"


async def spawn_vehicle_impl(query: str, count: int = 1, **kwargs) -> str:
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    # 强制转int，防止LLM传字符串进来
    count = int(count)
    
    vehicles = await carla_client.spawn_vehicles(query, count=count)
    if vehicles:
        if len(vehicles) == 1:
            return f"✅ 已生成1辆{query}车辆 (ID: {vehicles[0].id})"
        else:
            last_vehicle = vehicles[-1]
            return f"✅ 已生成{len(vehicles)}辆{query}车辆，最后一辆车ID: {last_vehicle.id}"
    return "❌ 车辆生成失败，请确保CARLA服务器已连接且地图有可用生成点"

async def spawn_bicycle_impl(query: str, count: int = 1, **kwargs) -> str:
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    count = int(count)  # ← 强制转int
    
    bicycles = await carla_client.spawn_bicycles(query, count=count)
    if bicycles:
        if len(bicycles) == 1:
            return f"✅ 已生成1辆{query}自行车 (ID: {bicycles[0].id})"
        else:
            ids = [b.id for b in bicycles]
            return f"✅ 已生成{len(bicycles)}辆{query}自行车，ID列表: {ids}"
    return "❌ 自行车生成失败，请确保CARLA服务器已连接且地图有可用生成点"

async def spawn_motorcycle_impl(query: str, count: int = 1, **kwargs) -> str:
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    count = int(count)  # ← 强制转int
    
    motorcycles = await carla_client.spawn_motorcycles(query, count=count)
    if motorcycles:
        if len(motorcycles) == 1:
            return f"✅ 已生成1辆{query}摩托车 (ID: {motorcycles[0].id})"
        else:
            ids = [m.id for m in motorcycles]
            return f"✅ 已生成{len(motorcycles)}辆{query}摩托车，ID列表: {ids}"
    return "❌ 摩托车生成失败"

async def spawn_prop_impl(query: str, count: int = 1, target_id: int = None, **kwargs) -> str:
    """（实际功能：生成道具/警示牌，可指定放在某个actor后方）"""
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    count = int(count)
    
    # 如果指定了 target_id，计算目标后方位置
    location = None
    if target_id is not None:
        target_actor = None
        for actor in carla_client.world.get_actors():
            if actor.id == int(target_id):
                target_actor = actor
                break
        
        if target_actor is None:
            return f"❌ 找不到目标 actor (ID: {target_id})，无法放置道具"
        
        # 获取目标位置和朝向，计算后方5米处
        target_loc = target_actor.get_location()
        target_rot = target_actor.get_transform().rotation
        
        # 将 yaw 转换为弧度，计算后方偏移
        import math
        yaw_rad = math.radians(target_rot.yaw)
        # 后方 = 当前位置 - 朝向向量 * 距离
        behind_x = target_loc.x - math.cos(yaw_rad) * 5.0
        behind_y = target_loc.y - math.sin(yaw_rad) * 5.0
        behind_z = target_loc.z + 0.1
        
        location = carla.Location(x=behind_x, y=behind_y, z=behind_z)
        app_logger.info(f"🚧 道具将放置在目标 {target_id} 后方5米处 ({behind_x:.1f}, {behind_y:.1f})")
    
    props = await carla_client.spawn_props(query, count=count, location=location)
    if props:
        if len(props) == 1:
            return f"✅ 已生成1个{query}道具 (ID: {props[0].id})" + (f"，位于目标 {target_id} 后方" if target_id else "")
        else:
            ids = [p.id for p in props]
            return f"✅ 已生成{len(props)}个{query}道具，ID列表: {ids}" + (f"，位于目标 {target_id} 后方" if target_id else "")
    return "❌ 道具生成失败"

async def spawn_overturned_vehicle_impl(vehicle_type: str = 'model3', **kwargs) -> str:
    """（实际功能：生成仰翻车辆）"""
    if carla_client.world is None:
        await carla_client.connect('localhost', 2000)
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    vehicle = await carla_client.spawn_overturned_vehicle(vehicle_type)
    if vehicle:
        return f"✅ 已生成仰翻的{vehicle_type} (ID: {vehicle.id})，物理已禁用以保持姿态"
    return "❌ 仰翻车辆生成失败"

async def set_weather_impl(weather_type: str) -> str:
    """（实际功能：设置天气）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    weather_presets = {'clear': '晴天', 'rain': '雨天', 'fog': '雾天', 'snow': '雪天', 'night': '夜晚',
                       'dawn': '清晨', 'dusk': '黄昏', 'overcast': '阴天'}
    success = await carla_client.set_weather(weather_type.lower())
    return f"✅ 天气已设置为 {weather_presets.get(weather_type.lower(), weather_type)}" if success else "❌ 不支持的天气类型"


async def get_traffic_lights_impl(query: str, **kwargs) -> str:
    """（实际功能：获取交通灯信息）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    lights = await carla_client.get_traffic_lights()
    if not lights:
        return "🚦 未找到交通灯或无法获取交通灯信息"
    
    result = ["🚦 交通灯状态:"]
    for i, light in enumerate(lights, 1):
        state = "绿色" if light.state == carla.TrafficLightState.Green else \
            "红色" if light.state == carla.TrafficLightState.Red else \
                "黄色"
        result.append(f"{i}. {light.type_id} - {state} (位置: {light.get_location()})")
    return "\n".join(result)


async def cleanup_scene_impl(**kwargs) -> str:
    """（实际功能：清理环境）"""
    await carla_client.cleanup()
    return "✅ 已清理所有车辆和物体"


async def spawn_pedestrian_impl(query: str, count: int = 1, speed: float = None, **kwargs) -> str:
    """（实际功能：生成行人）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    count = int(count)  # ← 强制转int
    
    pedestrians = await carla_client.spawn_pedestrians(query, count=count, speed=speed)
    if pedestrians:
        speed_info = f"，速度: {speed} m/s" if speed is not None else ""
        if len(pedestrians) == 1:
            return f"✅ 已生成1个{query}行人 (ID: {pedestrians[0].id}){speed_info}，行人已开始自动行走"
        else:
            last_pedestrian = pedestrians[-1]
            return f"✅ 已生成{len(pedestrians)}个{query}行人，最后一个行人ID: {last_pedestrian.id}{speed_info}，所有行人已开始自动行走"
    return "❌ 行人生成失败，请确保CARLA服务器已连接且地图有可用导航点"


async def setup_autopilot_impl(enable: bool = True, radius: float = 0.0, **kwargs) -> str:
    """（实际功能：设置车辆自动驾驶）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    success = await carla_client.setup_autopilot(enable, radius)
    if success:
        return f"✅ 车辆自动驾驶已{'启用' if enable else '禁用'}"
    return "❌ 设置自动驾驶失败"


async def setup_pedestrian_movement_impl(enable: bool = True, radius: float = 0.0, **kwargs) -> str:
    """（实际功能：设置行人自动移动）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    success = await carla_client.setup_pedestrian_movement(enable, radius)
    if success:
        return f"✅ 行人自动移动已{'启用' if enable else '禁用'}"
    return "❌ 设置行人移动失败"


# ============ 视角控制和视频录制实现函数 ============

async def switch_view_impl(view_mode: str, target_actor_id: int = None, **kwargs) -> str:
    """（实际功能：切换视角模式）"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    result = await carla_client.switch_view_mode(view_mode, target_actor_id)
    return result


async def start_recording_impl(fps: int = 30, **kwargs) -> str:
    """（实际功能：开始视频录制）"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"

    success = await carla_client.start_recording(fps=fps)

    if success:
        return f"🎥 开始录制 - 帧率: {fps}fps。录制过程中可以自由切换视角。"
    return "❌ 开始录制失败"


async def stop_recording_impl(**kwargs) -> str:
    """（实际功能：停止视频录制）"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"

    result = await carla_client.stop_recording()
    return result


# ============ 第1周场景任务: 底层实现函数 ============
def _format_scenario_result(result) -> str:
    """统一格式化场景构建结果"""
    if not result.get("success"):
        return f"❌ {result.get('scenario', '场景')}构建失败: {result.get('error', '未知错误')}"
    lines = [
        f"✅ 场景「{result['scenario']}」构建完成（地图: {result.get('map', '当前地图')}）",
        f"🚗 成功生成 {result['spawned_count']} 辆车（已开启自动驾驶汇流行驶）",
    ]
    if "merge_point" in result:
        mp = result["merge_point"]
        lines.append(f"📍 关键位置: ({mp['x']}, {mp['y']})")
    if "junction_id" in result:
        lines.append(f"📍 路口ID: {result['junction_id']}，进口臂数: {result.get('arms', '?')}")
    if "parallel_distance_m" in result:
        lines.append(f"📍 主路 road{result['main_road_id']} / 辅路 road{result['side_road_id']}，平行间距 {result['parallel_distance_m']}m")
    if result.get("details"):
        lines.append("🚙 明细:")
        lines.extend(f"  • {d}" for d in result["details"])
    return "\n".join(lines)


async def scenario_highway_ramp_impl(ramp_type: str = 'on', vehicle_count: int = 4,
                                     map_name: Optional[str] = None, **kwargs) -> str:
    """高速-进出匝道场景底层实现"""
    result = await carla_client.scenario_highway_ramp(ramp_type, vehicle_count, map_name)
    return _format_scenario_result(result)


async def scenario_lane_merge_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    """城市-车道合并场景底层实现"""
    result = await carla_client.scenario_lane_merge(vehicle_count, map_name)
    return _format_scenario_result(result)


async def scenario_diverge_merge_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    """城市-分合流路口场景底层实现"""
    result = await carla_client.scenario_diverge_merge(vehicle_count, map_name)
    return _format_scenario_result(result)


async def scenario_side_road_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    """城市-辅路场景底层实现"""
    result = await carla_client.scenario_side_road(vehicle_count, map_name)
    return _format_scenario_result(result)


# ============ 第2~4周场景任务: 底层实现函数 ============
def _format_scenario_result2(result) -> str:
    """带note字段的场景结果格式化"""
    text = _format_scenario_result(result)
    if result.get("note"):
        text += f"\n💡 说明: {result['note']}"
    if result.get("traffic_lights"):
        text += "\n🚦 信号灯: " + ", ".join(result["traffic_lights"])
    if result.get("sun"):
        s = result["sun"]
        text += f"\n☀️ 太阳方位角={s['azimuth']}° 高度角={s['altitude']}°"
    if result.get("overlap_note"):
        text += f"\n📐 {result['overlap_note']}"
    if result.get("protected") is False:
        text += "\n⚠️ 该路口无信号灯保护"
    return text


async def scenario_junction_light_impl(junction_shape: str = 'any', vehicle_count: int = 4,
                                       map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_junction_light(junction_shape, vehicle_count, map_name)
    return _format_scenario_result2(result)


async def scenario_tunnel_impl(vehicle_count: int = 4, map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_tunnel(vehicle_count, map_name)
    return _format_scenario_result2(result)


async def scenario_roundabout_impl(vehicle_count: int = 5, map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_roundabout(vehicle_count, map_name)
    return _format_scenario_result2(result)


async def spawn_pedestrian_pose_impl(pedestrian_type: str = 'child', pose: str = 'stand',
                                     count: int = 1, **kwargs) -> str:
    result = await carla_client.spawn_pedestrian_pose(pedestrian_type, pose, count)
    return _format_scenario_result2(result)


async def scenario_two_wheeler_impl(vehicle_type: str = 'bicycle', state: str = 'stand',
                                    count: int = 2, **kwargs) -> str:
    result = await carla_client.scenario_two_wheeler(vehicle_type, state, count)
    return _format_scenario_result2(result)


async def spawn_special_vehicle_impl(vehicle_type: str = 'ambulance', moving: bool = True,
                                     count: int = 1, **kwargs) -> str:
    result = await carla_client.spawn_special_vehicle(vehicle_type, moving, count)
    return _format_scenario_result2(result)


async def scenario_officer_impl(element: str = 'traffic_police', with_companion: bool = False, **kwargs) -> str:
    result = await carla_client.scenario_officer(element, with_companion)
    return _format_scenario_result2(result)


async def scenario_backlight_impl(map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_backlight(map_name)
    return _format_scenario_result2(result)


async def spawn_rollover_vehicle_impl(vehicle_type: str = 'car', rollover: str = 'side',
                                      map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.spawn_rollover_vehicle(vehicle_type, rollover, map_name)
    return _format_scenario_result2(result)


async def scenario_lead_vehicle_impl(mode: str = 'stationary', distance: float = 25.0,
                                     map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_lead_vehicle(mode, distance, map_name)
    return _format_scenario_result2(result)


async def scenario_cut_in_impl(direction: str = 'left', map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_cut_in(direction, map_name)
    return _format_scenario_result2(result)


async def scenario_lead_disappear_impl(map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_lead_disappear(map_name)
    return _format_scenario_result2(result)


async def scenario_crossing_hazard_impl(crosser: str = 'pedestrian', map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_crossing_hazard(crosser, map_name)
    return _format_scenario_result2(result)


async def scenario_low_overlap_impl(offset_ratio: float = 0.35, map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_low_overlap(offset_ratio, map_name)
    return _format_scenario_result2(result)


async def scenario_wrong_way_impl(speed: float = 8.0, map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_wrong_way(speed, map_name)
    return _format_scenario_result2(result)


async def scenario_unprotected_turn_impl(map_name: Optional[str] = None, **kwargs) -> str:
    result = await carla_client.scenario_unprotected_turn(map_name)
    return _format_scenario_result2(result)


# ============ FastMCP 工具装饰器版本 ============

@mcp.tool()
async def connect_carla(host: str = 'localhost', port: int = 2000) -> str:
    """（实际功能：连接CARLA）"""
    return await connect_carla_impl(host, port)


@mcp.tool()
async def spawn_vehicle(query: str, count: int = 1) -> str:
    """（实际功能：生成车辆）"""
    return await spawn_vehicle_impl(query, count=count)

@mcp.tool()
async def spawn_bicycle(query: str, count: int = 1) -> str:
    """（实际功能：生成自行车）
    
    支持类型: crossbike(BH Crossbike), century(Diamondback Century), omafiets(Gazelle Omafiets)
    """
    return await spawn_bicycle_impl(query, count=count)

@mcp.tool()
async def spawn_motorcycle(query: str, count: int = 1) -> str:
    """（实际功能：生成摩托车）
    
    支持类型: ninja(Kawasaki Ninja), yzf(Yamaha YZF), low_rider(Harley-Davidson Low Rider)
    """
    return await spawn_motorcycle_impl(query, count=count)

@mcp.tool()
async def spawn_prop(query: str, count: int = 1) -> str:
    """（实际功能：生成道具/警示牌）
    
    支持类型: cone(施工锥), barrier(路障), warning(三角警示牌/交通警示牌)
    """
    return await spawn_prop_impl(query, count=count)

@mcp.tool()
async def spawn_overturned_vehicle(vehicle_type: str = 'model3') -> str:
    """（实际功能：生成仰翻的车辆）
    
    支持类型: model3(特斯拉Model3), mustang(福特野马)等
    """
    return await spawn_overturned_vehicle_impl(vehicle_type)

@mcp.tool()
async def set_weather(weather_type: str) -> str:
    """设置仿真天气环境。weather_type 支持:
        clear(晴天),
        rain(雨天),
        fog(雾天),
        snow(雪天),
        night(夜晚/弱光)"""
    return await set_weather_impl(weather_type)


@mcp.tool()
async def get_traffic_lights(query: str, user_type: Optional[str] = None) -> str:
    """（实际功能：获取交通灯）"""
    return await get_traffic_lights_impl(query)


@mcp.tool()
async def cleanup_scene(language: Optional[str] = None, period: str = "daily") -> str:
    """（实际功能：清理环境）"""
    return await cleanup_scene_impl()


@mcp.tool()
async def spawn_pedestrian(query: str, count: int = 1, speed: float = None) -> str:
    """（实际功能：生成行人）"""
    return await spawn_pedestrian_impl(query, count=count, speed=speed)


@mcp.tool()
async def setup_autopilot(enable: bool = True, radius: float = 0.0) -> str:
    """（实际功能：设置车辆自动驾驶）"""
    return await setup_autopilot_impl(enable, radius=radius)


@mcp.tool()
async def setup_pedestrian_movement(enable: bool = True, radius: float = 0.0) -> str:
    """（实际功能：设置行人自动移动）"""
    return await setup_pedestrian_movement_impl(enable, radius=radius)


@mcp.tool()
async def switch_view(view_mode: str = "third_person", target_actor_id: int = None) -> str:
    """（实际功能：切换视角）"""
    return await switch_view_impl(view_mode, target_actor_id)


@mcp.tool()
async def start_recording(fps: int = 30) -> str:
    """（实际功能：开始录制视频）"""
    return await start_recording_impl(fps)


@mcp.tool()
async def stop_recording() -> str:
    """（实际功能：停止录制视频）"""
    return await stop_recording_impl()


@mcp.tool()
async def scenario_highway_ramp(ramp_type: str = "on", vehicle_count: int = 4,
                                map_name: Optional[str] = None) -> str:
    """（实际功能：高速-进出匝道场景）
    在高速公路主路与匝道的汇流/分流点自动布设车辆。
    ramp_type: "on"(匝道汇入) / "off"(主路驶出匝道)
    """
    return await scenario_highway_ramp_impl(ramp_type, vehicle_count, map_name)


@mcp.tool()
async def scenario_lane_merge(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：城市-车道合并场景）
    自动寻找车道消失（车道数减少）位置，在消失车道和延续车道布设车辆演示汇流。
    """
    return await scenario_lane_merge_impl(vehicle_count, map_name)


@mcp.tool()
async def scenario_diverge_merge(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：城市-分合流路口场景）
    找到多臂路口，在各进口臂布设车辆，经路口分流/合流。
    """
    return await scenario_diverge_merge_impl(vehicle_count, map_name)


@mcp.tool()
async def scenario_side_road(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：城市-辅路场景）
    自动寻找与主路平行的辅路，在主路和辅路上同时布设车辆。
    """
    return await scenario_side_road_impl(vehicle_count, map_name)


# ============ 第2~4周场景工具装饰器 ============
@mcp.tool()
async def scenario_junction_light(junction_shape: str = "any", vehicle_count: int = 4,
                                  map_name: Optional[str] = None) -> str:
    """（实际功能：城市-路口及红绿灯场景）
    junction_shape: "any"任意 / "cross"十字 / "t" T型 / "y" Y型
    """
    return await scenario_junction_light_impl(junction_shape, vehicle_count, map_name)


@mcp.tool()
async def scenario_tunnel(vehicle_count: int = 4, map_name: Optional[str] = None) -> str:
    """（实际功能：隧道场景）自动寻找下沉道路（隧道/地下道）布设车辆，推荐Town04/Town05"""
    return await scenario_tunnel_impl(vehicle_count, map_name)


@mcp.tool()
async def scenario_roundabout(vehicle_count: int = 5, map_name: Optional[str] = None) -> str:
    """（实际功能：环岛场景）自动寻找环形路口布设车辆，推荐Town05"""
    return await scenario_roundabout_impl(vehicle_count, map_name)


@mcp.tool()
async def spawn_pedestrian_pose(pedestrian_type: str = "child", pose: str = "stand",
                                count: int = 1) -> str:
    """（实际功能：儿童/成人姿态场景）
    pedestrian_type: child儿童/pedestrian成人/elderly老人/police警察
    pose: stand站立/walk行走/crouch蹲下/lie躺下/umbrella打伞（无动画，硬摆姿态）
    """
    return await spawn_pedestrian_pose_impl(pedestrian_type, pose, count)


@mcp.tool()
async def scenario_two_wheeler(vehicle_type: str = "bicycle", state: str = "stand",
                               count: int = 2) -> str:
    """（实际功能：自行车/摩托车-站立、行进、倒地场景）"""
    return await scenario_two_wheeler_impl(vehicle_type, state, count)


@mcp.tool()
async def spawn_special_vehicle(vehicle_type: str = "ambulance", moving: bool = True,
                                count: int = 1) -> str:
    """（实际功能：特殊任务车辆）ambulance救护车 / police警车（自动配送物流车无蓝图）"""
    return await spawn_special_vehicle_impl(vehicle_type, moving, count)


@mcp.tool()
async def scenario_officer(element: str = "traffic_police", with_companion: bool = False) -> str:
    """（实际功能：特殊群体场景）traffic_police交警 / wheelchair轮椅 / stroller婴儿车(无蓝图)"""
    return await scenario_officer_impl(element, with_companion)


@mcp.tool()
async def scenario_backlight(map_name: Optional[str] = None) -> str:
    """（实际功能：逆光场景）低角度太阳正对来车方向 + 对向车辆开大灯"""
    return await scenario_backlight_impl(map_name)


@mcp.tool()
async def spawn_rollover_vehicle(vehicle_type: str = "car", rollover: str = "side",
                                 map_name: Optional[str] = None) -> str:
    """（实际功能：翻车车辆）vehicle_type: car汽车/van货车近似；rollover: side侧翻/upside仰翻"""
    return await spawn_rollover_vehicle_impl(vehicle_type, rollover, map_name)


@mcp.tool()
async def scenario_lead_vehicle(mode: str = "stationary", distance: float = 25.0,
                                map_name: Optional[str] = None) -> str:
    """（实际功能：前车急刹/静止场景）mode: stationary静止 / brake急刹"""
    return await scenario_lead_vehicle_impl(mode, distance, map_name)


@mcp.tool()
async def scenario_cut_in(direction: str = "left", map_name: Optional[str] = None) -> str:
    """（实际功能：危险切入场景）邻道车突然变道到本车前方"""
    return await scenario_cut_in_impl(direction, map_name)


@mcp.tool()
async def scenario_lead_disappear(map_name: Optional[str] = None) -> str:
    """（实际功能：前车消失场景）前车切入邻道，露出前方静止障碍"""
    return await scenario_lead_disappear_impl(map_name)


@mcp.tool()
async def scenario_crossing_hazard(crosser: str = "pedestrian", map_name: Optional[str] = None) -> str:
    """（实际功能：路口危险横穿）crosser: pedestrian行人/vehicle机动车/bicycle自行车"""
    return await scenario_crossing_hazard_impl(crosser, map_name)


@mcp.tool()
async def scenario_low_overlap(offset_ratio: float = 0.35, map_name: Optional[str] = None) -> str:
    """（实际功能：低重叠率行驶目标）目标车贴车道线侵入本车道"""
    return await scenario_low_overlap_impl(offset_ratio, map_name)


@mcp.tool()
async def scenario_wrong_way(speed: float = 8.0, map_name: Optional[str] = None) -> str:
    """（实际功能：逆行场景）对向车道出现逆行车辆"""
    return await scenario_wrong_way_impl(speed, map_name)


@mcp.tool()
async def scenario_unprotected_turn(map_name: Optional[str] = None) -> str:
    """（实际功能：路口无保护通行场景）无信号灯路口多方向来车交汇"""
    return await scenario_unprotected_turn_impl(map_name)



# ============ AI助手类（集成Deepseek AI） ============

class FastMCPGitHubAssistant:
    """FastMCP GitHub AI助手 - 集成Deepseek AI与FastMCP工具"""

    VEHICLE_TYPES = {
        "model3": "Tesla Model 3",
        "a2": "Audi A2",
        "etron": "Audi e-tron",
        "tt": "Audi TT",
        "grandtourer": "BMW Grand Tourer",
        "i8": "BMW i8",
        "mini": "BMW Mini",
        "impala": "Chevrolet Impala",
        "c3": "Citroen C3",
        "charger_police": "Dodge Charger Police",
        "charger2020": "Dodge Charger 2020",
        "mustang": "Ford Mustang",
        "crown": "Ford Crown",
        "wrangler_rubicon": "Jeep Wrangler Rubicon",
        "mkz_2017": "Lincoln MKZ 2017",
        "mkz_2020": "Lincoln MKZ 2020",
        "benz_coupe": "Mercedes-Benz Coupe",
        "cabrio": "Mercedes-Benz Cabrio",
        "ccc": "Mercedes-Benz CCC",
        "cooper_s": "Mini Cooper S",
        "micra": "Nissan Micra",
        "patrol": "Nissan Patrol",
        "leon": "Seat Leon",
        "t2": "Volkswagen T2",
        "t3": "Volkswagen T3",
        "crossbike": "BH Crossbike",
        "century": "Diamondback Century",
        "omafiets": "Gazelle Omafiets"
    }

    VEHICLE_TYPE_MAP = {
        "特斯拉": "model3",
        "特斯拉model3": "model3",
        "model3": "model3",
        "奥迪": "a2",
        "奥迪a2": "a2",
        "a2": "a2",
        "奥迪etron": "etron",
        "etron": "etron",
        "奥迪tt": "tt",
        "tt": "tt",
        "宝马": "grandtourer",
        "宝马grandtourer": "grandtourer",
        "grandtourer": "grandtourer",
        "宝马i8": "i8",
        "i8": "i8",
        "宝马mini": "mini",
        "mini": "mini",
        "雪佛兰": "impala",
        "雪佛兰impala": "impala",
        "impala": "impala",
        "雪铁龙": "c3",
        "雪铁龙c3": "c3",
        "c3": "c3",
        "道奇": "charger2020",
        "道奇警车": "charger_police",
        "charger_police": "charger_police",
        "道奇charger": "charger2020",
        "charger2020": "charger2020",
        "福特": "mustang",
        "福特野马": "mustang",
        "野马": "mustang",
        "mustang": "mustang",
        "福特crown": "crown",
        "crown": "crown",
        "吉普": "wrangler_rubicon",
        "吉普牧马人": "wrangler_rubicon",
        "牧马人": "wrangler_rubicon",
        "wrangler_rubicon": "wrangler_rubicon",
        "林肯": "mkz_2020",
        "林肯mkz2017": "mkz_2017",
        "mkz_2017": "mkz_2017",
        "林肯mkz2020": "mkz_2020",
        "mkz_2020": "mkz_2020",
        "奔驰": "benz_coupe",
        "奔驰轿跑": "benz_coupe",
        "benz_coupe": "benz_coupe",
        "奔驰敞篷": "cabrio",
        "cabrio": "cabrio",
        "奔驰ccc": "ccc",
        "ccc": "ccc",
        "迷你": "cooper_s",
        "迷你cooper": "cooper_s",
        "cooper_s": "cooper_s",
        "日产": "patrol",
        "日产micra": "micra",
        "micra": "micra",
        "日产patrol": "patrol",
        "patrol": "patrol",
        "西雅特": "leon",
        "西雅特leon": "leon",
        "leon": "leon",
        "大众": "t2",
        "大众t2": "t2",
        "t2": "t2",
        "大众t3": "t3",
        "t3": "t3",
        "自行车": "crossbike",
        "单车": "crossbike",
        "山地自行车": "crossbike",
        "crossbike": "crossbike",
        "公路自行车": "century",
        "century": "century",
        "荷兰自行车": "omafiets",
        "omafiets": "omafiets"
    }

    PEDESTRIAN_TYPES = {
        "pedestrian": "普通行人",
        "elderly": "老年人",
        "child": "儿童",
        "police": "警察",
        "business": "商务人士",
        "jogger": "慢跑者"
    }

    PEDESTRIAN_TYPE_MAP = {
        "普通行人": "pedestrian",
        "行人": "pedestrian",
        "人": "pedestrian",
        "老年人": "elderly",
        "老人": "elderly",
        "儿童": "child",
        "小孩": "child",
        "孩子": "child",
        "警察": "police",
        "警官": "police",
        "商务人士": "business",
        "商人": "business",
        "白领": "business",
        "慢跑者": "jogger",
        "跑步者": "jogger",
        "跑步的人": "jogger"
    }

    def __init__(self):
        # 将FastMCP工具转换为标准MCP工具格式供AI使用
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "connect_carla",
                    "description": "连接CARLA服务器",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "host": {"type": "string", "description": "CARLA服务器地址", "default": "localhost"},
                            "port": {"type": "integer", "description": "CARLA服务器端口", "default": 2000}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_vehicle",
                    "description": "生成指定类型和数量的车辆。支持类型: model3(Tesla), a2/etron/tt(Audi), grandtourer/i8/mini(BMW), impala(Chevrolet), c3(Citroen), charger_police/charger2020(Dodge), mustang/crown(Ford), wrangler_rubicon(Jeep), mkz_2017/mkz_2020(Lincoln), benz_coupe/cabrio/ccc(Mercedes), cooper_s(Mini), micra/patrol(Nissan), leon(Seat), t2/t3(Volkswagen)。数据量: 取决于地图生成点数量，通常支持10-100+辆车",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "车辆型号，如model3, mustang, a2等", "enum": ["model3", "a2", "etron", "tt", "grandtourer", "i8", "mini", "impala", "c3", "charger_police", "charger2020", "mustang", "crown", "wrangler_rubicon", "mkz_2017", "mkz_2020", "benz_coupe", "cabrio", "ccc", "cooper_s", "micra", "patrol", "leon", "t2", "t3"]},
                            "count": {"type": "integer", "description": "生成车辆数量，默认为1", "default": 1}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_bicycle",
                    "description": "生成自行车。支持类型: crossbike(BH Crossbike), century(Diamondback Century), omafiets(Gazelle Omafiets)。数据量: 取决于地图生成点数量，通常支持1-20辆",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "自行车型号，如crossbike, century, omafiets", "enum": ["crossbike", "century", "omafiets"]},
                            "count": {"type": "integer", "description": "生成自行车数量，默认为1", "default": 1}
                        },
                        "required": ["query"]
                    }
                }
            },
                        {
                "type": "function",
                "function": {
                    "name": "spawn_motorcycle",
                    "description": "生成摩托车。支持类型: ninja(Kawasaki Ninja), yzf(Yamaha YZF), low_rider(Harley-Davidson Low Rider)。数据量: 取决于地图生成点数量，通常支持1-20辆",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "摩托车型号，如ninja, yzf, low_rider", "enum": ["ninja", "yzf", "low_rider"]},
                            "count": {"type": "integer", "description": "生成摩托车数量，默认为1", "default": 1}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_prop",
                    "description": "生成静态道具/警示牌，可指定放在某个actor（如仰翻车辆）后方。支持类型: cone(施工锥), barrier(路障), warning(三角警示牌)。数据量: 通常支持1-50个",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "道具类型，如cone, barrier, warning", "enum": ["cone", "barrier", "warning"]},
                            "count": {"type": "integer", "description": "生成道具数量，默认为1", "default": 1},
                            "target_id": {"type": "integer", "description": "目标actor ID，道具将放置在该目标后方5米处。如仰翻车辆的ID", "default": None}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_overturned_vehicle",
                    "description": "生成仰翻/侧翻的车辆，用于模拟事故场景。支持类型: model3(特斯拉Model3), mustang(福特野马), a2(奥迪A2)",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_type": {"type": "string", "description": "车辆类型，如model3, mustang, a2", "enum": ["model3", "mustang", "a2"], "default": "model3"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_weather",
                    "description": "设置天气（clear/rain/fog）",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "weather_type": {"type": "string", "enum": ["clear", "rain", "fog", "snow", "night"]}
                        },
                        "required": ["weather_type"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_traffic_lights",
                    "description": "获取交通灯状态",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "固定值traffic"}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "cleanup_scene",
                    "description": "清理仿真环境",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_pedestrian",
                    "description": "生成指定类型和数量的行人。支持类型: pedestrian(普通行人), elderly(老年人), child(儿童), police(警察), business(商务人士), jogger(慢跑者)。数据量: 取决于地图大小，通常支持10-100+个行人",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "行人类型，如pedestrian, elderly, child等", "enum": ["pedestrian", "elderly", "child", "police", "business", "jogger"]},
                            "count": {"type": "integer", "description": "生成行人数量，默认为1", "default": 1},
                            "speed": {"type": "number", "description": "行人移动速度（m/s），默认根据类型自动设置：普通行人1.4，老年人1.0，慢跑者2.8"}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "setup_autopilot",
                    "description": "设置车辆自动驾驶模式，可指定范围半径",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enable": {"type": "boolean", "description": "是否启用自动驾驶，默认为true", "default": True},
                            "radius": {"type": "number", "description": "自动驾驶范围半径（米），0表示全图，默认为0", "default": 0.0}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "setup_pedestrian_movement",
                    "description": "设置行人自动移动，可指定范围半径",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enable": {"type": "boolean", "description": "是否启用行人移动，默认为true", "default": True},
                            "radius": {"type": "number", "description": "移动范围半径（米），0表示全图，默认为0", "default": 0.0}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "switch_view",
                    "description": "切换视角模式。支持 third_person(第三人称跟随视角), first_person(第一人称视角), overhead(俯视/鸟瞰视角), free(自由/观察者视角)。切换视角时会自动将观察相机移动到对应位置",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "view_mode": {"type": "string", "description": "视角模式", "enum": ["third_person", "first_person", "overhead", "free"], "default": "third_person"},
                            "target_actor_id": {"type": "integer", "description": "目标actor ID，如果不指定则自动选择最新生成的车辆或行人", "default": None}
                        },
                        "required": ["view_mode"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "start_recording",
                    "description": "开始录制视频。录制的是当前窗口视角的内容，录制过程中可以自由切换视角",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "fps": {"type": "integer", "description": "帧率，默认30", "default": 30}
                        },
                        "required": []
                    }
                }
            },
             {
                "type": "function",
                "function": {
                    "name": "stop_recording",
                    "description": "停止视频录制并保存视频文件。视频将保存到recordings目录下",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            },
             {
                "type": "function",
                "function": {
                    "name": "generate_sumo_network",
                    "description": "生成 SUMO 路网和车流。当用户提到'路网'、'网格'、'SUMO'时，必须使用此工具。不要将其与 CARLA 车辆生成混淆。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "grid_x": {"type": "integer", "description": "X方向网格数，默认3"},
                            "grid_y": {"type": "integer", "description": "Y方向网格数，默认3"},
                            "duration": {"type": "integer", "description": "仿真时长（秒），默认200"},
                            "rate": {"type": "number", "description": "发车间隔（秒/辆），默认2.0"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "generate_openscenario",
                    "description": "生成 OpenSCENARIO 场景文件。参数：xodr_filename(OpenDRIVE文件名，留空则使用内建直路), scenario_name(场景名称), duration(仿真秒数), vehicle_speed(车辆速度m/s)。示例：'生成一个场景，基于 web_generated.xodr，车以10m/s行驶30秒'",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "xodr_filename": {"type": "string", "description": "OpenDRIVE文件名，默认空字符串（使用内建直路）"},
                            "scenario_name": {"type": "string", "description": "场景名称，默认my_scenario"},
                            "duration": {"type": "number", "description": "仿真时长（秒），默认30"},
                            "vehicle_speed": {"type": "number", "description": "车辆速度（m/s），默认10"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_walker",
                    "description": "控制行人停止或恢复移动。支持 stop(停止指定行人)、resume(恢复指定行人)、stop_all(停止所有行人)、resume_all(恢复所有行人)",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string", "enum": ["stop", "resume", "stop_all", "resume_all"], "description": "操作类型"},
                            "walker_id": {"type": "integer", "description": "行人ID，stop/resume时需要"}
                        },
                        "required": ["action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_vehicle_param",
                    "description": "参数化生成车辆，支持参照物/距离/角度/速度控制。当用户要求精确控制生成位置时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "count": {"type": "integer", "description": "生成数量", "default": 1},
                            "blueprint_filter": {"type": "string", "description": "蓝图过滤，如 vehicle.tesla.model3", "default": "vehicle.*"},
                            "autopilot": {"type": "boolean", "description": "是否开启自动驾驶", "default": True},
                            "reference_id": {"type": "integer", "description": "参照物actor ID，None则使用地图spawn point", "default": None},
                            "relative_distance": {"type": "number", "description": "相对参照物的距离（米）", "default": 10.0},
                            "relative_angle": {"type": "number", "description": "相对参照物的角度（度，0=正前方）", "default": 0.0},
                            "initial_speed": {"type": "number", "description": "初始速度（m/s）", "default": 0.0}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_highway_ramp",
                    "description": "高速-进出匝道场景：在高速公路主路与匝道的汇流/分流点自动布设车辆并开启自动驾驶。推荐地图Town04。当用户提到'匝道'、'高速进出匝道'、'汇入匝道'、'驶出匝道'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "ramp_type": {"type": "string", "enum": ["on", "off"], "description": "on=匝道汇入(默认), off=主路驶出匝道", "default": "on"},
                            "vehicle_count": {"type": "integer", "description": "总车辆数（主路+匝道），默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名如Town04", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_lane_merge",
                    "description": "城市-车道合并场景：自动寻找车道消失（车道数减少）位置，在消失车道与延续车道布设车辆演示汇流。当用户提到'车道合并'、'车道减少'、'汇流'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_diverge_merge",
                    "description": "城市-分合流路口场景：找到多臂路口，在各进口臂布设车辆，经路口分流/合流。当用户提到'分合流路口'、'分流'、'路口合流'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_side_road",
                    "description": "城市-辅路场景：自动寻找与主路平行的辅路，在主路和辅路上同时布设车辆。当用户提到'辅路'、'辅道'、'侧路'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_junction_light",
                    "description": "城市-路口（十字、T型、Y型）及红绿灯场景：寻找带信号灯的多臂路口并布设车辆。当用户提到'路口红绿灯'、'十字路口'、'T型路口'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "junction_shape": {"type": "string", "enum": ["any", "cross", "t", "y"], "description": "路口形状", "default": "any"},
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，指定加载的地图名", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_tunnel",
                    "description": "隧道场景：自动寻找下沉道路（隧道/地下道）布设车辆并开车灯。推荐Town04/Town05。当用户提到'隧道'、'地下道'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认4", "default": 4},
                            "map_name": {"type": "string", "description": "可选，推荐Town04/Town05", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_roundabout",
                    "description": "环岛场景：自动寻找环形路口布设车辆。推荐Town05。当用户提到'环岛'、'环形路口'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_count": {"type": "integer", "description": "总车辆数，默认5", "default": 5},
                            "map_name": {"type": "string", "description": "可选，推荐Town05", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_pedestrian_pose",
                    "description": "儿童/成人姿态场景：站立、行走、蹲下、躺下、打伞。蹲下/躺下/打伞无动画，仅能set_transform硬摆姿态。当用户提到'儿童'、'蹲下'、'躺下'、'打伞'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "pedestrian_type": {"type": "string", "enum": ["child", "pedestrian", "elderly", "police"], "description": "行人类型，默认child", "default": "child"},
                            "pose": {"type": "string", "enum": ["stand", "walk", "crouch", "lie", "umbrella"], "description": "姿态", "default": "stand"},
                            "count": {"type": "integer", "description": "数量，默认1", "default": 1}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_two_wheeler",
                    "description": "自行车/摩托车-站立、行进、倒地场景。当用户提到'自行车倒地'、'摩托车行驶'等时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_type": {"type": "string", "enum": ["bicycle", "motorcycle"], "description": "二轮车类型", "default": "bicycle"},
                            "state": {"type": "string", "enum": ["stand", "move", "fallen"], "description": "状态", "default": "stand"},
                            "count": {"type": "integer", "description": "数量，默认2", "default": 2}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_special_vehicle",
                    "description": "特殊任务车辆：救护车(vehicle.ford.ambulance)、警车(vehicle.dodge.charger_police)。自动配送物流车无蓝图无法完成。当用户提到'救护车'、'警车'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_type": {"type": "string", "enum": ["ambulance", "police"], "description": "车辆类型", "default": "ambulance"},
                            "moving": {"type": "boolean", "description": "是否行进", "default": True},
                            "count": {"type": "integer", "description": "数量，默认1", "default": 1}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_officer",
                    "description": "特殊群体场景：交警(walker.pedestrian.0030/0032)、轮椅行人(use_wheelchair属性)。婴儿车无蓝图无法完成。当用户提到'交警'、'轮椅'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "element": {"type": "string", "enum": ["traffic_police", "wheelchair", "stroller"], "description": "元素类型", "default": "traffic_police"},
                            "with_companion": {"type": "boolean", "description": "是否有成人伴行", "default": False}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_lighting",
                    "description": "弱光条件设置：清晨(dawn)/黄昏(dusk)/阴天(overcast)/夜晚(night)。当用户提到'清晨'、'黄昏'、'阴天'、'弱光'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "condition": {"type": "string", "enum": ["dawn", "dusk", "overcast", "night"], "description": "弱光条件", "default": "night"}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_backlight",
                    "description": "逆光场景：低角度太阳正对来车方向，对向车辆开大灯。当用户提到'逆光'、'太阳眩光'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "spawn_rollover_vehicle",
                    "description": "翻车车辆：侧翻/仰翻，汽车或货车（货车用Sprinter近似）。当用户提到'侧翻'、'翻车'、'仰翻'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "vehicle_type": {"type": "string", "enum": ["car", "van", "truck"], "description": "车辆类型", "default": "car"},
                            "rollover": {"type": "string", "enum": ["side", "upside"], "description": "侧翻/仰翻", "default": "side"},
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_lead_vehicle",
                    "description": "前车急刹/静止场景：同车道前车+自动驾驶后车。当用户提到'前车急刹'、'前车静止'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "mode": {"type": "string", "enum": ["stationary", "brake"], "description": "静止/急刹", "default": "stationary"},
                            "distance": {"type": "number", "description": "前后车距(米)", "default": 25.0},
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_cut_in",
                    "description": "危险切入场景：邻道车3秒后突然变道到本车前方。当用户提到'危险切入'、'cut-in'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "direction": {"type": "string", "enum": ["left", "right"], "description": "切入方向", "default": "left"},
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_lead_disappear",
                    "description": "前车消失场景：前车4秒后切入邻道，露出前方静止障碍物。当用户提到'前车消失'、'前车切出'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_crossing_hazard",
                    "description": "路口-危险横穿场景：行人/机动车/自行车横穿本车方向。当用户提到'危险横穿'、'鬼探头'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "crosser": {"type": "string", "enum": ["pedestrian", "vehicle", "bicycle"], "description": "横穿对象", "default": "pedestrian"},
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_low_overlap",
                    "description": "前方低重叠率行驶目标：目标车贴车道线侵入本车道行驶。当用户提到'低重叠'、'压线行驶'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "offset_ratio": {"type": "number", "description": "侵入比例0~0.5", "default": 0.35},
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_wrong_way",
                    "description": "逆行场景：对向车道出现逆行车辆。当用户提到'逆行'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "speed": {"type": "number", "description": "逆行速度(m/s)", "default": 8.0},
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "scenario_unprotected_turn",
                    "description": "路口无保护通行场景：无信号灯路口多方向来车交汇。当用户提到'无保护'、'无信号灯路口'时使用。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "map_name": {"type": "string", "description": "可选，指定地图", "default": None}
                        },
                        "required": []
                    }
                }
            }
        ] 

    def process_markdown(self, text):
        """在Python端处理Markdown格式"""
        result = text

        # 处理标题
        result = re.sub(r'^### (.+)$', r'<h3><strong>\1</strong></h3>', result, flags=re.MULTILINE)
        result = re.sub(r'^## (.+)$', r'<h2><strong>\1</strong></h2>', result, flags=re.MULTILINE)
        result = re.sub(r'^# (.+)$', r'<h1><strong>\1</strong></h1>', result, flags=re.MULTILINE)

        # 处理粗体链接 **[text](url)**
        result = re.sub(r'\*\*\[([^\]]+)\]\(([^)]+)\)\*\*', r'<strong><a href="\2" target="_blank">\1</a></strong>',
                        result)

        # 处理普通链接 [text](url)
        result = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2" target="_blank">\1</a>', result)

        # 处理粗体文本 **text**
        result = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', result)

        # 处理换行
        result = result.replace('\n', '<br>')

        return result

    async def call_deepseek_with_tools(self, messages):
        """调用Deepseek API，包含FastMCP工具定义"""
        headers = config.get_deepseek_headers()

        data = {
            "model": "deepseek-chat",
            "messages": messages,
            "tools": self.tools,
            "tool_choice": "auto",
            "max_tokens": 2000,
            "temperature": 0.7
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(config.DEEPSEEK_API_URL, headers=headers, json=data) as response:
                if response.status == 200:
                    return await response.json()
                else:
                    error_text = await response.text()
                    raise Exception(f"Deepseek API调用失败: {response.status} - {error_text}")

    async def execute_fastmcp_tool_call(self, tool_call):
        """执行FastMCP工具调用 - 桥接到FastMCP装饰器函数"""
        function_name = tool_call["function"]["name"]
        arguments = json.loads(tool_call["function"]["arguments"])

        app_logger.info(f"🔧 执行FastMCP工具: {function_name}")
        app_logger.info(f"📝 参数: {arguments}")

        try:
            # 调用实际的工具实现函数（避免FastMCP装饰器问题）
            if function_name == "connect_carla":
                host = arguments.get("host", "localhost")
                port = arguments.get("port", 2000)
                result = await carla_client.connect(host, port)
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_vehicle":
                app_logger.info(f"spawn_vehicle参数详情: {arguments}")
                result = await spawn_vehicle_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1))
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_bicycle":
                app_logger.info(f"spawn_bicycle参数详情: {arguments}")
                result = await spawn_bicycle_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1))
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_motorcycle":
                app_logger.info(f"spawn_motorcycle参数详情: {arguments}")
                result = await spawn_motorcycle_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1))
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_prop":
                app_logger.info(f"spawn_prop参数详情: {arguments}")
                result = await spawn_prop_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1)),
                    target_id=arguments.get("target_id")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_overturned_vehicle":
                app_logger.info(f"spawn_overturned_vehicle参数详情: {arguments}")
                result = await spawn_overturned_vehicle_impl(
                    vehicle_type=arguments.get("vehicle_type", "model3")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_pedestrian":
                app_logger.info(f"spawn_pedestrian参数详情: {arguments}")
                result = await spawn_pedestrian_impl(
                    query=arguments["query"],
                    count=int(arguments.get("count", 1)),
                    speed=arguments.get("speed")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_weather":
                result = await carla_client.set_weather(arguments["weather_type"])
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_synchronous_mode":
                result = await carla_client.set_synchronous_mode(
                    arguments.get("enabled", True),
                    arguments.get("fixed_delta_seconds", 0.05)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "setup_autopilot":
                result = await carla_client.setup_autopilot(
                    arguments.get("enable", True),
                    arguments.get("radius", 0.0)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "switch_view_mode":
                result = await carla_client.switch_view_mode(
                    arguments.get("view_mode", "third_person"),
                    arguments.get("target_id")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "start_recording":
                result = await carla_client.start_recording(
                    arguments.get("filename", "simulation_recording"),
                    arguments.get("duration", 30)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "stop_recording":
                result = await carla_client.stop_recording()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_third_person_view":
                result = await carla_client.set_third_person_view(arguments.get("target_id"))
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_first_person_view":
                result = await carla_client.set_first_person_view(arguments.get("target_id"))
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_overhead_view":
                result = await carla_client.set_overhead_view(arguments.get("target_id"))
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_free_view":
                result = await carla_client.set_free_view()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "start_view_follow":
                result = await carla_client.start_view_follow(
                    arguments.get("view_mode", "third_person"),
                    arguments.get("target_id")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "stop_view_follow":
                result = await carla_client.stop_view_follow()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_sumo_grid_network":
                result = await spawn_sumo_grid_network_impl(
                    grid_size=arguments.get("grid_size", 3),
                    simulation_time=arguments.get("simulation_time", 200),
                    vehicle_spawn_interval=arguments.get("vehicle_spawn_interval", 2)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "generate_openscenario":
                result = await generate_openscenario_impl(
                    xodr_file=arguments.get("xodr_file", "web_generated.xodr"),
                    vehicle_speed=arguments.get("vehicle_speed", 10.0),
                    duration=arguments.get("duration", 30.0)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_highway_ramp":
                result = await scenario_highway_ramp_impl(
                    ramp_type=arguments.get("ramp_type", "on"),
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_lane_merge":
                result = await scenario_lane_merge_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_diverge_merge":
                result = await scenario_diverge_merge_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_side_road":
                result = await scenario_side_road_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "cleanup_scene":
                result = await cleanup_scene_impl()
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "scenario_junction_light":
                result = await scenario_junction_light_impl(
                    junction_shape=arguments.get("junction_shape", "any"),
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_tunnel":
                result = await scenario_tunnel_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 4)),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_roundabout":
                result = await scenario_roundabout_impl(
                    vehicle_count=int(arguments.get("vehicle_count", 5)),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "spawn_pedestrian_pose":
                result = await spawn_pedestrian_pose_impl(
                    pedestrian_type=arguments.get("pedestrian_type", "child"),
                    pose=arguments.get("pose", "stand"),
                    count=int(arguments.get("count", 1))
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_two_wheeler":
                result = await scenario_two_wheeler_impl(
                    vehicle_type=arguments.get("vehicle_type", "bicycle"),
                    state=arguments.get("state", "stand"),
                    count=int(arguments.get("count", 2))
                )
                return {"success": True, "data": result}

            elif function_name == "spawn_special_vehicle":
                result = await spawn_special_vehicle_impl(
                    vehicle_type=arguments.get("vehicle_type", "ambulance"),
                    moving=bool(arguments.get("moving", True)),
                    count=int(arguments.get("count", 1))
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_officer":
                result = await scenario_officer_impl(
                    element=arguments.get("element", "traffic_police"),
                    with_companion=bool(arguments.get("with_companion", False))
                )
                return {"success": True, "data": result}

            elif function_name == "set_lighting":
                result = await carla_client.set_lighting(arguments.get("condition", "night"))
                if result.get("success"):
                    result = f"✅ 弱光环境已设置：{result['scenario']}"
                else:
                    result = f"❌ {result.get('error', '设置失败')}"
                return {"success": True, "data": result}

            elif function_name == "scenario_backlight":
                result = await scenario_backlight_impl(map_name=arguments.get("map_name"))
                return {"success": True, "data": result}

            elif function_name == "spawn_rollover_vehicle":
                result = await spawn_rollover_vehicle_impl(
                    vehicle_type=arguments.get("vehicle_type", "car"),
                    rollover=arguments.get("rollover", "side"),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_lead_vehicle":
                result = await scenario_lead_vehicle_impl(
                    mode=arguments.get("mode", "stationary"),
                    distance=float(arguments.get("distance", 25.0)),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_cut_in":
                result = await scenario_cut_in_impl(
                    direction=arguments.get("direction", "left"),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_lead_disappear":
                result = await scenario_lead_disappear_impl(map_name=arguments.get("map_name"))
                return {"success": True, "data": result}

            elif function_name == "scenario_crossing_hazard":
                result = await scenario_crossing_hazard_impl(
                    crosser=arguments.get("crosser", "pedestrian"),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_low_overlap":
                result = await scenario_low_overlap_impl(
                    offset_ratio=float(arguments.get("offset_ratio", 0.35)),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_wrong_way":
                result = await scenario_wrong_way_impl(
                    speed=float(arguments.get("speed", 8.0)),
                    map_name=arguments.get("map_name")
                )
                return {"success": True, "data": result}

            elif function_name == "scenario_unprotected_turn":
                result = await scenario_unprotected_turn_impl(map_name=arguments.get("map_name"))
                return {"success": True, "data": result}

            else:
                return {
                    "success": False,
                    "error": f"未知的工具: {function_name}"
                }

        except Exception as e:
            app_logger.error(f"❌ FastMCP工具执行失败: {str(e)}")
            return {
                "success": False,
                "error": str(e)
            }

    # 定义车辆和行人的类型信息
        VEHICLE_TYPES = {
        "model3": "Tesla Model 3",
        "a2": "Audi A2",
        "etron": "Audi e-tron",
        "tt": "Audi TT",
        "grandtourer": "BMW Grand Tourer",
        "i8": "BMW i8",
        "mini": "BMW Mini",
        "impala": "Chevrolet Impala",
        "c3": "Citroen C3",
        "charger_police": "Dodge Charger Police",
        "charger2020": "Dodge Charger 2020",
        "mustang": "Ford Mustang",
        "crown": "Ford Crown",
        "wrangler_rubicon": "Jeep Wrangler Rubicon",
        "mkz_2017": "Lincoln MKZ 2017",
        "mkz_2020": "Lincoln MKZ 2020",
        "benz_coupe": "Mercedes-Benz Coupe",
        "cabrio": "Mercedes-Benz Cabrio",
        "ccc": "Mercedes-Benz CCC",
        "cooper_s": "Mini Cooper S",
        "micra": "Nissan Micra",
        "patrol": "Nissan Patrol",
        "leon": "Seat Leon",
        "t2": "Volkswagen T2",
        "t3": "Volkswagen T3",
        "crossbike": "BH Crossbike",
        "century": "Diamondback Century",
        "omafiets": "Gazelle Omafiets"
    }

    # 车辆中文到英文的映射
    VEHICLE_TYPE_MAP = {
        "特斯拉": "model3",
        "特斯拉model3": "model3",
        "model3": "model3",
        "奥迪": "a2",
        "奥迪a2": "a2",
        "a2": "a2",
        "奥迪etron": "etron",
        "etron": "etron",
        "奥迪tt": "tt",
        "tt": "tt",
        "宝马": "grandtourer",
        "宝马grandtourer": "grandtourer",
        "grandtourer": "grandtourer",
        "宝马i8": "i8",
        "i8": "i8",
        "宝马mini": "mini",
        "mini": "mini",
        "雪佛兰": "impala",
        "雪佛兰impala": "impala",
        "impala": "impala",
        "雪铁龙": "c3",
        "雪铁龙c3": "c3",
        "c3": "c3",
        "道奇": "charger2020",
        "道奇警车": "charger_police",
        "警车": "charger_police",
        "charger_police": "charger_police",
        "道奇charger": "charger2020",
        "charger2020": "charger2020",
        "福特": "mustang",
        "福特野马": "mustang",
        "野马": "mustang",
        "mustang": "mustang",
        "福特crown": "crown",
        "crown": "crown",
        "吉普": "wrangler_rubicon",
        "吉普牧马人": "wrangler_rubicon",
        "牧马人": "wrangler_rubicon",
        "wrangler_rubicon": "wrangler_rubicon",
        "林肯": "mkz_2020",
        "林肯mkz2017": "mkz_2017",
        "mkz_2017": "mkz_2017",
        "林肯mkz2020": "mkz_2020",
        "mkz_2020": "mkz_2020",
        "奔驰": "benz_coupe",
        "奔驰轿跑": "benz_coupe",
        "benz_coupe": "benz_coupe",
        "奔驰敞篷": "cabrio",
        "cabrio": "cabrio",
        "奔驰ccc": "ccc",
        "ccc": "ccc",
        "迷你": "cooper_s",
        "迷你cooper": "cooper_s",
        "cooper_s": "cooper_s",
        "日产": "patrol",
        "日产micra": "micra",
        "micra": "micra",
        "日产patrol": "patrol",
        "patrol": "patrol",
        "西雅特": "leon",
        "西雅特leon": "leon",
        "leon": "leon",
        "大众": "t2",
        "大众t2": "t2",
        "t2": "t2",
        "大众t3": "t3",
        "t3": "t3",
        "自行车": "crossbike",
        "单车": "crossbike",
        "山地自行车": "crossbike",
        "crossbike": "crossbike",
        "公路自行车": "century",
        "century": "century",
        "荷兰自行车": "omafiets",
        "omafiets": "omafiets"
    }
    PEDESTRIAN_TYPES = {
        "pedestrian": "普通行人",
        "elderly": "老年人",
        "child": "儿童",
        "police": "警察",
        "business": "商务人士",
        "jogger": "慢跑者"
    }

    # 中文到英文的映射
    PEDESTRIAN_TYPE_MAP = {
        "普通行人": "pedestrian",
        "行人": "pedestrian",
        "人": "pedestrian",
        "老年人": "elderly",
        "老人": "elderly",
        "儿童": "child",
        "小孩": "child",
        "孩子": "child",
        "警察": "police",
        "警官": "police",
        "商务人士": "business",
        "商人": "business",
        "白领": "business",
        "慢跑者": "jogger",
        "跑步者": "jogger",
        "跑步的人": "jogger"
    }
    def _check_spawn_intent(self, message):
        """检测用户是否有生成车辆、自行车或行人的意图，但缺少必要参数"""
        message = message.lower()

        # 首先排除 SUMO 路网生成相关的指令
        sumo_keywords = ['路网', '网格', 'sumo', '仿真', '交通', '场景']
        for kw in sumo_keywords:
            if kw in message:
                return {
                    'needs_vehicle_type': False,
                    'needs_vehicle_count': False,
                    'needs_pedestrian_type': False,
                    'needs_pedestrian_count': False,
                    'needs_bicycle_type': False,
                    'needs_bicycle_count': False,
                    'is_ambiguous': False
                }

        # 排除视角控制相关的指令
        view_keywords = ['视角', '切换', '人称', '俯视', '鸟瞰', '自由视角', '录制', '录像', '视频']
        if any(kw in message for kw in view_keywords):
            return {
                'needs_vehicle_type': False,
                'needs_vehicle_count': False,
                'needs_pedestrian_type': False,
                'needs_pedestrian_count': False,
                'needs_bicycle_type': False,
                'needs_bicycle_count': False,
                'is_ambiguous': False
            }

        # 排除自动驾驶相关指令（不是生成车辆）
        autopilot_keywords = ['自动驾驶', '车辆运行', '车自己开', '开启自动驾驶', '让车', '让车辆']
        if any(kw in message for kw in autopilot_keywords):
            return {
                'needs_vehicle_type': False,
                'needs_vehicle_count': False,
                'needs_pedestrian_type': False,
                'needs_pedestrian_count': False,
                'needs_bicycle_type': False,
                'needs_bicycle_count': False,
                'is_ambiguous': False
            }

        import re
        has_count = bool(re.search(r'\d+\s*[辆个]', message))

        # ===== 先初始化 result =====
        result = {
            'needs_vehicle_type': False,
            'needs_vehicle_count': False,
            'needs_pedestrian_type': False,
            'needs_pedestrian_count': False,
            'needs_bicycle_type': False,
            'needs_bicycle_count': False,
            'is_ambiguous': False
        }

        # 1️⃣ 仰翻车辆检测（最优先，默认 model3，不提示用户）
        overturned_keywords = ['仰翻', '侧翻', '翻车', '事故车', '翻了', '底朝天']
        has_overturned_request = any(kw in message for kw in overturned_keywords)
        if has_overturned_request:
            return result  # 直接放行，AI 会默认用 model3

        # 2️⃣ 摩托车检测（在普通车辆之前，因为"摩托车"含"车"字）
        motorcycle_keywords = ['摩托车', '生成摩托车', '来辆摩托车', '添加摩托车']
        has_motorcycle_request = any(kw in message for kw in motorcycle_keywords)
        if has_motorcycle_request:
            return result  # 直接放行，AI 会默认用 ninja

        # 3️⃣ 道具检测
        prop_keywords = ['施工锥', '路障', '警示牌', '三角警示牌', '生成道具']
        has_prop_request = any(kw in message for kw in prop_keywords)
        if has_prop_request:
            return result  # 直接放行，AI 会默认用 cone

        # 4️⃣ 自行车检测
        bicycle_keywords = ['自行车', '单车', '生成自行车', '来辆自行车', '添加自行车']
        has_bicycle_request = any(kw in message for kw in bicycle_keywords)
        has_bicycle_type = any(btype in message for btype in ['山地', '公路', '荷兰', 'crossbike', 'century', 'omafiets'])
        if has_bicycle_request and not has_bicycle_type:
            result['needs_bicycle_type'] = True
            result['needs_bicycle_count'] = not has_count
            result['is_ambiguous'] = True
            return result

        # 5️⃣ 车辆检测（最后，避免被"摩托车"等含"车"字的词误触发）
        vehicle_keywords = ['车辆', '汽车', '生成车', '创建车', '来车', '加车', '添加车辆']
        # 排除已处理的类型，避免"摩托车"触发
        if has_motorcycle_request or has_prop_request or has_overturned_request:
            is_vehicle_request = False
        else:
            is_vehicle_request = any(kw in message for kw in vehicle_keywords) or ('车' in message and '摩托车' not in message and '自行车' not in message)
        
        has_vehicle_type = any(vtype in message for vtype in self.VEHICLE_TYPES.keys()) or \
                           any(vname in message for vname in self.VEHICLE_TYPE_MAP.keys())

        # 6️⃣ 行人检测
        pedestrian_keywords = ['行人', '生成行人', '创建行人', '添加行人', '路人']
        person_spawn_verbs = ['生成', '创建', '来', '加', '添加', '放', 'spawn']
        specific_pedestrian_keywords = set(self.PEDESTRIAN_TYPES.keys()) | set(self.PEDESTRIAN_TYPE_MAP.keys()) - {"行人", "人"}
        has_pedestrian_type = any(ptype in message for ptype in specific_pedestrian_keywords)
        is_pedestrian_request = any(kw in message for kw in pedestrian_keywords)
        if not is_pedestrian_request and '人' in message:
            has_spawn_verb = any(verb in message for verb in person_spawn_verbs)
            if has_spawn_verb and '人称' not in message:
                is_pedestrian_request = True

        # 设置结果
        if is_vehicle_request and not has_vehicle_type:
            result['needs_vehicle_type'] = True
            result['needs_vehicle_count'] = not has_count
            result['is_ambiguous'] = True

        if is_pedestrian_request and not has_pedestrian_type:
            result['needs_pedestrian_type'] = True
            result['needs_pedestrian_count'] = not has_count
            result['is_ambiguous'] = True

        return result

    def _generate_spawn_prompt(self, check_result):
        """生成参数询问提示"""
        prompt_parts = []

        if check_result.get('needs_vehicle_type'):
            vehicle_list = "\n".join([f"  • {name} ({key})" for key, name in self.VEHICLE_TYPES.items() if key not in ['crossbike', 'century', 'omafiets']])
            prompt_parts.append(f"🚗 **可用车辆类型：**\n{vehicle_list}")

        if check_result.get('needs_vehicle_count'):
            prompt_parts.append("🚗 **车辆数量：** 支持生成 1-100+ 辆车（取决于地图可用生成点数量）")

        if check_result.get('needs_bicycle_type'):
            prompt_parts.append("🚲 **可用自行车类型：**\n  • 山地自行车 (crossbike)\n  • 公路自行车 (century)\n  • 荷兰自行车 (omafiets)")

        if check_result.get('needs_motorcycle_type') or check_result.get('needs_motorcycle_count'):
                prompt_parts.append('  • "生成3辆川崎忍者摩托车"')
                prompt_parts.append('  • "来5辆雅马哈YZF"')
                prompt_parts.append('  • "生成2辆哈雷"')

        if check_result.get('needs_prop_type') or check_result.get('needs_prop_count'):
                prompt_parts.append('  • "生成3个施工锥"')
                prompt_parts.append('  • "来5个三角警示牌"')
                prompt_parts.append('  • "生成2个路障"')

        if check_result.get('needs_bicycle_count'):
            prompt_parts.append("🚲 **自行车数量：** 支持生成 1-20 辆")

        if check_result.get('needs_pedestrian_type'):
            pedestrian_list = "\n".join([f"  • {name} ({key})" for key, name in self.PEDESTRIAN_TYPES.items()])
            prompt_parts.append(f"🚶 **可用行人类型：**\n{pedestrian_list}")

        if check_result.get('needs_pedestrian_count'):
            prompt_parts.append("🚶 **行人数量：** 支持生成 1-100+ 个行人（取决于地图大小）")

        if prompt_parts:
            prompt_parts.insert(0, "请提供以下信息以完成生成：\n")
            prompt_parts.append("\n💡 **示例指令：**")
            
            if check_result.get('needs_vehicle_type') or check_result.get('needs_vehicle_count'):
                prompt_parts.append('  • "生成5辆特斯拉"')
                prompt_parts.append('  • "来10辆福特野马"')
                prompt_parts.append('  • "生成3辆宝马"')
                prompt_parts.append('  • "来5辆奔驰"')
            
            if check_result.get('needs_bicycle_type') or check_result.get('needs_bicycle_count'):
                prompt_parts.append('  • "生成3辆山地自行车"')
                prompt_parts.append('  • "来5辆公路自行车"')
                prompt_parts.append('  • "生成2辆荷兰自行车"')
            
            if check_result.get('needs_pedestrian_type') or check_result.get('needs_pedestrian_count'):
                prompt_parts.append('  • "生成3个老年人"')
                prompt_parts.append('  • "来5个警察"')
                prompt_parts.append('  • "生成2个儿童"')
                prompt_parts.append('  • "来10个普通行人"')

        return "\n\n".join(prompt_parts)

    def _check_view_switch_intent(self, message):
        """检测用户是否有切换视角的意图，如果有多个行人/车辆则询问选择

        Returns:
            dict: 包含是否需要询问、视角模式、可用目标列表等信息
        """
        import re
        message = message.lower()

        # 视角相关关键词
        view_keywords = ['视角', '人称', '俯视', '鸟瞰', '自由视角', '旁观者']
        has_view_intent = any(kw in message for kw in view_keywords)

        if not has_view_intent:
            return {'needs_target_selection': False}

        # 检测是否指定了特定的视角模式
        view_mode = None
        if '第一人称' in message or '第一视角' in message or 'first_person' in message:
            view_mode = 'first_person'
        elif '第三人称' in message or '第三视角' in message or 'third_person' in message:
            view_mode = 'third_person'
        elif '俯视' in message or '鸟瞰' in message or 'overhead' in message:
            view_mode = 'overhead'
        elif '自由' in message or 'free' in message:
            view_mode = 'free'
        elif '旁观者' in message or 'bystander' in message:
            view_mode = 'bystander'
        else:
            view_mode = 'third_person'
        # 旁观者视角不需要选择目标
        if view_mode == 'bystander':
            return {'needs_target_selection': False, 'view_mode': 'bystander'}

        # 尝试从消息中提取ID（支持 "ID26", "ID 26", "id26", "id 26" 等格式）
        target_id = None
        id_patterns = [
            r'id\s*(\d+)',  # ID 26, id26, ID26
            r'[^\d](\d+)$',  # 以数字结尾
            r'\s(\d+)\s',  # 中间有数字
        ]
        for pattern in id_patterns:
            match = re.search(pattern, message, re.IGNORECASE)
            if match:
                target_id = int(match.group(1))
                break

        # 获取当前所有行人和车辆
        pedestrians = carla_client.get_all_pedestrians()
        vehicles = carla_client.get_all_vehicles()

        all_targets = []
        for p in pedestrians:
            all_targets.append({'id': p['id'], 'type': p['type_name'], 'category': '行人'})
        for v in vehicles:
            all_targets.append({'id': v['id'], 'type': v['type_name'], 'category': '车辆'})

        # 如果提取到了ID，验证该ID是否存在
        if target_id is not None:
            target_exists = any(t['id'] == target_id for t in all_targets)
            if target_exists:
                return {
                    'needs_target_selection': False,
                    'view_mode': view_mode,
                    'target_id': target_id
                }

        # 如果只有一个目标，直接使用
        if len(all_targets) == 1:
            return {
                'needs_target_selection': False,
                'view_mode': view_mode,
                'target_id': all_targets[0]['id']
            }

        # 如果有多个目标，需要询问
        if len(all_targets) > 1:
            return {
                'needs_target_selection': True,
                'view_mode': view_mode,
                'targets': all_targets
            }

        # 没有可用的目标
        return {
            'needs_target_selection': False,
            'view_mode': view_mode,
            'no_targets': True
        }

    def _generate_view_selection_prompt(self, view_mode, targets):
        """生成视角目标选择提示"""
        view_mode_names = {
            'first_person': '第一人称视角',
            'third_person': '第三人称视角',
            'overhead': '俯视视角',
            'free': '自由视角'
        }

        prompt_parts = [f"👁️ 请选择要切换到{view_mode_names.get(view_mode, view_mode)}的目标：\n"]

        for i, target in enumerate(targets, 1):
            prompt_parts.append(f"  {i}. ID: {target['id']} - {target['type']} ({target['category']})")

        prompt_parts.append(f"\n💡 **示例指令：**")
        prompt_parts.append(f'  • "切换到{view_mode_names.get(view_mode, view_mode)} ID {targets[0]["id"]}"')
        prompt_parts.append(f'  • "用ID {targets[0]["id"]} 切换{view_mode_names.get(view_mode, view_mode)}"')

        return "\n".join(prompt_parts)

    async def chat(self, user_message):
        """处理聊天请求 - 使用FastMCP工具的AI对话"""

        # 检查是否有生成意图但缺少参数
        spawn_check = self._check_spawn_intent(user_message)
        if spawn_check['is_ambiguous']:
            prompt = self._generate_spawn_prompt(spawn_check)
            return {
                "message": self.process_markdown(prompt),
                "tool_calls": None,
                "conversation": [{"role": "user", "content": user_message}]
            }

        # 检查是否有视角切换意图
        view_check = self._check_view_switch_intent(user_message)
        if view_check.get('needs_target_selection'):
            # 有多个目标且用户没有指定ID，显示选择列表
            prompt = self._generate_view_selection_prompt(view_check['view_mode'], view_check['targets'])
            return {
                "message": self.process_markdown(prompt),
                "tool_calls": None,
                "conversation": [{"role": "user", "content": user_message}]
            }
        elif view_check.get('target_id'):
            # 用户指定了ID或只有一个目标，直接执行视角切换
            result = await switch_view_impl(
                view_mode=view_check['view_mode'],
                target_actor_id=view_check['target_id']
            )
            return {
                "message": self.process_markdown(result),
                "tool_calls": None,
                "conversation": [{"role": "user", "content": user_message}]
            }

        # 初始消息
        messages = [
    {
        "role": "system",
        "content": """## 🚦 关键路由规则（必须严格遵守）

- 如果用户消息中包含 "路网"、"网格"、"SUMO"、"生成路网" 这些词，**必须**调用 `generate_sumo_network` 工具。
- 如果用户消息中包含 "场景"、"OpenSCENARIO"、"生成场景"，**必须**调用 `generate_openscenario` 工具，**不要**将其理解为车辆生成。
- 如果用户消息中包含 "匝道"、"高速进出匝道"、"汇入匝道"、"驶出匝道"，**必须**调用 `scenario_highway_ramp` 工具。
- 如果用户消息中包含 "车道合并"、"车道减少"、"汇流"，**必须**调用 `scenario_lane_merge` 工具。
- 如果用户消息中包含 "分合流路口"、"分流"、"路口合流"，**必须**调用 `scenario_diverge_merge` 工具。
- 如果用户消息中包含 "辅路"、"辅道"，**必须**调用 `scenario_side_road` 工具。
- 如果用户消息中包含 "路口红绿灯"、"十字路口"、"T型路口"、"Y型路口"，**必须**调用 `scenario_junction_light` 工具。
- 如果用户消息中包含 "隧道"、"地下道"，**必须**调用 `scenario_tunnel` 工具。
- 如果用户消息中包含 "环岛"、"环形路口"，**必须**调用 `scenario_roundabout` 工具。
- 如果用户消息中包含 "儿童"、"蹲下"、"躺下"、"打伞"（行人姿态），**必须**调用 `spawn_pedestrian_pose` 工具。
- 如果用户消息中包含 "自行车倒地"、"摩托车倒地"、"二轮车"，**必须**调用 `scenario_two_wheeler` 工具。
- 如果用户消息中包含 "救护车"、"警车"（特殊任务车辆），**必须**调用 `spawn_special_vehicle` 工具。
- 如果用户消息中包含 "交警"、"轮椅"，**必须**调用 `scenario_officer` 工具。
- 如果用户消息中包含 "清晨"、"黄昏"、"阴天"、"弱光"，**必须**调用 `set_lighting` 工具。
- 如果用户消息中包含 "逆光"，**必须**调用 `scenario_backlight` 工具。
- 如果用户消息中包含 "侧翻"，**必须**调用 `spawn_rollover_vehicle` 工具（"仰翻"仍用spawn_overturned_vehicle）。
- 如果用户消息中包含 "前车急刹"、"前车静止"，**必须**调用 `scenario_lead_vehicle` 工具。
- 如果用户消息中包含 "危险切入"、"cut-in"，**必须**调用 `scenario_cut_in` 工具。
- 如果用户消息中包含 "前车消失"、"前车切出"，**必须**调用 `scenario_lead_disappear` 工具。
- 如果用户消息中包含 "危险横穿"、"鬼探头"，**必须**调用 `scenario_crossing_hazard` 工具。
- 如果用户消息中包含 "低重叠"、"压线行驶"，**必须**调用 `scenario_low_overlap` 工具。
- 如果用户消息中包含 "逆行"，**必须**调用 `scenario_wrong_way` 工具。
- 如果用户消息中包含 "无保护"、"无信号灯路口"，**必须**调用 `scenario_unprotected_turn` 工具。
- 如果用户消息中包含 "连接CARLA"、"CARLA服务器"，调用 `connect_carla` 工具。
- **绝对不要**将"路网"或"网格"理解为 CARLA 车辆生成请求。

你是一个GitHub搜索助手，基于FastMCP框架提供服务。你有以下工具可以使用：

CARLA仿真功能：
5. connect_carla - 连接CARLA服务器（默认localhost:2000）
6. spawn_vehicle - 生成车辆，支持参数：query(车型), count(数量)。支持车型：model3(Tesla), a2/etron/tt(Audi), grandtourer/i8/mini(BMW), impala(Chevrolet), c3(Citroen), charger_police/charger2020(Dodge), mustang/crown(Ford), wrangler_rubicon(Jeep), mkz_2017/mkz_2020(Lincoln), benz_coupe/cabrio/ccc(Mercedes), cooper_s(Mini), micra/patrol(Nissan), leon(Seat), t2/t3(Volkswagen)
7. spawn_pedestrian - 生成行人，支持参数：query(类型), count(数量), speed(速度)。支持类型：pedestrian(普通行人), elderly(老年人), child(儿童), police(警察), business(商务人士), jogger(慢跑者)。速度默认值：普通行人1.4m/s，老年人1.0m/s，慢跑者2.8m/s
8. setup_autopilot - 设置车辆自动驾驶，支持参数：enable(是否启用), radius(范围半径)
9. setup_pedestrian_movement - 设置行人自动移动，支持参数：enable(是否启用), radius(范围半径)
10. set_weather - 设置天气（clear/rain/fog）
11. get_traffic_lights - 查看交通灯状态
12. cleanup_scene - 清理仿真场景
13. switch_view - 切换视角模式，支持 third_person(第三人称跟随), first_person(第一人称), overhead(俯视/鸟瞰), free(自由视角), bystander(旁观者视角)
14. start_recording - 开始视频录制，录制当前窗口视角的内容
15. stop_recording - 停止视频录制
16. generate_openscenario - 基于已有的 OpenDRIVE 文件生成 OpenSCENARIO 场景文件。当用户提到"场景"、"OpenSCENARIO"、"生成场景"、"仿真场景"时使用。参数：xodr_filename(OpenDRIVE文件名), scenario_name(场景名称), duration(仿真时长秒), vehicle_speed(车辆速度m/s)
17. scenario_highway_ramp - 高速-进出匝道场景，参数：ramp_type("on"匝道汇入/"off"驶出匝道), vehicle_count(总车辆数，默认4), map_name(可选，推荐Town04)。当用户提到"匝道"、"高速进出匝道"时使用
18. scenario_lane_merge - 城市-车道合并场景，参数：vehicle_count(默认4), map_name(可选)。当用户提到"车道合并"、"车道减少"、"汇流"时使用
19. scenario_diverge_merge - 城市-分合流路口场景，参数：vehicle_count(默认4), map_name(可选)。当用户提到"分合流路口"、"分流"时使用
20. scenario_side_road - 城市-辅路场景，参数：vehicle_count(默认4), map_name(可选)。当用户提到"辅路"、"辅道"时使用
21. scenario_junction_light - 城市-路口（十字/T型/Y型）及红绿灯场景，参数：junction_shape(any/cross/t/y), vehicle_count(默认4), map_name(可选)
22. scenario_tunnel - 隧道场景，参数：vehicle_count(默认4), map_name(可选，推荐Town04/Town05)
23. scenario_roundabout - 环岛场景，参数：vehicle_count(默认5), map_name(可选，推荐Town05)
24. spawn_pedestrian_pose - 儿童/成人姿态场景（站立/行走/蹲下/躺下/打伞，后三者为硬摆姿态），参数：pedestrian_type(child/pedestrian/elderly/police), pose(stand/walk/crouch/lie/umbrella), count(默认1)
25. scenario_two_wheeler - 自行车/摩托车-站立/行进/倒地场景，参数：vehicle_type(bicycle/motorcycle), state(stand/move/fallen), count(默认2)
26. spawn_special_vehicle - 特殊任务车辆（救护车/警车），参数：vehicle_type(ambulance/police), moving(默认true), count(默认1)。自动配送物流车无蓝图
27. scenario_officer - 特殊群体场景（交警/轮椅/婴儿车），参数：element(traffic_police/wheelchair/stroller), with_companion(默认false)。婴儿车无蓝图
28. set_lighting - 弱光条件设置（dawn清晨/dusk黄昏/overcast阴天/night夜晚），参数：condition
29. scenario_backlight - 逆光场景（低角度太阳+对向车开大灯），参数：map_name(可选)
30. spawn_rollover_vehicle - 翻车车辆（侧翻side/仰翻upside，car/van货车近似），参数：vehicle_type, rollover, map_name(可选)
31. scenario_lead_vehicle - 前车急刹(brake)/静止(stationary)场景，参数：mode, distance(默认25), map_name(可选)
32. scenario_cut_in - 危险切入场景，参数：direction(left/right), map_name(可选)
33. scenario_lead_disappear - 前车消失场景（前车切出露出障碍），参数：map_name(可选)
34. scenario_crossing_hazard - 路口危险横穿（pedestrian/vehicle/bicycle），参数：crosser, map_name(可选)
35. scenario_low_overlap - 前方低重叠率行驶目标，参数：offset_ratio(默认0.35), map_name(可选)
36. scenario_wrong_way - 逆行场景，参数：speed(默认8), map_name(可选)
37. scenario_unprotected_turn - 路口无保护通行场景，参数：map_name(可选)


CARLA相关：
- 当用户提到"连接"、"服务器"、"CARLA"等明确要求连接时，使用connect_carla
- 当用户提到"车辆"、"生成"、"创建汽车"、"车"等，使用spawn_vehicle，count参数默认为1
- 当用户提到"多辆车"、"生成X辆车"、"几辆车"、指定数量（如5辆、10辆），必须设置count参数为对应数字
- 车辆类型支持中文：特斯拉(model3)、奥迪(a2/etron/tt)、宝马(grandtourer/i8/mini)、雪佛兰(impala)、雪铁龙(c3)、道奇(charger_police/charger2020)、福特(mustang/crown)、吉普(wrangler_rubicon)、林肯(mkz_2017/mkz_2020)、奔驰(benz_coupe/cabrio/ccc)、迷你(cooper_s)、日产(micra/patrol)、西雅特(leon)、大众(t2/t3)
- 当用户使用中文车辆类型（如"生成3辆特斯拉"），你需要将中文类型转换为对应的英文类型：model3、a2、etron、tt、grandtourer、i8、mini、impala、c3、charger_police、charger2020、mustang、crown、wrangler_rubicon、mkz_2017、mkz_2020、benz_coupe、cabrio、ccc、cooper_s、micra、patrol、leon、t2、t3
- 当用户提到"自行车"、"单车"、"生成自行车"、"来辆自行车"等，使用spawn_bicycle，count参数默认为1
- 自行车类型支持中文：山地自行车/crossbike(默认)、公路自行车/century、荷兰自行车/omafiets
- 当用户使用中文自行车类型（如"生成3辆山地自行车"），你需要将中文类型转换为对应的英文类型：crossbike、century、omafiets
- 示例指令：
  - "生成一辆自行车" -> spawn_bicycle(query="crossbike", count=1)
  - "生成5辆山地自行车" -> spawn_bicycle(query="crossbike", count=5)
  - "来3辆公路自行车" -> spawn_bicycle(query="century", count=3)
- 当用户说"生成一辆摩托车"没有指定类型时，默认使用 ninja（川崎忍者）
- 当用户说"生成一辆仰翻的车辆"没有指定类型时，默认使用 model3（特斯拉）
- 当用户说"生成道具"没有指定类型时，默认使用 cone（施工锥）
- 当用户提到"摩托车"、"生成摩托车"、"来辆摩托车"等，使用spawn_motorcycle，count参数默认为1
- 摩托车类型支持中文：川崎忍者/ninja(默认)、雅马哈YZF/yzf、哈雷low_rider
- 示例指令：
  - "生成一辆摩托车" -> spawn_motorcycle(query="ninja", count=1)
  - "生成3辆雅马哈" -> spawn_motorcycle(query="yzf", count=3)

- 当用户提到"施工锥"、"路障"、"警示牌"、"三角警示牌"、"生成道具"等，使用spawn_prop，count参数默认为1
- 道具类型支持中文：施工锥/cone(默认)、路障/barrier、警示牌/warning
- 示例指令：
  - "生成3个施工锥" -> spawn_prop(query="cone", count=3)
  - "来5个三角警示牌" -> spawn_prop(query="warning", count=5)
  - "生成路障" -> spawn_prop(query="barrier", count=1)
- 当用户说"在仰翻车辆后方放警示牌"、"在事故车后面放锥桶"时，需要：
  1. 先确认仰翻车辆的ID（如果刚生成，ID会在返回结果中）
  2. 使用 spawn_prop 并传入 target_id=仰翻车辆ID
- 示例指令：
  - "在ID 32的后方放3个施工锥" -> spawn_prop(query="cone", count=3, target_id=32)
  - "在仰翻车辆后面放三角警示牌" -> 先问用户仰翻车辆的ID，或如果刚生成则直接用该ID
  - "给事故车后方放路障" -> spawn_prop(query="barrier", count=2, target_id=事故车ID)

- 当用户提到"仰翻"、"侧翻"、"翻车"、"事故车"、"翻了的特斯拉"等，使用spawn_overturned_vehicle
- 示例指令：
  - "生成一辆仰翻的特斯拉" -> spawn_overturned_vehicle(vehicle_type="model3")
  - "来一辆侧翻的野马" -> spawn_overturned_vehicle(vehicle_type="mustang")

- 当用户提到"薄雾"、"轻雾"、"雾天（轻）"等，使用set_weather，weather_type设为"light_fog"
- 示例指令：
  - "设置薄雾天气" -> set_weather(weather_type="light_fog")
- 当用户提到"行人"、"生成行人"、"创建行人"、"人"等，直接使用spawn_pedestrian，count参数默认为1
- 当用户提到"多个行人"、"生成X个行人"、"几个行人"、指定数量（如5个、10个），必须设置count参数为对应数字
- 行人类型支持中文：普通行人/行人/人、老年人/老人、儿童/小孩/孩子、警察/警官、商务人士/商人/白领、慢跑者/跑步者/跑步的人
- 当用户使用中文行人类型（如"生成5个老年人"），你需要将中文类型转换为对应的英文类型：elderly、child、police、business、jogger、pedestrian
- 当用户提到"自动驾驶"、"车辆运行"、"车自己开"等，使用setup_autopilot
- 当用户提到"行人移动"、"行人走路"、"行人运行"等，使用setup_pedestrian_movement
- 当用户提到"天气"、"下雨"、"晴天"、"雾天"等，使用set_weather
- 当用户提到"交通灯"、"信号灯"、"红绿灯"等，使用get_traffic_lights
- 当用户提到"清理"、"重置"、"清除场景"等，使用cleanup_scene
- 当用户提到"视角"、"切换视角"、"第三人称"、"第一人称"、"俯视"、"鸟瞰"、"自由视角"、"旁观者"等，使用switch_view
  * third_person: 第三人称跟随视角，相机在目标后方跟随
  * first_person: 第一人称视角，模拟驾驶员或行人视角
  * overhead: 俯视/鸟瞰视角，从上方俯瞰场景
  * free: 自由视角/观察者视角，可以自由观察
  * bystander: 旁观者视角，回到默认观察者位置，不跟随任何目标
- 当用户提到"录制"、"录像"、"视频"、"开始录制"、"录屏"等，使用start_recording
  * 录制的是当前窗口视角的内容，与当前看到的画面一致
  * 录制过程中可以自由切换视角，录制不会中断
  * 可以指定帧率，默认30fps
- 当用户提到"停止录制"、"结束录像"、"保存视频"等，使用stop_recording
- 当用户提到"切换到第三人称视角"、"切换到第一人称"等，但没有指定目标ID时：
  * 如果只有一个行人/车辆，系统会自动选择它
  * 如果有多个行人/车辆，系统会询问用户选择哪个目标
  * 用户可以回复"切换到第三人称视角 ID xxx"来指定目标

重要规则：
- 如果用户已经连接过CARLA服务器，不要再重复调用connect_carla
- 当用户明确要求生成行人或车辆时，直接调用对应的生成工具，不要先调用connect_carla
- 只有当用户明确要求连接服务器时，才调用connect_carla

通用策略：
- 首先判断用户意图是GitHub相关还是CARLA仿真相关
- 搜索时使用英文关键词效果更好
- 必须先连接CARLA服务器才能使用CARLA相关功能
- 不要自动连接CARLA服务器，只在用户明确要求时连接
- 可以根据用户需求调用多个工具获得更全面的结果
- 必须先获取数据，再基于实际数据回答用户问题
- 如果没有找到结果，要明确告知用户

用户指令示例：
- "连接carla服务器" -> connect_carla(host="localhost", port=2000)
- "生成一辆model3" -> spawn_vehicle(query="model3", count=1)
- "生成5辆mustang" -> spawn_vehicle(query="mustang", count=5)
- "生成10辆车" -> spawn_vehicle(query="model3", count=10)
- "给我来3辆奥迪a2" -> spawn_vehicle(query="a2", count=3)
- "创建20辆车" -> spawn_vehicle(query="model3", count=20)
- "生成3辆特斯拉" -> spawn_vehicle(query="model3", count=3)
- "生成5辆宝马" -> spawn_vehicle(query="grandtourer", count=5)
- "生成2辆奔驰" -> spawn_vehicle(query="benz_coupe", count=2)
- "生成4辆福特野马" -> spawn_vehicle(query="mustang", count=4)
- "生成一个行人" -> spawn_pedestrian(query="pedestrian", count=1)
- "生成5个行人" -> spawn_pedestrian(query="pedestrian", count=5)
- "生成3个老年人" -> spawn_pedestrian(query="elderly", count=3)
- "生成10个警察" -> spawn_pedestrian(query="police", count=10)
- "生成一个人" -> spawn_pedestrian(query="pedestrian", count=1)
- "生成5个人" -> spawn_pedestrian(query="pedestrian", count=5)
- "生成3个小孩" -> spawn_pedestrian(query="child", count=3)
- "生成2个商务人士" -> spawn_pedestrian(query="business", count=2)
- "生成4个慢跑者" -> spawn_pedestrian(query="jogger", count=4)
- "生成3个慢跑者，速度3.0" -> spawn_pedestrian(query="jogger", count=3, speed=3.0)
- "开启车辆自动驾驶" -> setup_autopilot(enable=True, radius=0.0)
- "让车辆自己开" -> setup_autopilot(enable=True)
- "开启行人移动" -> setup_pedestrian_movement(enable=True, radius=0.0)
- "让行人走路" -> setup_pedestrian_movement(enable=True)
- "设置雨天" -> set_weather(weather_type="rain")
- "查看交通灯" -> get_traffic_lights()
- "清理场景" -> cleanup_scene()
- "切换到第三人称视角" -> switch_view(view_mode="third_person")
- "切换到第三人称视角 ID 123" -> switch_view(view_mode="third_person", target_actor_id=123)
- "切换到第一人称" -> switch_view(view_mode="first_person")
- "切换到第一人称 ID 456" -> switch_view(view_mode="first_person", target_actor_id=456)
- "切换到俯视视角" -> switch_view(view_mode="overhead")
- "切换到自由视角" -> switch_view(view_mode="free")
- "切换到旁观者视角" -> switch_view(view_mode="bystander")
- "回到默认视角" -> switch_view(view_mode="bystander")
- "开始录制视频" -> start_recording()
- "开始录制60fps视频" -> start_recording(fps=60)
- "停止录制" -> stop_recording()
- "结束录像" -> stop_recording()

重要提示：
- 当用户明确要求生成多辆车时（如"生成5辆车"、"来10辆车"），必须在spawn_vehicle的arguments中包含count参数
- 当用户明确要求生成多个行人时（如"生成5个行人"、"来10个行人"），必须在spawn_pedestrian的arguments中包含count参数
- count参数必须是整数，表示要生成的车辆或行人数量
- 如果不指定count，默认为1
- 当用户要求生成行人时，直接调用spawn_pedestrian，不要先调用connect_carla
- 当用户要求生成车辆时，直接调用spawn_vehicle，不要先调用connect_carla
- 只有当用户明确要求连接服务器时，才调用connect_carla

本助手基于FastMCP框架构建，提供高效、类型安全的工具调用体验。
"""
    },
    {"role": "user", "content": user_message}
]


        # 第一次API调用
        app_logger.info(f"💬 用户消息: {user_message}")
        response = await self.call_deepseek_with_tools(messages)
        assistant_message = response["choices"][0]["message"]

        # 检查是否有工具调用
        tool_calls = assistant_message.get("tool_calls", [])
        messages.append(assistant_message)

        # 执行FastMCP工具调用
        if tool_calls:
            app_logger.info(f"🔧 检测到 {len(tool_calls)} 个FastMCP工具调用")

            for tool_call in tool_calls:
                app_logger.info(f"🔨 执行FastMCP工具: {tool_call['function']['name']}")
                tool_result = await self.execute_fastmcp_tool_call(tool_call)
                app_logger.info(f"✅ FastMCP工具执行完成，结果长度: {len(str(tool_result))}")

                # 添加工具结果到消息历史
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call["id"],
                    "content": json.dumps(tool_result, ensure_ascii=False)
                })

            # 再次调用API获取最终回答
            app_logger.info("🤖 正在生成最终回答...")
            try:
                final_response = await self.call_deepseek_with_tools(messages)
                final_message = final_response["choices"][0]["message"]["content"]
                app_logger.info(f"✅ 最终回答生成成功，长度: {len(final_message)}")

                if not final_message or final_message.strip() == "":
                    app_logger.info("❌ 警告：最终回答为空")
                    final_message = "抱歉，我无法生成回答。请稍后重试。"

                return {
                    "message": self.process_markdown(final_message),
                    "tool_calls": tool_calls,
                    "conversation": messages
                }
            except Exception as e:
                app_logger.error(f"❌ 生成最终回答时出错: {str(e)}")
                return {
                    "message": f"FastMCP工具调用成功，但生成最终回答时出错: {str(e)}",
                    "tool_calls": tool_calls,
                    "conversation": messages
                }
        else:
            return {
                "message": self.process_markdown(assistant_message["content"]),
                "tool_calls": None,
                "conversation": messages
            }


# ============ FastAPI Web界面（AI对话版） ============

app = FastAPI(title="FastMCP GitHub Assistant")

# ===== 新增：文件下载接口 =====
from fastapi.responses import FileResponse
from pathlib import Path

@app.get("/download/{filename}")
async def download_file(filename: str):
    """下载生成的文件"""
    file_path = Path(__file__).parent / "output" / filename
    if file_path.exists():
        return FileResponse(
            path=file_path,
            filename=filename,
            media_type="application/octet-stream"
        )
    return {"error": "文件不存在"}
# ===== 新增结束 =====

def get_web_interface():
    """生成AI对话Web界面HTML"""
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>FastMCP GitHub Assistant - AI智能助手</title>
        <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
        <style>
            * { 
                margin: 0; 
                padding: 0; 
                box-sizing: border-box; 
            }

            body {
                font-family: 'Segoe UI', 'Microsoft YaHei', Tahoma, Geneva, Verdana, sans-serif;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                min-height: 100vh;
                line-height: 1.6;
            }

            .container {
                max-width: 900px;
                margin: 0 auto;
                padding: 20px;
                min-height: 100vh;
                display: flex;
                flex-direction: column;
            }

            .header {
                background: rgba(255, 255, 255, 0.95);
                backdrop-filter: blur(10px);
                padding: 12px 20px;
                border-radius: 15px;
                text-align: center;
                margin-bottom: 15px;
                box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }

            .header h1 {
                color: #2d3748;
                font-size: 1.5em;
                margin: 0;
                font-weight: 700;
                background: linear-gradient(135deg, #667eea, #764ba2);
                -webkit-background-clip: text;
                -webkit-text-fill-color: transparent;
                background-clip: text;
            }

            .chat-container {
                background: rgba(255, 255, 255, 0.95);
                backdrop-filter: blur(10px);
                border-radius: 20px;
                padding: 20px;
                flex: 1;
                display: flex;
                flex-direction: column;
                box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }

            .messages {
                order: 1;
                flex: 1;
                overflow-y: auto;
                overflow-x: hidden;
                padding: 15px;
                margin-bottom: 15px;
                background: rgba(248, 250, 252, 0.5);
                border-radius: 15px;
                border: 1px solid rgba(226, 232, 240, 0.5);
                height: calc(100vh - 280px);
                min-height: 400px;
                max-height: calc(100vh - 280px);
                scroll-behavior: smooth;
            }

            .message {
                margin-bottom: 15px;
                padding: 15px 20px;
                border-radius: 15px;
                max-width: 85%;
                word-wrap: break-word;
                position: relative;
                animation: messageSlide 0.3s ease-out;
            }

            @keyframes messageSlide {
                from {
                    opacity: 0;
                    transform: translateY(10px);
                }
                to {
                    opacity: 1;
                    transform: translateY(0);
                }
            }

            .user-message {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                margin-left: auto;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
                border-bottom-right-radius: 5px;
            }

            .assistant-message {
                background: linear-gradient(135deg, #f8fafc, #e2e8f0);
                color: #2d3748;
                margin-right: auto;
                border-left: 4px solid #667eea;
                box-shadow: 0 4px 15px rgba(0, 0, 0, 0.05);
                border-bottom-left-radius: 5px;
            }

            .tools-used {
                background: rgba(102, 126, 234, 0.05);
                margin-top: 10px;
                border-radius: 10px;
                font-size: 0.9em;
                border: 1px solid rgba(102, 126, 234, 0.2);
                overflow: hidden;
            }

            .tools-header {
                background: rgba(102, 126, 234, 0.1);
                padding: 10px 12px;
                cursor: pointer;
                display: flex;
                align-items: center;
                justify-content: space-between;
                font-weight: 600;
                color: #667eea;
                transition: all 0.3s ease;
            }

            .tools-header:hover {
                background: rgba(102, 126, 234, 0.15);
            }

            .tools-toggle {
                font-size: 0.9em;
                transition: all 0.3s ease;
                font-weight: bold;
            }

            .tools-content {
                padding: 12px;
                display: none;
                border-top: 1px solid rgba(102, 126, 234, 0.1);
            }

            .tools-content.show {
                display: block;
            }

            .input-form {
                order: 2;              /* ← 新增：输入框永远排在消息区下面 */
                position: sticky;      /* ← 新增：吸附在可视区域底部 */
                bottom: 0;             /* ← 新增 */
                z-index: 10;           /* ← 新增：不被消息盖住 */
                display: flex;
                gap: 12px;
                align-items: flex-end;
                background: linear-gradient(135deg, rgba(255, 255, 255, 0.95), rgba(248, 250, 252, 0.9));
                padding: 15px;
                border-radius: 15px;
                border: 1px solid rgba(102, 126, 234, 0.2);
                box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
                backdrop-filter: blur(10px);
            }

            .message-input {
                flex: 1;
                padding: 12px 16px;
                border: 2px solid transparent;
                border-radius: 12px;
                background: white;
                font-size: 0.95em;
                resize: none;
                min-height: 44px;
                max-height: 120px;
                transition: all 0.3s ease;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
                font-family: inherit;
                line-height: 1.4;
            }

            .message-input:focus {
                outline: none;
                border-color: #667eea;
                box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.15), 0 4px 15px rgba(0, 0, 0, 0.15);
                transform: translateY(-1px);
            }

            .message-input::placeholder {
                color: #9ca3af;
                font-style: italic;
            }

            .send-button {
                width: 44px;
                height: 44px;
                background: linear-gradient(135deg, #667eea, #764ba2);
                border: none;
                border-radius: 50%;
                cursor: pointer;
                transition: all 0.3s ease;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
                display: flex;
                align-items: center;
                justify-content: center;
                flex-shrink: 0;
                position: relative;
            }

            .send-button i {
                color: white;
                font-size: 16px;
            }

            .send-button:hover:not(:disabled) {
                transform: translateY(-2px);
                box-shadow: 0 6px 25px rgba(102, 126, 234, 0.4);
                background: linear-gradient(135deg, #5a67d8, #6b46c1);
            }

            .send-button:active:not(:disabled) {
                transform: translateY(0px);
                box-shadow: 0 2px 10px rgba(102, 126, 234, 0.3);
            }

            .send-button:disabled {
                opacity: 0.5;
                cursor: not-allowed;
                transform: none;
                box-shadow: 0 2px 8px rgba(102, 126, 234, 0.2);
                background: linear-gradient(135deg, #9ca3af, #6b7280);
            }

            .loading {
                display: none;
                text-align: center;
                padding: 25px;
                margin: 15px 0;
                background: linear-gradient(135deg, rgba(102, 126, 234, 0.1), rgba(118, 75, 162, 0.1));
                border-radius: 15px;
                border: 1px solid rgba(102, 126, 234, 0.2);
            }

            .loading.show { 
                display: block; 
            }

            .loading-content {
                display: flex;
                flex-direction: column;
                align-items: center;
                gap: 15px;
            }

            .loading-text {
                color: #667eea;
                font-weight: 600;
                font-size: 1.2em;
                display: flex;
                align-items: center;
                gap: 12px;
            }

            .loading-spinner {
                width: 24px;
                height: 24px;
                border: 3px solid rgba(102, 126, 234, 0.2);
                border-top: 3px solid #667eea;
                border-radius: 50%;
                animation: spin 1s linear infinite;
            }

            @keyframes spin {
                from { transform: rotate(0deg); }
                to { transform: rotate(360deg); }
            }

            .example-questions {
                background: linear-gradient(135deg, rgba(248, 250, 252, 0.8), rgba(241, 245, 249, 0.8));
                border-radius: 15px;
                padding: 20px;
                margin-bottom: 15px;
                border: 1px solid rgba(226, 232, 240, 0.5);
                backdrop-filter: blur(5px);
            }

            .welcome-message {
                color: #4a5568;
                margin-bottom: 15px;
                font-size: 1em;
                line-height: 1.5;
                text-align: center;
                padding: 15px;
                background: rgba(255, 255, 255, 0.6);
                border-radius: 12px;
                border-left: 4px solid #667eea;
            }

            .example-questions h3 {
                color: #2d3748;
                margin-bottom: 15px;
                font-size: 1em;
                text-align: center;
                font-weight: 600;
            }

            .examples-grid {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 12px;
            }

            .example-item {
                background: linear-gradient(135deg, rgba(255, 255, 255, 0.9), rgba(248, 250, 252, 0.9));
                border-radius: 10px;
                padding: 12px 16px;
                cursor: pointer;
                transition: all 0.3s ease;
                border-left: 3px solid #667eea;
                font-size: 0.9em;
                box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
                border: 1px solid rgba(226, 232, 240, 0.3);
                text-align: center;
            }

            .example-item:hover {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                transform: translateY(-2px) scale(1.02);
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
            }

            .assistant-message h1 {
                font-size: 1.4em;
                color: #2d3748;
                margin: 15px 0 10px 0;
                font-weight: 700;
            }

            .assistant-message h2 {
                font-size: 1.2em;
                color: #2d3748;
                margin: 12px 0 8px 0;
                font-weight: 600;
            }

            .assistant-message h3 {
                font-size: 1.1em;
                color: #2d3748;
                margin: 10px 0 6px 0;
                font-weight: 600;
            }

            /* 响应式设计 */
            @media (max-width: 768px) {
                .container {
                    padding: 10px;
                }

                .header h1 {
                    font-size: 1.5em;
                }

                .message {
                    max-width: 95%;
                    padding: 12px 15px;
                }

                .examples-grid {
                    grid-template-columns: 1fr;
                    gap: 8px;
                }

                .input-form {
                    flex-direction: column;
                    gap: 12px;
                    padding: 12px;
                }

                .message-input {
                    min-height: 40px;
                }

                .send-button {
                    width: 100%;
                    height: 44px;
                }

                .messages {
                    height: calc(100vh - 320px);
                }
            }

            /* 滚动条美化 */
            .messages::-webkit-scrollbar {
                width: 6px;
            }

            .messages::-webkit-scrollbar-track {
                background: rgba(226, 232, 240, 0.3);
                border-radius: 3px;
            }

            .messages::-webkit-scrollbar-thumb {
                background: linear-gradient(135deg, #667eea, #764ba2);
                border-radius: 3px;
            }

            .messages::-webkit-scrollbar-thumb:hover {
                background: linear-gradient(135deg, #5a67d8, #6b46c1);
            }

            /* ===== 周计划场景快捷按钮面板 ===== */
            .quick-panel {
                margin-top: 15px;
                border-top: 2px dashed #3b82f6;
                padding-top: 10px;
            }
            .quick-title {
                color: #3b82f6;
                font-size: 1.05em;
                font-weight: 700;
                margin-bottom: 8px;
            }
            .week-group { margin-bottom: 10px; }
            .week-label {
                font-size: 0.85em;
                font-weight: 600;
                color: #64748b;
                margin-bottom: 5px;
            }
            .week-label .w1 { color: #10b981; }
            .quick-btns {
                display: flex;
                flex-wrap: wrap;
                gap: 6px;
            }
            .qbtn {
                border: none;
                border-radius: 8px;
                padding: 7px 12px;
                font-size: 0.85em;
                cursor: pointer;
                color: #fff;
                background: linear-gradient(135deg, #3b82f6, #6366f1);
                box-shadow: 0 2px 8px rgba(59, 130, 246, 0.3);
                transition: transform 0.15s ease, box-shadow 0.15s ease;
            }
            .qbtn:hover:not(:disabled) {
                transform: translateY(-2px);
                box-shadow: 0 4px 12px rgba(59, 130, 246, 0.45);
            }
            .qbtn:disabled {
                background: #cbd5e1;
                color: #64748b;
                cursor: not-allowed;
                box-shadow: none;
            }
            .qbtn.running {
                background: linear-gradient(135deg, #f59e0b, #d97706);
            }
            .qbtn-w1 { background: linear-gradient(135deg, #10b981, #059669); box-shadow: 0 2px 8px rgba(16, 185, 129, 0.3); }
            .qbtn-util { background: linear-gradient(135deg, #64748b, #475569); }
            .quick-note { font-size: 0.8em; color: #94a3b8; margin-top: 4px; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🚀 HUTB 模拟器智能助手</h1>
            </div>

            <div class="chat-container">
                <div class="messages" id="messages">
                    <div class="example-questions">
                         <div class="welcome-message">
                            👋 欢迎使用基于FastMCP框架的 HUTB 模拟器智能助手！集成 HUTB 仿真控制。
                            <br><br>
                            🔧 <strong>技术特色</strong>：本助手使用FastMCP装饰器实现工具定义，提供类型安全、自动化的MCP体验！
                        </div>
                        <h3>💡 试试这些问题：</h3>
                        <div class="examples-grid">
                            <div class="example-item" onclick="askExample('连接CARLA仿真服务器')">
                            🔗 连接服务器
                            </div>
                            <div class="example-item" onclick="askExample('设置雨天天气条件')">
                                🌫️ 天气设置（默认雨天）
                            </div>
                            <div class="example-item" onclick="askExample('生成行人')">
                                🚶 生成行人
                            </div>
                            <div class="example-item" onclick="askExample('生成 model3 车辆')">
                                🚗 生成车辆
                            </div>
                        </div>

                        <!-- SUMO 功能（新增） -->
<div style="margin-top: 15px; border-top: 2px dashed #ff6b35; padding-top: 10px;">
    <h3 style="color: #ff6b35;">🚦 SUMO 交通仿真（新增功能）</h3>
    <div class="examples-grid">
        <!-- 原有的路网生成按钮 -->
        <div class="example-item" style="border-left-color: #ff6b35;" onclick="askExample('生成一个3x3网格路网，跑200秒，每2秒发一辆车')">
            🚦 生成默认网格路网
        </div>
        <!-- 新增 OpenSCENARIO 生成按钮 -->
        <div class="example-item" style="border-left-color: #ff6b35;" onclick="askExample('生成一个场景，基于 web_generated.xodr，车以10m/s行驶30秒')">
            🎬 生成 OpenSCENARIO 场景
        </div>
    </div>
    <div style="margin-top: 8px; font-size: 0.85em; color: #666; text-align: center;">
        💡 也支持自然语言自定义参数：<em>"生成4x4网格路网，跑300秒"</em> 或 <em>"生成5x5网格路网，跑500秒，每3秒发一辆车"</em>
    </div>
</div>
<div style="margin-top: 15px; border-top: 2px dashed #10b981; padding-top: 10px;">
        <h3 style="color: #10b981;">🎬 场景与道具（新增功能）</h3>
        <div class="examples-grid">
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆山地自行车')">🚲 生成自行车</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆摩托车')">🏍️ 生成摩托车</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成3个施工锥')">🚧 生成道具</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆仰翻的车辆')">🚓💥 仰翻汽车</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('设置薄雾天气')">🌫️ 薄雾天气</div>
            <div class="example-item" style="border-left-color: #10b981;" onclick="askExample('生成一辆警车')">🚓 生成警车</div>
        </div>
    </div>

    <!-- ===== 周计划场景快捷按钮 ===== -->
    <div class="quick-panel">
        <div class="quick-title">⚡ 场景快捷按钮（点击直接执行，无需输入）</div>

        <div class="week-group">
            <div class="week-label"><span class="w1">🛣️ 道路结构场景</span></div>
            <div class="quick-btns">
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_highway_ramp', {ramp_type: 'on'}, this)">🛣️ 匝道汇入</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_highway_ramp', {ramp_type: 'off'}, this)">🛣️ 匝道驶出</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_lane_merge', {}, this)">🔀 车道合并</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_diverge_merge', {}, this)">🚥 分合流路口</button>
                <button class="qbtn qbtn-w1" onclick="runQuickTool('scenario_side_road', {}, this)">🛤️ 辅路</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label"><span style="color:#3b82f6;">🚦 路口与特殊形态目标</span></div>
            <div class="quick-btns">
                <button class="qbtn" onclick="runQuickTool('scenario_junction_light', {}, this)">🚦 路口+红绿灯</button>
                <button class="qbtn" onclick="runQuickTool('scenario_tunnel', {}, this)">🚇 隧道</button>
                <button class="qbtn" onclick="runQuickTool('scenario_roundabout', {}, this)">🔄 环岛</button>
                <button class="qbtn" onclick="runQuickTool('spawn_pedestrian_pose', {pedestrian_type: 'child', pose: 'walk'}, this)">🧒 儿童姿态</button>
                <button class="qbtn" onclick="runQuickTool('scenario_two_wheeler', {vehicle_type: 'bicycle', state: 'move'}, this)">🚲 二轮车姿态</button>
                <button class="qbtn" onclick="runQuickTool('spawn_special_vehicle', {vehicle_type: 'ambulance'}, this)">🚑 特殊任务车辆</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label"><span style="color:#8b5cf6;">👮 特殊群体、光照与危险行为</span></div>
            <div class="quick-btns">
                <button class="qbtn" onclick="runQuickTool('scenario_officer', {element: 'traffic_police'}, this)">👮 交警/轮椅</button>
                <button class="qbtn" onclick="runQuickTool('set_lighting', {condition: 'dusk'}, this)">🌙 弱光</button>
                <button class="qbtn" onclick="runQuickTool('scenario_backlight', {}, this)">☀️ 逆光</button>
                <button class="qbtn" onclick="runQuickTool('spawn_rollover_vehicle', {vehicle_type: 'car', rollover: 'side'}, this)">🚚 侧翻车辆</button>
                <button class="qbtn" onclick="runQuickTool('scenario_lead_vehicle', {mode: 'brake'}, this)">🛑 前车急刹/静止</button>
                <button class="qbtn" onclick="runQuickTool('scenario_cut_in', {}, this)">⚠️ 危险切入</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label"><span style="color:#f59e0b;">⚠️ 复杂危险场景</span></div>
            <div class="quick-btns">
                <button class="qbtn" onclick="runQuickTool('scenario_lead_disappear', {}, this)">👻 前车消失</button>
                <button class="qbtn" onclick="runQuickTool('scenario_crossing_hazard', {crosser: 'pedestrian'}, this)">🏃 路口危险横穿</button>
                <button class="qbtn" onclick="runQuickTool('scenario_low_overlap', {}, this)">📐 低重叠率目标</button>
                <button class="qbtn" onclick="runQuickTool('scenario_wrong_way', {}, this)">↩️ 逆行</button>
                <button class="qbtn" onclick="runQuickTool('scenario_unprotected_turn', {}, this)">🚗 无保护转弯</button>
            </div>
        </div>

        <div class="week-group">
            <div class="week-label">常用操作</div>
            <div class="quick-btns">
                <button class="qbtn qbtn-util" onclick="runQuickTool('connect_carla', {}, this)">🔗 连接 CARLA</button>
                <button class="qbtn qbtn-util" onclick="runQuickTool('cleanup_scene', {}, this)">🧹 清理场景</button>
            </div>
        </div>
        <div class="quick-note">💡 场景按钮基于当前地图自动选址，推荐 Town04（高速匝道/隧道/环岛均可）；Town03 地图不可用。蹲下/躺下/打伞为硬摆姿态、货车用Sprinter近似等限制会在执行结果中说明。</div>
    </div>

</div> 
                </div>
                <div class="loading" id="loading">
                    <div class="loading-content">
                        <div class="loading-text">
                            <div class="loading-spinner"></div>
                            <span>FastMCP工具调用中...</span>
                        </div>
                    </div>
                </div>

                <form class="input-form" onsubmit="return submitForm(event)">
                    <textarea 
                        id="messageInput" 
                        class="message-input" 
                        placeholder="问我任何 HUTB 模拟器相关问题，我会使用 FastMCP 工具来帮你操作..."
                        rows="2"
                        onkeydown="handleKeyPress(event)"
                    ></textarea>
                    <button type="submit" class="send-button" id="sendButton">
                        <i class="fas fa-paper-plane"></i>
                    </button>
                </form>
            </div>
        </div>

        <script>
    function scrollToBottom() {
        var box = document.getElementById('messages');
        if (box) box.scrollTop = box.scrollHeight;
    }
    window.addEventListener('load', scrollToBottom);
    var observer = new MutationObserver(scrollToBottom);
    var msgBox = document.getElementById('messages');
    if (msgBox) observer.observe(msgBox, { childList: true, subtree: true });
    </script>

<script>
function askExample(text) {
    document.getElementById('messageInput').value = text;
    submitMessage();
}

// ===== 周计划快捷按钮：直接执行FastMCP工具（不走AI） =====
async function runQuickTool(tool, args, btn) {
    if (btn) { btn.disabled = true; btn.classList.add('running'); }
    const argStr = Object.entries(args || {}).map(([k, v]) => `${k}=${v}`).join(', ');
    addMessage(`⚡ 快捷执行：${tool}(${argStr})`, 'user');
    try {
        const form = new FormData();
        form.append('tool', tool);
        form.append('args', JSON.stringify(args || {}));
        const response = await fetch('/tool', { method: 'POST', body: form });
        const result = await response.json();
        const toolCalls = result.success
            ? [{ function: { name: tool, arguments: JSON.stringify(args || {}) } }]
            : null;
        addMessage(result.message || (result.success ? '✅ 执行完成' : '❌ 执行失败'), 'assistant', toolCalls);
    } catch (error) {
        addMessage('❌ 请求失败: ' + error, 'assistant');
    } finally {
        if (btn) { btn.disabled = false; btn.classList.remove('running'); }
    }
}

function handleKeyPress(event) {
    if (event.key === 'Enter' && !event.shiftKey) {
        event.preventDefault();
        submitMessage();
    }
}

function submitForm(event) {
    event.preventDefault();
    submitMessage();
    return false;
}

async function submitMessage() {
    const input = document.getElementById('messageInput');
    const message = input.value.trim();
    if (!message) return;

    addMessage(message, 'user');
    input.value = '';
    showLoading(true);

    try {
        const response = await fetch('/chat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'message=' + encodeURIComponent(message)
        });

        if (response.ok) {
            const result = await response.json();
            addMessage(result.message, 'assistant', result.tool_calls);
        } else {
            addMessage('抱歉，发生了错误，请稍后重试。', 'assistant');
        }
    } catch (error) {
        console.error('Error:', error);
        addMessage('网络连接错误，请检查网络后重试。', 'assistant');
    } finally {
        showLoading(false);
    }
}

function addMessage(content, sender, toolCalls) {
    const messages = document.getElementById('messages');
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}-message`;

    let html = `<div>${content}</div>`;

    if (toolCalls && toolCalls.length > 0) {
        const toolsId = 'tools-' + Date.now();
        html += `
            <div class="tools-used">
                <div class="tools-header" onclick="toggleTools('${toolsId}')">
                    <span>🔧 使用的FastMCP工具 (${toolCalls.length}个)</span>
                    <span class="tools-toggle" id="toggle-${toolsId}">▼</span>
                </div>
                <div class="tools-content" id="${toolsId}">`;

        for (let i = 0; i < toolCalls.length; i++) {
            const tool = toolCalls[i];
            const args = JSON.parse(tool.function.arguments);
            let argStr = '';
            for (const k in args) {
                if (argStr) argStr += ', ';
                argStr += `${k}: "${args[k]}"`;
            }
            html += `<div>• <strong>@mcp.tool() ${tool.function.name}</strong>(${argStr})</div>`;
        }

        html += `
                </div>
            </div>`;
    }

    messageDiv.innerHTML = html;
    messages.appendChild(messageDiv);
    messages.scrollTop = messages.scrollHeight;
}

function toggleTools(toolsId) {
    const content = document.getElementById(toolsId);
    const toggle = document.getElementById('toggle-' + toolsId);

    if (content.classList.contains('show')) {
        content.classList.remove('show');
        toggle.classList.remove('expanded');
        toggle.textContent = '▼';
    } else {
        content.classList.add('show');
        toggle.classList.add('expanded');
        toggle.textContent = '▲';
    }
}

function showLoading(show) {
    const loading = document.getElementById('loading');
    const sendButton = document.getElementById('sendButton');

    if (show) {
        loading.classList.add('show');
        sendButton.disabled = true;
    } else {
        loading.classList.remove('show');
        sendButton.disabled = false;
    }
}
</script>
    </body>
    </html>
    """
    return html_content


@app.get("/", response_class=HTMLResponse)
async def index():
    """主页面 - AI对话界面"""
    return get_web_interface()


@app.post("/chat")
async def chat(message: str = Form(...)):
    """处理聊天请求 - 使用FastMCP工具的AI对话"""
    try:
        result = await assistant.chat(message)
        return {
            "success": True,
            "message": result["message"],
            "tool_calls": result["tool_calls"]
        }
    except Exception as e:
        app_logger.error(f"❌ FastMCP聊天处理失败: {str(e)}")
        return {
            "success": False,
            "message": f"抱歉，处理您的请求时出现错误: {str(e)}",
            "tool_calls": None
        }


@app.post("/tool")
async def run_tool(tool: str = Form(...), args: str = Form("{}")):
    """快捷按钮接口 - 直接执行FastMCP工具（不经过AI），供网页快捷按钮调用"""
    try:
        arguments = json.loads(args) if args else {}
    except Exception:
        arguments = {}
    try:
        # 健康探测：客户端可能持有已失效的连接（如CARLA重启过），先验证再决定是否重连
        if tool != "connect_carla":
            healthy = False
            if carla_client.world is not None and getattr(carla_client, 'client', None) is not None:
                try:
                    carla_client.client.set_timeout(5)
                    carla_client.client.get_world()
                    healthy = True
                except Exception:
                    healthy = False
            if not healthy:
                connected = await carla_client.connect('localhost', 2000)
                if not connected:
                    return {"success": False, "message": "❌ 无法连接到CARLA服务器(localhost:2000)，请先启动仿真器", "tool": tool}

        tool_call = {
            "id": "quick-btn",
            "type": "function",
            "function": {"name": tool, "arguments": json.dumps(arguments)}
        }
        result = await assistant.execute_fastmcp_tool_call(tool_call)
        if result.get("success"):
            data = result.get("data")
            if isinstance(data, str):
                message = data
            else:
                message = "✅ 操作成功" if data else "❌ 操作失败"
            return {"success": True, "message": message, "tool": tool}
        return {"success": False, "message": result.get("error", "执行失败"), "tool": tool}
    except Exception as e:
        app_logger.error(f"❌ 快捷工具执行失败: {str(e)}")
        return {"success": False, "message": f"执行出错: {str(e)}", "tool": tool}


# 创建全局AI助手实例
assistant = FastMCPGitHubAssistant()

def main():
    """主函数 - 支持 Web界面、标准MCP、SSE-MCP 三种启动模式"""
    import sys
    import socket

    # 1. 提取公共逻辑：无论进入哪个模式，都先进行一次环境校验
    if not config.validate():
        print("[ERROR] 配置验证失败，请检查环境变量设置")
        print("[INFO] 请确保 .env 文件包含以下必要配置：")
        print("   - GITHUB_TOKEN=your_github_token")
        print("   - DEEPSEEK_API_KEY=your_deepseek_api_key")
        return
    print("[OK] 环境配置验证通过")

    # 2. 根据命令行参数进行路由分发
    if len(sys.argv) > 1 and sys.argv[1] == "mcp":
        print("[MCP] 启动 FastMCP AI助手 MCP/stdio 服务器...")
        mcp.run()

    elif len(sys.argv) > 1 and sys.argv[1] == "sse":
        print("[MCP] 启动 FastMCP AI助手 SSE 服务端 (OpenClaw专用)...")
        # 监听 0.0.0.0 允许 Docker 跨环境访问，使用 3001 端口与 Web 端物理隔离
        mcp.run(transport="sse", host="0.0.0.0", port=3001)

    else:
        print("[WEB] 启动 FastMCP AI助手 Web 对话界面...")
        host_ip = socket.gethostbyname(socket.gethostname())
        print(f"[INFO] 访问地址: http://{host_ip}:3000")
        uvicorn.run(app='main_ai:app', host=host_ip, port=3000, reload=True)

@mcp.tool()
async def generate_sumo_network(
    grid_x: int = 3,
    grid_y: int = 3,
    duration: int = 200,
    rate: float = 2.0
) -> str:
    """生成 SUMO 路网和车流。"""
    return await generate_sumo_network_impl(grid_x, grid_y, duration, rate)

async def generate_sumo_network_impl(
    grid_x: int = 3,
    grid_y: int = 3,
    duration: int = 200,
    rate: float = 2.0
) -> str:
    """SUMO 路网生成实现函数"""
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        return "❌ 错误：未设置 SUMO_HOME 环境变量。请在终端中设置：`set SUMO_HOME=D:\\mcp\\sumo\\sumo_install\\sumo-win64-1.27.0\\sumo-1.27.0`"

 # 创建输出目录
    output_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(output_dir, exist_ok=True)

    bin_dir = os.path.join(sumo_home, "bin")
    tools_dir = os.path.join(sumo_home, "tools")

    prefix = os.path.join(output_dir, "web_generated")
    net_file = f"{prefix}.net.xml"
    trips_file = f"{prefix}.trips.xml"
    rou_file = f"{prefix}.rou.xml"
    cfg_file = f"{prefix}.sumocfg"
    xodr_file = f"{prefix}.xodr"

    try:
        # 1. 生成路网
        subprocess.run([
            os.path.join(bin_dir, "netgenerate"),
            "--grid",
            f"--grid-x-number={grid_x}",
            f"--grid-y-number={grid_y}",
            f"--grid-x-length=500",
            f"--grid-y-length=500",
            f"--output-file={net_file}"
        ], check=True, capture_output=True, text=True)

        # 2. 生成出行
        subprocess.run([
            "python",
            os.path.join(tools_dir, "randomTrips.py"),
            f"-n={net_file}",
            f"-e={duration}",
            "-l",
            f"-p={rate}",
            f"-o={trips_file}"
        ], check=True, capture_output=True, text=True)

        # 3. 生成路由
        subprocess.run([
            os.path.join(bin_dir, "duarouter"),
            f"-n={net_file}",
            f"-t={trips_file}",
            f"-o={rou_file}",
            "--ignore-errors"
        ], check=True, capture_output=True, text=True)

        # 4. 创建配置文件
        with open(cfg_file, "w", encoding="utf-8") as f:
            f.write(f'''<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{net_file}"/>
        <route-files value="{rou_file}"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{duration}"/>
    </time>
</configuration>
''')

 # 4.5 转换为 OpenDRIVE 格式
        xodr_file = f"{prefix}.xodr"
        xodr_msg = ""
        try:
            subprocess.run([
                os.path.join(bin_dir, "netconvert"),
                f"--sumo-net-file={net_file}",
                f"--opendrive-output={xodr_file}"
            ], check=True, capture_output=True, text=True)
            xodr_msg = f"- OpenDRIVE: {xodr_file}"
        except subprocess.CalledProcessError as e:
            xodr_msg = f"- OpenDRIVE 转换失败：{e.stderr}"

 # 5. 返回结果（修改返回信息，添加 xodr_msg）
        download_url = f"/download/{os.path.basename(xodr_file)}"
        return f"""✅ SUMO 路网和车流生成成功！

📁 生成的文件：
- 路网: {net_file}
- 出行: {trips_file}
- 路由: {rou_file}
- 配置: {cfg_file}
{xodr_msg}

📊 参数：
- 网格: {grid_x}x{grid_y}
- 仿真时长: {duration} 秒
- 发车间隔: {rate} 秒/辆

📥 下载链接：
- [点击下载 OpenDRIVE 文件]({download_url})

▶️ 在终端中运行以下命令查看：
cd D:\\mcp\\sumo
sumo-gui -c {cfg_file}
"""

    except subprocess.CalledProcessError as e:
        return f"❌ 生成失败：{e.stderr}"
    
# ============ OpenSCENARIO 场景生成功能（新增） ============

async def generate_openscenario_impl(
    xodr_filename: str = "",
    scenario_name: str = "my_scenario",
    duration: float = 30.0,
    vehicle_speed: float = 10.0
) -> str:
    """
    生成 OpenSCENARIO 场景文件（支持外部 .xodr 或内建直路）。
    """
    import xml.etree.ElementTree as ET
    import os
    from datetime import datetime

    output_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(output_dir, exist_ok=True)

    try:
        root = ET.Element("OpenSCENARIO", {
            "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
            "xsi:noNamespaceSchemaLocation": "http://www.asam.net/xml/OpenSCENARIO/1.0.0/OpenSCENARIO.xsd"
        })

        header = ET.SubElement(root, "FileHeader", {
            "revMajor": "1",
            "revMinor": "0",
            "date": datetime.now().isoformat(),
            "description": f"Scenario: {scenario_name}",
            "author": "MCP Assistant"
        })

        # 路网
        road_network = ET.SubElement(root, "RoadNetwork")
        if xodr_filename:
            # 使用外部 OpenDRIVE 文件
            xodr_path = os.path.join(output_dir, xodr_filename)
            if not os.path.exists(xodr_path):
                return f"❌ 错误：找不到 OpenDRIVE 文件 {xodr_filename}，请先生成路网。"
            ET.SubElement(road_network, "LogicFile", {"filepath": xodr_path})
        # 否则不添加 LogicFile，CARLA 会使用默认地图

        # 实体
        entities = ET.SubElement(root, "Entities")
        obj = ET.SubElement(entities, "ScenarioObject", {"name": "ego_vehicle"})
        vehicle = ET.SubElement(obj, "Vehicle", {"name": "ego_vehicle", "vehicleCategory": "car"})
        ET.SubElement(vehicle, "Performance", {"maxSpeed": "20", "maxAcceleration": "5", "maxDeceleration": "5"})

        # 故事板
        storyboard = ET.SubElement(root, "Storyboard")

        # 初始化：车辆放在原点
        init = ET.SubElement(storyboard, "Init")
        private = ET.SubElement(init, "Private", {"entityRef": "ego_vehicle"})
        action = ET.SubElement(private, "Action")
        teleport = ET.SubElement(action, "TeleportAction")
        pos = ET.SubElement(teleport, "Position")
        ET.SubElement(pos, "WorldPosition", {"x": "0.0", "y": "0.0", "z": "0.0", "h": "0.0"})

        # 故事：匀速行驶
        story = ET.SubElement(storyboard, "Story", {"name": "drive_story"})
        act = ET.SubElement(story, "Act", {"name": "drive_act"})
        maneuver = ET.SubElement(act, "Maneuver", {"name": "drive_maneuver"})
        event = ET.SubElement(maneuver, "Event", {"name": "speed_event", "priority": "overwrite"})

        action = ET.SubElement(event, "Action")
        speed_action = ET.SubElement(action, "SpeedAction")
        ET.SubElement(speed_action, "SpeedActionDynamics", {"dynamicsShape": "step", "value": "0.0"})
        target = ET.SubElement(speed_action, "SpeedActionTarget")
        ET.SubElement(target, "AbsoluteSpeed", {"value": str(vehicle_speed)})

        # 开始触发
        start = ET.SubElement(event, "StartTrigger")
        cond_group = ET.SubElement(start, "ConditionGroup")
        cond = ET.SubElement(cond_group, "Condition", {"rule": "greaterThan", "edge": "rising"})
        by_val = ET.SubElement(cond, "ByValueCondition")
        ET.SubElement(by_val, "SimulationTimeCondition", {"value": "0.0"})

        # 结束触发
        stop = ET.SubElement(event, "StopTrigger")
        stop_group = ET.SubElement(stop, "ConditionGroup")
        stop_cond = ET.SubElement(stop_group, "Condition", {"rule": "greaterThan", "edge": "rising"})
        stop_by_val = ET.SubElement(stop_cond, "ByValueCondition")
        ET.SubElement(stop_by_val, "SimulationTimeCondition", {"value": str(duration)})

        tree = ET.ElementTree(root)
        output_file = os.path.join(output_dir, f"{scenario_name}.xosc")
        tree.write(output_file, encoding="UTF-8", xml_declaration=True)

        return f"""✅ OpenSCENARIO 场景文件生成成功！

📁 文件路径: {output_file}
📊 文件大小: {os.path.getsize(output_file)} 字节

📥 下载链接：
<a href="/download/{scenario_name}.xosc" target="_blank">点击下载 {scenario_name}.xosc</a>

▶️ 可直接在 CARLA 的 OpenSCENARIO 播放器中运行。
"""
    except Exception as e:
        return f"❌ 生成失败: {str(e)}"
    # 2. 工具函数（有 @mcp.tool() 装饰器）
@mcp.tool()
async def generate_openscenario(
    xodr_filename: str = "web_generated.xodr",
    scenario_name: str = "my_scenario",
    duration: float = 30.0,
    vehicle_speed: float = 10.0
) -> str:
    """生成 OpenSCENARIO 场景文件。"""
    return await generate_openscenario_impl(xodr_filename, scenario_name, duration, vehicle_speed)

# ============ 修复5: SpawnParameters 参数面板类 ============
class SpawnParameters:
    """生成参数面板"""
    def __init__(self):
        self.actor_type = "vehicle"
        self.blueprint_filter = "vehicle.*"
        self.reference_actor_id = None
        self.relative_distance = 10.0
        self.relative_angle = 0.0
        self.lane_type = "Driving"
        self.initial_speed = 0.0
        self.autopilot = False
        self.count = 1

# ============ 修复8: MCP Tool - 控制行人停止/恢复 ============
@mcp.tool()
async def control_walker(action: str, walker_id: int = None) -> str:
    """控制行人停止或恢复移动
    
    Args:
        action: "stop" | "resume" | "stop_all" | "resume_all"
        walker_id: 行人ID（stop/resume需要）
    """
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    if action == "stop":
        if walker_id is None:
            return "❌ stop需要指定walker_id"
        carla_client.stop_walker(walker_id)
        return f"✅ 行人 {walker_id} 已停止"
    elif action == "resume":
        if walker_id is None:
            return "❌ resume需要指定walker_id"
        if carla_client.resume_walker(walker_id):
            return f"✅ 行人 {walker_id} 已恢复移动"
        return f"❌ 行人 {walker_id} 恢复失败"
    elif action == "stop_all":
        count = carla_client.stop_all_walkers()
        return f"✅ 已停止 {count} 个行人"
    elif action == "resume_all":
        count = carla_client.resume_all_walkers()
        return f"✅ 已恢复 {count} 个行人"
    return f"❌ 未知操作: {action}"

# ============ 修复5+6+7: 底层实现函数 ============
async def spawn_vehicle_param_impl(
    count: int = 1,
    blueprint_filter: str = "vehicle.*",
    autopilot: bool = True,
    reference_id: int = None,
    relative_distance: float = 10.0,
    relative_angle: float = 0.0,
    initial_speed: float = 0.0
) -> str:
    """参数化生成车辆的底层实现"""
    try:
        if carla_client.world is None:
            return "❌ 未连接到CARLA服务器"
        
        params = SpawnParameters()
        params.actor_type = "vehicle"
        params.blueprint_filter = blueprint_filter
        params.reference_actor_id = reference_id
        params.relative_distance = relative_distance
        params.relative_angle = relative_angle
        params.autopilot = autopilot
        params.initial_speed = initial_speed
        params.count = count
        params.lane_type = "Driving"
        
        if reference_id is None:
            result = await carla_client.batch_spawn_vehicles_with_id(count, blueprint_filter, autopilot)
            lines = [f"✅ 生成完成：成功{result['success']}辆，失败{result['failed']}辆"]
            for v in result["vehicles"]:
                lines.append(f"  [{v['index']}] ID={v['id']} {v['type_id']} @({v['location']['x']},{v['location']['y']})")
            return "\n".join(lines)
        else:
            spawned = await carla_client.spawn_vehicles_with_params(params)
            return f"✅ 生成完成：成功{len(spawned)}辆，ID={[a.id for a in spawned]}"
    except Exception as e:
        app_logger.error(f"❌ spawn_vehicle_param_impl 异常: {e}")
        return f"❌ 生成失败: {str(e)}"


async def control_walker_impl(action: str, walker_id: int = None) -> str:
    """控制行人停止/恢复的底层实现"""
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器"
    
    if action == "stop":
        if walker_id is None:
            return "❌ stop需要指定walker_id"
        carla_client.stop_walker(walker_id)
        return f"✅ 行人 {walker_id} 已停止"
    elif action == "resume":
        if walker_id is None:
            return "❌ resume需要指定walker_id"
        if carla_client.resume_walker(walker_id):
            return f"✅ 行人 {walker_id} 已恢复移动"
        return f"❌ 行人 {walker_id} 恢复失败"
    elif action == "stop_all":
        count = carla_client.stop_all_walkers()
        return f"✅ 已停止 {count} 个行人"
    elif action == "resume_all":
        count = carla_client.resume_all_walkers()
        return f"✅ 已恢复 {count} 个行人"
    return f"❌ 未知操作: {action}"

# ============ 修复5+6+7: MCP Tool - 参数化生成车辆 ============
@mcp.tool()
async def spawn_vehicle_param(
    count: int = 1,
    blueprint_filter: str = "vehicle.*",
    autopilot: bool = True,
    reference_id: int = None,
    relative_distance: float = 10.0,
    relative_angle: float = 0.0,
    initial_speed: float = 0.0
) -> str:
    """参数化生成车辆，支持参照物/距离/角度/速度控制"""
    try:
        if carla_client.world is None:
            return "❌ 未连接到CARLA服务器"
        
        params = SpawnParameters()
        params.actor_type = "vehicle"
        params.blueprint_filter = blueprint_filter
        params.reference_actor_id = reference_id
        params.relative_distance = relative_distance
        params.relative_angle = relative_angle
        params.autopilot = autopilot
        params.initial_speed = initial_speed
        params.count = count
        params.lane_type = "Driving"
        
        if reference_id is None:
            result = await carla_client.batch_spawn_vehicles_with_id(count, blueprint_filter, autopilot)
            lines = [f"✅ 生成完成：成功{result['success']}辆，失败{result['failed']}辆"]
            for v in result["vehicles"]:
                lines.append(f"  [{v['index']}] ID={v['id']} {v['type_id']} @({v['location']['x']},{v['location']['y']})")
            return "\n".join(lines)
        else:
            spawned = await carla_client.spawn_vehicles_with_params(params)
            return f"✅ 生成完成：成功{len(spawned)}辆，ID={[a.id for a in spawned]}"
    except Exception as e:
        app_logger.error(f"❌ spawn_vehicle_param 异常: {e}")
        return f"❌ 生成失败: {str(e)}"

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()
