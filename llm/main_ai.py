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
# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

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

    async def connect(self, host='localhost', port=2000):
        """连接CARLA服务器"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10)
            self.world = self.client.get_world()
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
            self.world = self.client.load_world(map_name)
            app_logger.info(f"✅ 地图加载成功: {map_name}")
            return True
        except Exception as e:
            app_logger.error(f"❌ 加载地图失败: {str(e)}")
            return False

    async def set_synchronous_mode(self, enabled=True, fixed_delta_seconds=0.05):
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
        """启动后台tick循环，确保世界持续运行"""
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
                    await asyncio.sleep(0.05)  # 20 FPS
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

    async def spawn_vehicles(self, vehicle_type='model3', count=1):
        """生成多辆车辆，返回生成的车辆列表和最后一辆车
        
        支持的车辆类型:
        - tesla: model3
        - audi: a2, etron, tt
        - bmw: grandtourer, i8, mini
        - chevrolet: impala
        - citroen: c3
        - dodge: charger_police, charger2020
        - ford: mustang, crown
        - jeep: wrangler_rubicon
        - lincoln: mkz_2017, mkz_2020
        - mercedes: benz_coupe, cabrio, ccc
        - mini: cooper_s
        - nissan: micra, patrol
        - seat: leon
        - volkswagen: t2, t3
        
        数据量支持: 取决于地图生成点数量，通常支持10-100+辆车
        """
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return []
        
        try:
            # 车辆类型到蓝图路径的映射
            vehicle_blueprints = {
                # Tesla
                'model3': 'vehicle.tesla.model3',
                # Audi
                'a2': 'vehicle.audi.a2',
                'etron': 'vehicle.audi.etron',
                'tt': 'vehicle.audi.tt',
                # BMW
                'grandtourer': 'vehicle.bmw.grandtourer',
                'i8': 'vehicle.bmw.i8',
                'mini': 'vehicle.bmw.mini',
                # Chevrolet
                'impala': 'vehicle.chevrolet.impala',
                # Citroen
                'c3': 'vehicle.citroen.c3',
                # Dodge
                'charger_police': 'vehicle.dodge.charger_police',
                'charger2020': 'vehicle.dodge.charger2020',
                # Ford
                'mustang': 'vehicle.ford.mustang',
                'crown': 'vehicle.ford.crown',
                # Jeep
                'wrangler_rubicon': 'vehicle.jeep.wrangler_rubicon',
                # Lincoln
                'mkz_2017': 'vehicle.lincoln.mkz_2017',
                'mkz_2020': 'vehicle.lincoln.mkz_2020',
                # Mercedes
                'benz_coupe': 'vehicle.mercedes.benz_coupe',
                'cabrio': 'vehicle.mercedes.cabrio',
                'ccc': 'vehicle.mercedes.ccc',
                # Mini
                'cooper_s': 'vehicle.mini.cooper_s',
                # Nissan
                'micra': 'vehicle.nissan.micra',
                'patrol': 'vehicle.nissan.patrol',
                # Seat
                'leon': 'vehicle.seat.leon',
                # Volkswagen
                't2': 'vehicle.volkswagen.t2',
                't3': 'vehicle.volkswagen.t3',
            }
            
            # 获取蓝图路径
            blueprint_path = vehicle_blueprints.get(vehicle_type.lower(), f'vehicle.tesla.{vehicle_type}')
            
            # 获取可用生成点
            spawn_points = self.world.get_map().get_spawn_points()
            available_points = len(spawn_points)
            
            # 限制生成数量不超过可用生成点
            actual_count = min(count, available_points)
            if count > available_points:
                app_logger.warning(f"⚠️ 请求生成{count}辆车，但地图只有{available_points}个生成点，将生成{actual_count}辆")
            
            blueprint_library = self.world.get_blueprint_library()
            
            spawned_vehicles = []
            for i in range(actual_count):
                try:
                    # 尝试查找指定蓝图，如果不存在则使用随机车辆蓝图
                    blueprint = blueprint_library.find(blueprint_path)
                    if blueprint is None:
                        # 如果指定蓝图不存在，使用随机车辆蓝图
                        blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if bp.id.startswith('vehicle.')]
                        blueprint = blueprints[i % len(blueprints)] if blueprints else None
                    
                    if blueprint is None:
                        app_logger.error(f"❌ 无法找到车辆蓝图")
                        continue
                    
                    # 设置车辆颜色 - 修复颜色格式问题
                    if blueprint.has_attribute('color'):
                        # 生成随机的RGB颜色值 (0-255范围)
                        import random
                        r = random.randint(0, 255)
                        g = random.randint(0, 255)
                        b = random.randint(0, 255)
                        color_str = f"{r},{g},{b}"
                        blueprint.set_attribute('color', color_str)
                        app_logger.info(f"🎨 设置车辆颜色: RGB({color_str})")
                    
                    # 设置role_name属性避免错误
                    if blueprint.has_attribute('role_name'):
                        blueprint.set_attribute('role_name', 'autopilot')
                    
                    # 尝试多个位置生成车辆
                    spawn_success = False
                    for attempt in range(10):  # 尝试10个位置
                        try:
                            # 使用随机道路位置
                            spawn_location = self.world.get_random_location_from_navigation()
                            if not spawn_location:
                                # 如果无法获取随机道路位置，使用固定生成点
                                import random
                                spawn_point = random.choice(spawn_points)
                                spawn_location = spawn_point.location
                            
                            # 为避免碰撞，对位置进行更大范围的随机偏移
                            import random
                            offset_x = random.uniform(-3.0, 3.0)
                            offset_y = random.uniform(-3.0, 3.0)
                            offset_z = 0.1  # 稍微抬高一点，避免地面碰撞
                            
                            # 创建新的变换，添加偏移
                            from carla import Transform, Location
                            new_location = Location(
                                x=spawn_location.x + offset_x,
                                y=spawn_location.y + offset_y,
                                z=spawn_location.z + offset_z
                            )
                            
                            # 随机旋转角度
                            from carla import Rotation
                            random_yaw = random.uniform(0, 360)
                            new_rotation = Rotation(yaw=random_yaw)
                            new_transform = Transform(new_location, new_rotation)
                            
                            app_logger.info(f"📍 尝试生成车辆，位置: {new_location}, 朝向: {random_yaw:.1f}°")
                            
                            # 尝试生成车辆
                            vehicle = self.world.try_spawn_actor(blueprint, new_transform)
                            if vehicle:
                                self.actors.append(vehicle)
                                spawned_vehicles.append(vehicle)
                                app_logger.info(f"🚗 生成第{i+1}辆车: {blueprint.id} (ID: {vehicle.id})")
                                spawn_success = True
                                break
                            else:
                                app_logger.warning(f"⚠️ 尝试 {attempt+1}/10: 生成车辆失败，位置可能被占用")
                                
                        except Exception as loc_error:
                            app_logger.error(f"❌ 位置生成时出错: {loc_error}")
                            continue
                    
                    if not spawn_success:
                        app_logger.error(f"❌ 生成第{i+1}辆车失败，已尝试10个位置")
                        
                except Exception as e:
                    app_logger.error(f"❌ 生成第{i+1}辆车时出错: {str(e)}")
                    continue
            
            app_logger.info(f"✅ 共生成{len(spawned_vehicles)}辆车")
            return spawned_vehicles
            
        except Exception as e:
            app_logger.error(f"❌ 生成车辆失败: {str(e)}")
            return []

    async def spawn_vehicle(self, vehicle_type='model3'):
        """生成单辆车辆（兼容旧接口）"""
        vehicles = await self.spawn_vehicles(vehicle_type, count=1)
        return vehicles[0] if vehicles else None

    async def set_weather(self, weather_type='clear'):
        """设置天气"""
        # 检查是否已连接到CARLA服务器
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
            )
        }
        if weather_type in weather_presets:
            self.world.set_weather(weather_presets[weather_type])
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
            import random
            
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
                                        controller.go_to_location(self.world.get_random_location_from_navigation())
                                        controller.set_max_speed(speed)
                                        app_logger.info(f"🎯 为行人设置随机目标位置，速度: {speed} m/s")
                                        
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
        """设置车辆自动驾驶模式
        
        Args:
            enable: 是否启用自动驾驶
            radius: 自动驾驶范围半径（米），0表示全图
        """
        # 检查是否已连接到CARLA服务器
        if self.world is None:
            app_logger.error("❌ 未连接到CARLA服务器，请先调用connect_carla")
            return False
        
        try:
            # 获取所有车辆
            vehicles = [actor for actor in self.world.get_actors() if 'vehicle' in actor.type_id]
            
            enabled_count = 0
            for vehicle in vehicles:
                # 检查车辆是否在指定范围内
                if radius > 0:
                    # 以当前 spectator 位置为中心
                    spectator = self.world.get_spectator()
                    spectator_location = spectator.get_location()
                    vehicle_location = vehicle.get_location()
                    distance = spectator_location.distance(vehicle_location)
                    
                    if distance > radius:
                        continue
                
                # 启用或禁用自动驾驶
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

    async def cleanup(self):
        """清理环境"""
        # 停止后台tick循环
        await self.stop_tick_loop()

        # 停止视角跟随
        await self.stop_view_follow()

        # 停止视频录制
        await self.stop_recording()

        for actor in self.actors:
            if actor.is_alive:
                actor.destroy()
        self.actors = []
        app_logger.info("🧹 清理所有CARLA actor")

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
            import math
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
            import math
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
            app_logger.info(f"👁️  俯视视角已设置 - 高度: {height}m")
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

            import math
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

    async def start_view_follow(self, view_mode, target_actor):
        """启动视角跟随任务

        Args:
            view_mode: 视角模式 - third_person, first_person
            target_actor: 要跟随的目标actor
        """
        import asyncio

        # 停止之前的跟随
        self.stop_view_follow()

        self.is_view_following = True
        self.view_target = target_actor
        self.current_view_mode = view_mode

        app_logger.info(f"🎯 启动视角跟随 - 模式: {view_mode}, 目标: {target_actor.id}")

        while self.is_view_following and self.world:
            try:
                # 检查目标是否还存在
                if not target_actor.is_alive:
                    app_logger.warning(f"⚠️ 视角目标 {target_actor.id} 已不存在，停止跟随")
                    break

                # 根据视角模式更新视角
                if view_mode == "third_person":
                    self._update_third_person_view(target_actor)
                elif view_mode == "first_person":
                    self._update_first_person_view(target_actor)

                # 每50ms更新一次（约20fps）
                await asyncio.sleep(0.05)

            except Exception as e:
                app_logger.warning(f"⚠️ 视角跟随出错: {e}")
                await asyncio.sleep(0.1)

    async def stop_view_follow(self):
        """停止视角跟随"""
        if self.is_view_following:
            self.is_view_following = False
            app_logger.info("🛑 停止视角跟随")

        # 取消之前的跟随任务
        if self.view_follow_task and not self.view_follow_task.done():
            try:
                self.view_follow_task.cancel()
                # 等待任务真正结束
                await asyncio.sleep(0.1)
            except Exception:
                pass
            self.view_follow_task = None

    def _update_third_person_view(self, target_actor, distance=5.0, height=2.0):
        """更新第三人称视角位置（用于跟随）"""
        try:
            spectator = self.world.get_spectator()
            target_transform = target_actor.get_transform()
            target_location = target_transform.location

            import math
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

            import math
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
                    img = np.frombuffer(image.raw_data, dtype=np.uint8)
                    img = img.reshape((image.height, image.width, 4))  # BGRA
                    img = img[:, :, :3]  # 转换为BGR
                    img = img[:, :, ::-1]  # 转换为RGB

                    # 写入视频文件
                    if self.video_writer is None:
                        import cv2
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        self.video_writer = cv2.VideoWriter(
                            self.recording_output_path,
                            fourcc,
                            self.recording_fps,
                            (image.width, image.height)
                        )
                        app_logger.info(f"📹 视频写入器已创建")

                    self.video_writer.write(img)
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
                # 启动视角跟随任务
                self.view_follow_task = asyncio.create_task(
                    self.start_view_follow("third_person", target_actor)
                )
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
                # 启动视角跟随任务
                self.view_follow_task = asyncio.create_task(
                    self.start_view_follow("first_person", target_actor)
                )
                result = f"✅ 已切换到第一人称视角 - 目标: {target_actor.id} (已启用跟随)"
            else:
                result = "❌ 第一人称视角需要指定目标"

        elif view_mode == "overhead":
            # 停止之前的跟随
            await self.stop_view_follow()
            self.set_overhead_view(target_actor)
            self.current_view_mode = "overhead"
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
    """（实际功能：连接CARLA服务器）"""
    success = await carla_client.connect(host, port)
    return "✅ CARLA服务器连接成功" if success else "❌ 连接CARLA服务器失败"


async def spawn_vehicle_impl(query: str, count: int = 1, **kwargs) -> str:
    """（实际功能：生成车辆）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    vehicles = await carla_client.spawn_vehicles(query, count=count)
    if vehicles:
        if len(vehicles) == 1:
            return f"✅ 已生成1辆{query}车辆 (ID: {vehicles[0].id})"
        else:
            last_vehicle = vehicles[-1]
            return f"✅ 已生成{len(vehicles)}辆{query}车辆，最后一辆车ID: {last_vehicle.id}"
    return "❌ 车辆生成失败，请确保CARLA服务器已连接且地图有可用生成点"


async def set_weather_impl(owner: str, repo: str) -> str:
    """（实际功能：设置天气）"""
    # 检查是否已连接到CARLA服务器
    if carla_client.world is None:
        return "❌ 未连接到CARLA服务器，请先使用'连接CARLA服务器'命令进行连接"
    
    weather_types = {'clear': '晴天', 'rain': '雨天', 'fog': '雾天'}
    success = await carla_client.set_weather(repo.lower())
    return f"✅ 天气已设置为 {weather_types.get(repo.lower(), repo)}" if success else "❌ 不支持的天气类型"


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
async def set_weather(owner: str, repo: str) -> str:
    """（实际功能：设置天气）"""
    return await set_weather_impl(owner, repo)


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



# ============ AI助手类（集成Deepseek AI） ============

class FastMCPGitHubAssistant:
    """FastMCP GitHub AI助手 - 集成Deepseek AI与FastMCP工具"""

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
                    "name": "set_weather",
                    "description": "设置天气（clear/rain/fog）",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "owner": {"type": "string", "description": "固定值weather"},
                            "repo": {"type": "string", "enum": ["clear", "rain", "fog"]}
                        },
                        "required": ["owner", "repo"]
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
                result = await connect_carla_impl(
                    host=arguments.get("host", "localhost"),
                    port=arguments.get("port", 2000)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "spawn_vehicle":
                app_logger.info(f"spawn_vehicle参数详情: {arguments}")
                result = await spawn_vehicle_impl(
                    query=arguments["query"],
                    count=arguments.get("count", 1)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "set_weather":
                result = await set_weather_impl(
                    owner=arguments["owner"],
                    repo=arguments["repo"]
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "get_traffic_lights":
                result = await get_traffic_lights_impl(
                    query=arguments["query"],
                    user_type=arguments.get("user_type")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "cleanup_scene":
                result = await cleanup_scene_impl(
                    language=arguments.get("language"),
                    period=arguments.get("period", "daily")
                )
                return {
                    "success": True,
                    "data": result
                }
            
            elif function_name == "spawn_pedestrian":
                result = await spawn_pedestrian_impl(
                    query=arguments["query"],
                    count=arguments.get("count", 1),
                    speed=arguments.get("speed")
                )
                return {
                    "success": True,
                    "data": result
                }
            elif function_name == "setup_autopilot":
                result = await setup_autopilot_impl(
                    enable=arguments.get("enable", True),
                    radius=arguments.get("radius", 50.0)
                )
                return {
                    "success": True,
                    "data": result
                }
            elif function_name == "setup_pedestrian_movement":
                result = await setup_pedestrian_movement_impl(
                    enable=arguments.get("enable", True),
                    radius=arguments.get("radius", 50.0)
                )
                return {
                    "success": True,
                    "data": result
                }
            elif function_name == "switch_view":
                result = await switch_view_impl(
                    view_mode=arguments.get("view_mode", "third_person"),
                    target_actor_id=arguments.get("target_actor_id")
                )
                return {
                    "success": True,
                    "data": result
                }
            elif function_name == "start_recording":
                result = await start_recording_impl(
                    fps=arguments.get("fps", 30)
                )
                return {
                    "success": True,
                    "data": result
                }
            elif function_name == "stop_recording":
                result = await stop_recording_impl()
                return {
                    "success": True,
                    "data": result
                }
            elif function_name == "search_github_repositories":
                result = await search_github_repositories_impl(
                    query=arguments["query"],
                    language=arguments.get("language"),
                    sort=arguments.get("sort", "stars"),
                    limit=arguments.get("limit", 8)
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "get_repository_details":
                result = await get_repository_details_impl(
                    owner=arguments["owner"],
                    repo=arguments["repo"]
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "search_github_users":
                result = await search_github_users_impl(
                    query=arguments["query"],
                    user_type=arguments.get("user_type")
                )
                return {
                    "success": True,
                    "data": result
                }

            elif function_name == "get_trending_repositories":
                result = await get_trending_repositories_impl(
                    language=arguments.get("language"),
                    period=arguments.get("period", "daily")
                )
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
        "t3": "Volkswagen T3"
    }

    # 车辆中文到英文的映射
    VEHICLE_TYPE_MAP = {
        # Tesla
        "特斯拉": "model3",
        "特斯拉model3": "model3",
        "model3": "model3",
        # Audi
        "奥迪": "a2",
        "奥迪a2": "a2",
        "a2": "a2",
        "奥迪etron": "etron",
        "etron": "etron",
        "奥迪tt": "tt",
        "tt": "tt",
        # BMW
        "宝马": "grandtourer",
        "宝马grandtourer": "grandtourer",
        "grandtourer": "grandtourer",
        "宝马i8": "i8",
        "i8": "i8",
        "宝马mini": "mini",
        "mini": "mini",
        # Chevrolet
        "雪佛兰": "impala",
        "雪佛兰impala": "impala",
        "impala": "impala",
        # Citroen
        "雪铁龙": "c3",
        "雪铁龙c3": "c3",
        "c3": "c3",
        # Dodge
        "道奇": "charger2020",
        "道奇警车": "charger_police",
        "charger_police": "charger_police",
        "道奇charger": "charger2020",
        "charger2020": "charger2020",
        # Ford
        "福特": "mustang",
        "福特野马": "mustang",
        "野马": "mustang",
        "mustang": "mustang",
        "福特crown": "crown",
        "crown": "crown",
        # Jeep
        "吉普": "wrangler_rubicon",
        "吉普牧马人": "wrangler_rubicon",
        "牧马人": "wrangler_rubicon",
        "wrangler_rubicon": "wrangler_rubicon",
        # Lincoln
        "林肯": "mkz_2020",
        "林肯mkz2017": "mkz_2017",
        "mkz_2017": "mkz_2017",
        "林肯mkz2020": "mkz_2020",
        "mkz_2020": "mkz_2020",
        # Mercedes
        "奔驰": "benz_coupe",
        "奔驰轿跑": "benz_coupe",
        "benz_coupe": "benz_coupe",
        "奔驰敞篷": "cabrio",
        "cabrio": "cabrio",
        "奔驰ccc": "ccc",
        "ccc": "ccc",
        # Mini
        "迷你": "cooper_s",
        "迷你cooper": "cooper_s",
        "cooper_s": "cooper_s",
        # Nissan
        "日产": "patrol",
        "日产micra": "micra",
        "micra": "micra",
        "日产patrol": "patrol",
        "patrol": "patrol",
        # Seat
        "西雅特": "leon",
        "西雅特leon": "leon",
        "leon": "leon",
        # Volkswagen
        "大众": "t2",
        "大众t2": "t2",
        "t2": "t2",
        "大众t3": "t3",
        "t3": "t3"
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
        """检测用户是否有生成车辆或行人的意图，但缺少必要参数

        只有当缺少类型时才询问，数量默认为1，不询问
        """
        message = message.lower()

        # 首先排除视角控制相关的指令（这些不是生成请求）
        view_keywords = ['视角', '切换', '人称', '俯视', '鸟瞰', '自由视角', '录制', '录像', '视频']
        if any(kw in message for kw in view_keywords):
            return {
                'needs_vehicle_type': False,
                'needs_vehicle_count': False,
                'needs_pedestrian_type': False,
                'needs_pedestrian_count': False,
                'is_ambiguous': False
            }

        # 车辆相关关键词
        vehicle_keywords = ['车', '车辆', '汽车', '生成车', '创建车', '来车', '加车', '添加车辆']
        # 行人相关关键词（更精确，避免误判）
        pedestrian_keywords = ['行人', '生成行人', '创建行人', '添加行人', '路人']
        # 单独的"人"字需要结合生成类动词才认为是生成行人
        person_spawn_verbs = ['生成', '创建', '来', '加', '添加', '放', 'spawn']

        # 数量相关模式
        import re
        has_count = bool(re.search(r'\d+\s*[辆个]', message))

        # 检测车辆类型（支持英文和中文）
        has_vehicle_type = any(vtype in message for vtype in self.VEHICLE_TYPES.keys()) or \
                           any(vname in message for vname in self.VEHICLE_TYPE_MAP.keys())
        # 检测行人类型（支持英文和中文）- 但排除泛指的"行人"
        specific_pedestrian_keywords = set(self.PEDESTRIAN_TYPES.keys()) | set(self.PEDESTRIAN_TYPE_MAP.keys()) - {"行人", "人"}
        has_pedestrian_type = any(ptype in message for ptype in specific_pedestrian_keywords)

        # 检查是否是模糊的车辆生成请求
        is_vehicle_request = any(kw in message for kw in vehicle_keywords)

        # 检查是否是模糊的行人生成请求
        is_pedestrian_request = any(kw in message for kw in pedestrian_keywords)
        # 单独的"人"需要配合生成动词才算
        if not is_pedestrian_request and '人' in message:
            has_spawn_verb = any(verb in message for verb in person_spawn_verbs)
            # 排除"人称"（第一人称、第三人称等）
            if has_spawn_verb and '人称' not in message:
                is_pedestrian_request = True

        result = {
            'needs_vehicle_type': False,
            'needs_vehicle_count': False,
            'needs_pedestrian_type': False,
            'needs_pedestrian_count': False,
            'is_ambiguous': False
        }

        # 如果是车辆请求但没有指定类型，需要询问
        if is_vehicle_request and not has_vehicle_type:
            result['needs_vehicle_type'] = True
            result['needs_vehicle_count'] = not has_count  # 记录是否也缺少数量
            result['is_ambiguous'] = True

        # 如果是行人请求但没有指定类型，需要询问
        if is_pedestrian_request and not has_pedestrian_type:
            result['needs_pedestrian_type'] = True
            result['needs_pedestrian_count'] = not has_count  # 记录是否也缺少数量
            result['is_ambiguous'] = True

        return result

    def _generate_spawn_prompt(self, check_result):
        """生成参数询问提示"""
        prompt_parts = []

        if check_result['needs_vehicle_type']:
            # 使用中文展示可用的车辆类型
            vehicle_list = "\n".join([f"  • {name} ({key})" for key, name in self.VEHICLE_TYPES.items()])
            prompt_parts.append(f"🚗 **可用车辆类型：**\n{vehicle_list}")

        if check_result['needs_vehicle_count']:
            prompt_parts.append("🚗 **车辆数量：** 支持生成 1-100+ 辆车（取决于地图可用生成点数量）")

        if check_result['needs_pedestrian_type']:
            # 使用中文展示可用的行人类型
            pedestrian_list = "\n".join([f"  • {name} ({key})" for key, name in self.PEDESTRIAN_TYPES.items()])
            prompt_parts.append(f"🚶 **可用行人类型：**\n{pedestrian_list}")

        if check_result['needs_pedestrian_count']:
            prompt_parts.append("🚶 **行人数量：** 支持生成 1-100+ 个行人（取决于地图大小）")

        if prompt_parts:
            prompt_parts.insert(0, "请提供以下信息以完成生成：\n")
            prompt_parts.append("\n💡 **示例指令：**")
            if check_result['needs_vehicle_type'] or check_result['needs_vehicle_count']:
                prompt_parts.append('  • "生成5辆特斯拉"')
                prompt_parts.append('  • "来10辆福特野马"')
                prompt_parts.append('  • "生成3辆宝马"')
                prompt_parts.append('  • "来5辆奔驰"')
            if check_result['needs_pedestrian_type'] or check_result['needs_pedestrian_count']:
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
        if '第一人称' in message or 'first_person' in message:
            view_mode = 'first_person'
        elif '第三人称' in message or 'third_person' in message:
            view_mode = 'third_person'
        elif '俯视' in message or '鸟瞰' in message or 'overhead' in message:
            view_mode = 'overhead'
        elif '自由' in message or 'free' in message:
            view_mode = 'free'
        elif '旁观者' in message or 'bystander' in message:
            view_mode = 'bystander'
        else:
            # 默认第三人称
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
                "content": """你是一个GitHub搜索助手，基于FastMCP框架提供服务。你有以下工具可以使用：


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


CARLA相关：
- 当用户提到"连接"、"服务器"、"CARLA"等明确要求连接时，使用connect_carla
- 当用户提到"车辆"、"生成"、"创建汽车"、"车"等，使用spawn_vehicle，count参数默认为1
- 当用户提到"多辆车"、"生成X辆车"、"几辆车"、指定数量（如5辆、10辆），必须设置count参数为对应数字
- 车辆类型支持中文：特斯拉(model3)、奥迪(a2/etron/tt)、宝马(grandtourer/i8/mini)、雪佛兰(impala)、雪铁龙(c3)、道奇(charger_police/charger2020)、福特(mustang/crown)、吉普(wrangler_rubicon)、林肯(mkz_2017/mkz_2020)、奔驰(benz_coupe/cabrio/ccc)、迷你(cooper_s)、日产(micra/patrol)、西雅特(leon)、大众(t2/t3)
- 当用户使用中文车辆类型（如"生成3辆特斯拉"），你需要将中文类型转换为对应的英文类型：model3、a2、etron、tt、grandtourer、i8、mini、impala、c3、charger_police、charger2020、mustang、crown、wrangler_rubicon、mkz_2017、mkz_2020、benz_coupe、cabrio、ccc、cooper_s、micra、patrol、leon、t2、t3
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

本助手基于FastMCP框架构建，提供高效、类型安全的工具调用体验。"""
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
function askExample(text) {
    document.getElementById('messageInput').value = text;
    submitMessage();
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


# 创建全局AI助手实例
assistant = FastMCPGitHubAssistant()


def main():
    """主函数 - 可以选择启动Web界面或MCP服务器"""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "mcp":
        # 启动MCP服务器模式
        print("[MCP] 启动FastMCP AI助手MCP服务器...")

        # 验证配置
        if not config.validate():
            print("[ERROR] 配置验证失败")
            print("[INFO] 请确保环境变量包含:")
            print("[INFO] 请确保 .env 文件包含以下必要配置：")
            print("   - GITHUB_TOKEN=your_github_token")
            print("   - DEEPSEEK_API_KEY=your_deepseek_api_key")
            return

        print("[OK] 配置验证通过")
        print("[TOOLS] 已注册MCP工具:")
        print("   - search_github_repositories")
        print("   - get_repository_details")
        print("   - search_github_users")
        print("   - get_trending_repositories")
        print("[READY] 等待AI连接...")

        # 启动FastMCP服务器
        mcp.run()
    else:
        # 默认启动Web AI对话界面
        print("[WEB] 启动FastMCP AI助手对话界面...")
        print("[AI] 集成Deepseek AI + FastMCP工具")

        # 验证配置
        if not config.validate():
            print("[ERROR] 配置验证失败，请检查环境变量设置")
            print("[INFO] 请确保 .env 文件包含以下必要配置：")
            print("   - GITHUB_TOKEN=your_github_token")
            print("   - DEEPSEEK_API_KEY=your_deepseek_api_key")
            return

        print("[OK] 配置验证通过")
        print("[TOOLS] FastMCP工具已注册:")
        print("   - @mcp.tool() search_github_repositories")
        print("   - @mcp.tool() get_repository_details")
        print("   - @mcp.tool() search_github_users")
        print("   - @mcp.tool() get_trending_repositories")
        print("[INFO] 基于FastMCP框架 + Deepseek AI智能对话")
        print()

        # 指定 0.0.0.0 无效
        # 获取本地机器的ip地址
        host_ip = socket.gethostbyname(socket.gethostname())
        print(f"[INFO] 访问地址: http://{host_ip}:3000")
        uvicorn.run(app='main_ai:app', host=host_ip, port=3000, reload=True)


if __name__ == "__main__":
    main()
