"""
天气控制工具模块

提供天气参数设置、预设天气、时间控制等功能。
"""

import logging
from typing import Dict, Any
from mcp.server.fastmcp import FastMCP, Context

logger = logging.getLogger("hutb_mcp.tools.weather")

# 天气预设
WEATHER_PRESETS = {
    "clear": {
        "cloudiness": 0.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 0.0,
        "sun_altitude_angle": 70.0,
        "fog_density": 0.0,
        "wetness": 0.0
    },
    "cloudy": {
        "cloudiness": 80.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 20.0,
        "sun_altitude_angle": 45.0,
        "fog_density": 0.0,
        "wetness": 0.0
    },
    "rainy": {
        "cloudiness": 100.0,
        "precipitation": 80.0,
        "precipitation_deposits": 50.0,
        "wind_intensity": 30.0,
        "sun_altitude_angle": 30.0,
        "fog_density": 10.0,
        "wetness": 80.0
    },
    "foggy": {
        "cloudiness": 50.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 10.0,
        "sun_altitude_angle": 30.0,
        "fog_density": 80.0,
        "wetness": 20.0
    },
    "stormy": {
        "cloudiness": 100.0,
        "precipitation": 100.0,
        "precipitation_deposits": 100.0,
        "wind_intensity": 100.0,
        "sun_altitude_angle": 10.0,
        "fog_density": 30.0,
        "wetness": 100.0
    }
}


def register_weather_tools(mcp: FastMCP) -> None:
    """注册天气控制工具"""
    
    @mcp.tool()
    def get_weather(ctx: Context) -> Dict[str, Any]:
        """
        获取当前天气参数
        
        Returns:
            天气参数字典
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            weather = conn.world.get_weather()
            
            result = {
                "success": True,
                "cloudiness": weather.cloudiness,
                "precipitation": weather.precipitation,
                "precipitation_deposits": weather.precipitation_deposits,
                "wind_intensity": weather.wind_intensity,
                "sun_azimuth_angle": weather.sun_azimuth_angle,
                "sun_altitude_angle": weather.sun_altitude_angle,
                "fog_density": weather.fog_density,
                "fog_distance": weather.fog_distance,
                "wetness": weather.wetness,
                "fog_falloff": weather.fog_falloff,
                "scattering_intensity": weather.scattering_intensity,
                "mie_scattering_scale": weather.mie_scattering_scale,
                "rayleigh_scattering_scale": weather.rayleigh_scattering_scale
            }
            
            logger.info("获取天气参数成功")
            return result
            
        except Exception as e:
            logger.error(f"获取天气参数失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_weather(
        ctx: Context,
        cloudiness: float = None,
        precipitation: float = None,
        precipitation_deposits: float = None,
        wind_intensity: float = None,
        sun_azimuth_angle: float = None,
        sun_altitude_angle: float = None,
        fog_density: float = None,
        fog_distance: float = None,
        wetness: float = None
    ) -> Dict[str, Any]:
        """
        设置天气参数
        
        Args:
            cloudiness: 云量 (0-100)
            precipitation: 降水量 (0-100)
            precipitation_deposits: 降水沉积 (0-100)
            wind_intensity: 风力强度 (0-100)
            sun_azimuth_angle: 太阳方位角 (0-360)
            sun_altitude_angle: 太阳高度角 (-90 到 90)
            fog_density: 雾密度 (0-100)
            fog_distance: 雾距离
            wetness: 湿度 (0-100)
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            # 获取当前天气
            weather = conn.world.get_weather()
            
            # 更新指定参数（带范围限制）
            if cloudiness is not None:
                weather.cloudiness = max(0.0, min(100.0, cloudiness))
            if precipitation is not None:
                weather.precipitation = max(0.0, min(100.0, precipitation))
            if precipitation_deposits is not None:
                weather.precipitation_deposits = max(0.0, min(100.0, precipitation_deposits))
            if wind_intensity is not None:
                weather.wind_intensity = max(0.0, min(100.0, wind_intensity))
            if sun_azimuth_angle is not None:
                weather.sun_azimuth_angle = sun_azimuth_angle % 360.0
            if sun_altitude_angle is not None:
                weather.sun_altitude_angle = max(-90.0, min(90.0, sun_altitude_angle))
            if fog_density is not None:
                weather.fog_density = max(0.0, min(100.0, fog_density))
            if fog_distance is not None:
                weather.fog_distance = max(0.0, fog_distance)
            if wetness is not None:
                weather.wetness = max(0.0, min(100.0, wetness))
            
            # 应用天气
            conn.world.set_weather(weather)
            
            logger.info("设置天气参数成功")
            return {
                "success": True,
                "cloudiness": weather.cloudiness,
                "precipitation": weather.precipitation,
                "sun_altitude_angle": weather.sun_altitude_angle,
                "fog_density": weather.fog_density,
                "wetness": weather.wetness
            }
            
        except Exception as e:
            logger.error(f"设置天气参数失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_weather_preset(ctx: Context, preset: str) -> Dict[str, Any]:
        """
        应用天气预设
        
        Args:
            preset: 预设名称，可选值: clear, cloudy, rainy, foggy, stormy
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            preset = preset.lower()
            if preset not in WEATHER_PRESETS:
                return {
                    "success": False,
                    "error": f"未知预设: {preset}，可选值: {list(WEATHER_PRESETS.keys())}"
                }
            
            params = WEATHER_PRESETS[preset]
            
            # 创建天气对象
            weather = carla.WeatherParameters(
                cloudiness=params["cloudiness"],
                precipitation=params["precipitation"],
                precipitation_deposits=params["precipitation_deposits"],
                wind_intensity=params["wind_intensity"],
                sun_altitude_angle=params["sun_altitude_angle"],
                fog_density=params["fog_density"],
                wetness=params["wetness"]
            )
            
            conn.world.set_weather(weather)
            
            logger.info(f"应用天气预设成功: {preset}")
            return {
                "success": True,
                "preset": preset,
                "parameters": params
            }
            
        except Exception as e:
            logger.error(f"应用天气预设失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_time_of_day(ctx: Context, hour: int, minute: int = 0) -> Dict[str, Any]:
        """
        设置一天中的时间（通过调整太阳位置）
        
        Args:
            hour: 小时 (0-23)
            minute: 分钟 (0-59)
            
        Returns:
            操作结果
        """
        from ..connection import get_connection
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            # 限制范围
            hour = max(0, min(23, hour))
            minute = max(0, min(59, minute))
            
            # 计算太阳高度角
            # 简化模型：6:00 日出 (0°), 12:00 正午 (90°), 18:00 日落 (0°)
            time_decimal = hour + minute / 60.0
            
            if 6 <= time_decimal <= 18:
                # 白天
                sun_altitude = 90.0 * (1 - abs(time_decimal - 12) / 6)
            else:
                # 夜晚
                sun_altitude = -45.0
            
            # 计算太阳方位角
            sun_azimuth = (time_decimal - 6) * 15  # 每小时15度
            
            weather = conn.world.get_weather()
            weather.sun_altitude_angle = sun_altitude
            weather.sun_azimuth_angle = sun_azimuth % 360
            conn.world.set_weather(weather)
            
            logger.info(f"设置时间成功: {hour:02d}:{minute:02d}")
            return {
                "success": True,
                "time": f"{hour:02d}:{minute:02d}",
                "sun_altitude_angle": sun_altitude,
                "sun_azimuth_angle": sun_azimuth % 360
            }
            
        except Exception as e:
            logger.error(f"设置时间失败: {e}")
            return {"success": False, "error": str(e)}
    
    @mcp.tool()
    def set_extreme_weather(ctx: Context, weather_type: str) -> Dict[str, Any]:
        """
        设置极端天气（用于压力测试）
        
        Args:
            weather_type: 极端天气类型，可选值: 
                - heavy_rain: 暴雨
                - dense_fog: 浓雾
                - blizzard: 暴风雪
                - sandstorm: 沙尘暴
                
        Returns:
            操作结果
        """
        from ..connection import get_connection
        import carla
        
        extreme_presets = {
            "heavy_rain": {
                "cloudiness": 100.0,
                "precipitation": 100.0,
                "precipitation_deposits": 100.0,
                "wind_intensity": 80.0,
                "sun_altitude_angle": 5.0,
                "fog_density": 20.0,
                "wetness": 100.0
            },
            "dense_fog": {
                "cloudiness": 100.0,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "wind_intensity": 5.0,
                "sun_altitude_angle": 20.0,
                "fog_density": 100.0,
                "wetness": 50.0
            },
            "blizzard": {
                "cloudiness": 100.0,
                "precipitation": 100.0,
                "precipitation_deposits": 100.0,
                "wind_intensity": 100.0,
                "sun_altitude_angle": 10.0,
                "fog_density": 50.0,
                "wetness": 0.0
            },
            "sandstorm": {
                "cloudiness": 80.0,
                "precipitation": 0.0,
                "precipitation_deposits": 50.0,
                "wind_intensity": 100.0,
                "sun_altitude_angle": 30.0,
                "fog_density": 70.0,
                "wetness": 0.0
            }
        }
        
        try:
            conn = get_connection()
            if not conn.is_connected():
                return {"success": False, "error": "未连接到仿真器"}
            
            weather_type = weather_type.lower()
            if weather_type not in extreme_presets:
                return {
                    "success": False,
                    "error": f"未知极端天气类型: {weather_type}，可选值: {list(extreme_presets.keys())}"
                }
            
            params = extreme_presets[weather_type]
            
            weather = carla.WeatherParameters(
                cloudiness=params["cloudiness"],
                precipitation=params["precipitation"],
                precipitation_deposits=params["precipitation_deposits"],
                wind_intensity=params["wind_intensity"],
                sun_altitude_angle=params["sun_altitude_angle"],
                fog_density=params["fog_density"],
                wetness=params["wetness"]
            )
            
            conn.world.set_weather(weather)
            
            logger.info(f"设置极端天气成功: {weather_type}")
            return {
                "success": True,
                "weather_type": weather_type,
                "parameters": params
            }
            
        except Exception as e:
            logger.error(f"设置极端天气失败: {e}")
            return {"success": False, "error": str(e)}
    
    logger.info("天气控制工具注册完成")
