#!/usr/bin/env python3
"""
FastMCP CARLA Assistant - 基于FastMCP框架的CARLA自动驾驶模拟器助手
保留原始GitHub助手的完整界面，仅替换功能逻辑
"""

import sys
import json
import re
from pathlib import Path
from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse
import uvicorn
import aiohttp
from typing import Optional
import carla  # CARLA Python API

# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

from fastmcp import FastMCP
from src.config import config
from src.utils.logger import app_logger

# 创建FastMCP实例
mcp = FastMCP("CARLA智能助手")


# ============ CARLA工具函数定义 ============

class CarlaClient:
    """CARLA客户端封装类（保持与原始代码相同的工具函数名）"""

    def __init__(self):
        self.client = None
        self.world = None
        self.actors = []

    async def connect(self, host='localhost', port=2000):
        """连接CARLA服务器"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10)
            self.world = self.client.get_world()
            print("1")
            print(self.client)
            print(self.world)
            app_logger.info("✅ CARLA服务器连接成功")
            return True
        except Exception as e:
            app_logger.error(f"❌ 连接CARLA失败: {str(e)}")
            return False

    async def spawn_vehicle(self, vehicle_type='model3'):
        """生成车辆（对应原search_repositories）"""
        try:
            blueprint = self.world.get_blueprint_library().find(f'vehicle.tesla.{vehicle_type}')
            spawn_point = self.world.get_map().get_spawn_points()[0]
            vehicle = self.world.spawn_actor(blueprint, spawn_point)
            self.actors.append(vehicle)
            app_logger.info(f"🚗 生成车辆: {vehicle_type}")
            return vehicle
        except Exception as e:
            app_logger.error(f"❌ 生成车辆失败: {str(e)}")
            return None

    async def set_weather(self, weather_type='clear'):
        """设置天气"""
        weather_presets = {
            'clear': carla.WeatherParameters(
                    cloudiness=0,           # 无云
                    precipitation=0,        # 无雨
                    precipitation_deposits=0,  # 无积水
                    wind_intensity=10,     # 微风
                    sun_azimuth_angle=0,   # 太阳方位角（0-360°）
                    sun_altitude_angle=75, # 太阳高度角（正午）
                    fog_density=0,         # 无雾
                    fog_distance=0,        # 无雾
                    wetness=0,             # 地面干燥
                    fog_falloff=0,
                    scattering_intensity=0,
                    mie_scattering_scale=0,
                    rayleigh_scattering_scale=0,
                    dust_storm=0
                ),
            'rain': carla.WeatherParameters(
                    cloudiness=100,        # 乌云密布
                    precipitation=80,       # 中到大雨
                    precipitation_deposits=50,  # 地面有积水
                    wind_intensity=30,     # 中等风力（影响雨滴方向）
                    sun_azimuth_angle=0,
                    sun_altitude_angle=15, # 低角度光照（模拟阴天）
                    fog_density=10,        # 轻微雾气（雨雾效果）
                    fog_distance=100,      # 雾气起始距离
                    wetness=60,            # 地面湿润
                    fog_falloff=1,
                    scattering_intensity=1,
                    mie_scattering_scale=0.5,
                    rayleigh_scattering_scale=0.1,
                    dust_storm=0
                ),
            'fog': carla.WeatherParameters(
                    cloudiness=80,         # 多云
                    precipitation=0,
                    precipitation_deposits=0,
                    wind_intensity=5,      # 无风
                    sun_azimuth_angle=0,
                    sun_altitude_angle=30, # 中等光照（雾中阳光）
                    fog_density=90,        # 浓雾
                    fog_distance=50,       # 近距离起雾
                    wetness=20,            # 轻微潮湿（晨雾效果）
                    fog_falloff=5,         # 低矮雾气（贴近地面）
                    scattering_intensity=2,
                    mie_scattering_scale=1.2,
                    rayleigh_scattering_scale=0.2,
                    dust_storm=0
                )
        }
        if weather_type in weather_presets:
            self.world.set_weather(weather_presets[weather_type])
            return True
        return False

    async def get_traffic_lights(self):
        """获取交通灯（对应原search_users）"""
        lights = [light for light in self.world.get_actors() if 'traffic_light' in light.type_id]
        return lights[:5]  # 只返回前5个

    async def cleanup(self):
        """清理环境（对应原get_trending_repositories）"""
        for actor in self.actors:
            if actor.is_alive:
                actor.destroy()
        self.actors = []
        app_logger.info("🧹 清理所有CARLA actor")


# 全局CARLA客户端实例
carla_client = CarlaClient()


# ============ 工具实现（保持与原GitHub助手相同的函数名） ============

async def connect_github_impl(host: str = 'localhost', port: int = 2000) -> str:
    """（实际功能：连接CARLA服务器）"""
    success = await carla_client.connect(host, port)
    return "✅ CARLA服务器连接成功" if success else "❌ 连接CARLA服务器失败"


async def search_github_repositories_impl(query: str, **kwargs) -> str:
    """（实际功能：生成车辆）"""
    vehicle = await carla_client.spawn_vehicle(query)
    if vehicle:
        return f"✅ 已生成车辆: {query} (ID: {vehicle.id})"
    return "❌ 车辆生成失败"


async def get_repository_details_impl(owner: str, repo: str) -> str:
    """（实际功能：设置天气）"""
    weather_types = {'clear': '晴天', 'rain': '雨天', 'fog': '雾天'}
    success = await carla_client.set_weather(repo.lower())
    return f"✅ 天气已设置为 {weather_types.get(repo.lower(), repo)}" if success else "❌ 不支持的天气类型"


async def search_github_users_impl(query: str, **kwargs) -> str:
    """（实际功能：获取交通灯信息）"""
    lights = await carla_client.get_traffic_lights()
    result = ["🚦 交通灯状态:"]
    for i, light in enumerate(lights, 1):
        state = "绿色" if light.state == carla.TrafficLightState.Green else \
            "红色" if light.state == carla.TrafficLightState.Red else \
                "黄色"
        result.append(f"{i}. {light.type_id} - {state} (位置: {light.get_location()})")
    return "\n".join(result)


async def get_trending_repositories_impl(**kwargs) -> str:
    """（实际功能：清理环境）"""
    await carla_client.cleanup()
    return "✅ 已清理所有车辆和物体"


# ============ FastMCP 工具装饰器（保持与原GitHub助手完全相同的定义） ============

@mcp.tool()
async def connect_github(host: str = 'localhost', port: int = 2000) -> str:
    """（实际功能：连接CARLA）"""
    return await connect_github_impl(host, port)


@mcp.tool()
async def search_github_repositories(query: str, language: Optional[str] = None,
                                     sort: str = "stars", limit: int = 8) -> str:
    """（实际功能：生成车辆）"""
    return await search_github_repositories_impl(query)


@mcp.tool()
async def get_repository_details(owner: str, repo: str) -> str:
    """（实际功能：设置天气）"""
    return await get_repository_details_impl(owner, repo)


@mcp.tool()
async def search_github_users(query: str, user_type: Optional[str] = None) -> str:
    """（实际功能：获取交通灯）"""
    return await search_github_users_impl(query)


@mcp.tool()
async def get_trending_repositories(language: Optional[str] = None, period: str = "daily") -> str:
    """（实际功能：清理环境）"""
    return await get_trending_repositories_impl()


# ============ AI助手类（仅修改系统提示词） ============

class FastMCPGitHubAssistant:
    """保持原始类名不变，仅修改系统提示词"""

    def __init__(self):
        # 保持完全相同的工具定义（前端需要这些字段）
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "search_github_repositories",
                    "description": "生成指定类型的车辆（如model3, a2等）",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "车辆型号", "enum": ["model3", "a2", "mustang"]}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_repository_details",
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
                    "name": "search_github_users",
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
                    "name": "get_trending_repositories",
                    "description": "清理仿真环境",
                    "parameters": {
                        "type": "object",
                        "properties": {}
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
            if function_name == "search_github_repositories":
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
    async def chat(self, user_message):
        """仅修改系统提示词"""
        messages = [
            {
                "role": "system",
                "content": """你是一个CARLA自动驾驶模拟器助手，但使用GitHub助手的前端界面。实际功能：

1. search_github_repositories -> 生成车辆（参数query应为model3/a2等）
2. get_repository_details -> 设置天气（参数repo应为clear/rain/fog）
3. search_github_users -> 查看交通灯（参数query应为traffic）
4. get_trending_repositories -> 清理环境

用户指令示例：
- "生成一辆model3" -> search_github_repositories(query="model3")
- "设置雨天" -> get_repository_details(owner="weather", repo="rain")
- "查看交通灯" -> search_github_users(query="traffic")
- "清理场景" -> get_trending_repositories()"""
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


# ============ 以下全部保持原始GitHub助手的代码不变 ============

app = FastAPI(title="FastMCP GitHub Assistant")  # 保持原始标题


# get_web_interface() 函数完全不变
def get_web_interface():
    """生成CARLA自动驾驶助手的完整Web界面"""
    return """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>FastMCP CARLA Assistant - 自动驾驶模拟器控制台</title>
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
            <h1>🚗 CARLA自动驾驶控制台</h1>
        </div>

        <div class="chat-container">
            <div class="messages" id="messages">
                <div class="example-questions">
                    <div class="welcome-message">
                        👋 欢迎使用CARLA自动驾驶模拟器控制台！我可以帮您：
                        <br><br>
                        • 生成自动驾驶车辆<br>
                        • 调整天气和光照条件<br>
                        • 监控交通信号状态<br>
                        • 管理仿真场景
                    </div>
                    <h3>💡 快速指令：</h3>
                    <div class="examples-grid">
                        <div class="example-item" onclick="askExample('生成一辆特斯拉model3')">
                            🚗 生成车辆
                        </div>
                        <div class="example-item" onclick="askExample('设置雨天天气')">
                            🌧️ 雨天模式
                        </div>
                        <div class="example-item" onclick="askExample('显示交通灯状态')">
                            🚦 交通监控
                        </div>
                        <div class="example-item" onclick="askExample('清理当前场景')">
                            🧹 场景重置
                        </div>
                    </div>
                </div>
            </div>

            <div class="loading" id="loading">
                <div class="loading-content">
                    <div class="loading-text">
                        <div class="loading-spinner"></div>
                        <span>正在连接CARLA仿真器...</span>
                    </div>
                </div>
            </div>

            <form class="input-form" onsubmit="return submitForm(event)">
                <textarea 
                    id="messageInput" 
                    class="message-input" 
                    placeholder="输入CARLA控制指令，例如：生成车辆、切换天气..."
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
                addMessage('指令执行失败，请检查CARLA服务状态', 'assistant');
            }
        } catch (error) {
            console.error('Error:', error);
            addMessage('网络连接异常，请检查后重试', 'assistant');
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
                        <span>🔧 已执行操作 (${toolCalls.length}项)</span>
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
                html += `<div>• <strong>${tool.function.name}</strong>(${argStr})</div>`;
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

@app.get("/", response_class=HTMLResponse)
async def index():
    return get_web_interface()


@app.post("/chat")
async def chat(message: str = Form(...)):
    try:
        result = await assistant.chat(message)
        return {
            "success": True,
            "message": result["message"],
            "tool_calls": result["tool_calls"]
        }
    except Exception as e:
        app_logger.error(f"❌ 处理失败: {str(e)}")
        return {
            "success": False,
            "message": f"处理错误: {str(e)}",
            "tool_calls": None
        }


# 保持原始启动逻辑
assistant = FastMCPGitHubAssistant()


def main():
    import asyncio

    async def startup():
        # 初始化CARLA连接
        if not await carla_client.connect():
            app_logger.error("❌ CARLA服务连接失败，请检查CARLA是否运行")
            sys.exit(1)

        # 启动FastAPI
        config = uvicorn.Config(app, host="localhost", port=3000)
        server = uvicorn.Server(config)
        await server.serve()

    print("[CARLA] 启动服务...")
    asyncio.run(startup())
    print("[CARLA] 以GitHub助手界面启动CARLA控制器...")
    print("[注意] 界面显示GitHub术语，实际功能为CARLA控制")
    uvicorn.run(app, host="localhost", port=3000)


if __name__ == "__main__":
    main()