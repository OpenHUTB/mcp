# 基于 FastMCP 的人车仿真器交互控制系统

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://python.org)
[![FastMCP](https://img.shields.io/badge/FastMCP-Latest-brightgreen)](https://github.com/jlowin/fastmcp)
[![Unreal Engine](https://img.shields.io/badge/Unreal%20Engine-4.x%2F5.x-orange)](https://www.unrealengine.com)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

基于 MCP (Model Context Protocol) 实现与 HUTB 人车仿真器的大模型交互控制系统，支持通过自然语言控制车辆、无人机、场景编辑和天气等功能。

**作者**: 徐杨杨 | **学校**: 湖南工商大学

---

## 🎯 项目目标

1. **实现 car、air、editor 的常用 API 控制** - 车辆、无人机、场景编辑器的完整控制接口
2. **集成其他模块功能** - 天气控制、传感器管理、极端天气生成、Blueprint 编辑等
3. **打包成可直接部署的服务** - 一键启动，支持 MCP 协议与 AI 助手交互

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                         用户交互层                                    │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                  │
│  │  语音输入    │  │  Web界面    │  │  AI助手     │                  │
│  │  (麦克风)    │  │  (浏览器)   │  │ (Claude等)  │                  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                  │
└─────────┼────────────────┼────────────────┼─────────────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      MCP 服务层 (FastMCP)                            │
│  ┌────────────────────────────┐  ┌────────────────────────────┐     │
│  │     HUTB MCP Server        │  │    Unreal MCP Server       │     │
│  │  (sim/hutb_mcp/)           │  │    (unreal/)               │     │
│  │  ├─ Vehicle Tools          │  │  ├─ Editor Tools           │     │
│  │  ├─ Air Tools              │  │  ├─ Blueprint Tools        │     │
│  │  ├─ Editor Tools           │  │  ├─ Node Tools             │     │
│  │  ├─ Weather Tools          │  │  ├─ Project Tools          │     │
│  │  └─ Sensor Tools           │  │  └─ UMG Tools              │     │
│  └────────────────────────────┘  └────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      仿真器层                                        │
│  ┌────────────────────────────┐  ┌────────────────────────────┐     │
│  │   HUTB/CARLA 仿真器         │  │   Unreal Engine 编辑器      │     │
│  │   (CarlaUE4.exe)           │  │   (UnrealMCP Plugin)       │     │
│  │   Port: 2000               │  │   Port: 55557              │     │
│  └────────────────────────────┘  └────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ✨ 功能模块

### HUTB MCP Server (仿真器控制)

| 模块 | 功能 | 状态 |
|------|------|------|
| **车辆控制** | 生成、销毁、自动驾驶、手动控制、状态获取 | ✅ |
| **无人机控制** | 生成、飞行控制、悬停、降落 | ✅ |
| **编辑器控制** | Actor 管理、场景编辑、地图加载 | ✅ |
| **天气控制** | 天气参数、预设天气、极端天气 | ✅ |
| **传感器管理** | 相机、激光雷达、雷达 | ✅ |

### Unreal MCP Server (引擎编辑)

| 模块 | 功能 | 状态 |
|------|------|------|
| **Editor Tools** | Actor 创建/删除/变换、视口控制 | ✅ |
| **Blueprint Tools** | Blueprint 创建、组件添加、属性设置 | ✅ |
| **Node Tools** | 蓝图节点图编辑、事件/函数节点 | ✅ |
| **Project Tools** | 输入映射配置 | ✅ |
| **UMG Tools** | UI Widget 创建和编辑 | ✅ |

---

## 📁 项目结构

```
mcp-main/
├── sim/                        # HUTB MCP 服务器 (仿真器控制)
│   ├── hutb_mcp/              # 核心包
│   │   ├── server.py          # FastMCP 服务器
│   │   ├── connection.py      # CARLA 连接管理
│   │   └── tools/             # 工具模块
│   │       ├── vehicle_tools.py   # 车辆控制
│   │       ├── air_tools.py       # 无人机控制
│   │       ├── editor_tools.py    # 编辑器控制
│   │       ├── weather_tools.py   # 天气控制
│   │       └── sensor_tools.py    # 传感器管理
│   └── pyproject.toml         # 项目配置
│
├── unreal/                     # Unreal MCP 服务器 (引擎编辑)
│   ├── unreal_mcp_server.py   # FastMCP 服务器
│   ├── tools/                 # 工具模块
│   │   ├── editor_tools.py    # 编辑器工具
│   │   ├── blueprint_tools.py # Blueprint 工具
│   │   ├── node_tools.py      # 节点图工具
│   │   ├── project_tools.py   # 项目工具
│   │   └── umg_tools.py       # UI 工具
│   └── scripts/               # 测试脚本
│
├── MCPGameProject/             # Unreal Engine 插件
│   └── Plugins/UnrealMCP/     # C++ 插件源码
│
├── llm/                        # AI 助手 Web 界面
│   ├── main_ai.py             # Web 服务入口
│   └── src/                   # 前端源码
│
├── Docs/                       # 文档
│   └── Tools/                 # API 文档
│
├── model/                      # 人形机器人模型
└── hutb.bat                   # 一键启动脚本
```

---

## 🚀 快速开始

> **🎉 项目已升级为虚拟环境 (venv) 管理！** 无需 Conda，更轻量、更简单！

### 方式一：一键启动（推荐）⭐

#### 步骤 1: 创建虚拟环境

```cmd
setup_venv.bat
```

首次运行会自动:
- 创建 Python 虚拟环境 `env.UE4-hutb`
- 安装所有依赖包
- 配置 CARLA Python API (如果存在)

#### 步骤 2: 启动项目

```cmd
hutb.bat
```

自动完成:
- 激活虚拟环境
- 检查依赖
- 启动 CARLA 仿真器 (如果存在)
- 启动 AI Web 界面 (http://localhost:3000)

### 方式二：手动安装

#### 1. HUTB MCP Server (仿真器控制)

```cmd
# 创建并激活虚拟环境
python -m venv env.UE4-hutb
env.UE4-hutb\Scripts\activate.bat

# 安装 HUTB Python API
pip install D:\hutb\PythonAPI\carla\dist\hutb-2.9.16-cp310-cp310-win_amd64.whl

# 安装项目依赖
cd sim
pip install -e .

# 配置环境变量
copy .env.example .env

# 启动服务（先启动 CarlaUE4.exe）
python -m hutb_mcp
```

#### 2. AI Web 界面

```cmd
# 激活虚拟环境
env.UE4-hutb\Scripts\activate.bat

# 安装依赖
cd llm
pip install -r requirements.txt

# 配置 API 密钥
copy .env.example .env
# 编辑 .env 文件，填入 DEEPSEEK_API_KEY

# 启动 Web 界面
python main_ai.py
```

#### 3. Unreal MCP Server (引擎编辑)

```cmd
# 激活虚拟环境
env.UE4-hutb\Scripts\activate.bat

# 进入目录
cd unreal

# 安装依赖
pip install -e .

# 启动服务（先启动 Unreal Editor 并加载 UnrealMCP 插件）
python unreal_mcp_server.py
```

### 📖 详细文档

- [🚀 快速启动指南](QUICKSTART.md) - 最简单的入门方式
- [⚙️ 虚拟环境设置](VENV_SETUP.md) - 详细的环境配置说明

---

## 🔧 MCP 客户端配置

### HUTB MCP (仿真器控制)

```json
{
  "mcpServers": {
    "hutb-mcp": {
      "command": "python",
      "args": ["-m", "hutb_mcp"],
      "cwd": "D:/path/to/mcp-main/sim",
      "env": {
        "HUTB_HOST": "localhost",
        "HUTB_PORT": "2000"
      }
    }
  }
}
```

### Unreal MCP (引擎编辑)

```json
{
  "mcpServers": {
    "unreal-mcp": {
      "command": "python",
      "args": ["unreal_mcp_server.py"],
      "cwd": "D:/path/to/mcp-main/unreal"
    }
  }
}
```

---

## 📖 API 文档

### 车辆控制 (HUTB MCP)

```python
get_vehicle_blueprints()                    # 获取可用车辆蓝图
spawn_vehicle(blueprint_id, location, rotation)  # 生成车辆
destroy_vehicle(vehicle_id)                 # 销毁车辆
set_vehicle_autopilot(vehicle_id, enabled)  # 设置自动驾驶
apply_vehicle_control(vehicle_id, throttle, steer, brake)  # 手动控制
get_vehicle_state(vehicle_id)               # 获取车辆状态
```

### 天气控制 (HUTB MCP)

```python
get_weather()                               # 获取当前天气
set_weather_preset(preset)                  # 设置预设天气 (clear/cloudy/rainy/foggy/stormy)
set_extreme_weather(weather_type)           # 设置极端天气 (heavy_rain/dense_fog/blizzard/sandstorm)
set_time_of_day(hour, minute)               # 设置时间
```

### 无人机控制 (HUTB MCP)

```python
spawn_drone(location, rotation)             # 生成无人机
get_drone_state(drone_id)                   # 获取状态
set_drone_destination(drone_id, destination)  # 设置目标
drone_hover(drone_id)                       # 悬停
drone_land(drone_id)                        # 降落
```

### 编辑器控制 (Unreal MCP)

```python
get_actors_in_level()                       # 获取场景 Actor
spawn_actor(name, type, location, rotation) # 生成 Actor
delete_actor(name)                          # 删除 Actor
set_actor_transform(name, location, rotation, scale)  # 设置变换
create_blueprint(name, parent_class)        # 创建 Blueprint
compile_blueprint(blueprint_name)           # 编译 Blueprint
```

---

## 🧪 测试场景

- **Town10** (优先) - 主要测试场景
- **Town01** - 备选测试场景

---

## 📚 参考资料

- [FastMCP 框架](https://github.com/jlowin/fastmcp)
- [CARLA Python API](https://carla.readthedocs.io/en/latest/python_api/)
- [HUTB 文档](https://openhutb.github.io/doc/python_api/)
- [Unreal MCP](https://github.com/chongdashu/unreal-mcp)

---

## 📄 许可证

MIT License
