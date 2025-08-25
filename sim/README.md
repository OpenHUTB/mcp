# HutbMCP - Unreal 模型上下文协议集成

## 概述

HutbMCP 是一个 Python 软件包，提供将 Hutb 与模型上下文协议 (Model Context Protocol, MCP) 集成的接口。它允许用户与 Hutb 的仿真环境交互，并使用 MCP 管理模型上下文。该软件包旨在通过在 Hutb 和 MCP 之间提供无缝连接来促进机器人应用程序的开发。


## 特性
- [x] 与 Hutb 仿真环境集成
- [x] 支持模型上下文协议 (MCP)
- [x] 用于管理模型上下文的易于使用的 API
- [ ] 支持各种 Hutb 传感器和参与者
- [ ] 运行模拟和收集数据的能力


## 安装

要安装 HutbMCP，您可以使用 uv。请确保您已安装 Python 3.10 或更高版本。

```bash
pip install D:/hutb/PythonAPI/carla/dist/hutb-2.9.16-cp310-cp310-win_amd64.whl
# 启动 WindowsNoEditor/CarlaUE4.exe
python main.py
```

#### 警告 ⚠️：该项目处于初始阶段，希望贡献者帮助开发。

## 参考

* [MuJoCo-Unreal-Engine-Plugin](https://github.com/oneclicklabs/MuJoCo-Unreal-Engine-Plugin) ，其他 [使用 ROS 与 WebSockets 连接实现机器人可视化的 Unreal 插件](https://github.com/HoangGiang93/URoboViz)
