title: 环境配置

# 环境配置

## 1. 软硬件版本

| 项目 | 版本 | 说明 |
|---|---|---|
| Python | 3.10 | CARLA PythonAPI 的 wheel 只提供到 3.10 |
| CARLA | 0.9.16 | 对应 `carla==0.9.16` |
| 操作系统 | Windows 10 / 11 | CARLA 服务端只在这里跑 |
| 显卡 | 集显可用 | 测试在 AMD Radeon 880M 集显 + 低画质下完成 |

## 2. 安装 Python 依赖

```bash
cd mcp
pip install -r llm/requirements.txt
```

本次新增的关键依赖：

| 包 | 版本 | 用途 |
|---|---|---|
| `scenariogeneration` | >=0.16.0 | OpenSCENARIO / OpenDRIVE 对象模型 |
| `lxml` | >=5.0.0 | XSD 校验 |
| `carla` | 0.9.16 | CARLA 客户端 |

> `scenariogeneration` 在库内是**延迟导入**的。缺失时不影响 MCP 服务启动，
> 只有调用场景/路网相关工具时才报错。

## 3. 配置 DeepSeek API Key

在项目根目录创建 `.env`：

```text
DEEPSEEK_API_KEY=sk-xxxxxxxxxxxxxxxx
```

## 4. 启动 CARLA

无界面模式（省显存，适合集显/服务器）：

```bat
cd D:\CARLA_0.9.16
CarlaUE4.exe -RenderOffScreen -quality-level=Low -nosound -carla-port=2000
```

启动参数：

| 参数 | 作用 |
|---|---|
| `-RenderOffScreen` | 不弹窗口，离屏渲染（截图仍可用） |
| `-quality-level=Low` | 降低画质，集显可用 |
| `-nosound` | 关闭音频，减少资源占用 |
| `-carla-port=2000` | 服务端口，默认 2000 |

成功标志：进程 `CarlaUE4-Win64-Shipping` 存在，端口 2000 处于监听状态。

可以在对话里问「CARLA 连上了吗」，会调用 `list_carla_maps` 返回服务端版本与可用地图列表。

## 5. 启动 MCP 服务

```bat
mcp.bat
```

或手动：

```bash
python llm/main_ai.py
```

浏览器打开 `http://127.0.0.1:3000`。页面顶部有「场景快捷按钮」面板，
可以不输入文字直接执行场景。

## 6. 验证安装

依次执行下面三个对话，都返回成功即环境就绪：

| 对话 | 预期结果 |
|---|---|
| `列出所有场景模板` | 返回 8 个模板 |
| `列出所有路网布局` | 返回 7 种布局 |
| `CARLA 连上了吗` | 返回版本 0.9.16 与 13 个内置地图 |
