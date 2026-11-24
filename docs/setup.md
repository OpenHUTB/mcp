title: 环境配置

# 环境配置

## 1. 软硬件版本

| 项目 | 版本 | 说明 |
|---|---|---|
| Python | 3.10 | HUTB / CARLA PythonAPI 的 wheel 只提供到 3.10 |
| HUTB 模拟器 | 2.10.1 | 同步至 CARLA 0.9.16 API，客户端为 `hutb-2.10.1` wheel |
| 操作系统 | Windows 10 / 11 | 模拟器服务端只在这里跑 |
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
| `carla`（模块） | 2.10.1 | HUTB 客户端，来自模拟器发行包 `PythonAPI/carla/dist/hutb-*.whl` |

> CARLA 客户端不由 `requirements.txt` 安装：官方 `carla` 与 HUTB 的 `hutb`
> 两个包都提供 `carla` 模块，混装会互相覆盖。请按第 4 节用发行包自带的 wheel 安装。

> `scenariogeneration` 在库内是**延迟导入**的。缺失时不影响 MCP 服务启动，
> 只有调用场景/路网相关工具时才报错。

## 3. 配置 DeepSeek API Key

在项目根目录创建 `.env`：

```text
DEEPSEEK_API_KEY=sk-xxxxxxxxxxxxxxxx
```

## 4. 安装 HUTB 客户端并启动模拟器

### 4.1 安装客户端

```bat
pip uninstall -y carla
pip install D:\hutb\PythonAPI\carla\dist\hutb-2.10.1-cp310-cp310-win_amd64.whl
```

`pip list` 中出现 `hutb 2.10.1` 即为成功。代码通过分发名识别 HUTB
（`carla_runner.hutb_version()`），据此决定是否给地图追加 `?GAME=CARLA`。

### 4.2 启动

无界面模式（省显存，适合集显/服务器）：

```bat
cd D:\hutb
CarlaUE4.exe -RenderOffScreen -nosound -carla-rpc-port=2000
```

启动参数：

| 参数 | 作用 |
|---|---|
| `-RenderOffScreen` | 不弹窗口，离屏渲染（截图仍可用） |
| `-nosound` | 关闭音频，减少资源占用 |
| `-carla-rpc-port=2000` | 服务端口，默认 2000 |

不要额外加 `-quality-level=Low`。HUTB 2.10.1 只烘焙了默认画质的着色器，
Low 画质所需的着色器要运行时编译，实测会在启动后 20–40 秒内以
`Shader compilation failures are Fatal` 退出（切到 Town03 时同样会崩）。

HUTB 的 `settings.json` 默认 `SimMode: Multirotor`（无人机模式）。车辆场景
不需要手工改它：加载内置地图时 `carla_runner.map_url()` 会自动追加
`?GAME=CARLA`，把世界切到 CARLA 车辆模式；不带该选项重载地图会按无人机
模式重建并崩溃。

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
| `列出所有场景模板` | 返回 12 个模板 |
| `列出所有路网布局` | 返回 7 种布局 |
| `CARLA 连上了吗` | 返回服务端版本（HUTB 为提交哈希，如 `3ea2130ed`）与内置地图数量 |

## 7. 常见问题

### 视角切换提示成功，但画面没有跟着变

日志里能看到 `视角已设置`、`视角跟随已启用`，模拟器窗口却还是原来的机位。
HUTB 默认以无人机模式（`settings.json` 的 `SimMode: Multirotor`）启动，
会持续覆盖客户端设置的相机。加载内置地图时本项目会自动追加 `?GAME=CARLA`
切到车辆模式（见 `llm/src/carla_runner.py` 的 `map_url()`），HUTB 2.10.1
下已实测正常。若仍不生效，请确认地图是通过 `run_scenario_in_carla(map_name=...)`
加载的，而不是在模拟器界面里手工切图。

切换视角后代码会回读一次相机位置做自检，只要相机没有停在目标附近，
对话里会直接给出这条提示，不会再静默成功。

### 模拟器启动后立刻退出

- 确认已安装 VC++ 2015-2022 x64 运行库（发行包根目录带 `vc_redist.x64.exe`）。
- Windows 11 的 **Smart App Control** 会拦截无签名的 `CarlaUE4-Win64-Shipping.exe`
  （事件日志 `Microsoft-Windows-CodeIntegrity/Operational` 中为 3077/3118）。
  需要在「Windows 安全中心 → 应用和浏览器控制 → Smart App Control」中关闭，
  该操作不可逆，请自行评估。
- HUTB 发行包是分片压缩包；用系统 `tar` 解压 20 GB 以上的包可能中途静默停止，
  建议用 7-Zip 或 Python `zipfile`（加 `\\?\` 长路径前缀）解压。

### 模拟器运行十几秒到几十秒后退出

崩溃转储里写着 `LowLevelFatalError [File:Unknown] [Line: 139]
Shader compilation failures are Fatal.`，说明进程是按需编着色器时挂掉的。
最常见的原因是启动参数带了 `-quality-level=Low`：该画质的着色器没有烘焙
进发行包。去掉这个参数、用默认画质启动即可（集显也能跑，实测 AMD Radeon
880M 正常）。崩溃转储位于
`%LOCALAPPDATA%\CarlaUE4\Saved\Crashes\`，可用其中的
`CrashContext.runtime-xml` 确认原因。

