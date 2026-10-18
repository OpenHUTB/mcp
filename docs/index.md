title: 首页

# HUTB 模拟器 MCP —— 场景生成与 CARLA 执行

基于 MCP 实现大模型与具身人、无人车、无人机的交互。本部分文档介绍
**场景生成、路网生成、CARLA 执行与实采轨迹批量合成**四组能力。

![cut_in 场景在 CARLA 中执行](https://openhutb.github.io/mcp/img/osc_cut_in_t2.png)

---

## 1. 这四组能力解决什么问题

| 能力 | 解决的问题 |
|---|---|
| 场景生成 | 原来只会生成"一辆车匀速直行"，没有中间表示，无法批量与复现 |
| 路网生成 | 原来唯一的 `.xodr` 出自 netconvert 规则网格，且没有几何数据 |
| CARLA 执行 | 生成的 `.xosc` 无法播放，回答不了"跑起来是什么样" |
| 轨迹批量合成 | 少量实采轨迹难以扩增，缺少分布校验手段 |

## 2. 功能总览

| 功能 | MCP 工具 | 说明 |
|---|---|---|
| 场景模板生成 | `generate_scenario_from_template` | 按模板批量生成 `.xosc`，参数区间内随机 |
| 列出场景模板 | `list_scenario_templates` | 返回 8 个模板及参数说明 |
| 场景校验 | `validate_scenario` | 结构往返解析 + 语义断言 |
| 路网生成 | `generate_rich_xodr` | 生成 7 种布局的 `.xodr` |
| 列出路网布局 | `list_road_layouts` | 返回布局名与默认参数 |
| 轨迹导入 | `import_trajectory` | CSV / FCD / JSON 三种格式 |
| 轨迹批量合成 | `synthesize_trajectory_dataset` | 增强算子扩增，带 manifest |
| 分布对比 | `compare_trajectory_distribution` | KS 距离与 Wasserstein |
| 导出 SUMO 需求 | `export_sumo_demand` | `.rou.xml` |
| 路网进 CARLA | `run_road_in_carla` | 加载、布景、截图 |
| 场景进 CARLA | `run_scenario_in_carla` | 播放 `.xosc` 并截图 |
| CARLA 状态 | `list_carla_maps` | 版本、当前地图、地图列表 |

## 3. 文档导航

| 页面 | 内容 |
|---|---|
| [New Features](new-features.md) | 新增功能记录：能力、用法、示例、效果 |
| [环境配置](setup.md) | 软硬件版本、依赖安装、API Key、CARLA 启动 |
| [场景生成](scenario.md) | 8 个模板、参数、批量生成与校验 |
| [路网生成](road.md) | 7 种布局、默认参数、几何校验 |
| [CARLA 执行](carla.md) | 路网加载、场景播放、截图 |
| [轨迹合成](trajectory.md) | 轨迹导入、增强算子、分布对比、SUMO 导出 |
| [效果展示](gallery.md) | 动图与截图 |
| [实测数据与限制](limits.md) | 各环节指标、已知边界 |

## 4. 快速开始

```bash
# 1. 安装依赖
pip install -r llm/requirements.txt

# 2. 启动 CARLA（另一个终端）
CarlaUE4.exe -RenderOffScreen -quality-level=Low -nosound -carla-port=2000

# 3. 启动 MCP 服务
mcp.bat
```

浏览器打开 `http://127.0.0.1:3000`，然后直接说：

```text
生成 20 个前车急刹的场景
生成一个十字路口路网并加载进 CARLA 看看
在 CARLA 里播放 cut_in 场景，第 2 秒和第 5 秒各截一张图
```
