title: 效果展示

# 效果展示

以下全部由本项目生成的 `.xodr` 与 `.xosc` 在 CARLA 0.9.16 中渲染，
**不是导入的现成地图或示意图**。

## 1. 场景执行

![cut_in 场景播放](https://openhutb.github.io/mcp/img/scenario_cut_in.gif)
<div align="center"><i>cut_in（危险切入）播放 8 秒：后车跟随、切入车变道（16 帧，0.5 秒/帧）</i></div>

## 2. 路网布局

![路网布局轮播](https://openhutb.github.io/mcp/img/road_layouts.gif)
<div align="center"><i>6 种路网布局在 CARLA 中的实际渲染（动图）</i></div>

### 2.1 路口

| 十字路口 | 丁字路口 |
|---|---|
| ![十字路口](https://openhutb.github.io/mcp/img/carla_cross_junction.png) | ![丁字路口](https://openhutb.github.io/mcp/img/carla_t_junction.png) |
| 4 条臂 + 菱形路口环，6 辆车已布景 | 东西横杠 + 朝南竖杠 |

### 2.2 曲线路段

| 高速弯道 | S 弯 |
|---|---|
| ![高速弯道](https://openhutb.github.io/mcp/img/carla_highway_curve.png) | ![S 弯](https://openhutb.github.io/mcp/img/carla_s_curve.png) |
| 缓和曲线 + 圆弧 + 缓和曲线 | 两段反向曲线 |

### 2.3 直路与匝道

| 匝道合流 | 直路 |
|---|---|
| ![匝道合流](https://openhutb.github.io/mcp/img/carla_on_ramp_merge.png) | ![直路](https://openhutb.github.io/mcp/img/carla_straight.png) |
| 车道数 1→2（`s=0 左1右1 → s=80 左1右2`） | 4 车道 |

## 3. 场景截图

| 前车急刹 | 无保护通行 |
|---|---|
| ![前车急刹](https://openhutb.github.io/mcp/img/osc_lead_brake_t4.png) | ![无保护通行](https://openhutb.github.io/mcp/img/osc_unprotected_turn_t4.png) |
| `lead_brake` t=4.0 s：前车已停住 | `unprotected_turn` t=4.0 s |

| 低重叠目标 | 路口信号 |
|---|---|
| ![低重叠](https://openhutb.github.io/mcp/img/osc_low_overlap_t4.png) | ![路口信号](https://openhutb.github.io/mcp/img/osc_junction_light_t4.png) |
| `low_overlap` t=4.0 s | `junction_light` t=4.0 s |

| 危险切入 |
|---|
| ![危险切入](https://openhutb.github.io/mcp/img/osc_cut_in_t2.png) |
| `cut_in` t=2.0 s：三辆车在直路上，切入车已触发变道 |

| 近距跟驰 | 超车 |
|---|---|
| ![close_follow](https://openhutb.github.io/mcp/img/osc_close_follow_t8.png) | ![overtake](https://openhutb.github.io/mcp/img/osc_overtake_t8.png) |
| `close_follow` t=8.0 s：TTC 触发后减速跟驰 | `overtake` t=8.0 s：变道后以相对速度超越 |

| 紧急避让 |
|---|
| ![evasive_swerve](https://openhutb.github.io/mcp/img/osc_evasive_swerve_t4.png) |
| `evasive_swerve` t=4.0 s：沿 S 形轨迹绕过静止障碍后回正 |

## 4. 生成结果对照

### 4.1 路网

| 布局 | 文件大小 | 道路数 | 几何段数 | 生成点 |
|---|---|---|---|---|
| 直路 | 2626 B | 1 | 1 | 2 |
| 高速弯道 | 3071 B | 1 | 1 | 2 |
| S 弯 | 3694 B | 1 | 1 | 2 |
| 十字路口 | 22708 B | 12 | 28 | 12 |
| 丁字路口 | 17023 B | 9 | 21 | 9 |
| 匝道合流 | 4195 B | 1 | 1 | 4 |
| 匝道分流 | 3496 B | 1 | 1 | 3 |

### 4.2 场景

| 模板 | 实体 | 触发事件 | 未支持项 |
|---|---|---|---|
| `lead_stationary` | 2 | 1/1 | 0 |
| `wrong_way` | 2 | 2/2 | 0 |
| `crossing_hazard` | 2 | 2/2 | 0 |
| `junction_light` | 3 | 3/3 | 0 |
| `lead_brake` | 3 | 3/4 | 0 |
| `low_overlap` | 3 | 4/4 | 0 |
| `unprotected_turn` | 3 | 4/4 | 0 |
| `cut_in` | 4 | 4/5 | 0 |
