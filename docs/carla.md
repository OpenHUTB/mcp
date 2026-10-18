title: CARLA 执行

# CARLA 执行

把生成的路网与场景真正跑进 CARLA：加载 `.xodr` 建成可驾驶世界、
播放 `.xosc` 驱动车辆、并截图。

## 1. 工具参数

### 1.1 `run_road_in_carla`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `layout` | string | `straight` | 路网布局 |
| `params_json` | string | `{}` | 覆盖默认参数 |
| `vehicle_count` | int | 5 | 布景车辆数 |
| `capture` | bool | True | 是否截图 |
| `out_dir` | string | `llm/output/carla/shots/<layout>` | 截图目录 |
| `host` | string | `127.0.0.1` | CARLA 地址 |
| `port` | int | 2000 | CARLA 端口 |

### 1.2 `run_scenario_in_carla`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `template` | string | `lead_brake` | 场景模板（与 `xosc_path` 二选一） |
| `xosc_path` | string | 空 | 直接指定 `.xosc` 文件 |
| `xodr_layout` | string | `straight` | 承载场景的路网 |
| `duration` | float | 0 | 播放时长；0 表示用场景自身 `StopTrigger` |
| `capture_times` | string | `2,5` | 截图时刻，逗号分隔 |
| `out_dir` | string | `llm/output/xosc_run/shots/<template>` | 截图目录 |

### 1.3 `list_carla_maps`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `host` / `port` | string / int | `127.0.0.1` / 2000 | CARLA 地址 |

返回服务端版本、当前地图、可用地图列表。

## 2. 对话示例

```text
CARLA 连上了吗
把十字路口路网加载进 CARLA 并截图
在 CARLA 里播放 cut_in 场景，第 2 秒和第 5 秒各截一张图
把 wrong_way 场景跑 6 秒看看效果
```

## 3. 代码示例

```python
import carla_runner as CR
from scenario_templates import build
from osc_builder import write_scenario
from xosc_player import parse_xosc, play

# 1. 连接并加载路网
client = CR.connect()
world = CR.load_xodr_world(client, "output/straight.xodr")
CR.clear_actors(world)

# 2. 准备场景
spec = build("cut_in", seed=1)
write_scenario(spec, "output/cut_in.xosc")
sc = parse_xosc("output/cut_in.xosc")

# 3. 播放并截图
res = play(world, sc, duration=8.0, capture_at=[2.0, 5.0],
           out_dir="output/shots")
print(res["events_fired"], "/", res["events_total"])
```

## 4. 路网加载流程

```mermaid
graph LR
    A[.xodr] --> B[generate_opendrive_world]
    B --> C[可驾驶世界]
    C --> D[生成点布景]
    D --> E[相机取图]
```

`load_xodr_world()` 会先把文件内容读出来，检查 `<planView/>` 是否有几何，
再交给 `Client.generate_opendrive_world()`。加载耗时约 2 秒。

## 5. 播放器支持范围

### 5.1 支持

| 类型 | 条件/动作 | 说明 |
|---|---|---|
| 实体 | `Vehicle` / `Pedestrian` | |
| 初始化 | `TeleportAction` | `LanePosition` / `WorldPosition` |
| 速度动作 | `SpeedAction` | step 与 linear 动力学 |
| 跟驰 | `LongitudinalDistanceAction` | P 控制维持间距 |
| 变道 | `LaneChangeAction` | 纯追踪转向控制 |
| 横向偏移 | `LaneOffsetAction` | |
| 横向间距 | `LateralDistanceAction` | 保持与参照实体的横向间距 |
| 相对速度 | `RelativeSpeedAction` | 目标速度 = 参照速度 + delta |
| 轨迹跟随 | `FollowTrajectoryAction` | 折线轨迹 + 时间戳，纯追踪 + 前瞻 |
| 时间触发 | `SimulationTimeCondition` | |
| 时距触发 | `TimeHeadwayCondition` | 间距 / 自身车速 |
| 距离触发 | `DistanceCondition` | 到指定位置 |
| 速度触发 | `SpeedCondition` | |
| 静止触发 | `StandStillCondition` | |
| 到达触发 | `ReachPositionCondition` | 按 `tolerance` |
| 相对速度触发 | `RelativeSpeedCondition` | 自身速度 − 参照实体速度 |
| TTC 触发 | `TimeToCollisionCondition` | 间距 / 接近速度 |
| 碰撞触发 | `CollisionCondition` | 包围球近似，中心距 ≤ 2.6 m |

边沿检测支持 `rising` / `falling` / `risingOrFalling` / `none` 四种 `ConditionEdge`。

### 5.2 不支持

以下会记入 `unsupported` 并明确返回，**不会静默跳过**：

| 类别 | 项目 |
|---|---|
| 条件 | `AccelerationCondition`、`OffroadCondition`、`EndOfRoadCondition` |
| 结构 | `Story` 分支、`Catalog`、`TrafficSignalControllerAction` |

## 6. 返回结果

`run_scenario_in_carla` 返回执行统计：

```text
✅ OpenSCENARIO 场景已在 CARLA 中播放

场景：lead_brake.xosc
实体 3 个，成功布景 3 个
时长 6.00s（120 帧）
事件 4 个，已触发 3 个
已触发的动作：
   t=0.00s ego   ego_longitudinal_distance_ev follow=15.00
   t=0.05s lead  lead_speed_ev              speed=13.00
   t=2.05s lead  lead_speed_ev              speed=0.00
未执行（超出播放器范围）：
   - 事件 ego_standstill_ev 的条件 StandStillCondition 未满足
截图：
   - output/shots/t02.00.png
   - output/shots/t04.00.png
```

## 7. 8 个模板的实测结果

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

`lead_brake` 与 `cut_in` 各有一个事件未触发，是**条件本身未满足**
（前车没在时限内完全静止 / 时距没降到阈值），不是播放器缺陷。

效果见[效果展示](gallery.md)。

## 8. 注意事项

| 现象 | 原因 | 处理 |
|---|---|---|
| 加载一直超时 | `.xodr` 没有几何 | 检查 `inspect_road()` 的 `valid` |
| `_episode != nullptr` 断言 | 一个进程内连续切世界 | 每种布局跑独立进程 |
| 截图是空路 | 相机复用了旧帧 | 每次截图重新创建相机 |
