title: New Features

# New Features

按时间倒序记录新增功能。每个功能都给出：**能力说明、调用方式、示例、效果**。

图片全部使用绝对链接。

---

## 2026-10 · OpenSCENARIO 轨迹跟随

### 能力

支持 OpenSCENARIO 的 `FollowTrajectoryAction`：让实体沿一条任意形状的
折线轨迹行驶，轨迹点带时间戳，播放器按时间插值并用纯追踪控制转向。

这条能力把**轨迹模块**和**场景模块**接了起来——实采或合成的轨迹现在可以直接
变成场景里的运动。

### 之前的状态

`osc_builder` 对 `follow_trajectory` 直接抛异常：

```text
OSCBuildError: follow_trajectory 需要先由 trajectory 模块转换为 Polyline，暂未接线
```

### 用法

方式一，直接给点（世界坐标，米 / 秒）：

```python
from scenario_ir import BehaviorSpec, BehaviorKind

BehaviorSpec(
    kind=BehaviorKind.FOLLOW_TRAJECTORY,
    entity_ref="ego",
    trajectory_points=[
        [0.0, 50.0, 1.8],
        [1.2, 62.0, 5.0],
        [2.3, 85.0, 5.0],
        [3.1, 100.0, 1.8],
    ],
)
```

方式二，读轨迹文件（JSON 或 CSV，需要 `t` / `x` / `y` 三列）：

```python
BehaviorSpec(
    kind=BehaviorKind.FOLLOW_TRAJECTORY,
    entity_ref="ego",
    trajectory_file="output/synthetic/track_0007.csv",
)
```

对话方式：

```text
用 evasive_swerve 模板生成一个紧急避让场景
在 CARLA 里播放这个场景，第 2 秒和第 4 秒各截一张图
```

### 生成的 OpenSCENARIO

```xml
<PrivateAction>
  <RoutingAction>
    <FollowTrajectoryAction>
      <TrajectoryRef>
        <Trajectory name="ego_traj" closed="false">
          <Shape><Polyline>
            <Vertex time="0.0"><Position><WorldPosition x="50.0" y="1.8" z="0.0"/></Position></Vertex>
            <Vertex time="1.2"><Position><WorldPosition x="62.0" y="5.0" z="0.0"/></Position></Vertex>
            ...
          </Polyline></Shape>
        </Trajectory>
      </TrajectoryRef>
      <TrajectoryFollowingMode followingMode="follow"/>
    </FollowTrajectoryAction>
  </RoutingAction>
</PrivateAction>
```

### 新模板：紧急避让 `evasive_swerve`

| 项目 | 值 |
|---|---|
| 实体 | 自车、静止障碍车、对向车 |
| 轨迹点 | 5 个（直行 → 向左避让 → 越过障碍 → 回正 → 继续直行） |
| 参数 | `ego_speed` / `obstacle_gap` / `lateral` |

![紧急避让](https://openhutb.github.io/mcp/img/osc_evasive_swerve_t4.png)
<div align="center"><i>evasive_swerve 在 t=4.0 s：已越过障碍并回到原车道</i></div>

实测轨迹（t 为动作触发后的相对时间）：

```text
t=1.0  (57.2, 3.5)  yaw= 16.0°   开始向左避让
t=2.0  (70.1, 4.6)  yaw=  1.3°   在左车道直行
t=3.0  (89.6, 3.3)  yaw= -7.5°   越过障碍后回正
t=4.0  (110.9, 1.9) yaw= -1.0°   回到原车道
```

### 踩到的两个坑

**1. 目标落在车后导致打转**

轨迹第一点就是车辆初始位置，如果触发延迟到 1 秒，车已开出 8 米，
插值出的目标点落在车后，纯追踪转向直接饱和，车原地转 75° 后停住。

修法：触发改成立即（`TIME > 0.0`），并在控制器里加一条——
目标纵向分量 ≤ 0.5 m 时不打方向，保持直行等下一帧。

**2. 恒速模式与转向控制打架**

实体在转向类动作触发前用的是 `enable_constant_velocity`，触发后改用
`apply_control`，但不退出恒速模式时恒速会一直覆盖转向。
现在进入变道 / 偏移 / 横向间距 / 轨迹跟随前统一调用
`disable_constant_velocity()`。

---

## 2026-10 · OpenSCENARIO 相对速度 / 碰撞 / TTC

### 能力

| 类型 | 条件/动作 | 实现 |
|---|---|---|
| 条件 | `RelativeSpeedCondition` | 自身速度 − 参照实体速度 |
| 条件 | `TimeToCollisionCondition` | TTC = 间距 / 接近速度 |
| 条件 | `CollisionCondition` | 包围球近似，中心距 ≤ 2.6 m |
| 动作 | `RelativeSpeedAction` | 目标速度 = 参照速度 + delta，每帧跟随 |
| 动作 | `LateralDistanceAction` | 保持横向间距，比例控制打方向 |

### 用法

```python
# 条件：TTC 低于 3 秒时触发
TriggerSpec(kind=TriggerKind.TIME_TO_COLLISION, value=3.0,
            rule=Rule.LESS_THAN, entity_ref="lead",
            triggering_entity="ego")

# 动作：保持比前车快 4 m/s
BehaviorSpec(kind=BehaviorKind.SPEED, entity_ref="ego",
             target_speed=4.0, relative_speed=True,
             speed_entity_ref="lead")
```

### 修掉的语义错误

`osc_builder` 原来无论传什么 `entity_ref`，TTC 都输出
`WorldPosition(0, 0, 0)`，等于在算"到世界原点的碰撞时间"。
现在正确输出 `<EntityRef entityRef="lead"/>`。

### 新模板

| 模板 | 场景 | 触发 / 动作 |
|---|---|---|
| `close_follow` | 高速接近慢车，TTC 低于阈值时减速 | `TimeToCollisionCondition` + 线性速度动作 |
| `overtake` | 接近 → 变道 → 相对速度超越 → 回原车道 | `DistanceCondition` + `RelativeSpeedAction` + 连续变道 |

| 近距跟驰 | 超车 |
|---|---|
| ![close_follow](https://openhutb.github.io/mcp/img/osc_close_follow_t8.png) | ![overtake](https://openhutb.github.io/mcp/img/osc_overtake_t8.png) |
| `close_follow` t=8.0 s | `overtake` t=8.0 s |

---

## 2026-10 · 全部模板实测

11 个模板在 CARLA 0.9.16 中逐个播放，**未支持项合计 0**：

| 模板 | 实体 | 触发事件 |
|---|---|---|
| `lead_stationary` | 2 | 1/1 |
| `wrong_way` | 2 | 2/2 |
| `crossing_hazard` | 2 | 2/2 |
| `junction_light` | 3 | 3/3 |
| `lead_brake` | 3 | 3/4 |
| `low_overlap` | 3 | 4/4 |
| `unprotected_turn` | 3 | 4/4 |
| `cut_in` | 4 | 4/5 |
| `close_follow` | 2 | 1/1 |
| `overtake` | 3 | 3/3 |
| `evasive_swerve` | 3 | 2/2 |

合计 **29/31 事件触发**。未触发的 2 个是条件本身未满足，不是播放器缺陷。

---

## 相关页面

| 页面 | 内容 |
|---|---|
| [场景生成](scenario.md) | 模板、参数、批量生成 |
| [CARLA 执行](carla.md) | 路网加载、场景播放、支持范围 |
| [效果展示](gallery.md) | 动图与截图 |
| [实测数据与限制](limits.md) | 指标与已知边界 |
