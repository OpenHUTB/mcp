title: 实测数据与限制

# 实测数据与已知限制

## 1. 实测数据

测试环境：Windows 11 + Python 3.10 + CARLA 0.9.16 + AMD Radeon 880M 集显。

### 1.1 场景

| 项目 | 结果 |
|---|---|
| OSC 生成器往返解析 | 4 场景 × 4 版本 = **16/16 通过** |
| 场景模板编译与校验 | **8/8 通过**，同 seed 指纹一致 |
| 批量生成 50 个 | **6.86 s**（要求 < 60 s） |
| 批量生成 80 个混合 | **10.89 s** |
| 重复场景 | **0** |

### 1.2 路网与 CARLA

| 项目 | 结果 |
|---|---|
| 布局生成 | **7/7** 通过几何校验 |
| CARLA 路网加载 | **7/7** 成功，每种约 7 秒（含 2 秒世界加载） |
| 场景播放 | **11/11** 模板跑通，未支持项 0 |

### 1.3 轨迹

| 项目 | 结果 |
|---|---|
| 轨迹导入 | CSV / FCD / JSON **三种格式全部通过** |
| 批量合成 100 条 | **0.17 s**（595 条/秒） |
| 合成成功率 | **100/100** |
| 速度分布 KS 距离 | **0.029** |

### 1.4 工程一致性

| 项目 | 结果 |
|---|---|
| 工具装饰器 | 51 个 |
| `self.tools` 列表 | 60 个 |
| 派发链分支 | 61 个 |
| "可见但调用失败"的工具 | **0 个**（修复前 5 个） |
| 重复派发键 | 0 |
| 调用了未定义函数的分支 | 0 |

---

## 2. 已知限制

### 2.1 播放器是子集实现

不支持的条件与动作会记入 `unsupported` 并明确返回，不会静默跳过。

| 类别 | 不支持项 |
|---|---|
| 条件 | `CollisionCondition`、`TimeToCollisionCondition`、`RelativeSpeedCondition`、`AccelerationCondition`、`OffroadCondition`、`EndOfRoadCondition` |
| 动作 | `FollowTrajectoryAction` |
| 结构 | `Story` 分支、`Catalog`、`TrafficSignalControllerAction` |

即便支持的部分也有简化：

| 项目 | 简化 |
|---|---|
| 跟驰 | 用 P 控制，未实现 OSC 完整的时距控制与加速度限制 |
| 变道 | 用纯追踪，未做横向加速度约束 |
| 速度动力学 | `linear` 近似为分帧线性插值 |
| 位置语义 | `freespace` / `alongroute` 未实现，一律按直线距离 |

### 2.2 路口环是折线拼接

`CommonJunctionCreator` 用直线连接各臂端点，十字路口呈现为**菱形环**
而不是正方形环。拓扑正确、可通行，但几何不是真实交叉口的样子。

### 2.3 CARLA 客户端断言

一个进程内连续切换多个世界会偶发 `_episode != nullptr` 断言（引擎侧问题）。
目前的规避方式是每种布局跑独立进程，未在代码里根治。

### 2.4 测试环境

集显 + `-quality-level=Low` 下完成，未在独立显卡 / 高画质复验。

### 2.5 轨迹数据

当前没有真实车队数据，测试用符合 NGSIM 格式的合成样本
（3 条 / 180 帧 / 10 Hz）。拿到实采数据后需要重跑分布对比，
确认增强强度是否合适。

`duration` 的 KS 距离偏大（0.68），原因是 3 条种子时长完全相同、
真实方差为 0，这个指标在退化分布上没有意义。

### 2.6 轨迹到 SUMO

只实现了起点终点法（按轨迹首尾生成 flow），没有按 edge 序列精确还原路径。

### 2.7 OpenSCENARIO 默认版本

默认输出 OSC 1.1。若下游播放器只支持 1.0，需要改默认值并补行人 `model` 定义。

---

## 3. 参考

| 内容 | 链接 |
|---|---|
| CARLA | <https://carla.org/> |
| OpenSCENARIO | <https://www.asam.net/standards/detail/openscenario/> |
| OpenDRIVE | <https://www.asam.net/standards/detail/opendrive/> |
| scenariogeneration | <https://github.com/pyoscx/scenariogeneration> |
| NGSIM | <https://ops.fhwa.dot.gov/trafficanalysistools/ngsim.htm> |
