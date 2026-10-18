title: 路网生成

# 路网生成

用 pyodrx 生成参数化 OpenDRIVE（`.xodr`），支持 7 种布局。

## 1. 工具参数

### 1.1 `generate_rich_xodr`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `layout` | string | `straight` | 布局名，见第 2 节 |
| `params_json` | string | `{}` | 覆盖默认参数 |
| `out_dir` | string | `llm/output/xodr` | 输出目录 |
| `filename` | string | 自动 | 文件名 |

### 1.2 `list_road_layouts`

无参数，返回全部布局名、中文名与默认参数。

## 2. 7 种布局与默认参数

| 布局 | 含义 | 默认参数 |
|---|---|---|
| `straight` | 直路 | `length=200.0, lanes=2, lane_width=3.5` |
| `highway_curve` | 高速弯道 | `radius=250.0, angle=90.0, lanes=2, spiral_len=40.0` |
| `s_curve` | S 弯 | `radius=200.0, angle=45.0, lanes=2, spiral_len=30.0` |
| `cross_junction` | 十字路口 | `arm_length=80.0, radius=12.0, lanes=2, lane_width=3.5` |
| `t_junction` | 丁字路口 | `arm_length=80.0, radius=12.0, lanes=2, lane_width=3.5` |
| `on_ramp_merge` | 匝道合流 | `length=200.0, merge_len=80.0, lane_width=3.5` |
| `off_ramp_diverge` | 匝道分流 | `length=200.0, diverge_len=80.0, lane_width=3.5` |

## 3. 参数说明

| 参数 | 适用布局 | 含义 |
|---|---|---|
| `length` | straight / on_ramp_merge / off_ramp_diverge | 路段总长（米） |
| `lanes` | straight / 弯道 / 路口 | 单侧车道数 |
| `lane_width` | 全部 | 车道宽（米） |
| `radius` | 弯道 / 路口 | 曲线半径或路口环半径（米） |
| `angle` | 弯道 | 圆弧转角（度） |
| `spiral_len` | 弯道 | 缓和曲线长度（米） |
| `arm_length` | 路口 | 各条臂的长度（米） |
| `merge_len` | on_ramp_merge | 车道数 1→2 的渐变长度（米） |
| `diverge_len` | off_ramp_diverge | 车道数 2→1 的渐变长度（米） |

## 4. 对话示例

```text
列出所有路网布局
生成一个十字路口路网
生成一个匝道合流路网，主路 4 车道、匝道 2 车道
```

## 5. 代码示例

```python
from road_builder import write, inspect_road

write("cross_junction", "output/cross_junction.xodr",
      {"arm_length": 100.0, "lanes": 2})

info = inspect_road("output/cross_junction.xodr")
print(info["valid"], info["road_count"], info["geometry_count"])
# True 12 28
```

## 6. 几何校验

`inspect_road()` 返回：

| 字段 | 含义 |
|---|---|
| `valid` | 是否通过几何校验 |
| `road_count` | 道路条数 |
| `geometry_count` | 几何段数 |
| `size_bytes` | 文件大小 |
| `lane_profile` | 车道数变化剖面（如匝道 1→2） |

### 必须调用 `adjust_roads_and_lanes()`

`PlanView.add_geometry()` 只把几何存进 `_raw_geometries`，而 `get_element()`
遍历的是 `_adjusted_geometries`。**不调用 `OpenDrive.adjust_roads_and_lanes()`
写出的 `.xodr` 没有几何**，CARLA 加载会一直等到超时。

`road_builder.build()` 内部已无条件调用，`inspect_road()` 会检查这一点。

## 7. 生成结果

| 布局 | 文件大小 | 道路数 | 几何段数 | 生成点 |
|---|---|---|---|---|
| 直路 | 2626 B | 1 | 1 | 2 |
| 高速弯道 | 3071 B | 1 | 1 | 2 |
| S 弯 | 3694 B | 1 | 1 | 2 |
| 十字路口 | 22708 B | 12 | 28 | 12 |
| 丁字路口 | 17023 B | 9 | 21 | 9 |
| 匝道合流 | 4195 B | 1 | 1 | 4 |
| 匝道分流 | 3496 B | 1 | 1 | 3 |

效果图见[效果展示](gallery.md)。
