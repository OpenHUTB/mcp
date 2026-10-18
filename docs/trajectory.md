title: 轨迹合成

# 实采轨迹批量合成

把有限的实采轨迹扩增为大批量合成数据，并量化合成数据与真实数据的分布差异。

## 1. 工具参数

### 1.1 `import_trajectory`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `file_path` | string | 必填 | 轨迹文件 |
| `format` | string | 自动推断 | `csv` / `fcd` / `json` |
| `column_map_json` | string | `{}` | 自定义列名映射 |
| `preview` | int | 5 | 预览条数 |

### 1.2 `synthesize_trajectory_dataset`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `seed_path` | string | 必填 | 种子轨迹文件 |
| `n` | int | 100 | 合成数量 |
| `seed` | int | None | 随机种子 |
| `n_ops` | int | 2 | 每条轨迹叠加的算子数 |
| `strength` | string | `mild` | 增强强度 `mild` / `medium` / `strong` |
| `max_per_seed` | int | None | 每个种子的最大衍生数 |
| `out_dir` | string | `llm/output/synthetic` | 输出目录 |
| `format` | string | `csv` | 输出格式 |

### 1.3 `compare_trajectory_distribution`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `real_path` | string | 必填 | 真实轨迹 |
| `synthetic_dir` | string | 必填 | 合成数据目录 |
| `format` | string | 自动 | 文件格式 |

### 1.4 `export_sumo_demand`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `trajectory_path` | string | 必填 | 轨迹文件 |
| `out_path` | string | `llm/output/sumo_demand.rou.xml` | 输出路径 |
| `depart_interval` | float | 0.5 | 起始时刻相同时的错峰间隔（秒） |

## 2. 支持的格式

| 格式 | 扩展名 | 典型来源 |
|---|---|---|
| CSV | `.csv` | NGSIM、HighD 等数据集 |
| FCD | `.xml` | SUMO 的浮车数据输出 |
| JSON | `.json` | 自建采集工具 |

### 列名映射

不同数据集列名不一致时，用 `column_map_json` 指定：

```json
{"time": "Global_Time", "x": "Local_X", "y": "Local_Y", "v": "v_Vel"}
```

## 3. 5 类增强算子

| 算子 | 作用 | 强度影响 |
|---|---|---|
| 加噪 | 位置/速度叠加高斯噪声 | 噪声标准差 |
| 时间缩放 | 整体加速或减速 | 缩放系数范围 |
| 平移 | 起点位置与航向偏移 | 平移幅度 |
| 法向偏移 | 沿车道法向横向偏移 | 偏移量 |
| 速度变形 | 速度曲线重新塑形 | 变形幅度 |

| 强度 | 适用场景 |
|---|---|
| `mild` | 保守扩增，分布几乎不变 |
| `medium` | 一般扩增 |
| `strong` | 压力测试，分布明显偏离 |

## 4. 对话示例

```text
导入 _traj_samples 里的轨迹数据
基于这些轨迹合成 100 条数据
对比合成数据和真实数据的分布
把轨迹导出成 SUMO 的需求文件
```

## 5. 代码示例

### 5.1 导入并查看

```python
from trajectory_io import load_trajectories

seeds = load_trajectories("data/ngsim_sample.csv")
print(len(seeds), "条轨迹")
print(seeds[0].frames[:3])
```

### 5.2 批量合成

```python
from synthesis_batch import synthesize

result = synthesize(seeds, n=100, seed=42, n_ops=2, strength="mild")
print(result["generated"], result["skipped"])
# 100 0
```

### 5.3 分布对比

```python
from distribution_check import compare

report = compare("data/ngsim_sample.csv", "output/synthetic")
print(report["speed"]["ks"], report["duration"]["ks"])
```

## 6. 单位换算

NGSIM 格式的量纲与 SI 不一致，导入时自动处理：

| 列 | 原始单位 | 换算 |
|---|---|---|
| `Local_X` / `Local_Y` | 英尺 | ×0.3048 → 米 |
| `v_Vel` | ft/s | ×0.3048 → m/s |
| `Global_Time` | 毫秒 | ÷1000 → 秒 |

> 导入后会校验"报告速度 vs 位置差分"，偏差超过 50% 会告警。
> 早期版本只换算速度不换算位置，导致 `lane_offset` 重算时放大到 44 m/s。

## 7. 时间归一化

如果每条轨迹各自归零，100 辆车会全部在 t=0 出发。现在的做法是
**全局最小时间归一化**，保留车与车之间的时间差：

```text
种子轨迹起始时刻：0.00 s / 0.50 s / 1.20 s
```

## 8. 输出结构

```
output/synthetic/
├── track_0000.csv
├── track_0001.csv
├── ...
├── manifest.json     # 每条轨迹的来源种子与算子链
└── summary.json      # 汇总统计
```

`manifest.json` 记录可追溯信息：

```json
{
  "track": "track_0007.csv",
  "seed_track": "ngsim_0002",
  "seed_value": 42,
  "ops": ["noise", "time_scale"]
}
```

## 9. 实测

| 项目 | 结果 |
|---|---|
| 轨迹导入 | CSV / FCD / JSON **三种格式全部通过** |
| 批量合成 | 100 条 **0.17 s**（595 条/秒），100/100 成功 |
| 复现性 | 同 seed 结果完全一致 |
| 分布对比 | 速度 KS 距离 **0.029** |
