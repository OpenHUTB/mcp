"""
distribution_check.py - 真实数据 vs 合成数据的分布对比

批量合成最容易出的问题是"能生成但不像"：速度分布被拉偏、加速度爆表、
轨迹长度全挤在一个值上。没有量化对比就无法判断合成数据能不能用。

本模块提供：
  - 单变量分布指标：min/max/mean/p50/p95、标准差、偏度
  - 两个样本的分布距离：KS 统计量、Wasserstein 距离
  - 质量门限：越界的合成样本会被标出

不依赖 scipy：KS 与 Wasserstein 都是一维情形，直接排序算即可，
避免给项目再加一个大依赖。
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

from trajectory import Trajectory, TrajectorySet


# =====================================================================
# 基础统计
# =====================================================================
def _pct(sorted_vals: Sequence[float], p: float) -> float:
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = (len(sorted_vals) - 1) * p
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (idx - lo)


def describe_values(vals: Sequence[float]) -> Dict[str, float]:
    v = sorted(x for x in vals if x is not None and not math.isnan(x))
    if not v:
        return {}
    n = len(v)
    mean = sum(v) / n
    var = sum((x - mean) ** 2 for x in v) / n if n > 1 else 0.0
    std = math.sqrt(var)
    skew = 0.0
    if std > 1e-12 and n > 1:
        skew = sum(((x - mean) / std) ** 3 for x in v) / n
    return {
        "n": n,
        "min": round(v[0], 4),
        "max": round(v[-1], 4),
        "mean": round(mean, 4),
        "std": round(std, 4),
        "p05": round(_pct(v, 0.05), 4),
        "p50": round(_pct(v, 0.50), 4),
        "p95": round(_pct(v, 0.95), 4),
        "skew": round(skew, 4),
    }


# =====================================================================
# 分布距离
# =====================================================================
def ks_statistic(a: Sequence[float], b: Sequence[float]) -> float:
    """两样本 Kolmogorov-Smirnov 统计量（0 = 完全相同，1 = 完全分离）。"""
    if not a or not b:
        return 1.0
    sa, sb = sorted(a), sorted(b)
    na, nb = len(sa), len(sb)
    i = j = 0
    d = 0.0
    while i < na and j < nb:
        if sa[i] <= sb[j]:
            i += 1
        else:
            j += 1
        d = max(d, abs(i / na - j / nb))
    return round(d, 4)


def wasserstein_1d(a: Sequence[float], b: Sequence[float]) -> float:
    """一维 Wasserstein 距离（单位同输入），衡量"搬多少质量"。

    一维情形的解析解就是两个经验分位数函数之差的积分，
    这里用合并排序后按 CDF 步长加权求和近似。
    """
    if not a or not b:
        return float("inf")
    sa, sb = sorted(a), sorted(b)
    na, nb = len(sa), len(sb)
    # 取并集的所有取值点，逐段积分 |F_a(x) - F_b(x)|
    pts = sorted(set(sa) | set(sb))
    total = 0.0
    ia = ib = 0
    prev = pts[0]
    for x in pts:
        if x > prev:
            ca = ia / na
            cb = ib / nb
            total += abs(ca - cb) * (x - prev)
        while ia < na and sa[ia] <= x:
            ia += 1
        while ib < nb and sb[ib] <= x:
            ib += 1
        prev = x
    return round(total, 4)


# =====================================================================
# 特征提取
# =====================================================================
FEATURES = {
    "speed": "逐帧速度 m/s",
    "accel": "逐帧加速度 m/s^2",
    "path_length": "单条轨迹里程 m",
    "duration": "单条轨迹时长 s",
    "curvature": "轨迹平均曲率 1/m",
}


def extract_features(tset: TrajectorySet) -> Dict[str, List[float]]:
    """把轨迹集抽成若干一维特征序列，供分布对比。"""
    speed: List[float] = []
    accel: List[float] = []
    path_len: List[float] = []
    duration: List[float] = []
    curvature: List[float] = []

    for tr in tset.tracks:
        speed.extend(tr.speeds())
        accel.extend(tr.accelerations())
        path_len.append(tr.path_length())
        duration.append(tr.duration)
        curvature.append(_mean_curvature(tr))

    return {
        "speed": speed,
        "accel": accel,
        "path_length": path_len,
        "duration": duration,
        "curvature": curvature,
    }


def _mean_curvature(tr: Trajectory) -> float:
    """用相邻三点的转角除以弧长估计平均曲率。"""
    fs = tr.frames
    if len(fs) < 3:
        return 0.0
    total = 0.0
    length = 0.0
    for i in range(1, len(fs) - 1):
        a, b, c = fs[i - 1], fs[i], fs[i + 1]
        v1 = (b.x - a.x, b.y - a.y)
        v2 = (c.x - b.x, c.y - b.y)
        l1 = math.hypot(*v1)
        l2 = math.hypot(*v2)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        ang = abs(math.atan2(cross, dot))
        total += ang
        length += (l1 + l2) / 2.0
    return (total / length) if length > 1e-9 else 0.0


# =====================================================================
# 对比
# =====================================================================
def compare(real: TrajectorySet, synthetic: TrajectorySet,
            ks_warn: float = 0.25,
            wass_warn_ratio: float = 0.5) -> Dict[str, Any]:
    """对比真实与合成数据的分布。

    wass_warn_ratio: Wasserstein 距离超过真实数据 p95 的这个比例时告警。
    """
    fr = extract_features(real)
    fs = extract_features(synthetic)

    rows: Dict[str, Any] = {}
    warnings: List[str] = []
    for name in FEATURES:
        a, b = fr.get(name, []), fs.get(name, [])
        da, db = describe_values(a), describe_values(b)
        ks = ks_statistic(a, b)
        wass = wasserstein_1d(a, b)
        base = abs(da.get("p95", 0.0)) or 1.0
        ratio = wass / base

        verdict = "ok"
        if ks > ks_warn:
            verdict = "偏"
            warnings.append("%s 的 KS 距离 %.3f 超过 %.2f" % (name, ks, ks_warn))
        if ratio > wass_warn_ratio:
            verdict = "偏"
            warnings.append("%s 的 Wasserstein 距离 %.3f（相对 p95 %.2f）偏大"
                            % (name, wass, ratio))

        rows[name] = {
            "real": da,
            "synthetic": db,
            "ks": ks,
            "wasserstein": wass,
            "wass_ratio": round(ratio, 4),
            "verdict": verdict,
        }

    return {
        "real_tracks": len(real.tracks),
        "synthetic_tracks": len(synthetic.tracks),
        "features": rows,
        "warnings": warnings,
        "ok": not warnings,
    }


def format_report(rep: Dict[str, Any]) -> str:
    lines = []
    lines.append("真实 %d 条  vs  合成 %d 条"
                 % (rep["real_tracks"], rep["synthetic_tracks"]))
    lines.append("")
    lines.append("%-12s %9s %9s %9s %9s %9s  %s" %
                 ("特征", "真实均值", "合成均值", "KS", "Wass", "相对", "判定"))
    for name, r in rep["features"].items():
        da, db = r["real"] or {}, r["synthetic"] or {}
        lines.append("%-12s %9.3f %9.3f %9.3f %9.3f %9.2f  %s" % (
            name,
            da.get("mean", 0.0), db.get("mean", 0.0),
            r["ks"], r["wasserstein"], r["wass_ratio"], r["verdict"]))
    if rep["warnings"]:
        lines.append("")
        lines.append("告警：")
        for w in rep["warnings"]:
            lines.append("  - %s" % w)
    else:
        lines.append("")
        lines.append("所有特征均在阈值内")
    return "\n".join(lines)


def filter_implausible(tset: TrajectorySet,
                       max_speed: float = 55.0,
                       max_accel: float = 12.0,
                       min_frames: int = 5) -> Tuple[TrajectorySet, List[str]]:
    """剔除物理上不合理的合成轨迹，返回 (保留集, 剔除原因列表)。"""
    keep: List[Trajectory] = []
    dropped: List[str] = []
    for tr in tset.tracks:
        sp = tr.speeds()
        ac = tr.accelerations()
        if len(tr) < min_frames:
            dropped.append("%s: 帧数不足 %d" % (tr.id, len(tr)))
            continue
        if sp and max(sp) > max_speed:
            dropped.append("%s: 速度 %.1f 超过 %.1f m/s"
                           % (tr.id, max(sp), max_speed))
            continue
        if ac and max(ac) > max_accel:
            dropped.append("%s: 加速度 %.1f 超过 %.1f m/s^2"
                           % (tr.id, max(ac), max_accel))
            continue
        keep.append(tr)
    out = TrajectorySet(tracks=keep, hz=tset.hz, source=tset.source,
                        coordinate_system=tset.coordinate_system,
                        units=dict(tset.units), extra=dict(tset.extra))
    return out, dropped
