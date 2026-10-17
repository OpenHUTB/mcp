"""
augmentation.py - 轨迹增强算子

目标②的核心：从少量实采轨迹（种子）派生出大量合成轨迹。

五类基础算子 + 一个流水线组合器：

  add_noise             位置/速度高斯噪声
  time_scale            时间轴拉伸/压缩（改变节奏，不改变空间路径）
  spatial_translate     整体平移（换到别的车道/路口）
  lane_offset           车道内横向偏移（贴线、压线）
  multipath_replicate   时间错位复制（一条轨迹变成多车流）
  speed_profile_warp    速度曲线变形（整体加速/减速/分段变速）

设计约束：
- 每个算子都只接受并返回 Trajectory，不产生副作用；
- 所有随机来源都走传入的 random.Random，保证同 seed 可复现；
- 算子不做物理合理性判断（那是 distribution_check 的事），
  但会保持基本一致性：速度与位置差分一致，加速度同步更新。
"""
from __future__ import annotations

import math
import random
from typing import Any, Dict, List, Optional, Sequence

from trajectory import Frame, Trajectory


class AugmentError(ValueError):
    pass


def _clone(tr: Trajectory, new_id: Optional[str] = None) -> Trajectory:
    return Trajectory(
        id=new_id or tr.id,
        frames=[Frame(**f.__dict__) for f in tr.frames],
        kind=tr.kind,
        meta=dict(tr.meta),
    )


def _recompute_kinematics(tr: Trajectory, smooth: int = 1) -> None:
    """位置改了之后，用差分把速度和加速度重新算一遍，保证自洽。

    smooth: 速度平滑的半窗宽。位置里本来就有毫米级抖动，
    直接做差分会把它放大成米/秒级的速度毛刺，所以默认做 3 点平滑。
    传 0 关闭平滑。
    """
    n = len(tr.frames)
    if n < 2:
        return

    # 1) 原始差商
    raw_vx: List[float] = []
    raw_vy: List[float] = []
    for i in range(n):
        if i == 0:
            a, b = tr.frames[0], tr.frames[1]
        elif i == n - 1:
            a, b = tr.frames[-2], tr.frames[-1]
        else:
            a, b = tr.frames[i - 1], tr.frames[i + 1]
        dt = b.t - a.t
        if dt > 0:
            raw_vx.append((b.x - a.x) / dt)
            raw_vy.append((b.y - a.y) / dt)
        else:
            raw_vx.append(0.0)
            raw_vy.append(0.0)

    # 2) 平滑
    def smooth_series(series: List[float], half: int) -> List[float]:
        if half <= 0:
            return list(series)
        out = []
        for i in range(len(series)):
            lo = max(0, i - half)
            hi = min(len(series), i + half + 1)
            chunk = series[lo:hi]
            out.append(sum(chunk) / len(chunk))
        return out

    vx_s = smooth_series(raw_vx, smooth)
    vy_s = smooth_series(raw_vy, smooth)

    for f, vx, vy in zip(tr.frames, vx_s, vy_s):
        f.vx, f.vy = vx, vy
        if abs(vx) + abs(vy) > 1e-9:
            f.heading = math.degrees(math.atan2(vy, vx)) % 360.0

    # 3) 加速度由平滑后的速度再差分
    for i, f in enumerate(tr.frames):
        if i == 0:
            a, b = tr.frames[0], tr.frames[1]
        elif i == n - 1:
            a, b = tr.frames[-2], tr.frames[-1]
        else:
            a, b = tr.frames[i - 1], tr.frames[i + 1]
        dt = b.t - a.t
        if dt > 0:
            f.ax = (b.vx - a.vx) / dt
            f.ay = (b.vy - a.vy) / dt


# =====================================================================
# 算子 1：加噪
# =====================================================================
def add_noise(tr: Trajectory, sigma_pos: float = 0.05,
              sigma_vel: float = 0.0, sigma_heading: float = 0.0,
              rng: Optional[random.Random] = None,
              new_id: Optional[str] = None) -> Trajectory:
    """位置/速度/朝向加高斯噪声。

    sigma_pos 单位米，日常取值 0.02–0.2；过大会破坏轨迹平滑性
    （加速度会爆），distribution_check 能查出来。
    """
    rng = rng or random.Random(0)
    out = _clone(tr, new_id)
    for f in out.frames:
        f.x += rng.gauss(0.0, sigma_pos)
        f.y += rng.gauss(0.0, sigma_pos)
        if sigma_vel:
            f.vx += rng.gauss(0.0, sigma_vel)
            f.vy += rng.gauss(0.0, sigma_vel)
        if sigma_heading:
            f.heading = (f.heading or 0.0) + rng.gauss(0.0, sigma_heading)
    out.derive_kinematics()
    out.meta["augment"] = {"op": "add_noise", "sigma_pos": sigma_pos}
    return out


# =====================================================================
# 算子 2：时间缩放
# =====================================================================
def time_scale(tr: Trajectory, factor: float,
               new_id: Optional[str] = None) -> Trajectory:
    """时间轴拉伸/压缩。factor>1 变慢，<1 变快。

    空间路径不变，只改节奏；速度按 1/factor 变化。
    """
    if factor <= 0:
        raise AugmentError("factor 必须为正")
    out = _clone(tr, new_id)
    for f in out.frames:
        f.t *= factor
        f.vx /= factor
        f.vy /= factor
        f.ax /= factor * factor
        f.ay /= factor * factor
    out.meta["augment"] = {"op": "time_scale", "factor": factor}
    return out


# =====================================================================
# 算子 3：空间平移
# =====================================================================
def spatial_translate(tr: Trajectory, dx: float = 0.0, dy: float = 0.0,
                      new_id: Optional[str] = None) -> Trajectory:
    out = _clone(tr, new_id)
    for f in out.frames:
        f.x += dx
        f.y += dy
    out.meta["augment"] = {"op": "spatial_translate", "dx": dx, "dy": dy}
    return out


# =====================================================================
# 算子 4：车道内横向偏移
# =====================================================================
def lane_offset(tr: Trajectory, offset_m: float,
                new_id: Optional[str] = None) -> Trajectory:
    """沿轨迹法向整体平移 offset_m（左正右负）。

    与 spatial_translate 的区别：这是**跟随轨迹朝向**的横向偏移，
    用于造贴线/压线场景；平移是固定方向。

    实采轨迹的位置通常带毫米级抖动，逐点算法向会让偏移量跟着抖，
    所以先对法向做一次滑动平均再施加。
    """
    out = _clone(tr, new_id)
    n = len(out.frames)
    if n == 0:
        return out
    if n < 2:
        for f in out.frames:
            f.y += offset_m
        return out

    # 1) 逐点法向
    normals = []
    for i in range(n):
        a = out.frames[max(i - 1, 0)]
        b = out.frames[min(i + 1, n - 1)]
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        if length < 1e-9:
            normals.append((0.0, 1.0))
        else:
            normals.append((-dy / length, dx / length))

    # 2) 3 点滑动平均，抑制抖动放大
    smoothed = []
    for i in range(n):
        lo, hi = max(0, i - 1), min(n, i + 2)
        chunk = normals[lo:hi]
        sx = sum(v[0] for v in chunk) / len(chunk)
        sy = sum(v[1] for v in chunk) / len(chunk)
        norm = math.hypot(sx, sy)
        smoothed.append((sx / norm, sy / norm) if norm > 1e-9 else (0.0, 1.0))

    # 3) 施加偏移
    for f, (nx, ny) in zip(out.frames, smoothed):
        f.x += nx * offset_m
        f.y += ny * offset_m

    _recompute_kinematics(out)
    out.meta["augment"] = {"op": "lane_offset", "offset_m": offset_m}
    return out


# =====================================================================
# 算子 5：多轨迹复制（时间错位）
# =====================================================================
def multipath_replicate(tr: Trajectory, n_copies: int = 2,
                        gap_s: float = 1.5, lateral_gap: float = 0.0,
                        rng: Optional[random.Random] = None,
                        id_prefix: str = None) -> List[Trajectory]:
    """把一条轨迹复制成 n_copies 条，按时间错位、可选横向错开。

    用于把单车轨迹变成有跟驰关系的车流。返回列表包含原轨迹。
    """
    if n_copies < 1:
        raise AugmentError("n_copies 至少为 1")
    rng = rng or random.Random(0)
    out: List[Trajectory] = [_clone(tr)]
    prefix = id_prefix or tr.id

    for k in range(1, n_copies):
        tid = "%s_r%d" % (prefix, k)
        dup = _clone(tr, tid)
        jitter = rng.uniform(-0.2, 0.2) * gap_s
        shift = -(gap_s * k + jitter)
        for f in dup.frames:
            f.t += shift
        if lateral_gap:
            dup = lane_offset(dup, lateral_gap * (1 if k % 2 else -1))
        dup.meta["augment"] = {"op": "multipath_replicate", "copy": k,
                               "gap_s": shift}
        out.append(dup)
    return out


# =====================================================================
# 算子 6：速度曲线变形
# =====================================================================
def speed_profile_warp(tr: Trajectory, scale: float = 1.0,
                       start_frac: float = 0.0, end_frac: float = 1.0,
                       new_id: Optional[str] = None) -> Trajectory:
    """按时间段对速度做缩放，再用积分重建位置。

    scale > 1 加速，< 1 减速。start_frac/end_frac 指定作用区间（0~1）。
    """
    if scale <= 0:
        raise AugmentError("scale 必须为正")
    if not (0.0 <= start_frac < end_frac <= 1.0):
        raise AugmentError("需要 0 <= start_frac < end_frac <= 1")

    out = _clone(tr, new_id)
    n = len(out.frames)
    if n < 2:
        return out

    # 先把速度按区间缩放
    for i, f in enumerate(out.frames):
        frac = i / (n - 1)
        if start_frac <= frac <= end_frac:
            f.vx *= scale
            f.vy *= scale

    # 再用缩放后的速度积分回位置
    x, y = out.frames[0].x, out.frames[0].y
    for i in range(1, n):
        dt = out.frames[i].t - out.frames[i - 1].t
        x += out.frames[i - 1].vx * dt
        y += out.frames[i - 1].vy * dt
        out.frames[i].x = x
        out.frames[i].y = y
    _recompute_kinematics(out)
    out.meta["augment"] = {"op": "speed_profile_warp", "scale": scale,
                           "range": [start_frac, end_frac]}
    return out


# =====================================================================
# 流水线
# =====================================================================
OPERATORS = {
    "add_noise": add_noise,
    "time_scale": time_scale,
    "spatial_translate": spatial_translate,
    "lane_offset": lane_offset,
    "speed_profile_warp": speed_profile_warp,
}


def apply_pipeline(tr: Trajectory, steps: Sequence[Dict[str, Any]],
                   rng: Optional[random.Random] = None,
                   new_id: Optional[str] = None) -> Trajectory:
    """按顺序执行算子链。steps 形如 [{"op": "add_noise", "sigma_pos": 0.05}, ...]"""
    rng = rng or random.Random(0)
    cur = _clone(tr, new_id or tr.id)
    applied = []
    for s in steps:
        name = s.get("op")
        if name not in OPERATORS:
            raise AugmentError("未知算子 %r，可用: %s"
                               % (name, sorted(OPERATORS)))
        kwargs = {k: v for k, v in s.items() if k != "op"}
        fn = OPERATORS[name]
        if name == "add_noise":
            kwargs.setdefault("rng", rng)
        cur = fn(cur, **kwargs)
        applied.append({"op": name, "params": kwargs})
    cur.meta["augment"] = {"pipeline": applied}
    return cur


def random_pipeline(rng: random.Random, n_ops: int = 2,
                    strength: str = "mild") -> List[Dict[str, Any]]:
    """随机组合一个算子链，用于批量合成。"""
    if strength == "mild":
        noise_range, scale_range, time_range = (0.02, 0.10), (0.85, 1.15), (0.85, 1.2)
    elif strength == "medium":
        noise_range, scale_range, time_range = (0.05, 0.25), (0.7, 1.35), (0.7, 1.5)
    else:
        noise_range, scale_range, time_range = (0.1, 0.5), (0.5, 1.8), (0.5, 2.0)

    pool = [
        lambda: {"op": "add_noise",
                 "sigma_pos": round(rng.uniform(*noise_range), 4)},
        lambda: {"op": "speed_profile_warp",
                 "scale": round(rng.uniform(*scale_range), 3),
                 "start_frac": round(rng.uniform(0.0, 0.3), 2),
                 "end_frac": round(rng.uniform(0.7, 1.0), 2)},
        lambda: {"op": "time_scale",
                 "factor": round(rng.uniform(*time_range), 3)},
        lambda: {"op": "lane_offset",
                 "offset_m": round(rng.uniform(-1.2, 1.2), 3)},
        lambda: {"op": "spatial_translate",
                 "dx": round(rng.uniform(-30, 30), 2),
                 "dy": round(rng.uniform(-30, 30), 2)},
    ]
    rng.shuffle(pool)
    return [pool[i]() for i in range(min(n_ops, len(pool)))]


def describe_pipeline(steps: Sequence[Dict[str, Any]]) -> str:
    parts = []
    for s in steps:
        name = s.get("op", "?")
        params = ", ".join("%s=%s" % (k, v) for k, v in s.items() if k != "op")
        parts.append("%s(%s)" % (name, params) if params else name)
    return " -> ".join(parts)
