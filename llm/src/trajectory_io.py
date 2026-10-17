"""
trajectory_io.py - 轨迹多格式导入/导出

支持三种实采数据来源：

  CSV   通用轨迹表，带列名别名自动识别（兼容 NGSIM / highD / 自定义）
  FCD   SUMO 的 Floating Car Data XML
  JSON  本模块导出的规范格式，或其他工具的自定义 JSON

原项目的问题：sumo/src/mcp_tools/analysis.py:20 的 FCD 解析只取
`vehicle.speed` 一列，把 x/y/angle/lane 全丢了——明明拿到的是完整轨迹，
却只做了一维统计。本模块把 FCD 完整读出来。

单位处理
--------
CSV 常见单位混杂：NGSIM 的 v_Vel 是英尺/秒，高速数据常用 km/h。
通过 column_map 的 `*_unit` 键或自动探测声明，统一换算成 m/s。
"""
from __future__ import annotations

import csv
import json
import math
import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

from trajectory import (FT_TO_M, KMH_TO_MS, MPH_TO_MS, Frame, Trajectory,
                        TrajectoryError, TrajectorySet)

# =====================================================================
# 列名别名表：左为标准字段，右为常见实际列名（全部小写比较）
# =====================================================================
COLUMN_ALIASES: Dict[str, List[str]] = {
    "id":        ["id", "vehicle_id", "track_id", "veh_id", "obj_id",
                  "agent_id", "car_id", "vehicleid", "trackid"],
    "frame":     ["frame", "frame_id", "frameid", "f", "index", "step"],
    "t":         ["t", "time", "timestamp", "global_time", "sim_time",
                  "time_s", "sec"],
    "x":         ["x", "local_x", "global_x", "xcoord", "pos_x", "px",
                  "east", "utm_x"],
    "y":         ["y", "local_y", "global_y", "ycoord", "pos_y", "py",
                  "north", "utm_y"],
    "vx":        ["vx", "x_velocity", "xvelocity", "vel_x", "v_x"],
    "vy":        ["vy", "y_velocity", "yvelocity", "vel_y", "v_y"],
    "speed":     ["speed", "v_vel", "velocity", "vel", "v", "speed_ms",
                  "speed_kmh", "speed_mph"],
    "ax":        ["ax", "x_acceleration", "xacceleration", "acc_x"],
    "ay":        ["ay", "y_acceleration", "yacceleration", "acc_y"],
    "accel":     ["acc", "acceleration", "a"],
    "heading":   ["heading", "angle", "yaw", "theta", "direction",
                  "heading_deg"],
    "lane":      ["lane", "lane_id", "laneid", "lane_index"],
    "s":         ["s", "longitudinal", "pos", "lane_pos"],
}


def _norm(name: str) -> str:
    return name.strip().lower().replace(" ", "_").replace("-", "_")


def detect_columns(headers: List[str]) -> Dict[str, str]:
    """从表头自动推断标准字段 → 实际列名的映射。"""
    norm = {h: _norm(h) for h in headers}
    out: Dict[str, str] = {}
    used = set()
    for std, aliases in COLUMN_ALIASES.items():
        for h, n in norm.items():
            if h in used:
                continue
            if n in aliases:
                out[std] = h
                used.add(h)
                break
    return out


def _pick_unit(column_map: Dict[str, str], headers: List[str],
               speed_col: Optional[str]) -> float:
    """返回速度列 → m/s 的换算系数。"""
    # 显式声明优先
    unit = column_map.get("speed_unit", "").lower()
    if unit in ("m/s", "ms"):
        return 1.0
    if unit in ("km/h", "kmh", "kph"):
        return KMH_TO_MS
    if unit in ("mph", "mi/h"):
        return MPH_TO_MS
    if unit in ("ft/s", "fps"):
        return FT_TO_M
    # 按列名猜
    if speed_col:
        n = _norm(speed_col)
        if "kmh" in n or "km_h" in n or "kph" in n:
            return KMH_TO_MS
        if "mph" in n:
            return MPH_TO_MS
        if "fps" in n or "ft" in n:
            return FT_TO_M
        if "v_vel" in n:     # NGSIM 约定
            return FT_TO_M
    return 1.0


def _pick_pos_unit(column_map: Dict[str, str]) -> float:
    """位置列 → 米的换算系数。

    NGSIM 的 Local_X / Local_Y 是英尺，必须和 v_Vel 一起换算，
    否则会出现"位置按英尺、速度按米/秒"的自相矛盾，
    后续差分重算速度时会被放大成离谱的数值。
    """
    unit = (column_map.get("position_unit") or "").lower()
    if unit in ("ft", "feet"):
        return FT_TO_M
    if unit in ("m", "meter", "metre"):
        return 1.0
    xcol = _norm(column_map.get("x", ""))
    ycol = _norm(column_map.get("y", ""))
    if xcol.startswith("local_") or ycol.startswith("local_"):
        return FT_TO_M      # NGSIM 约定
    if xcol.startswith("global_") or ycol.startswith("global_"):
        return FT_TO_M      # NGSIM 的 Global_XY 同样是英尺
    return 1.0


# =====================================================================
# CSV
# =====================================================================
def load_csv(path: str, column_map: Optional[Dict[str, str]] = None,
             hz: Optional[float] = None, id_col: Optional[str] = None,
             delimiter: Optional[str] = None) -> TrajectorySet:
    """读 CSV 轨迹表。

    column_map 可只给部分字段，其余自动探测。
    如果给了 id 但表格里没有 id 列，则整表当作单条轨迹。
    """
    if not os.path.exists(path):
        raise TrajectoryError("文件不存在: %s" % path)

    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        sample = f.read(4096)
        f.seek(0)
        if delimiter is None:
            try:
                delimiter = csv.Sniffer().sniff(sample, delimiters=",;\t").delimiter
            except csv.Error:
                delimiter = ","
        rows = list(csv.DictReader(f, delimiter=delimiter))

    if not rows:
        raise TrajectoryError("CSV 为空: %s" % path)

    headers = list(rows[0].keys())
    auto = detect_columns(headers)
    cmap = dict(auto)
    if column_map:
        # 用户显式给的覆盖自动探测；保留 *_unit 这类非列名键
        for k, v in column_map.items():
            cmap[k] = v

    need = ["x", "y"]
    for n in need:
        if n not in cmap:
            raise TrajectoryError(
                "无法识别坐标列（需要 x/y）。表头: %s\n可显式传 column_map，"
                "例如 {\"x\": \"Local_X\", \"y\": \"Local_Y\"}" % headers)

    speed_scale = _pick_unit(cmap, headers, cmap.get("speed"))
    pos_scale = _pick_pos_unit(cmap)

    # 时间基
    if hz is None:
        hz = 10.0  # 兜底，后面会用实际间隔校正

    def get(row, key, default=None):
        col = cmap.get(key)
        if not col:
            return default
        v = row.get(col)
        if v is None or str(v).strip() == "":
            return default
        return v

    def fnum(row, key, default=None, scale=1.0):
        v = get(row, key)
        if v is None:
            return default
        try:
            return float(v) * scale
        except (TypeError, ValueError):
            return default

    # 按 id 分组
    groups: Dict[str, List[Dict[str, str]]] = {}
    id_key = cmap.get("id") or id_col
    for i, row in enumerate(rows):
        if id_key:
            gid = str(row.get(id_key, "")).strip() or "0"
        else:
            gid = "0"
        groups.setdefault(gid, []).append(row)

    # ---------------- 时间基判定 ----------------
    # 实采 CSV 的时间列单位常见三种：秒、毫秒时间戳、纯帧号。
    # 直接当秒用会把 100 ms 的间隔读成 100 s（NGSIM 的 Global_Time 就是毫秒）。
    raw_t: List[float] = []
    t_col = cmap.get("t")
    if t_col:
        for r in rows:
            try:
                raw_t.append(float(r.get(t_col)))
            except (TypeError, ValueError):
                pass

    t_scale = 1.0
    use_frame = False
    if len(raw_t) > 2:
        uniq = sorted(set(raw_t))
        diffs = sorted(b - a for a, b in zip(uniq, uniq[1:]) if b - a > 1e-12)
        if diffs:
            med = diffs[len(diffs) // 2]
            if med >= 50.0:
                t_scale = 0.001          # 毫秒时间戳
            elif med >= 0.5 and cmap.get("frame"):
                use_frame = True         # 间隔过大，改用帧号
    elif not t_col:
        use_frame = True

    tracks: List[Trajectory] = []
    all_t: List[float] = []

    # 先算出每条轨迹的原始时间序列（未归一化）
    resolved: List[Tuple[str, List[Dict[str, str]], List[Optional[float]]]] = []
    for gid, grows in groups.items():
        tvals: List[Optional[float]] = []
        for r in grows:
            if use_frame:
                fv = fnum(r, "frame")
                tv = (fv / hz) if fv is not None else None
            else:
                tv = fnum(r, "t")
                if tv is not None:
                    tv *= t_scale
                elif cmap.get("frame"):
                    fv = fnum(r, "frame")
                    tv = (fv / hz) if fv is not None else None
            tvals.append(tv)
        if all(v is None for v in tvals):
            tvals = [i / hz for i in range(len(grows))]
        resolved.append((gid, grows, tvals))

    # 用全局最小时间做归一化，而不是每条轨迹各自归零。
    # 各自归零会丢掉车与车之间的出发时间差（100 辆车全在 t=0 出发），
    # 对交通流分析是致命的。
    global_first = min(
        (v for _, _, tv in resolved for v in tv if v is not None),
        default=0.0)

    for gid, grows, tvals in resolved:
        frames: List[Frame] = []
        for idx, (r, tv) in enumerate(zip(grows, tvals)):
            t = (tv if tv is not None else idx / hz) - global_first
            x = fnum(r, "x", 0.0, pos_scale)
            y = fnum(r, "y", 0.0, pos_scale)
            speed = fnum(r, "speed", None, speed_scale)
            vx = fnum(r, "vx", None, speed_scale)
            vy = fnum(r, "vy", None, speed_scale)
            if vx is None and vy is None and speed is not None:
                hd = fnum(r, "heading", 0.0)
                rr = math.radians(hd)
                vx, vy = speed * math.cos(rr), speed * math.sin(rr)
            frames.append(Frame(
                t=t, x=x, y=y,
                vx=vx or 0.0, vy=vy or 0.0,
                ax=fnum(r, "ax", 0.0), ay=fnum(r, "ay", 0.0),
                heading=fnum(r, "heading", None),
                lane=(str(get(r, "lane")) if get(r, "lane") is not None else None),
                s=fnum(r, "s", None),
            ))
            all_t.append(t)
        frames.sort(key=lambda f: f.t)
        tracks.append(Trajectory(id=gid, frames=frames))

    # 用实际相邻间隔校正 hz（只在估计值落在合理区间时采用）
    if len(all_t) > 2:
        ts = sorted(set(round(v, 6) for v in all_t))
        diffs = [b - a for a, b in zip(ts, ts[1:]) if b - a > 1e-9]
        if diffs:
            diffs.sort()
            median = diffs[len(diffs) // 2]
            if median > 0:
                est = 1.0 / median
                if 0.5 <= est <= 200.0:
                    hz = round(est, 3)

    tset = TrajectorySet(tracks=tracks, hz=hz,
                         source=os.path.basename(path),
                         coordinate_system="local",
                         extra={"column_map": cmap,
                                "speed_scale": speed_scale,
                                "position_scale": pos_scale,
                                "time_scale": t_scale,
                                "used_frame_as_time": use_frame,
                                "rows": len(rows)})
    tset.derive_all()

    # 自洽性检查：报告里给的速度应与位置差分吻合。
    # 不一致通常意味着单位没换算对（例如位置英尺、速度米/秒）。
    dev = _consistency_deviation(tset)
    tset.extra["speed_position_deviation"] = round(dev, 4)
    if dev > 0.5:
        tset.extra["warning"] = (
            "速度与位置差分偏差 %.0f%%，单位可能没换算对"
            "（检查 position_unit / speed_unit）" % (dev * 100))
    return tset


def _consistency_deviation(tset: TrajectorySet) -> float:
    """返回速度与位置差分的相对偏差（最大）。"""
    worst = 0.0
    for tr in tset.tracks:
        for a, b in zip(tr.frames, tr.frames[1:]):
            dt = b.t - a.t
            if dt <= 0:
                continue
            v_disp = math.hypot((b.x - a.x) / dt, (b.y - a.y) / dt)
            v_rep = (a.speed() + b.speed()) / 2.0
            if v_rep > 1.0:
                worst = max(worst, abs(v_disp - v_rep) / v_rep)
    return worst


# =====================================================================
# SUMO FCD
# =====================================================================
def load_fcd(path: str) -> TrajectorySet:
    """读 SUMO FCD XML，完整提取 x/y/angle/speed/lane。

    对比原 analysis.py 只取 speed：这里保留全部字段。
    """
    if not os.path.exists(path):
        raise TrajectoryError("文件不存在: %s" % path)

    root = ET.parse(path).getroot()
    groups: Dict[str, List[Frame]] = {}
    times: List[float] = []

    for ts in root.iter("timestep"):
        try:
            t = float(ts.get("time", 0.0))
        except ValueError:
            continue
        times.append(t)
        for veh in ts.iter("vehicle"):
            vid = veh.get("id", "0")
            try:
                x = float(veh.get("x", 0.0))
                y = float(veh.get("y", 0.0))
                sp = float(veh.get("speed", 0.0))
                ang = float(veh.get("angle", 0.0))
            except (TypeError, ValueError):
                continue
            # SUMO 的 angle 是度，且 y 轴朝北、angle 从正东逆时针，与我们的约定一致
            r = math.radians(ang)
            groups.setdefault(vid, []).append(Frame(
                t=t, x=x, y=y,
                vx=sp * math.cos(r), vy=sp * math.sin(r),
                heading=ang,
                lane=veh.get("lane"),
            ))

    if not groups:
        raise TrajectoryError("FCD 中没有 vehicle 记录: %s" % path)

    hz = 1.0
    if len(times) > 2:
        ts = sorted(set(times))
        diffs = [b - a for a, b in zip(ts, ts[1:]) if b - a > 1e-9]
        if diffs:
            diffs.sort()
            hz = round(1.0 / diffs[len(diffs) // 2], 3)

    tracks = [Trajectory(id=k, frames=sorted(v, key=lambda f: f.t))
              for k, v in groups.items()]
    tset = TrajectorySet(tracks=tracks, hz=hz,
                         source=os.path.basename(path),
                         coordinate_system="sumo",
                         extra={"timesteps": len(times)})
    tset.derive_all()
    return tset


# =====================================================================
# JSON
# =====================================================================
def load_json(path: str) -> TrajectorySet:
    if not os.path.exists(path):
        raise TrajectoryError("文件不存在: %s" % path)
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    if isinstance(d, dict) and "tracks" in d:
        return TrajectorySet.from_dict(d)
    if isinstance(d, list):
        return TrajectorySet(tracks=[Trajectory.from_dict(x) for x in d],
                             source=os.path.basename(path))
    raise TrajectoryError("无法识别的 JSON 结构：需要 {tracks: [...]} 或 [...]")


# =====================================================================
# 统一入口
# =====================================================================
def load(path: str, fmt: Optional[str] = None, **kw) -> TrajectorySet:
    """按扩展名或显式 fmt 加载。fmt: csv / fcd / json"""
    if fmt is None:
        low = path.lower()
        if low.endswith(".csv") or low.endswith(".txt"):
            fmt = "csv"
        elif low.endswith(".json"):
            fmt = "json"
        elif ".fcd" in low or low.endswith(".xml"):
            fmt = "fcd"
        else:
            raise TrajectoryError("无法从扩展名判断格式，请显式传 fmt: %s" % path)
    if fmt == "csv":
        return load_csv(path, **kw)
    if fmt == "fcd":
        return load_fcd(path)
    if fmt == "json":
        return load_json(path)
    raise TrajectoryError("未知格式: %s" % fmt)


# =====================================================================
# 导出
# =====================================================================
CANON_COLUMNS = ["id", "t", "x", "y", "vx", "vy", "ax", "ay",
                 "heading", "speed", "lane", "s"]


def save_csv(tset: TrajectorySet, path: str, with_header: bool = True) -> str:
    """导出为规范 CSV（UTF-8 BOM，Excel 可直接打开）。"""
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        if with_header:
            w.writerow(CANON_COLUMNS)
        for tr in tset.tracks:
            for fr in tr.frames:
                w.writerow([tr.id, round(fr.t, 6), round(fr.x, 6), round(fr.y, 6),
                            round(fr.vx, 6), round(fr.vy, 6),
                            round(fr.ax, 6), round(fr.ay, 6),
                            "" if fr.heading is None else round(fr.heading, 4),
                            round(fr.speed(), 6),
                            "" if fr.lane is None else fr.lane,
                            "" if fr.s is None else round(fr.s, 4)])
    return path


def save_json(tset: TrajectorySet, path: str, indent: int = 2) -> str:
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(tset.to_dict(), f, ensure_ascii=False, indent=indent)
    return path


def describe(tset: TrajectorySet) -> str:
    s = tset.summary()
    lines = [
        "轨迹集：%d 条轨迹，%d 帧，%g Hz" % (s["tracks"], s["total_frames"], s["hz"]),
        "来源：%s   坐标系：%s" % (tset.source or "(未知)", tset.coordinate_system),
        "时间跨度：%.2f s" % (s["time_span"][1] - s["time_span"][0]),
        "总里程：%.1f m" % s["path_length_total"],
    ]
    for t in tset.tracks[:5]:
        st = t.stats()
        lines.append("  - %s: %d 帧, %.1fs, 均速 %.1f m/s, 最高 %.1f m/s"
                     % (st["id"], st["frames"], st["duration"],
                        st["speed_mean"], st["speed_max"]))
    if len(tset.tracks) > 5:
        lines.append("  ... 共 %d 条" % len(tset.tracks))
    return "\n".join(lines)
