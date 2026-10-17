"""
trajectory.py - 车辆轨迹数据模型

目标②（基于实采的有限数据生成批量合成数据）的地基。

原项目没有任何轨迹数据结构：FCD 解析只取了 vehicle.speed 一列
（sumo/src/mcp_tools/analysis.py:20），x/y/angle/lane 全部丢弃。
没有统一的数据模型，就谈不上导入、增强、批量导出。

坐标与单位契约（显式声明，原项目缺失）
--------------------------------------
- 位置：米 (m)
- 速度：米/秒 (m/s)，注意 NGSIM 的 v_Vel 是英尺/秒
- 加速度：米/秒² (m/s²)
- 时间：秒 (s)，帧号从 0 开始
- 朝向 heading：度 (°)，0 = 正东（+x），逆时针为正
- 侧向偏移 lateral：米，相对车道中心线，左正右负
- 车道 ID：沿用 OpenDRIVE 约定，右车道为负

帧率由 TrajectorySet.hz 统一声明；加载时会校验相邻帧间隔是否与 hz 相符。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

FT_TO_M = 0.3048
MPH_TO_MS = 0.44704
KMH_TO_MS = 1.0 / 3.6


class TrajectoryError(ValueError):
    pass


# =====================================================================
# 单帧
# =====================================================================
@dataclass
class Frame:
    """一帧状态。t 为相对开始的时间（秒）。"""
    t: float
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    heading: Optional[float] = None   # 度
    lane: Optional[str] = None
    s: Optional[float] = None         # 沿车道纵向弧长

    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def to_dict(self) -> Dict[str, Any]:
        d = {"t": round(self.t, 6), "x": round(self.x, 6), "y": round(self.y, 6),
             "vx": round(self.vx, 6), "vy": round(self.vy, 6),
             "ax": round(self.ax, 6), "ay": round(self.ay, 6)}
        if self.heading is not None:
            d["heading"] = round(self.heading, 4)
        if self.lane is not None:
            d["lane"] = self.lane
        if self.s is not None:
            d["s"] = round(self.s, 4)
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Frame":
        return cls(
            t=float(d["t"]), x=float(d["x"]), y=float(d["y"]),
            vx=float(d.get("vx", 0.0)), vy=float(d.get("vy", 0.0)),
            ax=float(d.get("ax", 0.0)), ay=float(d.get("ay", 0.0)),
            heading=None if d.get("heading") is None else float(d["heading"]),
            lane=None if d.get("lane") is None else str(d["lane"]),
            s=None if d.get("s") is None else float(d["s"]),
        )


# =====================================================================
# 单条轨迹
# =====================================================================
@dataclass
class Trajectory:
    """一个交通参与者的完整轨迹。"""
    id: str
    frames: List[Frame] = field(default_factory=list)
    kind: str = "vehicle"       # vehicle / pedestrian / bicycle ...
    meta: Dict[str, Any] = field(default_factory=dict)

    # ---------------------------------------------------------- 基础
    def __len__(self) -> int:
        return len(self.frames)

    @property
    def duration(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        return self.frames[-1].t - self.frames[0].t

    def start(self) -> Frame:
        if not self.frames:
            raise TrajectoryError("轨迹 %s 无帧" % self.id)
        return self.frames[0]

    def end(self) -> Frame:
        if not self.frames:
            raise TrajectoryError("轨迹 %s 无帧" % self.id)
        return self.frames[-1]

    def path_length(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(self.frames, self.frames[1:]):
            total += math.hypot(b.x - a.x, b.y - a.y)
        return total

    def speeds(self) -> List[float]:
        return [f.speed() for f in self.frames]

    def accelerations(self) -> List[float]:
        return [math.hypot(f.ax, f.ay) for f in self.frames]

    # ---------------------------------------------------------- 派生量
    def derive_kinematics(self) -> None:
        """缺速度/加速度时用位置差分补上；缺朝向时用相邻点方向补上。"""
        n = len(self.frames)
        if n == 0:
            return
        if n >= 2:
            for i, f in enumerate(self.frames):
                if f.vx == 0.0 and f.vy == 0.0:
                    if i == 0:
                        a, b = self.frames[0], self.frames[1]
                    elif i == n - 1:
                        a, b = self.frames[-2], self.frames[-1]
                    else:
                        a, b = self.frames[i - 1], self.frames[i + 1]
                    dt = b.t - a.t
                    if dt > 0:
                        f.vx = (b.x - a.x) / dt
                        f.vy = (b.y - a.y) / dt
        # 加速度
        if n >= 2:
            for i, f in enumerate(self.frames):
                if f.ax == 0.0 and f.ay == 0.0:
                    if i == 0:
                        a, b = self.frames[0], self.frames[1]
                    elif i == n - 1:
                        a, b = self.frames[-2], self.frames[-1]
                    else:
                        a, b = self.frames[i - 1], self.frames[i + 1]
                    dt = b.t - a.t
                    if dt > 0:
                        f.ax = (b.vx - a.vx) / dt
                        f.ay = (b.vy - a.vy) / dt
        # 朝向
        for f in self.frames:
            if f.heading is None:
                if abs(f.vx) + abs(f.vy) > 1e-9:
                    f.heading = math.degrees(math.atan2(f.vy, f.vx)) % 360.0

    def resample(self, hz: float) -> "Trajectory":
        """线性插值到固定帧率。"""
        if hz <= 0:
            raise TrajectoryError("hz 必须为正")
        if len(self.frames) < 2:
            return Trajectory(self.id, list(self.frames), self.kind, dict(self.meta))
        dt = 1.0 / hz
        t0, t1 = self.frames[0].t, self.frames[-1].t
        n = int(round((t1 - t0) * hz)) + 1
        out: List[Frame] = []
        j = 0
        for i in range(n):
            t = t0 + i * dt
            while j + 2 < len(self.frames) and self.frames[j + 1].t < t:
                j += 1
            a, b = self.frames[j], self.frames[min(j + 1, len(self.frames) - 1)]
            span = b.t - a.t
            r = 0.0 if span <= 0 else max(0.0, min(1.0, (t - a.t) / span))
            out.append(Frame(
                t=t,
                x=a.x + (b.x - a.x) * r,
                y=a.y + (b.y - a.y) * r,
                vx=a.vx + (b.vx - a.vx) * r,
                vy=a.vy + (b.vy - a.vy) * r,
                ax=a.ax + (b.ax - a.ax) * r,
                ay=a.ay + (b.ay - a.ay) * r,
                heading=None if a.heading is None or b.heading is None
                else a.heading + (b.heading - a.heading) * r,
                lane=a.lane,
            ))
        return Trajectory(self.id, out, self.kind, dict(self.meta))

    # ---------------------------------------------------------- 序列化
    def to_dict(self) -> Dict[str, Any]:
        return {"id": self.id, "kind": self.kind, "meta": dict(self.meta),
                "frames": [f.to_dict() for f in self.frames]}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Trajectory":
        return cls(id=str(d["id"]), kind=d.get("kind", "vehicle"),
                   meta=d.get("meta", {}),
                   frames=[Frame.from_dict(x) for x in d.get("frames", [])])

    def stats(self) -> Dict[str, Any]:
        sp = self.speeds()
        ac = self.accelerations()
        return {
            "id": self.id,
            "frames": len(self.frames),
            "duration": round(self.duration, 3),
            "path_length": round(self.path_length(), 3),
            "speed_mean": round(sum(sp) / len(sp), 3) if sp else 0.0,
            "speed_max": round(max(sp), 3) if sp else 0.0,
            "speed_min": round(min(sp), 3) if sp else 0.0,
            "accel_max": round(max(ac), 3) if ac else 0.0,
        }


# =====================================================================
# 轨迹集合
# =====================================================================
@dataclass
class TrajectorySet:
    """多参与者 + 元数据。这是导入器统一输出的类型。"""
    tracks: List[Trajectory] = field(default_factory=list)
    hz: float = 10.0
    source: str = ""
    coordinate_system: str = "local"   # local / global / carla / sumo
    units: Dict[str, str] = field(default_factory=lambda: {
        "position": "m", "speed": "m/s", "acceleration": "m/s^2", "time": "s"})
    extra: Dict[str, Any] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.tracks)

    def add(self, tr: Trajectory) -> None:
        self.tracks.append(tr)

    def by_id(self, tid: str) -> Optional[Trajectory]:
        for t in self.tracks:
            if t.id == tid:
                return t
        return None

    def derive_all(self) -> None:
        for t in self.tracks:
            t.derive_kinematics()

    def resample_all(self, hz: float) -> "TrajectorySet":
        return TrajectorySet(
            tracks=[t.resample(hz) for t in self.tracks],
            hz=hz, source=self.source,
            coordinate_system=self.coordinate_system,
            units=dict(self.units), extra=dict(self.extra))

    def time_span(self) -> Tuple[float, float]:
        if not self.tracks:
            return (0.0, 0.0)
        return (min(t.start().t for t in self.tracks),
                max(t.end().t for t in self.tracks))

    # ---------------------------------------------------------- 序列化
    def to_dict(self) -> Dict[str, Any]:
        return {
            "hz": self.hz,
            "source": self.source,
            "coordinate_system": self.coordinate_system,
            "units": self.units,
            "extra": self.extra,
            "track_count": len(self.tracks),
            "tracks": [t.to_dict() for t in self.tracks],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TrajectorySet":
        return cls(
            tracks=[Trajectory.from_dict(x) for x in d.get("tracks", [])],
            hz=float(d.get("hz", 10.0)),
            source=d.get("source", ""),
            coordinate_system=d.get("coordinate_system", "local"),
            units=d.get("units", {}),
            extra=d.get("extra", {}),
        )

    def summary(self) -> Dict[str, Any]:
        lo, hi = self.time_span()
        return {
            "tracks": len(self.tracks),
            "hz": self.hz,
            "source": self.source,
            "time_span": [round(lo, 3), round(hi, 3)],
            "total_frames": sum(len(t) for t in self.tracks),
            "path_length_total": round(sum(t.path_length() for t in self.tracks), 3),
        }


# =====================================================================
# 几何辅助（增强算子会用到）
# =====================================================================
def heading_between(a: Frame, b: Frame) -> float:
    return math.degrees(math.atan2(b.y - a.y, b.x - a.x)) % 360.0


def rotate_point(x: float, y: float, deg: float,
                 ox: float = 0.0, oy: float = 0.0) -> Tuple[float, float]:
    """绕 (ox, oy) 旋转 deg 度。"""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    dx, dy = x - ox, y - oy
    return (ox + dx * c - dy * s, oy + dx * s + dy * c)


def translate_trajectory(tr: Trajectory, dx: float, dy: float) -> Trajectory:
    out = Trajectory(tr.id, [], tr.kind, dict(tr.meta))
    for f in tr.frames:
        g = Frame(**{**f.__dict__})
        g.x += dx
        g.y += dy
        out.frames.append(g)
    return out
