"""
trajectory_to_sumo.py - 轨迹 → SUMO 需求文件

把实采/合成轨迹转成 SUMO 能吃的 rou.xml。

两条路径：
  1. 起终点法（默认）：每条轨迹只贡献一个 flow（起点位置 + 出发时刻 +
     出发速度 + 到达终点），由 SUMO 自己算路径。适合大批量、不需要
     逐帧还原的场景。
  2. 完整路径法：把轨迹按距离抽稀成 edge 序列，用 <route> 显式指定。
     需要 netconvert 提供 edge id 映射，实现更重，暂未启用。

注意本模块只产出 XML，不校验路网；轨迹点是否落在路网上由调用方或
SUMO 的 duarouter 负责（它会报 unused/ignored 的 flow）。
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Tuple

from trajectory import Trajectory, TrajectorySet


class SumoExportError(ValueError):
    pass


def trajectory_to_flow_xml(
    tset: TrajectorySet,
    out_path: str,
    vtype_id: str = "passenger",
    depart_lane: str = "best",
    max_vehicles: Optional[int] = None,
    begin_offset: float = 0.0,
    depart_interval: float = 0.5,
) -> Dict[str, Any]:
    """把轨迹集写成 rou.xml（每条轨迹一个 flow）。

    depart_interval: 当所有轨迹的起始时刻相同时（例如各自从 0 开始的
    独立样本），按这个间隔依次发出，避免上百辆车全在 t=0 涌出。
    若轨迹本身带相对时间差，则按实际时刻发出。

    返回统计信息，含被跳过的轨迹与原因。
    """
    if not tset.tracks:
        raise SumoExportError("轨迹集为空")

    root = ET.Element("routes")
    ET.SubElement(root, "vType", {
        "id": vtype_id, "vClass": "passenger",
        "length": "5.0", "maxSpeed": "55.0",
        "accel": "2.6", "decel": "4.5", "sigma": "0.5",
    })

    written: List[str] = []
    skipped: List[Dict[str, str]] = []
    t_min = min(t.start().t for t in tset.tracks)
    t_max = max(t.start().t for t in tset.tracks)
    # 起始时刻是否有实际差异
    has_spread = (t_max - t_min) > 0.01

    for idx, tr in enumerate(tset.tracks):
        if max_vehicles is not None and len(written) >= max_vehicles:
            break
        if len(tr) < 2:
            skipped.append({"id": tr.id, "reason": "帧数不足"})
            continue
        start = tr.start()
        if has_spread:
            depart = begin_offset + (start.t - t_min)
        else:
            depart = begin_offset + idx * depart_interval
        depart = round(depart, 3)
        speed = round(start.speed(), 3)
        if speed < 0.1:
            # 静止起步：给一个很小的出发速度，避免 SUMO 拒绝
            speed = 0.1
        ET.SubElement(root, "flow", {
            "id": tr.id,
            "type": vtype_id,
            "begin": str(depart),
            "end": str(depart),
            "number": "1",
            "departSpeed": str(speed),
            "departLane": depart_lane,
            "departPos": "base",
            # 自定义属性记录来源坐标，供后续匹配路网
            "from": "%.3f,%.3f" % (start.x, start.y),
        })
        written.append(tr.id)

    tree = ET.ElementTree(root)
    d = os.path.dirname(out_path)
    if d:
        os.makedirs(d, exist_ok=True)
    tree.write(out_path, encoding="UTF-8", xml_declaration=True)

    return {
        "file": out_path,
        "written": len(written),
        "skipped": len(skipped),
        "skip_details": skipped[:10],
        "size_bytes": os.path.getsize(out_path),
    }


def trajectory_to_route_skeleton(
    tr: Trajectory, step_m: float = 20.0
) -> List[Tuple[float, float]]:
    """按距离抽稀轨迹，得到一串关键点（供手工映射成 edge 序列）。"""
    if len(tr.frames) < 2:
        return []
    pts = [(tr.frames[0].x, tr.frames[0].y)]
    acc = 0.0
    last = tr.frames[0]
    for f in tr.frames[1:]:
        acc += math.hypot(f.x - last.x, f.y - last.y)
        last = f
        if acc >= step_m:
            pts.append((f.x, f.y))
            acc = 0.0
    if (tr.frames[-1].x, tr.frames[-1].y) != pts[-1]:
        pts.append((tr.frames[-1].x, tr.frames[-1].y))
    return pts


def inspect_flow_xml(path: str) -> Dict[str, Any]:
    """回读校验。"""
    root = ET.parse(path).getroot()
    flows = root.findall("flow")
    begins = []
    for f in flows:
        try:
            begins.append(float(f.get("begin", 0)))
        except ValueError:
            pass
    return {
        "flows": len(flows),
        "vtypes": len(root.findall("vType")),
        "begin_min": min(begins) if begins else None,
        "begin_max": max(begins) if begins else None,
        "size_bytes": os.path.getsize(path),
    }
