"""
road_builder.py - 参数化 OpenDRIVE 路网生成

补齐 PR #113 正文描述、但当时未实现的 generate_rich_xodr 能力。
在此之前，仓库里唯一的 .xodr 产出路径是 generate_sumo_network 调
netconvert 生成的规则网格，没有任何道路几何/车道数变化的控制能力。

7 种布局（与 #113 描述一致）：
    straight          直路
    highway_curve     高速弯道（带缓和曲线）
    s_curve           S 弯
    cross_junction    十字路口
    t_junction        丁字路口
    on_ramp_merge     匝道合流（车道数 1→2）
    off_ramp_diverge  匝道分流（车道数 2→1）

单位与坐标约定（沿用 scenario_ir 的约定）：
    长度 m，角度 °，车道右侧为负 lane_id（OpenDRIVE 约定）
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Sequence, Tuple

from scenariogeneration import xodr
from scenariogeneration.xodr import LaneDef

# 各布局的默认参数（同时作为参数空间文档）
DEFAULT_PARAMS: Dict[str, Dict[str, Any]] = {
    "straight":         {"length": 200.0, "lanes": 2, "lane_width": 3.5},
    "highway_curve":    {"radius": 250.0, "angle": 90.0, "lanes": 2,
                         "lane_width": 3.5, "spiral_len": 40.0},
    "s_curve":          {"radius": 200.0, "angle": 45.0, "lanes": 2,
                         "lane_width": 3.5, "spiral_len": 30.0},
    "cross_junction":   {"arm_length": 80.0, "radius": 12.0, "lanes": 2,
                         "lane_width": 3.5},
    "t_junction":       {"arm_length": 80.0, "radius": 12.0, "lanes": 2,
                         "lane_width": 3.5},
    "on_ramp_merge":    {"length": 200.0, "merge_len": 80.0, "lane_width": 3.5},
    "off_ramp_diverge": {"length": 200.0, "diverge_len": 80.0, "lane_width": 3.5},
}

LAYOUT_TITLES = {
    "straight": "直路",
    "highway_curve": "高速弯道",
    "s_curve": "S 弯",
    "cross_junction": "十字路口",
    "t_junction": "丁字路口",
    "on_ramp_merge": "匝道合流",
    "off_ramp_diverge": "匝道分流",
}


class RoadBuildError(RuntimeError):
    pass


# =====================================================================
# 几何辅助
# =====================================================================
def _clothoid_arc_clothoid(radius: float, angle: float,
                           spiral_len: float) -> List[Any]:
    """缓和曲线 + 圆弧 + 缓和曲线，比纯圆弧更接近真实道路。"""
    if radius <= 0:
        raise RoadBuildError("radius 必须为正")
    if spiral_len <= 0:
        # 退化为纯圆弧
        arc_len = radius * abs(angle) * 3.141592653589793 / 180.0
        return [xodr.Arc(1.0 / radius * (1 if angle >= 0 else -1), length=arc_len)]

    curv = 1.0 / radius * (1 if angle >= 0 else -1)
    arc_len = max(radius * abs(angle) * 3.141592653589793 / 180.0
                  - spiral_len, 1.0)
    return [xodr.Spiral(0.0, curv, length=spiral_len),
            xodr.Arc(curv, length=arc_len),
            xodr.Spiral(curv, 0.0, length=spiral_len)]


# =====================================================================
# 单路布局
# =====================================================================
def build_straight(p: Dict[str, Any]) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    length = float(p["length"])
    lanes = int(p["lanes"])
    w = float(p["lane_width"])
    odr = xodr.OpenDrive("straight")
    road = xodr.create_road(xodr.Line(length), 1,
                            left_lanes=lanes, right_lanes=lanes,
                            lane_width=w)
    odr.add_road(road)
    return odr, [road]


def build_highway_curve(p: Dict[str, Any]) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    radius = float(p["radius"])
    angle = float(p["angle"])
    lanes = int(p["lanes"])
    w = float(p["lane_width"])
    sl = float(p.get("spiral_len", 40.0))
    geos = _clothoid_arc_clothoid(radius, angle, sl)
    odr = xodr.OpenDrive("highway_curve")
    road = xodr.create_road(geos, 1, left_lanes=lanes, right_lanes=lanes,
                            lane_width=w)
    odr.add_road(road)
    return odr, [road]


def build_s_curve(p: Dict[str, Any]) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    """两段反向圆弧，中间各带缓和曲线。"""
    radius = float(p["radius"])
    angle = float(p["angle"])
    lanes = int(p["lanes"])
    w = float(p["lane_width"])
    sl = float(p.get("spiral_len", 30.0))
    geos = (_clothoid_arc_clothoid(radius, angle, sl)
            + _clothoid_arc_clothoid(radius, -angle, sl))
    odr = xodr.OpenDrive("s_curve")
    road = xodr.create_road(geos, 1, left_lanes=lanes, right_lanes=lanes,
                            lane_width=w)
    odr.add_road(road)
    return odr, [road]


# =====================================================================
# 车道数渐变（合流/分流）
# =====================================================================
def build_on_ramp_merge(p: Dict[str, Any]) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    """右侧新增一条车道（1 → 2），新车道宽度由 0 渐变到 lane_width。

    sub_lane 指"要创建的那条车道"。右侧车道 id 为负，-2 即外侧那条。
    """
    length = float(p["length"])
    merge_len = float(p["merge_len"])
    w = float(p["lane_width"])
    lane_defs = [LaneDef(0.0, merge_len, 1, 2, sub_lane=-2,
                         lane_start_widths=[w], lane_end_widths=[w, w])]
    odr = xodr.OpenDrive("on_ramp_merge")
    road = xodr.create_road(xodr.Line(length), 1,
                            left_lanes=1, right_lanes=lane_defs,
                            lane_width=w)
    odr.add_road(road)
    return odr, [road]


def build_off_ramp_diverge(p: Dict[str, Any]) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    """右侧减少一条车道（2 → 1），外侧车道宽度由 lane_width 渐变到 0。"""
    length = float(p["length"])
    div_len = float(p["diverge_len"])
    w = float(p["lane_width"])
    lane_defs = [LaneDef(0.0, div_len, 2, 1, sub_lane=-2,
                         lane_start_widths=[w, w], lane_end_widths=[w])]
    odr = xodr.OpenDrive("off_ramp_diverge")
    road = xodr.create_road(xodr.Line(length), 1,
                            left_lanes=1, right_lanes=lane_defs,
                            lane_width=w)
    odr.add_road(road)
    return odr, [road]


# =====================================================================
# 路口布局
# =====================================================================
def _build_junction(p: Dict[str, Any], angles_deg: Sequence[float], name: str
                    ) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    """按给定角度列表构造路口。

    角度是各条臂向外伸出的方向（度，0=东，逆时针）。
    十字路口用 [0, 90, 180, 270]；
    丁字路口用 [0, 180, 270]（横杠东西向 + 竖杠朝南），
    而不是简单均分 120°——均分出来的是 Y 形，不是丁字。
    """
    arm_len = float(p["arm_length"])
    w = float(p["lane_width"])

    odr = xodr.OpenDrive(name)
    jc = xodr.CommonJunctionCreator(id=100, name="junction_%s" % name)
    roads: List[xodr.Road] = []

    # 每条臂一条路。用笛卡尔定位，而不是圆周定位：
    # 圆周定位 (radius, angle) 把 angle 当作道路自身朝向，而所有臂路都是
    # 从原点沿 +X 生成的，摆不出对称的十字/T 字（实测 4 臂被摆成了
    # 0°/116.6°/233.2°/349.9°，渲染出来像个 Y 路口）。
    # 笛卡尔定位的 heading 语义是"指向路口中心"，据此把每条臂摆到
    # 半径 arm_len 的圆周上、朝向圆心。
    for i, deg in enumerate(angles_deg):
        road = xodr.create_road(xodr.Line(arm_len), i + 1,
                                left_lanes=1, right_lanes=1, lane_width=w)
        odr.add_road(road)
        roads.append(road)

        ang = math.radians(deg)
        px = arm_len * math.cos(ang)
        py = arm_len * math.sin(ang)
        heading = ang + math.pi          # 指向圆心
        jc.add_incoming_road_cartesian_geometry(
            road, px, py, heading, road_connection="successor")

    # 相邻臂两两互连（双向）
    n = len(angles_deg)
    for i in range(n):
        a = i + 1
        b = (i + 1) % n + 1
        for l1 in (1, -1):
            jc.add_connection(a, b, lane_one_id=l1, lane_two_id=-l1)

    odr.add_junction_creator(jc)
    return odr, roads


def build_cross_junction(p: Dict[str, Any]):
    """十字路口：四条臂朝东南西北。"""
    return _build_junction(p, (0.0, 90.0, 180.0, 270.0), "cross_junction")


def build_t_junction(p: Dict[str, Any]):
    """丁字路口：横杠东西向 + 竖杠朝南。"""
    return _build_junction(p, (0.0, 180.0, 270.0), "t_junction")


# =====================================================================
# 注册表与入口
# =====================================================================
_BUILDERS = {
    "straight": build_straight,
    "highway_curve": build_highway_curve,
    "s_curve": build_s_curve,
    "cross_junction": build_cross_junction,
    "t_junction": build_t_junction,
    "on_ramp_merge": build_on_ramp_merge,
    "off_ramp_diverge": build_off_ramp_diverge,
}


def list_layouts() -> List[Dict[str, Any]]:
    return [{"name": k, "title": LAYOUT_TITLES[k],
             "defaults": dict(DEFAULT_PARAMS[k])}
            for k in _BUILDERS]


def build(layout: str, params: Optional[Dict[str, Any]] = None
          ) -> Tuple[xodr.OpenDrive, List[xodr.Road]]:
    if layout not in _BUILDERS:
        raise RoadBuildError("未知布局 %r，可用: %s"
                             % (layout, sorted(_BUILDERS)))
    p = dict(DEFAULT_PARAMS[layout])
    p.update(params or {})
    odr, roads = _BUILDERS[layout](p)

    # 这一步必须无条件调用。
    # PlanView.add_geometry() 只把几何存进 _raw_geometries，而
    # PlanView.get_element() 遍历的是 _adjusted_geometries —— 不调
    # adjust_roads_and_lanes() 的话写出的 <planView/> 是空的，
    # 生成的文件对 CARLA / SUMO 都是无效的。
    odr.adjust_roads_and_lanes()
    return odr, roads


def write(layout: str, path: str,
          params: Optional[Dict[str, Any]] = None) -> str:
    odr, _ = build(layout, params)
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    odr.write_xml(path)
    return path


def _effective_lanes(section, side: str) -> int:
    """统计某一侧宽度大于 0 的车道数。

    车道渐变是通过宽度 0 表达"尚未存在/已消失"的，
    直接数 <lane> 元素会把零宽车道也算进去，造成误读。
    """
    el = section.find(side)
    if el is None:
        return 0
    n = 0
    for lane in el.findall("lane"):
        w = lane.find("width")
        try:
            if w is not None and float(w.get("a", 0)) > 1e-6:
                n += 1
        except (TypeError, ValueError):
            pass
    return n


def inspect_road(path: str) -> Dict[str, Any]:
    """回读校验：Road 数、车道数、车道数随 s 的变化、以及几何是否存在。

    geometries 必须与 road_count 一致：如果 <planView/> 是空的，说明
    生成时漏了 adjust_roads_and_lanes()，文件无效。
    """
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    roads = root.findall("road")
    info = []
    total_geoms = 0
    for r in roads:
        lsecs = r.findall(".//laneSection")
        profile = []
        for s in lsecs:
            profile.append({
                "s": s.get("s"),
                "left": _effective_lanes(s, "left"),
                "right": _effective_lanes(s, "right"),
            })
        first = lsecs[0] if lsecs else None
        n_geom = len(r.findall("./planView/geometry"))
        total_geoms += n_geom
        info.append({
            "id": r.get("id"),
            "length": r.get("length"),
            "lane_sections": len(lsecs),
            "lanes_first": (_effective_lanes(first, "left")
                            + _effective_lanes(first, "right")) if first else 0,
            "lane_profile": profile,
            "geometries": n_geom,
        })
    juncs = root.findall("junction")
    valid = total_geoms >= len(roads) and len(roads) > 0
    return {
        "road_count": len(roads),
        "junction_count": len(juncs),
        "geometry_count": total_geoms,
        "roads": info,
        "size_bytes": os.path.getsize(path),
        "valid": valid,
        "error": "" if valid else "有空 planView，几何没写进去（缺 adjust_roads_and_lanes）",
    }
