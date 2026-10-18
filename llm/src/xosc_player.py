"""
xosc_player.py - OpenSCENARIO 场景播放器

仓库里没有 ScenarioRunner，`.xosc` 一直是"生成完就躺在那儿"。这个模块
解析生成的 `.xosc`，把实体布景进 CARLA，按触发条件下发动作，跑固定时长。

支持范围
--------
实体      Vehicle / Pedestrian
初始化    TeleportAction -> LanePosition / WorldPosition
动作      SpeedAction（step / linear 渐变）
          LongitudinalDistanceAction（跟驰）
          LaneChangeAction（转向控制实现）
          LaneOffsetAction（车道内横向偏移）
条件      SimulationTimeCondition（仿真时间）
          TimeHeadwayCondition（车头时距）
          DistanceCondition（到某位置的距离）
          SpeedCondition（自身速度）
          StandStillCondition（静止一段时间）
          ReachPositionCondition（到达某位置）
触发边沿  rising / falling / none / risingOrFalling

不支持（记入 unsupported，不静默跳过）
------------------------------------
CollisionCondition、TimeToCollisionCondition、RelativeSpeedCondition、
AccelerationCondition、Story 分支、EntitySelection、Catalog、
TrafficSignalControllerAction、FollowTrajectoryAction

已知简化
--------
- 跟驰用 P 控制而非 OSC 的完整语义
- 变道用纯追踪转向，不保证与真实车辆动力学一致
- ConditionEdge 只做基本的沿检测（按上一帧比较）
- 不考虑 freespace / alongroute 的坐标语义，一律按直线距离
"""
from __future__ import annotations

import math
import os
import queue
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import carla


class XoscPlayerError(RuntimeError):
    pass


# =====================================================================
# 数据模型
# =====================================================================
@dataclass
class XoscEntity:
    name: str
    kind: str = "vehicle"
    position_kind: str = "lane"
    lane_s: float = 0.0
    lane_offset: float = 0.0
    lane_id: str = "-1"
    road_id: str = "1"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    h: float = 0.0
    initial_speed: float = 0.0


@dataclass
class XoscPosition:
    """条件里引用的位置。"""
    kind: str = "world"        # world / lane
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    s: float = 0.0
    lane_offset: float = 0.0


@dataclass
class XoscTrigger:
    kind: str = "time"         # time / distance / time_headway / speed / standstill / reach_position
    value: float = 0.0
    rule: str = "greaterThan"
    entity_ref: str = ""
    position: Optional[XoscPosition] = None
    tolerance: float = 1.0
    edge: str = "rising"
    # 运行时状态
    _prev: Optional[bool] = None
    _since: Optional[float] = None

    def reset_runtime(self) -> None:
        self._prev = None
        self._since = None


@dataclass
class XoscAction:
    kind: str = "speed"        # speed / follow / lane_change / lane_offset
    value: float = 0.0
    shape: str = "step"
    dynamics_value: float = 0.0
    target_entity: str = ""
    lane_delta: int = 0
    # follow_trajectory 用：[[t, x, y], ...]
    points: List[List[float]] = field(default_factory=list)
    # 运行时状态
    _started_at: Optional[float] = None
    _start_value: float = 0.0
    _done: bool = False


@dataclass
class XoscEvent:
    name: str
    entity: str
    trigger: XoscTrigger
    actions: List[XoscAction] = field(default_factory=list)
    fired: bool = False


@dataclass
class XoscScenario:
    name: str = ""
    entities: List[XoscEntity] = field(default_factory=list)
    events: List[XoscEvent] = field(default_factory=list)
    stop_time: float = 30.0
    unsupported: List[str] = field(default_factory=list)

    def entity(self, name: str) -> Optional[XoscEntity]:
        for e in self.entities:
            if e.name == name:
                return e
        return None

    def reset_runtime(self) -> None:
        for ev in self.events:
            ev.fired = False
            ev.trigger.reset_runtime()
            for a in ev.actions:
                a._started_at = None
                a._start_value = 0.0
                a._done = False


# =====================================================================
# 解析
# =====================================================================
_RULES = {
    "greaterThan": lambda a, b: a > b,
    "lessThan": lambda a, b: a < b,
    "equalTo": lambda a, b: abs(a - b) < 1e-6,
    "greaterOrEqual": lambda a, b: a >= b,
    "lessOrEqual": lambda a, b: a <= b,
    "notEqualTo": lambda a, b: abs(a - b) >= 1e-6,
}

_SUPPORTED_CONDITIONS = {
    "SimulationTimeCondition": "time",
    "TimeHeadwayCondition": "time_headway",
    "DistanceCondition": "distance",
    "SpeedCondition": "speed",
    "StandStillCondition": "standstill",
    "ReachPositionCondition": "reach_position",
}

_KNOWN_UNSUPPORTED = {
    "CollisionCondition", "TimeToCollisionCondition",
    "RelativeSpeedCondition", "AccelerationCondition",
    "OffroadCondition", "EndOfRoadCondition", "TraveledDistanceCondition",
    "RelativeDistanceCondition", "RelativeAngleCondition",
}


def _find_target_speed(node) -> Optional[float]:
    """找速度目标值。OSC 1.0 用 AbsoluteSpeed，1.1+ 用 AbsoluteTargetSpeed。"""
    for tag in ("AbsoluteTargetSpeed", "AbsoluteSpeed"):
        el = node.find(".//" + tag)
        if el is not None:
            try:
                return float(el.get("value", 0) or 0)
            except (TypeError, ValueError):
                return None
    return None


def _parse_position(pos_el) -> Optional[XoscPosition]:
    if pos_el is None:
        return None
    wp = pos_el.find("WorldPosition")
    if wp is not None:
        return XoscPosition(kind="world",
                            x=float(wp.get("x", 0) or 0),
                            y=float(wp.get("y", 0) or 0),
                            z=float(wp.get("z", 0) or 0))
    lp = pos_el.find("LanePosition")
    if lp is not None:
        return XoscPosition(kind="lane",
                            s=float(lp.get("s", 0) or 0),
                            lane_offset=float(lp.get("offset", 0) or 0))
    return None


def _parse_condition(cond_el, fallback_entity: str) -> Optional[XoscTrigger]:
    """把一个 <Condition> 解析成 XoscTrigger；不支持的返回 None 并记录。"""
    edge = cond_el.get("conditionEdge", "rising")

    for deep in cond_el.iter():
        tag = deep.tag
        if tag == "SimulationTimeCondition":
            return XoscTrigger(kind="time",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "greaterThan"),
                               edge=edge)
        if tag == "TimeHeadwayCondition":
            return XoscTrigger(kind="time_headway",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               entity_ref=deep.get("entityRef", fallback_entity),
                               edge=edge)
        if tag == "DistanceCondition":
            return XoscTrigger(kind="distance",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               position=_parse_position(deep.find("Position")),
                               edge=edge)
        if tag == "SpeedCondition":
            return XoscTrigger(kind="speed",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "greaterThan"),
                               edge=edge)
        if tag == "StandStillCondition":
            return XoscTrigger(kind="standstill",
                               value=float(deep.get("duration", 0) or 0),
                               edge=edge)
        if tag == "ReachPositionCondition":
            return XoscTrigger(kind="reach_position",
                               position=_parse_position(deep.find("Position")),
                               tolerance=float(deep.get("tolerance", 1.0) or 1.0),
                               edge=edge)
        if tag == "RelativeSpeedCondition":
            return XoscTrigger(kind="relative_speed",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               entity_ref=deep.get("entityRef", ""),
                               edge=edge)
        if tag == "TimeToCollisionCondition":
            # TTC 的目标可能写成 EntityRef，也可能写成 Position。
            # scenariogeneration 目前输出的是 Position，所以两种都收。
            # 注意不要 fallback 成触发主体自己——那会让距离恒为 0。
            ref = deep.get("entityRef", "")
            tgt = deep.find("TimeToCollisionConditionTarget")
            if tgt is not None:
                er = tgt.find("EntityRef")
                if er is not None:
                    ref = er.get("entityRef", ref)
            return XoscTrigger(kind="time_to_collision",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               entity_ref=ref,
                               position=_parse_position(deep.find(".//Position")),
                               edge=edge)
        if tag == "CollisionCondition":
            er = deep.find(".//EntityRef")
            ref = er.get("entityRef", "") if er is not None else ""
            return XoscTrigger(kind="collision", rule="equalTo",
                               entity_ref=ref, edge=edge)
    return None


def _parse_actions(pa_list, unsupported: List[str], event_name: str
                   ) -> List[XoscAction]:
    out: List[XoscAction] = []
    for pa in pa_list:
        # ---- 轨迹跟随：PrivateAction > RoutingAction > FollowTrajectoryAction
        routing = pa.find("RoutingAction")
        if routing is not None:
            fta = routing.find("FollowTrajectoryAction")
            if fta is not None:
                pts: List[List[float]] = []
                for v in fta.findall(".//Vertex"):
                    wp = v.find(".//WorldPosition")
                    if wp is None:
                        continue
                    try:
                        pts.append([float(v.get("time", 0) or 0),
                                    float(wp.get("x", 0) or 0),
                                    float(wp.get("y", 0) or 0)])
                    except ValueError:
                        continue
                if len(pts) >= 2:
                    out.append(XoscAction(kind="follow_trajectory", points=pts))
                else:
                    unsupported.append(
                        "事件 %s 的轨迹跟随没有可用的 WorldPosition 顶点"
                        % event_name)
                continue
            unsupported.append("事件 %s 的路由动作未识别" % event_name)
            continue

        longi = pa.find("LongitudinalAction")
        if longi is not None:
            sp = longi.find("SpeedAction")
            if sp is not None:
                # 先看是不是相对速度（RelativeTargetSpeed）
                rel = sp.find(".//RelativeTargetSpeed")
                target = _find_target_speed(sp)
                if rel is not None:
                    try:
                        rv = float(rel.get("value", 0) or 0)
                    except ValueError:
                        rv = 0.0
                    shape = "step"
                    dv = 0.0
                    dyn = sp.find("SpeedActionDynamics")
                    if dyn is not None:
                        shape = dyn.get("dynamicsShape", "step") or "step"
                        try:
                            dv = float(dyn.get("value", 0) or 0)
                        except ValueError:
                            dv = 0.0
                    out.append(XoscAction(kind="relative_speed", value=rv,
                                          shape=shape, dynamics_value=dv,
                                          target_entity=rel.get("entityRef", "")))
                    continue
                if target is not None:
                    shape = "step"
                    dv = 0.0
                    dyn = sp.find("SpeedActionDynamics")
                    if dyn is not None:
                        shape = dyn.get("dynamicsShape", "step") or "step"
                        try:
                            dv = float(dyn.get("value", 0) or 0)
                        except ValueError:
                            dv = 0.0
                    out.append(XoscAction(kind="speed", value=target,
                                          shape=shape, dynamics_value=dv))
                    continue
            dist = longi.find("LongitudinalDistanceAction")
            if dist is not None:
                try:
                    gap = float(dist.get("distance", 0) or 0)
                except ValueError:
                    gap = 0.0
                out.append(XoscAction(kind="follow", value=gap,
                                      target_entity=dist.get("entityRef", "")))
                continue
            unsupported.append("事件 %s 的纵向动作未识别" % event_name)
            continue

        lat = pa.find("LateralAction")
        if lat is not None:
            ld = lat.find("LateralDistanceAction")
            if ld is not None:
                try:
                    d = float(ld.get("distance", 0) or 0)
                except ValueError:
                    d = 0.0
                out.append(XoscAction(kind="lateral_distance", value=d,
                                      target_entity=ld.get("entityRef", "")))
                continue
            lc = lat.find("LaneChangeAction")
            if lc is not None:
                lc_el = lc.find(".//RelativeTargetLane")
                delta = 0
                if lc_el is not None:
                    try:
                        delta = int(float(lc_el.get("value", 0) or 0))
                    except ValueError:
                        delta = 0
                else:
                    tgt = lc.find(".//AbsoluteTargetLane")
                    if tgt is not None:
                        # AbsoluteTargetLane 只给目标车道号，无法直接算增量
                        unsupported.append(
                            "事件 %s 的绝对目标车道变道未执行" % event_name)
                        continue
                shape = "linear"
                dv = 1.0
                dyn = lc.find("LaneChangeActionDynamics")
                if dyn is not None:
                    shape = dyn.get("dynamicsShape", "linear") or "linear"
                    try:
                        dv = float(dyn.get("value", 1.0) or 1.0)
                    except ValueError:
                        dv = 1.0
                out.append(XoscAction(kind="lane_change", value=float(delta),
                                      shape=shape, dynamics_value=dv))
                continue

            lo = lat.find("LaneOffsetAction")
            if lo is not None:
                target_el = lo.find(".//AbsoluteLaneOffset")
                val = 0.0
                if target_el is not None:
                    try:
                        val = float(target_el.get("value", 0) or 0)
                    except ValueError:
                        val = 0.0
                shape = "linear"
                dv = 1.0
                dyn = lo.find("LaneOffsetActionDynamics")
                if dyn is not None:
                    shape = dyn.get("dynamicsShape", "linear") or "linear"
                    try:
                        dv = float(dyn.get("value", 1.0) or 1.0)
                    except ValueError:
                        dv = 1.0
                out.append(XoscAction(kind="lane_offset", value=val,
                                      shape=shape, dynamics_value=dv))
                continue

            unsupported.append("事件 %s 的横向动作未识别" % event_name)
            continue

        unsupported.append("事件 %s 的动作类型未识别" % event_name)
    return out


def parse_xosc(path: str) -> XoscScenario:
    if not os.path.exists(path):
        raise XoscPlayerError("文件不存在: %s" % path)

    root = ET.parse(path).getroot()
    sc = XoscScenario(name=os.path.basename(path))

    # ---- 实体 ----
    for obj in root.findall("./Entities/ScenarioObject"):
        name = obj.get("name", "")
        inner = list(obj)
        if not inner:
            sc.unsupported.append("实体 %s 没有类型节点" % name)
            continue
        kind = {"Vehicle": "vehicle", "Pedestrian": "pedestrian"}.get(
            inner[0].tag, "unknown")
        sc.entities.append(XoscEntity(name=name, kind=kind))
    if not sc.entities:
        raise XoscPlayerError("文件里没有实体")

    # ---- 初始化 ----
    for private in root.findall("./Storyboard/Init/Actions/Private"):
        ref = private.get("entityRef", "")
        ent = sc.entity(ref)
        if ent is None:
            continue
        for pa in private.findall("PrivateAction"):
            tele = pa.find("TeleportAction")
            if tele is not None:
                pos = _parse_position(tele.find("Position"))
                if pos is not None:
                    if pos.kind == "world":
                        ent.position_kind = "world"
                        ent.x, ent.y, ent.z = pos.x, pos.y, pos.z
                    else:
                        ent.position_kind = "lane"
                        ent.lane_s = pos.s
                        ent.lane_offset = pos.lane_offset
            longi = pa.find("LongitudinalAction")
            if longi is not None:
                v = _find_target_speed(longi)
                if v is not None:
                    ent.initial_speed = v

    # ---- 事件 ----
    for mg in root.findall("./Storyboard/Story/Act/ManeuverGroup"):
        actors = mg.findall("./Actors/EntityRef")
        if not actors:
            sc.unsupported.append("ManeuverGroup %s 没有 Actor" % mg.get("name"))
            continue
        actor = actors[0].get("entityRef", "")

        for ev in mg.findall("./Maneuver/Event"):
            name = ev.get("name", "")
            trig = None
            # 先找起始触发
            for cond in ev.findall("./StartTrigger/ConditionGroup/Condition"):
                trig = _parse_condition(cond, actor)
                if trig is not None:
                    break
                # 记录不支持的
                for deep in cond.iter():
                    if deep.tag in _KNOWN_UNSUPPORTED:
                        sc.unsupported.append(
                            "事件 %s 的条件 %s 未执行" % (name, deep.tag))
                        break
            if trig is None:
                if not any(name in u for u in sc.unsupported):
                    sc.unsupported.append("事件 %s 没有可识别的触发条件" % name)
                continue

            actions = _parse_actions(ev.findall("./Action/PrivateAction"),
                                     sc.unsupported, name)
            if not actions:
                continue
            sc.events.append(XoscEvent(name=name, entity=actor,
                                       trigger=trig, actions=actions))

    # ---- 结束时间 ----
    stop = root.find("./Storyboard/StopTrigger")
    if stop is not None:
        el = stop.find(".//SimulationTimeCondition")
        if el is not None:
            try:
                sc.stop_time = float(el.get("value", 30) or 30)
            except ValueError:
                pass

    return sc


# =====================================================================
# 运行时
# =====================================================================
def _yaw_of(actor) -> float:
    return actor.get_transform().rotation.yaw


def _loc_of(actor) -> carla.Location:
    return actor.get_transform().location


def _distance(a: carla.Location, b: carla.Location) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _resolve_position(world: carla.World, pos: XoscPosition) -> carla.Location:
    if pos.kind == "world":
        return carla.Location(x=pos.x, y=pos.y, z=pos.z)
    wps = world.get_map().generate_waypoints(4.0)
    if not wps:
        raise XoscPlayerError("路网没有车道点，无法解析 LanePosition")
    target = carla.Location(x=pos.s, y=pos.lane_offset, z=0.0)
    wp = min(wps, key=lambda w: w.transform.location.distance(target))
    return wp.transform.location


def _speed_of(actor) -> float:
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def _evaluate(world: carla.World, trig: XoscTrigger, actors: Dict[str, carla.Actor],
              self_actor: carla.Actor, now: float) -> bool:
    """求值一个触发条件（不含边沿检测）。"""
    rule = _RULES.get(trig.rule, _RULES["greaterThan"])

    if trig.kind == "time":
        return rule(now, trig.value)

    if trig.kind == "speed":
        return rule(_speed_of(self_actor), trig.value)

    if trig.kind == "standstill":
        slow = _speed_of(self_actor) < 0.15
        if not slow:
            trig._since = None
            return False
        if trig._since is None:
            trig._since = now
        return (now - trig._since) >= trig.value

    if trig.kind == "time_headway":
        other = actors.get(trig.entity_ref)
        if other is None:
            return False
        gap = _distance(_loc_of(self_actor), _loc_of(other))
        sp = max(_speed_of(self_actor), 0.1)
        return rule(gap / sp, trig.value)

    if trig.kind == "distance":
        if trig.position is None:
            return False
        target = _resolve_position(world, trig.position)
        return rule(_distance(_loc_of(self_actor), target), trig.value)

    if trig.kind == "reach_position":
        if trig.position is None:
            return False
        target = _resolve_position(world, trig.position)
        return _distance(_loc_of(self_actor), target) <= max(trig.tolerance, 0.5)

    if trig.kind == "relative_speed":
        other = actors.get(trig.entity_ref) if trig.entity_ref else None
        if other is None or other is self_actor:
            return False
        return rule(_speed_of(self_actor) - _speed_of(other), trig.value)

    if trig.kind == "time_to_collision":
        other = actors.get(trig.entity_ref) if trig.entity_ref else None
        if other is not None and other is not self_actor:
            gap = _distance(_loc_of(self_actor), _loc_of(other))
            closing = _speed_of(self_actor) - _speed_of(other)
        elif trig.position is not None:
            # 目标是固定位置：按静止目标算，接近速度就是自身车速
            target = _resolve_position(world, trig.position)
            gap = _distance(_loc_of(self_actor), target)
            closing = _speed_of(self_actor)
        else:
            return False
        # 没有接近趋势时 TTC 视为无穷大
        ttc = gap / closing if closing > 0.1 else float("inf")
        return rule(ttc, trig.value)

    if trig.kind == "collision":
        other = actors.get(trig.entity_ref) if trig.entity_ref else None
        if other is None or other is self_actor:
            return False
        # 用包围球近似：两个实体中心距离小于各自半长之和
        gap = _distance(_loc_of(self_actor), _loc_of(other))
        return gap <= 2.6

    return False


def _edge_ok(trig: XoscTrigger, now_value: bool) -> bool:
    """按 conditionEdge 判断是否应该触发。"""
    prev = trig._prev
    trig._prev = now_value
    edge = trig.edge or "rising"
    if edge == "rising":
        return now_value and (prev is not True)
    if edge == "falling":
        return (not now_value) and (prev is True)
    if edge == "risingOrFalling":
        return prev is not None and prev != now_value
    return now_value


# ---------------------------------------------------------------- 控制
def _drive_towards(actor, target: carla.Location, target_speed: float,
                   speed_now: float) -> None:
    """简单纯追踪：朝目标点转向，按速度误差给油门/刹车。"""
    tf = actor.get_transform()
    loc = tf.location
    yaw = math.radians(tf.rotation.yaw)

    dx = target.x - loc.x
    dy = target.y - loc.y
    # 转到车体坐标
    fx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    fy = dx * math.sin(-yaw) + dy * math.cos(-yaw)

    dist = math.hypot(fx, fy)
    if dist < 0.5:
        steer = 0.0
    elif fx <= 0.5:
        # 目标落到车后或几乎在正侧方：不要再打方向，否则会原地画圈。
        # 保持直行，等下一帧目标回到前方。
        steer = 0.0
    else:
        angle = math.degrees(math.atan2(fy, fx))
        steer = max(-1.0, min(1.0, angle / 40.0))

    err = target_speed - speed_now
    if err > 0.5:
        throttle, brake = min(0.9, 0.3 + err * 0.15), 0.0
    elif err < -0.5:
        throttle, brake = 0.0, min(0.9, -err * 0.2)
    else:
        throttle, brake = 0.35, 0.0

    actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steer,
                                             brake=brake))


def _traj_target(points: List[List[float]], elapsed: float,
                 lookahead: float = 0.45
                 ) -> Tuple[Optional[carla.Location], float, bool]:
    """按经过时间在轨迹上插值。

    转向目标取"当前时间 + lookahead"对应的点（纯追踪的前瞻点）。
    否则车一旦落后于轨迹，目标会落到车后，转向直接饱和导致打转。
    期望速度仍按当前所在段计算。

    返回 (目标点, 该段期望速度, 是否已走完)。
    """
    if len(points) < 2:
        return None, 0.0, True

    if elapsed >= points[-1][0]:
        p = points[-1]
        return carla.Location(x=p[1], y=p[2], z=0.0), 0.0, True

    def interp(tt: float) -> Tuple[carla.Location, float]:
        tt = min(tt, points[-1][0])
        for i in range(len(points) - 1):
            t0, x0, y0 = points[i]
            t1, x1, y1 = points[i + 1]
            if not (t0 <= tt <= t1):
                continue
            span = max(t1 - t0, 1e-3)
            frac = (tt - t0) / span
            seg = math.hypot(x1 - x0, y1 - y0)
            return (carla.Location(x=x0 + (x1 - x0) * frac,
                                   y=y0 + (y1 - y0) * frac, z=0.0),
                    seg / span)
        p = points[-1]
        return carla.Location(x=p[1], y=p[2], z=0.0), 0.0

    _, v_now = interp(elapsed)
    tgt, _ = interp(elapsed + lookahead)
    return tgt, v_now, False


def keep_lateral(actor, other, target: float, speed: float) -> None:
    """保持与参照实体的横向间距（单位米，正值表示参照实体在自己左侧）。

    把参照实体的位置转到车体坐标，取横向分量 ly，与目标值比较后用
    比例控制打方向。
    """
    tf = actor.get_transform()
    loc = tf.location
    yaw = math.radians(tf.rotation.yaw)
    o = other.get_transform().location
    dx, dy = o.x - loc.x, o.y - loc.y
    # 转到车体坐标：x 向前，y 向左
    ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    err = target - ly
    steer = max(-1.0, min(1.0, err * 0.30))
    err_v = speed - _speed_of(actor)
    if err_v > 0.5:
        throttle, brake = min(0.9, 0.3 + err_v * 0.15), 0.0
    elif err_v < -0.5:
        throttle, brake = 0.0, min(0.9, -err_v * 0.2)
    else:
        throttle, brake = 0.35, 0.0
    actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steer,
                                             brake=brake))


def _lane_target(actor, delta: int, lookahead: float = 8.0
                 ) -> Optional[carla.Location]:
    """取相邻车道上、前方 lookahead 米处的位置。"""
    try:
        wp = actor.get_world().get_map().get_waypoint(actor.get_transform().location)
        if wp is None:
            return None
        if delta > 0:
            lane = wp.get_left_lane()
        else:
            lane = wp.get_right_lane()
        if lane is None:
            return None
        ahead = lane.next(lookahead)
        if ahead:
            return ahead[0].transform.location
        return lane.transform.location
    except Exception:
        return None


# =====================================================================
# 播放
# =====================================================================
def play(world: carla.World, scenario: XoscScenario,
         duration: Optional[float] = None,
         capture_at: Optional[List[float]] = None,
         out_dir: str = "", width: int = 960, height: int = 540,
         dt: float = 0.05, verbose: bool = True) -> Dict[str, Any]:
    """在已有世界里播放场景。"""
    dur = duration if duration is not None else scenario.stop_time
    if dur <= 0:
        raise XoscPlayerError("时长必须为正")
    capture_at = sorted(capture_at or [])

    scenario.reset_runtime()

    bps = world.get_blueprint_library()
    veh_bp = bps.find("vehicle.tesla.model3") or next(iter(bps.filter("vehicle.*")))
    ped_bp = bps.find("walker.pedestrian.0001")

    settings = world.get_settings()
    old = (settings.synchronous_mode, settings.fixed_delta_seconds)
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    world.apply_settings(settings)

    actors: Dict[str, carla.Actor] = {}
    cams: List[Any] = []
    saved: List[str] = []
    applied: List[Dict[str, Any]] = []
    errors: List[str] = []
    snapshots: List[Dict[str, Any]] = []

    # 每个实体的目标速度（保持用）
    target_speed: Dict[str, float] = {}
    # 正在变道的：entity -> {delta, until, target}
    lane_change: Dict[str, Dict[str, Any]] = {}
    # 正在偏移的
    lane_off: Dict[str, Dict[str, Any]] = {}
    # 保持横向间距的：entity -> {target, ref}
    lateral_ctrl: Dict[str, Dict[str, Any]] = {}
    # 轨迹跟随的：entity -> {points, start}
    traj_ctrl: Dict[str, Dict[str, Any]] = {}

    try:
        # 1) 布景
        for ent in scenario.entities:
            try:
                if ent.position_kind == "world":
                    tf = carla.Transform(
                        carla.Location(x=ent.x, y=ent.y, z=max(ent.z, 0.0) + 0.3),
                        carla.Rotation(yaw=ent.h))
                else:
                    wps = world.get_map().generate_waypoints(4.0)
                    if not wps:
                        raise XoscPlayerError("路网没有车道点")
                    tgt = carla.Location(x=ent.lane_s, y=ent.lane_offset, z=0.0)
                    wp = min(wps, key=lambda w: w.transform.location.distance(tgt))
                    tf = carla.Transform(
                        wp.transform.location + carla.Location(z=0.35),
                        wp.transform.rotation)
            except Exception as e:
                errors.append("%s 定位失败: %s" % (ent.name, e))
                continue

            bp = ped_bp if ent.kind == "pedestrian" else veh_bp
            if bp is None:
                errors.append("%s 没有可用蓝图" % ent.name)
                continue
            actor = world.try_spawn_actor(bp, tf)
            if actor is None:
                errors.append("%s 生成点被占用" % ent.name)
                continue
            actors[ent.name] = actor
            target_speed[ent.name] = ent.initial_speed

        if not actors:
            raise XoscPlayerError("没有任何实体成功布景")

        # 2) 相机模板（每次截图时重新创建，见下方 capture 逻辑）
        cam_bp = None
        if out_dir and capture_at:
            os.makedirs(out_dir, exist_ok=True)
            cam_bp = bps.find("sensor.camera.rgb")
            cam_bp.set_attribute("image_size_x", str(width))
            cam_bp.set_attribute("image_size_y", str(height))
            cam_bp.set_attribute("fov", "90")

        # 3) 逐帧
        steps = int(round(dur / dt))
        cap_idx = 0

        for step in range(steps + 1):
            t = step * dt

            # --- 触发 ---
            for ev in scenario.events:
                if ev.fired:
                    continue
                actor = actors.get(ev.entity)
                if actor is None:
                    continue
                try:
                    val = _evaluate(world, ev.trigger, actors, actor, t)
                except Exception as e:
                    errors.append("条件求值失败 %s: %s" % (ev.name, e))
                    continue
                if not _edge_ok(ev.trigger, val):
                    continue

                ev.fired = True
                for act in ev.actions:
                    act._started_at = t
                    act._start_value = _speed_of(actor)
                    applied.append({"t": round(t, 3), "event": ev.name,
                                    "entity": ev.entity, "action": act.kind,
                                    "value": act.value})
                    # 转向类动作要用 apply_control，必须先退出恒速模式，
                    # 否则恒速会一直覆盖转向，车会被推着直走或打转。
                    if act.kind in ("lane_change", "lane_offset",
                                    "lateral_distance", "follow_trajectory"):
                        try:
                            actor.disable_constant_velocity()
                        except Exception:
                            pass
                    if act.kind == "speed" and act.shape not in ("linear",):
                        target_speed[ev.entity] = act.value
                        act._done = True
                    elif act.kind == "lane_change":
                        lane_change[ev.entity] = {
                            "delta": int(act.value),
                            "until": t + max(act.dynamics_value, 1.0),
                            "speed": target_speed.get(ev.entity, 8.0),
                            "done": False,
                        }
                    elif act.kind == "lane_offset":
                        lane_off[ev.entity] = {
                            "offset": act.value,
                            "until": t + max(act.dynamics_value, 1.0),
                            "done": False,
                        }
                    elif act.kind == "lateral_distance":
                        lateral_ctrl[ev.entity] = {
                            "target": act.value,
                            "ref": act.target_entity,
                        }
                    elif act.kind == "follow_trajectory":
                        traj_ctrl[ev.entity] = {
                            "points": act.points,
                            "start": t,
                        }

            # --- 持续动作 ---
            for ev in scenario.events:
                if not ev.fired:
                    continue
                actor = actors.get(ev.entity)
                if actor is None:
                    continue
                sp = _speed_of(actor)

                for act in ev.actions:
                    if act._done:
                        continue
                    if act.kind == "speed" and act.shape == "linear":
                        span = max(act.dynamics_value, 1e-3)
                        frac = min(1.0, (t - (act._started_at or 0)) / span)
                        v = act._start_value + (act.value - act._start_value) * frac
                        target_speed[ev.entity] = v
                        if frac >= 1.0:
                            act._done = True
                    elif act.kind == "follow":
                        other = actors.get(act.target_entity)
                        if other is not None:
                            gap = _distance(_loc_of(actor), _loc_of(other))
                            err = gap - act.value
                            target_speed[ev.entity] = max(
                                0.0, min(25.0, sp + err * 0.6))
                    elif act.kind == "relative_speed":
                        # 目标速度 = 参照实体速度 + delta，每帧跟随
                        other = actors.get(act.target_entity)
                        if other is not None:
                            target_speed[ev.entity] = max(
                                0.0, min(30.0, _speed_of(other) + act.value))
                    elif act.kind == "lateral_distance":
                        # 横向间距由控制段统一处理，这里只更新目标值
                        other = actors.get(act.target_entity)
                        if other is not None:
                            lateral_ctrl[ev.entity] = {
                                "target": act.value,
                                "ref": act.target_entity,
                            }

            # --- 变道 / 偏移用转向控制，其余用恒速 ---
            for name, actor in actors.items():
                if name in lane_change and not lane_change[name]["done"]:
                    lc = lane_change[name]
                    tgt = _lane_target(actor, lc["delta"])
                    if tgt is None:
                        errors.append("%s 找不到相邻车道" % name)
                        lc["done"] = True
                    else:
                        _drive_towards(actor, tgt, lc["speed"], _speed_of(actor))
                        if t >= lc["until"]:
                            lc["done"] = True
                    continue
                if name in lane_off and not lane_off[name]["done"]:
                    lo = lane_off[name]
                    lane = actor.get_world().get_map().get_waypoint(
                        actor.get_transform().location)
                    if lane is not None:
                        tgt = lane.transform.location + carla.Location(
                            z=0.0)  # 用当前车道中心
                        _drive_towards(actor, tgt, target_speed.get(name, 8.0),
                                       _speed_of(actor))
                    if t >= lo["until"]:
                        lo["done"] = True
                    continue

                if name in lateral_ctrl:
                    lc2 = lateral_ctrl[name]
                    other = actors.get(lc2["ref"])
                    if other is not None and actor.type_id.startswith("vehicle."):
                        keep_lateral(actor, other, lc2["target"],
                                     target_speed.get(name, 8.0))
                        continue

                if name in traj_ctrl and actor.type_id.startswith("vehicle."):
                    tc = traj_ctrl[name]
                    tgt, v_des, finished = _traj_target(
                        tc["points"], t - tc["start"])
                    if finished and tgt is not None:
                        # 走完轨迹就刹停，避免继续漂移
                        actor.apply_control(carla.VehicleControl(
                            throttle=0.0, steer=0.0, brake=0.6))
                        continue
                    if tgt is not None:
                        _drive_towards(actor, tgt, v_des, _speed_of(actor))
                        continue

                v = target_speed.get(name, 0.0)
                if actor.type_id.startswith("vehicle."):
                    yaw = math.radians(_yaw_of(actor))
                    actor.enable_constant_velocity(carla.Vector3D(
                        x=v * math.cos(yaw), y=v * math.sin(yaw), z=0.0))

            # --- 截图 ---
            while cap_idx < len(capture_at) and capture_at[cap_idx] <= t + 1e-6:
                if cam_bp is not None:
                    cam = None
                    try:
                        snap = {}
                        for nm, ac in actors.items():
                            if ac.is_alive:
                                tf2 = ac.get_transform()
                                snap[nm] = (round(tf2.location.x, 1),
                                            round(tf2.location.y, 1),
                                            round(_speed_of(ac), 1),
                                            round(tf2.rotation.yaw, 1))

                        # 以 ego 为中心构图。
                        # 不能按全体质心：有的场景里行人在世界原点、
                        # 车在几十米外，质心被拉偏，车会缩成小点。
                        center = actors.get("ego")
                        if center is None or not center.is_alive:
                            alive0 = [a for a in actors.values() if a.is_alive]
                            center = alive0[0] if alive0 else None
                        if center is None:
                            raise XoscPlayerError("没有存活的实体可供构图")

                        c0 = center.get_transform().location
                        others = [a.get_transform().location
                                  for a in actors.values() if a.is_alive]
                        span = max(
                            [math.hypot(l.x - c0.x, l.y - c0.y) for l in others]
                            or [20.0])
                        # 相机高度 = 视野半宽（水平 FOV 90°）。取值 40~80 m：
                        # 太小切掉远处实体，太大车缩成几个像素。不能直接
                        # 用 span——场景里常有远处的行人/对向车。
                        cam_z = min(max(span * 0.9, 40.0), 80.0)

                        # 每次截图新建相机，而不是复用并 set_transform。
                        # 复用 + set_transform 实测拿到的仍是移动前的画面
                        # （sensor 的变换要 tick 后才生效，队列里的旧帧很
                        # 容易被误取），重建最稳。
                        q: "queue.Queue" = queue.Queue()
                        cam = world.spawn_actor(cam_bp, carla.Transform(
                            carla.Location(x=c0.x, y=c0.y, z=cam_z),
                            carla.Rotation(pitch=-89.0)))
                        cam.listen(q.put)
                        cams.append((cam, q))

                        # 空转几帧让渲染稳定（与 capture_views 一致）
                        for _ in range(6):
                            world.tick()
                        frame = q.get(timeout=20)
                        snapshots.append({"t": round(t, 2), "actors": snap,
                                          "cam": (round(c0.x, 1), round(c0.y, 1),
                                                  round(cam_z, 1)),
                                          "frame": getattr(frame, "frame", -1)})
                        p = os.path.join(out_dir, "t%05.2f.png" % t)
                        frame.save_to_disk(p)
                        if os.path.exists(p):
                            saved.append(p)
                    except queue.Empty:
                        errors.append("截图 %.2fs 超时" % t)
                    except Exception as e:
                        errors.append("截图 %.2fs 失败: %s" % (t, e))
                    finally:
                        if cam is not None:
                            try:
                                cam.stop()
                                cam.destroy()
                            except Exception:
                                pass
                            if cams and cams[-1][0] is cam:
                                cams.pop()
                cap_idx += 1

            world.tick()

    finally:
        for a in actors.values():
            try:
                if hasattr(a, "disable_constant_velocity"):
                    a.disable_constant_velocity()
            except Exception:
                pass
        for cam, q in cams:
            try:
                cam.stop()
                cam.destroy()
            except Exception:
                pass
        for a in actors.values():
            try:
                a.destroy()
            except Exception:
                pass
        settings = world.get_settings()
        settings.synchronous_mode = old[0]
        settings.fixed_delta_seconds = old[1]
        world.apply_settings(settings)

    return {
        "scenario": scenario.name,
        "entities": len(scenario.entities),
        "spawned": len(actors),
        "duration": dur,
        "steps": int(round(dur / dt)),
        "events_total": len(scenario.events),
        "events_fired": sum(1 for e in scenario.events if e.fired),
        "applied": applied[:20],
        "unsupported": scenario.unsupported,
        "errors": errors,
        "images": saved,
        "snapshots": snapshots,
    }


def summarize(result: Dict[str, Any]) -> str:
    lines = [
        "场景：%s" % result["scenario"],
        "实体 %d 个，成功布景 %d 个" % (result["entities"], result["spawned"]),
        "时长 %.2fs（%d 帧）" % (result["duration"], result["steps"]),
        "事件 %d 个，已触发 %d 个"
        % (result["events_total"], result["events_fired"]),
    ]
    if result["applied"]:
        lines.append("已触发的动作：")
        for a in result["applied"]:
            lines.append("   t=%.2fs %-10s %-12s %s=%.2f"
                         % (a["t"], a["entity"], a["event"],
                            a["action"], a["value"]))
    if result["unsupported"]:
        lines.append("未执行（超出播放器范围）：")
        for u in result["unsupported"][:8]:
            lines.append("   - %s" % u)
    if result["errors"]:
        lines.append("错误：")
        for e in result["errors"][:8]:
            lines.append("   - %s" % e)
    if result["images"]:
        lines.append("截图：")
        for p in result["images"]:
            lines.append("   - %s" % p)
    return "\n".join(lines)
