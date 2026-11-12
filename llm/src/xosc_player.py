"""
xosc_player.py - OpenSCENARIO 场景播放器

仓库里没有 ScenarioRunner，`.xosc` 一直是"生成完就躺在那儿"。这个模块
解析生成的 `.xosc`，把实体布景进 CARLA，按触发条件下发动作，跑固定时长。

支持范围
--------
实体      Vehicle / Pedestrian
初始化    TeleportAction -> LanePosition(roadId/laneId/s/offset) / WorldPosition
动作      SpeedAction（step / linear 渐变，含 RelativeTargetSpeed delta/factor）
          LongitudinalDistanceAction（IDM 跟驰）
          LaneChangeAction（转向控制 + 横向加速度限幅）
          LaneOffsetAction（车道内横向偏移）
          FollowTrajectoryAction（轨迹跟随）
          TrafficSignalControllerAction（切换信号灯相位）
条件      SimulationTimeCondition（仿真时间）
          TimeHeadwayCondition（车头时距）
          DistanceCondition（到某位置的距离）
          SpeedCondition（自身速度）
          StandStillCondition（静止一段时间）
          ReachPositionCondition（到达某位置）
          CollisionCondition（碰撞）
          TimeToCollisionCondition（碰撞时间）
          RelativeSpeedCondition（相对速度）
          AccelerationCondition（自身加速度）
          OffroadCondition（驶出路网）
          EndOfRoadCondition（到达路段末端）
          TraveledDistanceCondition（累计里程）
          RelativeDistanceCondition（与参照实体的距离）
          RelativeAngleCondition（与参照实体的航向夹角）
          TrafficSignalCondition（信号灯相位）
触发组合  ConditionGroup 之间 OR、组内 Condition 之间 AND；
          Condition@delay 需连续成立；StartTrigger 的 TriggeringEntities
          决定条件求值主体（参照 esmini 语义）
执行结构  Story / Act / Sequence / ManeuverGroup 全层级遍历；
          一个 ManeuverGroup 的多个 Actor 各展开一份事件
参数      <ParameterDeclarations> 的 $name / ${name} 静态替换
速度曲线  SpeedActionDynamics 的 step / linear / cubic / sinusoidal，
          dynamicsDimension 的 time（时长）与 rate（变化率）
跟驰      LongitudinalDistanceAction 的 distance +
          maxAcceleration / maxDeceleration（作为 IDM 的 a_max / b）
停止      StopTrigger 与 StartTrigger 同一套条件语义，满足即提前结束
触发边沿  rising / falling / none / risingOrFalling

不支持（记入 unsupported，不静默跳过）
------------------------------------
EntitySelection、Catalog、运行期 ParameterAssignment、绝对目标车道

已知简化
--------
- 跟驰用 IDM（智能驾驶员模型），参数取舒适型标定，而非 OSC 的完整语义
- 变道用纯追踪转向，并按横向加速度上限对转向做限幅
- 复合条件先对每个叶子做边沿检测，再 AND/OR 组合；不实现"整个条件组
  再判一次边沿"的细节
- 不考虑 freespace / alongroute 的坐标语义，一律按直线距离
- 信号灯条件按"自车前方最近的红绿灯"取实际状态，忽略控制器内部的
  相位时长模型
"""
from __future__ import annotations

import copy
import math
import os
import queue
import re
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
    lane_id: str = ""
    road_id: str = ""


@dataclass
class XoscTrigger:
    # time / distance / time_headway / speed / standstill / reach_position /
    # collision / time_to_collision / relative_speed / acceleration /
    # offroad / end_of_road / traveled_distance / relative_distance /
    # relative_angle / traffic_signal
    kind: str = "time"
    value: float = 0.0
    rule: str = "greaterThan"
    entity_ref: str = ""
    position: Optional[XoscPosition] = None
    tolerance: float = 1.0
    edge: str = "rising"
    # 条件求值的主体（TriggeringEntities）；空表示用事件所属实体
    triggering_entity: str = ""
    # OSC 的 Condition/@delay：条件需连续成立 delay 秒才触发
    delay: float = 0.0
    # traffic_signal 用：控制器名（player 里映射到前方红绿灯）与目标相位
    signal_controller: str = ""
    signal_phase: str = ""
    # 复合条件（kind="any"）：children 是 OR 的 ConditionGroup；
    # 每个 group 的 kind="all"，children 是 AND 的叶子条件。
    children: List["XoscTrigger"] = field(default_factory=list)
    # 运行时状态
    _prev: Optional[bool] = None
    _since: Optional[float] = None
    _delay_since: Optional[float] = None

    def reset_runtime(self) -> None:
        self._prev = None
        self._since = None
        self._delay_since = None
        for c in self.children:
            c.reset_runtime()

    def leaves(self) -> List["XoscTrigger"]:
        """展开成叶子条件（非复合条件）。"""
        if self.children:
            out: List[XoscTrigger] = []
            for c in self.children:
                out.extend(c.leaves())
            return out
        return [self]


@dataclass
class XoscAction:
    # speed / relative_speed / follow / lane_change / lane_offset /
    # lateral_distance / follow_trajectory / traffic_light
    kind: str = "speed"
    value: float = 0.0
    shape: str = "step"
    dynamics_value: float = 0.0
    # OSC 的 SpeedActionDynamics/@dynamicsDimension：time 表示渐变时长，
    # rate 表示变化率（此时渐变时长 = |Δv| / rate）
    dynamics_dimension: str = "time"
    target_entity: str = ""
    lane_delta: int = 0
    # LongitudinalDistanceAction 的加减速上限（0 表示用 IDM 默认值）
    max_accel: float = 0.0
    max_decel: float = 0.0
    # relative_speed 用：delta = 参照速度 + value；factor = 参照速度 * value
    relative_mode: str = "delta"
    # traffic_light 用：目标相位（green / yellow / red / off）
    phase: str = ""
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
    # Storyboard/StopTrigger：满足即提前结束（None 表示只按 stop_time 跑满）
    stop_trigger: Optional[XoscTrigger] = None
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
        if self.stop_trigger is not None:
            self.stop_trigger.reset_runtime()


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
    "CollisionCondition": "collision",
    "TimeToCollisionCondition": "time_to_collision",
    "RelativeSpeedCondition": "relative_speed",
    "AccelerationCondition": "acceleration",
    "OffroadCondition": "offroad",
    "EndOfRoadCondition": "end_of_road",
    "TraveledDistanceCondition": "traveled_distance",
    "RelativeDistanceCondition": "relative_distance",
    "RelativeAngleCondition": "relative_angle",
    "TrafficSignalCondition": "traffic_signal",
}

# 仍未实现的条件（解析时记入 unsupported）
_KNOWN_UNSUPPORTED = {
    "ParameterCondition", "TimeOfDayCondition", "UserDefinedValueCondition",
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
                            lane_offset=float(lp.get("offset", 0) or 0),
                            lane_id=lp.get("laneId", "") or "",
                            road_id=lp.get("roadId", "") or "")
    return None


def _parse_condition(cond_el, fallback_entity: str) -> Optional[XoscTrigger]:
    """把一个 <Condition> 解析成 XoscTrigger；不支持的返回 None 并记录。"""
    edge = cond_el.get("conditionEdge", "rising")
    try:
        delay = float(cond_el.get("delay", 0) or 0)
    except (TypeError, ValueError):
        delay = 0.0

    for deep in cond_el.iter():
        tag = deep.tag
        if tag == "SimulationTimeCondition":
            return XoscTrigger(kind="time",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "greaterThan"),
                               edge=edge, delay=delay)
        if tag == "TimeHeadwayCondition":
            return XoscTrigger(kind="time_headway",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               entity_ref=deep.get("entityRef", fallback_entity),
                               edge=edge, delay=delay)
        if tag == "DistanceCondition":
            return XoscTrigger(kind="distance",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               position=_parse_position(deep.find("Position")),
                               edge=edge, delay=delay)
        if tag == "SpeedCondition":
            return XoscTrigger(kind="speed",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "greaterThan"),
                               edge=edge, delay=delay)
        if tag == "StandStillCondition":
            return XoscTrigger(kind="standstill",
                               value=float(deep.get("duration", 0) or 0),
                               edge=edge, delay=delay)
        if tag == "ReachPositionCondition":
            return XoscTrigger(kind="reach_position",
                               position=_parse_position(deep.find("Position")),
                               tolerance=float(deep.get("tolerance", 1.0) or 1.0),
                               edge=edge, delay=delay)
        if tag == "RelativeSpeedCondition":
            return XoscTrigger(kind="relative_speed",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               entity_ref=deep.get("entityRef", ""),
                               edge=edge, delay=delay)
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
                               edge=edge, delay=delay)
        if tag == "CollisionCondition":
            er = deep.find(".//EntityRef")
            ref = er.get("entityRef", "") if er is not None else ""
            return XoscTrigger(kind="collision", rule="equalTo",
                               entity_ref=ref, edge=edge, delay=delay)
        if tag == "AccelerationCondition":
            return XoscTrigger(kind="acceleration",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "greaterThan"),
                               edge=edge, delay=delay)
        if tag == "OffroadCondition":
            return XoscTrigger(kind="offroad", rule="greaterOrEqual",
                               value=float(deep.get("duration", 0) or 0),
                               edge=edge, delay=delay)
        if tag == "EndOfRoadCondition":
            return XoscTrigger(kind="end_of_road", rule="greaterOrEqual",
                               value=float(deep.get("duration", 0) or 0),
                               edge=edge, delay=delay)
        if tag == "TraveledDistanceCondition":
            return XoscTrigger(kind="traveled_distance",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "greaterThan"),
                               edge=edge, delay=delay)
        if tag == "RelativeDistanceCondition":
            return XoscTrigger(kind="relative_distance",
                               value=float(deep.get("value", 0) or 0),
                               rule=deep.get("rule", "lessThan"),
                               entity_ref=deep.get("entityRef", ""),
                               edge=edge, delay=delay)
        if tag == "RelativeAngleCondition":
            return XoscTrigger(kind="relative_angle",
                               value=float(deep.get("angle", 0) or 0),
                               tolerance=float(
                                   deep.get("angleTolerance", 0) or 0),
                               rule="equalTo",
                               entity_ref=deep.get("entityRef", ""),
                               edge=edge, delay=delay)
        if tag == "TrafficSignalCondition":
            return XoscTrigger(kind="traffic_signal", rule="equalTo",
                               signal_controller=deep.get("name", "") or "",
                               signal_phase=deep.get("state", "") or "",
                               edge=edge, delay=delay)
    return None


def _parse_condition_groups(trigger_el, fallback_entity: str,
                            unsupported: List[str], label: str
                            ) -> Optional[XoscTrigger]:
    """把 StartTrigger / StopTrigger 解析成复合条件。

    多个 ConditionGroup 之间是 OR；组内多个 Condition 之间是 AND。
    只有一个叶子时直接返回它，便于日志与回归读取 kind。
    """
    if trigger_el is None:
        return None
    groups: List[XoscTrigger] = []
    for cg in trigger_el.findall("ConditionGroup"):
        conds: List[XoscTrigger] = []
        for cond in cg.findall("Condition"):
            trig = _parse_condition(cond, fallback_entity)
            if trig is None:
                for deep in cond.iter():
                    if deep.tag in _KNOWN_UNSUPPORTED:
                        unsupported.append("%s条件 %s 未执行" % (label, deep.tag))
                        break
                continue
            # OSC 的条件作用在 TriggeringEntities 上，未声明时才回落到所属实体
            ref = cond.find("./ByEntityCondition/TriggeringEntities/EntityRef")
            if ref is not None:
                trig.triggering_entity = ref.get("entityRef", "")
            conds.append(trig)
        if conds:
            groups.append(XoscTrigger(kind="all", edge="none", children=conds))
    if not groups:
        return None
    if len(groups) == 1 and len(groups[0].children) == 1:
        return groups[0].children[0]
    return XoscTrigger(kind="any", edge="none", children=groups)


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
                                          target_entity=rel.get("entityRef", ""),
                                          relative_mode=rel.get(
                                              "speedTargetValueType", "delta")
                                          or "delta"))
                    continue
                if target is not None:
                    shape = "step"
                    dv = 0.0
                    dim = "time"
                    dyn = sp.find("SpeedActionDynamics")
                    if dyn is not None:
                        shape = dyn.get("dynamicsShape", "step") or "step"
                        dim = dyn.get("dynamicsDimension", "time") or "time"
                        try:
                            dv = float(dyn.get("value", 0) or 0)
                        except ValueError:
                            dv = 0.0
                    out.append(XoscAction(kind="speed", value=target,
                                          shape=shape, dynamics_value=dv,
                                          dynamics_dimension=dim))
                    continue
            dist = longi.find("LongitudinalDistanceAction")
            if dist is not None:
                try:
                    gap = float(dist.get("distance", 0) or 0)
                except ValueError:
                    gap = 0.0

                def _limit(attr: str) -> float:
                    try:
                        return max(0.0, float(dist.get(attr, 0) or 0))
                    except ValueError:
                        return 0.0

                out.append(XoscAction(kind="follow", value=gap,
                                      target_entity=dist.get("entityRef", ""),
                                      max_accel=_limit("maxAcceleration"),
                                      max_decel=_limit("maxDeceleration")))
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


def _parse_global_actions(ga_list, unsupported: List[str],
                          event_name: str) -> List[XoscAction]:
    """解析 Event 里的 GlobalAction（目前只支持信号灯控制）。"""
    out: List[XoscAction] = []
    for ga in ga_list:
        tsc = ga.find(".//TrafficSignalControllerAction")
        if tsc is not None:
            out.append(XoscAction(
                kind="traffic_light",
                phase=tsc.get("phase", "") or "",
                target_entity=tsc.get("trafficSignalControllerRef", "") or ""))
            continue
        unsupported.append("事件 %s 的全局动作未识别" % event_name)
    return out


_PARAM_REF = re.compile(r"\$\{?([A-Za-z_]\w*)\}?")


def _apply_parameters(root) -> int:
    """把 <ParameterDeclarations> 里的值替换进所有属性。

    esmini 支持 `$param` / `${param}` 两种写法；这里只做场景级的静态替换，
    不做运行期参数赋值（ParameterAssignment 仍记入 unsupported）。
    """
    params = {}
    for pd in root.findall("./ParameterDeclarations/ParameterDeclaration"):
        name = pd.get("name", "")
        if name:
            params[name] = pd.get("value", "")
    if not params:
        return 0

    def repl(m):
        return params.get(m.group(1), m.group(0))

    for el in root.iter():
        for k, v in list(el.attrib.items()):
            if "$" in v:
                el.set(k, _PARAM_REF.sub(repl, v))
    return len(params)


def parse_xosc(path: str) -> XoscScenario:
    if not os.path.exists(path):
        raise XoscPlayerError("文件不存在: %s" % path)

    root = ET.parse(path).getroot()
    sc = XoscScenario(name=os.path.basename(path))

    # ---- 参数声明（esmini 语义：$name / ${name} 就地替换）----
    _apply_parameters(root)

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
                        if pos.lane_id:
                            ent.lane_id = pos.lane_id
                        if pos.road_id:
                            ent.road_id = pos.road_id
            longi = pa.find("LongitudinalAction")
            if longi is not None:
                v = _find_target_speed(longi)
                if v is not None:
                    ent.initial_speed = v

    # ---- 事件 ----
    # OSC 的 Story 可以嵌套 Sequence，ManeuverGroup 也可能一次作用于多个 Actor，
    # 这里都按"每个 Actor 一份事件"展开（esmini / ScenarioRunner 的做法）。
    storyboard = root.find("./Storyboard")
    mgroups = list(storyboard.iter("ManeuverGroup")) if storyboard is not None else []
    for mg in mgroups:
        actors = [r.get("entityRef", "")
                  for r in mg.findall("./Actors/EntityRef")]
        actors = [a for a in actors if a]
        if not actors:
            sc.unsupported.append("ManeuverGroup %s 没有 Actor" % mg.get("name"))
            continue

        for ev in mg.findall("./Maneuver/Event"):
            name = ev.get("name", "")
            trigger = _parse_condition_groups(ev.find("./StartTrigger"),
                                              actors[0], sc.unsupported,
                                              "事件 %s 的" % name)
            if trigger is None:
                sc.unsupported.append("事件 %s 没有可识别的触发条件" % name)
                continue

            actions = _parse_actions(ev.findall("./Action/PrivateAction"),
                                     sc.unsupported, name)
            actions += _parse_global_actions(ev.findall("./Action/GlobalAction"),
                                             sc.unsupported, name)
            if not actions:
                continue
            for actor in actors:
                sc.events.append(XoscEvent(
                    name=(name if len(actors) == 1
                          else "%s[%s]" % (name, actor)),
                    entity=actor,
                    trigger=copy.deepcopy(trigger),
                    actions=copy.deepcopy(actions)))

    # ---- 停止触发 ----
    # StopTrigger 与 StartTrigger 用同一套条件语义（组间 OR、组内 AND），
    # 满足即提前结束，而不是只认 SimulationTimeCondition。
    stop = root.find("./Storyboard/StopTrigger")
    if stop is not None:
        first = sc.entities[0].name if sc.entities else ""
        sc.stop_trigger = _parse_condition_groups(stop, first,
                                                  sc.unsupported, "停止触发")
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


def _lane_waypoint(world: carla.World, road_id: str, lane_id: str,
                   s: float):
    """按 OpenDRIVE 的 (roadId, laneId, s) 取路点。

    优先用 CARLA 的 get_waypoint_xodr 精确解析；失败时退化为"同车道里
    s 最接近"的路点，避免把 s 当成世界坐标 x 去最近邻（旧实现的 bug）。
    """
    amap = world.get_map()
    try:
        if road_id and lane_id:
            wp = amap.get_waypoint_xodr(int(float(road_id)),
                                        int(float(lane_id)), float(s))
            if wp is not None:
                return wp
    except Exception:
        pass
    if not lane_id:
        return None
    cand = [w for w in amap.generate_waypoints(2.0)
            if str(w.lane_id) == str(lane_id)]
    if not cand:
        return None
    return min(cand, key=lambda w: abs(float(w.s) - float(s)))


def _apply_lane_offset(wp, offset: float) -> carla.Location:
    """车道中心 + 横向偏移（OSC 约定：正值向行驶方向左侧）。"""
    if abs(offset) < 1e-6:
        return wp.transform.location
    yaw = math.radians(wp.transform.rotation.yaw)
    return wp.transform.location + carla.Location(
        x=math.sin(yaw) * offset, y=-math.cos(yaw) * offset, z=0.0)


def _resolve_position(world: carla.World, pos: XoscPosition) -> carla.Location:
    if pos.kind == "world":
        return carla.Location(x=pos.x, y=pos.y, z=pos.z)
    wp = _lane_waypoint(world, pos.road_id, pos.lane_id, pos.s)
    if wp is not None:
        return _apply_lane_offset(wp, pos.lane_offset)
    # 极端兜底：路网里找不到该车道时，按世界坐标找最近路点
    wps = world.get_map().generate_waypoints(4.0)
    if not wps:
        raise XoscPlayerError("路网没有车道点，无法解析 LanePosition")
    target = carla.Location(x=pos.s, y=pos.lane_offset, z=0.0)
    return min(wps, key=lambda w: w.transform.location.distance(target)
               ).transform.location


def _speed_of(actor) -> float:
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def _accel_of(actor) -> float:
    a = actor.get_acceleration()
    return math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)


def _yaw_diff_deg(a, b) -> float:
    """两车航向夹角（度，0~180）。"""
    d = (a.get_transform().rotation.yaw - b.get_transform().rotation.yaw) % 360.0
    return abs(d - 360.0) if d > 180.0 else abs(d)


def _distance_to_road_end(world: carla.World, actor) -> float:
    """沿当前车道向前探测，估算到路段末端的距离（米）。"""
    amap = world.get_map()
    wp = amap.get_waypoint(_loc_of(actor), project_to_road=True)
    if wp is None:
        return 0.0
    road_id, lane_id = wp.road_id, wp.lane_id
    cur = wp
    remaining = 0.0
    for _ in range(60):
        nxt = [w for w in cur.next(5.0)
               if w.road_id == road_id and w.lane_id == lane_id]
        if not nxt:
            return remaining
        cur = nxt[0]
        remaining += 5.0
    return remaining


# 信号灯相位别名（OSC 的相位名是自由文本，这里做常见别名归一）
_PHASE_ALIASES = {
    "green": {"green", "go", "绿", "绿灯"},
    "yellow": {"yellow", "amber", "黄", "黄灯"},
    "red": {"red", "stop", "红", "红灯"},
    "off": {"off", "关", "关闭"},
}


def _canonical_phase(name: str) -> str:
    low = (name or "").strip().lower()
    for canon, aliases in _PHASE_ALIASES.items():
        if low in aliases:
            return canon
    return low


_CARLA_SIGNAL_STATES = {
    "green": carla.TrafficLightState.Green,
    "yellow": carla.TrafficLightState.Yellow,
    "red": carla.TrafficLightState.Red,
    "off": carla.TrafficLightState.Off,
}


def _traffic_light_ahead(actor, max_dist: float = 80.0):
    """取自车前方最近的一盏红绿灯；没有则返回 None。"""
    world = actor.get_world()
    wp = world.get_map().get_waypoint(_loc_of(actor))
    if wp is None:
        return None
    try:
        lights = world.get_traffic_lights_from_waypoint(wp, max_dist)
    except Exception:
        return None
    loc = _loc_of(actor)
    yaw = math.radians(_yaw_of(actor))
    fx, fy = math.cos(yaw), math.sin(yaw)
    best, best_d = None, float("inf")
    for lt in lights:
        d = _loc_of(lt) - loc
        if d.x * fx + d.y * fy <= 0:
            continue
        dist = math.hypot(d.x, d.y)
        if dist < best_d:
            best, best_d = lt, dist
    return best


def _evaluate(world: carla.World, trig: XoscTrigger, actors: Dict[str, carla.Actor],
              self_actor: carla.Actor, now: float,
              state: Optional[Dict[str, Any]] = None) -> bool:
    """求值一个触发条件（不含边沿检测）。"""
    rule = _RULES.get(trig.rule, _RULES["greaterThan"])
    state = state or {}

    if trig.kind == "time":
        return rule(now, trig.value)

    if trig.kind == "speed":
        return rule(_speed_of(self_actor), trig.value)

    if trig.kind == "acceleration":
        return rule(_accel_of(self_actor), trig.value)

    if trig.kind == "traveled_distance":
        name = self_actor.attributes.get("role_name", "")
        traveled = (state.get("traveled") or {}).get(name, 0.0)
        return rule(traveled, trig.value)

    if trig.kind == "standstill":
        slow = _speed_of(self_actor) < 0.15
        if not slow:
            trig._since = None
            return False
        if trig._since is None:
            trig._since = now
        return (now - trig._since) >= trig.value

    if trig.kind == "offroad":
        wp = world.get_map().get_waypoint(_loc_of(self_actor),
                                          project_to_road=False)
        off = wp is None or wp.lane_type != carla.LaneType.Driving
        if not off:
            trig._since = None
            return False
        if trig._since is None:
            trig._since = now
        return (now - trig._since) >= trig.value

    if trig.kind == "end_of_road":
        near_end = _distance_to_road_end(world, self_actor) <= 2.0
        if not near_end:
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

    if trig.kind == "relative_distance":
        other = actors.get(trig.entity_ref) if trig.entity_ref else None
        if other is None or other is self_actor:
            return False
        return rule(_distance(_loc_of(self_actor), _loc_of(other)), trig.value)

    if trig.kind == "relative_angle":
        other = actors.get(trig.entity_ref) if trig.entity_ref else None
        if other is None or other is self_actor:
            return False
        diff = _yaw_diff_deg(self_actor, other)
        return abs(diff - trig.value) <= max(trig.tolerance, 1e-3)

    if trig.kind == "traffic_signal":
        light = _traffic_light_ahead(self_actor)
        if light is None:
            return False
        try:
            actual = light.state.name
        except Exception:
            return False
        return _canonical_phase(actual) == _canonical_phase(trig.signal_phase)

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


def _leaf_ready(trig: XoscTrigger, value: bool, now: float) -> bool:
    """叶子条件：先满足 @delay 的持续时长，再做 conditionEdge 判边沿。"""
    if value:
        if trig._delay_since is None:
            trig._delay_since = now
    else:
        trig._delay_since = None
    if trig.delay > 0:
        sustained = (trig._delay_since is not None
                     and (now - trig._delay_since) >= trig.delay - 1e-6)
    else:
        sustained = value
    return _edge_ok(trig, sustained)


def _trigger_ready(world: carla.World, trigger: XoscTrigger,
                   actors: Dict[str, carla.Actor], default_actor,
                   now: float, state: Dict[str, Any]) -> bool:
    """复合条件是否就绪（StartTrigger 与 StopTrigger 共用同一套语义）。"""
    if trigger.children:
        groups = trigger.children
    else:
        groups = [XoscTrigger(kind="all", children=[trigger])]

    any_true = False
    for group in groups:
        group_true = True
        for cond in group.children or []:
            evaluator = actors.get(cond.triggering_entity) or default_actor
            val = _evaluate(world, cond, actors, evaluator, now, state)
            if not _leaf_ready(cond, val, now):
                group_true = False
        if group_true:
            any_true = True
    return any_true


def _event_ready(world: carla.World, ev: XoscEvent,
                 actors: Dict[str, carla.Actor], default_actor,
                 now: float, state: Dict[str, Any]) -> bool:
    """事件是否就绪：ConditionGroup 之间 OR，组内 Condition 之间 AND。

    所有叶子条件每帧都求值（不短路），否则未求值的叶子边沿状态会失真。
    """
    return _trigger_ready(world, ev.trigger, actors, default_actor, now, state)


# ---------------------------------------------------------------- 控制
# 横向加速度限幅用：steer=1 时曲率约 0.12 1/m（按 CARLA 车辆几何标定），
# 由 kappa = g * steer 近似，用于把转向约束在 ay = v^2 * kappa 以内。
_STEER_TO_KAPPA = 0.12


def _steer_towards(actor, target: carla.Location,
                   max_lat_acc: float = 0.0) -> float:
    """纯追踪求转向量；给出 max_lat_acc 时按横向加速度上限限幅。"""
    tf = actor.get_transform()
    loc = tf.location
    yaw = math.radians(tf.rotation.yaw)

    dx = target.x - loc.x
    dy = target.y - loc.y
    # 转到车体坐标
    fx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    fy = dx * math.sin(-yaw) + dy * math.cos(-yaw)

    dist = math.hypot(fx, fy)
    if dist < 0.5 or fx <= 0.5:
        # 目标落到车后或几乎在正侧方：不要再打方向，否则会原地画圈。
        return 0.0
    angle = math.degrees(math.atan2(fy, fx))
    steer = max(-1.0, min(1.0, angle / 40.0))

    if max_lat_acc > 0:
        v = max(_speed_of(actor), 1.0)
        # ay = v^2 * kappa <= max_lat_acc  =>  steer <= max_lat_acc/(v^2*g)
        lim = max_lat_acc / (v * v * _STEER_TO_KAPPA)
        steer = max(-min(1.0, lim), min(min(1.0, lim), steer))
    return steer


def _accel_to_control(accel: float, speed_now: float):
    """把期望加速度映射成油门/刹车。"""
    if accel >= 0:
        throttle = min(0.85, 0.12 + accel * 0.22)
        return throttle, 0.0
    return 0.0, min(1.0, -accel / 6.0)


def _drive_towards(actor, target: carla.Location, target_speed: float,
                   speed_now: float, max_lat_acc: float = 0.0) -> None:
    """简单纯追踪：朝目标点转向，按速度误差给油门/刹车。"""
    steer = _steer_towards(actor, target, max_lat_acc)
    err = target_speed - speed_now
    if err > 0.5:
        throttle, brake = min(0.9, 0.3 + err * 0.15), 0.0
    elif err < -0.5:
        throttle, brake = 0.0, min(0.9, -err * 0.2)
    else:
        throttle, brake = 0.35, 0.0

    actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steer,
                                             brake=brake))


def _idm_accel(speed: float, desired_speed: float, gap: float,
               rel_speed: float, s0: float = 2.0,
               a_max: float = 2.0, b: float = 3.0) -> float:
    """IDM（智能驾驶员模型）加速度。

    a = a_max * [1 - (v/v0)^4 - (s*/gap)^2]
    s* = s0 + max(0, v*T + v*dv/(2*sqrt(a_max*b)))
    gap 为车头净距（前车尾到自车头），rel_speed 为自车 - 前车速度，
    s0 为期望的最小净距（OSC LongitudinalDistanceAction 的 distance）。
    a_max / b 对应 OSC 动作里的 maxAcceleration / maxDeceleration。
    """
    t_head = 1.2
    v0 = max(desired_speed, 0.1)
    s_star = s0 + max(0.0, speed * t_head
                      + speed * rel_speed / (2.0 * math.sqrt(a_max * b)))
    net = max(gap, 0.5)
    return a_max * (1.0 - (speed / v0) ** 4 - (s_star / net) ** 2)


def _idm_follow(actor, lead, gap_target: float, speed_now: float,
                max_speed: float, steer_target=None,
                max_accel: float = 0.0, max_decel: float = 0.0) -> None:
    """IDM 跟驰：保持 gap_target 的净距，必要时刹停。

    max_accel / max_decel 来自 OSC 动作；为 0 时用 IDM 默认标定。
    """
    gap = _distance(_loc_of(actor), _loc_of(lead)) - 5.0
    rel = speed_now - _speed_of(lead)
    # 期望速度取场景给定值；前车速度只通过 IDM 的交互项影响减速度。
    # （若把 v0 绑成"前车速度+1.5"，前车停住后自车会一直以 1.5 m/s 蠕行）
    v0 = max(min(max_speed, 30.0), 0.1)
    a_max = max_accel if max_accel > 0 else 2.0
    b = max_decel if max_decel > 0 else 3.0
    a = _idm_accel(speed_now, v0, gap, rel,
                   s0=max(2.0, min(40.0, gap_target)), a_max=a_max, b=b)
    # IDM 输出再按动作给的上限夹一次：超车加速或急刹都不越界
    if max_accel > 0:
        a = min(a, max_accel)
    if max_decel > 0:
        a = max(a, -max_decel)
    throttle, brake = _accel_to_control(a, speed_now)
    steer = _steer_towards(actor, steer_target) if steer_target is not None else 0.0
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


def _lane_center_ahead(actor, lookahead: float = 8.0
                       ) -> Optional[carla.Location]:
    """当前车道上、前方 lookahead 米处的车道中心点（用于保持直线）。"""
    try:
        wp = actor.get_world().get_map().get_waypoint(
            actor.get_transform().location)
        if wp is None:
            return None
        ahead = wp.next(lookahead)
        return ahead[0].transform.location if ahead else None
    except Exception:
        return None


# 支持渐变的速度动作曲线（OSC SpeedActionDynamics/dynamicsShape）
_SHAPED_SPEEDS = ("linear", "cubic", "sinusoidal")


def _shape_frac(shape: str, frac: float) -> float:
    """把线性进度映射成各 dynamicsShape 的插值系数。"""
    frac = max(0.0, min(1.0, frac))
    if shape == "cubic":
        # 三次平滑，起止速度为 0
        return frac * frac * (3.0 - 2.0 * frac)
    if shape == "sinusoidal":
        return 0.5 * (1.0 - math.cos(math.pi * frac))
    return frac


def _dynamics_span(act: XoscAction) -> float:
    """速度渐变的时长：time 维度直接取值，rate 维度按 |Δv| / rate 换算。"""
    if act.dynamics_dimension == "rate" and act.dynamics_value > 1e-6:
        return abs(act.value - act._start_value) / act.dynamics_value
    return max(act.dynamics_value, 1e-3)


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
    # IDM 跟驰的：entity -> {ref, gap}
    follow_ctrl: Dict[str, Dict[str, Any]] = {}
    # 运行期状态：累计里程 / 上一帧位置
    state: Dict[str, Any] = {"traveled": {}, "last": {}}

    try:
        # 1) 布景
        for ent in scenario.entities:
            try:
                if ent.position_kind == "world":
                    tf = carla.Transform(
                        carla.Location(x=ent.x, y=ent.y, z=max(ent.z, 0.0) + 0.3),
                        carla.Rotation(yaw=ent.h))
                else:
                    wp = _lane_waypoint(world, ent.road_id, ent.lane_id,
                                        ent.lane_s)
                    if wp is None:
                        raise XoscPlayerError(
                            "找不到车道 roadId=%s laneId=%s s=%.1f"
                            % (ent.road_id, ent.lane_id, ent.lane_s))
                    loc = _apply_lane_offset(wp, ent.lane_offset)
                    tf = carla.Transform(
                        loc + carla.Location(z=0.35),
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
            try:
                actor.set_attribute("role_name", ent.name)
            except Exception:
                pass

        if not actors:
            raise XoscPlayerError("没有任何实体成功布景")

        # 布景后先 tick 一帧，让 actor 的 transform 真正生效。
        # 否则 t=0 求值条件时会读到未初始化的位置（StopTrigger 会被误判为已满足）。
        world.tick()

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
        # StopTrigger 的默认求值主体：优先 ego，其次首个实体
        stop_trigger = scenario.stop_trigger
        stop_actor_name = next((e.name for e in scenario.entities
                                if e.name in actors), "")
        stop_reason = "duration"
        ended_at = dur

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
                    ready = _event_ready(world, ev, actors, actor, t, state)
                except Exception as e:
                    errors.append("条件求值失败 %s: %s" % (ev.name, e))
                    continue
                if not ready:
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
                                    "lateral_distance", "follow_trajectory",
                                    "follow"):
                        try:
                            actor.disable_constant_velocity()
                        except Exception:
                            pass
                    if act.kind == "speed" and act.shape not in _SHAPED_SPEEDS:
                        target_speed[ev.entity] = act.value
                        act._done = True
                    elif act.kind == "speed":
                        # 渐变从当前速度出发，逐帧更新 target_speed
                        target_speed[ev.entity] = act._start_value
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
                    elif act.kind == "follow":
                        follow_ctrl[ev.entity] = {
                            "ref": act.target_entity, "gap": act.value,
                            "max_accel": act.max_accel,
                            "max_decel": act.max_decel}
                    elif act.kind == "traffic_light":
                        light = _traffic_light_ahead(actor)
                        if light is None:
                            errors.append(
                                "%s 附近没有红绿灯，跳过相位切换" % ev.name)
                        else:
                            try:
                                # 冻结自动循环，保证相位可控可复现
                                light.freeze(True)
                                light.set_state(_CARLA_SIGNAL_STATES[
                                    _canonical_phase(act.phase)])
                            except Exception as e:
                                errors.append("设置红绿灯失败 %s: %s"
                                              % (ev.name, e))
                        act._done = True

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
                    if act.kind == "speed" and act.shape in _SHAPED_SPEEDS:
                        frac = min(1.0, (t - (act._started_at or 0))
                                   / _dynamics_span(act))
                        frac = _shape_frac(act.shape, frac)
                        v = act._start_value + (act.value - act._start_value) * frac
                        target_speed[ev.entity] = v
                        if frac >= 1.0:
                            act._done = True
                    elif act.kind == "follow":
                        other = actors.get(act.target_entity)
                        if other is not None:
                            follow_ctrl[ev.entity] = {
                                "ref": act.target_entity, "gap": act.value,
                                "max_accel": act.max_accel,
                                "max_decel": act.max_decel}
                    elif act.kind == "relative_speed":
                        # delta: 目标速度 = 参照速度 + 值；factor: 参照速度 * 值
                        other = actors.get(act.target_entity)
                        if other is not None:
                            base = _speed_of(other)
                            v_t = (base * act.value
                                   if act.relative_mode == "factor"
                                   else base + act.value)
                            target_speed[ev.entity] = max(0.0, min(30.0, v_t))
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
                        _drive_towards(actor, tgt, lc["speed"], _speed_of(actor),
                                       max_lat_acc=3.0)
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

                if name in follow_ctrl and actor.type_id.startswith("vehicle."):
                    fc = follow_ctrl[name]
                    lead = actors.get(fc["ref"])
                    if lead is not None and lead is not actor:
                        _idm_follow(actor, lead, fc["gap"], _speed_of(actor),
                                    max(target_speed.get(name, 12.0), 8.0),
                                    _lane_center_ahead(actor),
                                    max_accel=fc.get("max_accel", 0.0),
                                    max_decel=fc.get("max_decel", 0.0))
                        continue

                v = target_speed.get(name, 0.0)
                if actor.type_id.startswith("vehicle."):
                    # enable_constant_velocity 期望**车体坐标系**下的速度
                    # （+x 为车头方向）。之前按世界坐标传入，在朝向 ±180°
                    # 的路上会把车开成倒车——内置地图上尤其明显。
                    actor.enable_constant_velocity(
                        carla.Vector3D(x=v, y=0.0, z=0.0))

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
                        # 相机高度按最远实体撑开（垂直 FOV 约 59°，半高约
                        # 0.56*z）。取 1.85 倍 span 才装得下"前后都有车"的
                        # 跟驰场景；上限 80 m 避免车缩成像素。
                        cam_z = min(max(span * 1.85, 40.0), 80.0)

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

            # --- 停止触发（满足即提前结束，截图已在本帧完成）---
            if stop_trigger is not None:
                default_actor = actors.get(stop_actor_name)
                if default_actor is not None:
                    try:
                        hit = _trigger_ready(world, stop_trigger, actors,
                                             default_actor, t, state)
                    except Exception as e:
                        errors.append("停止条件求值失败: %s" % e)
                        hit = False
                    if hit:
                        stop_reason = "stop_trigger"
                        ended_at = t
                        break

            # --- 累计里程（traveled_distance 条件用）---
            for nm, ac in actors.items():
                if not ac.is_alive:
                    continue
                cur = _loc_of(ac)
                last = state["last"].get(nm)
                if last is not None:
                    state["traveled"][nm] = (
                        state["traveled"].get(nm, 0.0) + _distance(cur, last))
                state["last"][nm] = cur

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
        "ended_at": ended_at,
        "stop_reason": stop_reason,
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
    if result.get("stop_reason") == "stop_trigger":
        lines.append("实际运行 %.2fs（StopTrigger 满足，提前结束）"
                     % result.get("ended_at", result["duration"]))
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
