"""
osc_builder.py - 把 ScenarioSpec(IR) 编译成 OpenSCENARIO (.xosc)

替代原先 main_ai.py:7858 的手拼 XML 实现。原实现只能产出
1 实体 / 1 行为 / 1 触发条件，本模块支持任意多实体、多变道/变速行为
与多类触发条件。

实测约束（scenariogeneration 0.16.7）
-------------------------------------
- OSC 1.0 要求行人提供 model，否则抛 OpenSCENARIOVersionError → 默认输出 1.1
- Priority.overwrite 自 OSC 1.1 起废弃 → 内部统一映射为 override
- 实体类条件（时距/距离）必须走 EntityTrigger，值类条件（时间）走 ValueTrigger
- StoryBoard 的 stoptrigger 必须带 triggeringpoint="stop"
- Entities.add_scenario_object(name, obj) —— name 在前
"""
from __future__ import annotations

from typing import Dict, List, Optional

from scenariogeneration import xosc

from scenario_ir import (
    BehaviorKind, BehaviorSpec, ConditionEdge, DynamicsDimension, DynamicsShape,
    EntityKind, EntitySpec, PedestrianCategory, PositionKind, PositionSpec,
    Priority, Rule, ScenarioSpec, TriggerKind, TriggerSpec, VehicleCategory,
)

# ---------------------------------------------------------------- 映射表
_SHAPE = {
    DynamicsShape.STEP: xosc.DynamicsShapes.step,
    DynamicsShape.LINEAR: xosc.DynamicsShapes.linear,
    DynamicsShape.CUBIC: xosc.DynamicsShapes.cubic,
    DynamicsShape.SINUSOIDAL: xosc.DynamicsShapes.sinusoidal,
}
_DIM = {
    DynamicsDimension.TIME: xosc.DynamicsDimension.time,
    DynamicsDimension.DISTANCE: xosc.DynamicsDimension.distance,
    DynamicsDimension.RATE: xosc.DynamicsDimension.rate,
}
_RULE = {
    Rule.EQUAL_TO: xosc.Rule.equalTo,
    Rule.GREATER_THAN: xosc.Rule.greaterThan,
    Rule.LESS_THAN: xosc.Rule.lessThan,
    Rule.GREATER_OR_EQUAL: xosc.Rule.greaterOrEqual,
    Rule.LESS_OR_EQUAL: xosc.Rule.lessOrEqual,
    Rule.NOT_EQUAL_TO: xosc.Rule.notEqualTo,
}
_EDGE = {
    ConditionEdge.NONE: xosc.ConditionEdge.none,
    ConditionEdge.RISING: xosc.ConditionEdge.rising,
    ConditionEdge.FALLING: xosc.ConditionEdge.falling,
    ConditionEdge.RISING_OR_FALLING: xosc.ConditionEdge.risingOrFalling,
}
_PRIORITY_COMMON = {
    Priority.PARALLEL: xosc.Priority.parallel,
    Priority.SKIP: xosc.Priority.skip,
}


def _priority(p: Priority, osc_minor: int):
    """先验优先级随 OSC 版本而异。

    - OSC 1.0       : overwrite
    - OSC 1.1       : overwrite（已废弃但可用），override 尚未引入
    - OSC 1.2 及以后 : override
    """
    if p in _PRIORITY_COMMON:
        return _PRIORITY_COMMON[p]
    # OVERRIDE / OVERWRITE 均映射为"覆盖"语义
    if osc_minor >= 2:
        return xosc.Priority.override
    return xosc.Priority.overwrite
_VCAT = {v.value: getattr(xosc.VehicleCategory, v.value) for v in VehicleCategory}
_PCAT = {v.value: getattr(xosc.PedestrianCategory, v.value)
         for v in PedestrianCategory}


class OSCBuildError(RuntimeError):
    pass


# ------------------------------------------------------------- 位置
def _position(p: PositionSpec):
    if p.kind == PositionKind.LANE:
        return xosc.LanePosition(p.s, p.lane_offset, str(p.lane_id), str(p.road_id))
    if p.kind == PositionKind.WORLD:
        return xosc.WorldPosition(p.x, p.y, p.z, p.h)
    if p.kind == PositionKind.RELATIVE_LANE:
        if not p.entity_ref:
            raise OSCBuildError("relative_lane 位置需要 entity_ref")
        return xosc.RelativeLanePosition(
            p.d_lane, p.entity_ref, p.ds, 0.0, 0.0, 0.0, xosc.RelativeDistanceType.longitudinal)
    if p.kind == PositionKind.RELATIVE_OBJECT:
        if not p.entity_ref:
            raise OSCBuildError("relative_object 位置需要 entity_ref")
        return xosc.RelativeObjectPosition(p.entity_ref, p.dx, p.dy, p.dz, p.h)
    raise OSCBuildError("未知位置类型: %s" % p.kind)


# ------------------------------------------------------------- 实体
def _make_entity(e: EntitySpec):
    bb = xosc.BoundingBox(e.bounding_box.length, e.bounding_box.width,
                          e.bounding_box.height, e.bounding_box.x,
                          e.bounding_box.y, e.bounding_box.z)
    if e.kind == EntityKind.VEHICLE:
        axle_f = xosc.Axle(0.5, 0.6, 1.5, 2.8, 1.5)
        axle_r = xosc.Axle(0.5, 0.6, 1.5, 2.8, 1.5)
        cat = _VCAT.get(e.vehicle_category.value, xosc.VehicleCategory.car)
        return xosc.Vehicle(e.id, cat, bb, axle_f, axle_r,
                            e.max_speed, e.max_acceleration, e.max_deceleration)
    if e.kind == EntityKind.PEDESTRIAN:
        cat = _PCAT.get(e.pedestrian_category.value,
                        xosc.PedestrianCategory.pedestrian)
        # OSC 1.0 强制 model；给出一个占位 model 以便低版本也能导出
        return xosc.Pedestrian(e.id, e.mass, cat, bb,
                               model=e.model3d or "pedestrian_default")
    if e.kind == EntityKind.MISC_OBJECT:
        return xosc.MiscObject(e.id, e.mass, xosc.MiscObjectCategory.obstacle, bb)
    raise OSCBuildError("未知实体类型: %s" % e.kind)


# ------------------------------------------------------------- 行为
def _dynamics(b: BehaviorSpec):
    if b.dynamics_value <= 0:
        return xosc.TransitionDynamics(
            xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, 0.0)
    return xosc.TransitionDynamics(_SHAPE[b.dynamics_shape],
                                   _DIM[b.dynamics_dimension], b.dynamics_value)


def _make_action(b: BehaviorSpec):
    if b.kind == BehaviorKind.SPEED:
        if b.relative_speed:
            if not b.speed_entity_ref:
                raise OSCBuildError("relative speed 需要 speed_entity_ref")
            return xosc.RelativeSpeedAction(
                b.target_speed or 0.0, b.speed_entity_ref, _dynamics(b))
        return xosc.AbsoluteSpeedAction(b.target_speed, _dynamics(b))

    if b.kind == BehaviorKind.LANE_CHANGE:
        return xosc.RelativeLaneChangeAction(b.lane_change, b.entity_ref,
                                             _dynamics(b))

    if b.kind == BehaviorKind.LANE_OFFSET:
        # 签名: (value, shape, maxlatacc=None, continuous=True)
        return xosc.AbsoluteLaneOffsetAction(
            b.lane_offset_target or 0.0, _SHAPE[b.dynamics_shape], 1.0)

    if b.kind == BehaviorKind.LONGITUDINAL_DISTANCE:
        if not b.distance_entity_ref:
            raise OSCBuildError("longitudinal_distance 需要 distance_entity_ref")
        return xosc.LongitudinalDistanceAction(
            b.distance_entity_ref, freespace=False, distance=b.distance)

    if b.kind == BehaviorKind.LATERAL_DISTANCE:
        if not b.distance_entity_ref:
            raise OSCBuildError("lateral_distance 需要 distance_entity_ref")
        return xosc.LateralDistanceAction(
            b.distance_entity_ref, distance=b.distance, freespace=False)

    if b.kind == BehaviorKind.TELEPORT:
        raise OSCBuildError("teleport 属于初始化动作，请放在 initial_position")

    if b.kind == BehaviorKind.FOLLOW_TRAJECTORY:
        raise OSCBuildError(
            "follow_trajectory 需要先由 trajectory 模块转换为 Polyline，暂未接线")

    raise OSCBuildError("未知行为类型: %s" % b.kind)


# ------------------------------------------------------------- 触发
def _make_trigger(t: TriggerSpec, fallback_entity: str):
    rule = _RULE[t.rule]
    edge = _EDGE[t.edge]
    subject = t.triggering_entity or fallback_entity
    name = t.resolved_name()

    # --- 值类条件 → ValueTrigger ---
    if t.kind == TriggerKind.TIME:
        return xosc.ValueTrigger(
            name, t.delay, edge,
            xosc.SimulationTimeCondition(t.value, rule))

    if t.kind == TriggerKind.STORYBOARD_STATE:
        return xosc.ValueTrigger(
            name, t.delay, edge,
            xosc.StoryboardElementStateCondition(
                xosc.StoryboardElementType.act, t.entity_ref or "act",
                xosc.StoryboardElementState.completeState))

    # --- 实体类条件 → EntityTrigger ---
    if t.kind == TriggerKind.TIME_HEADWAY:
        if not t.entity_ref:
            raise OSCBuildError("time_headway 需要 entity_ref 作为参照车")
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.TimeHeadwayCondition(t.entity_ref, t.value, rule), subject)

    if t.kind == TriggerKind.DISTANCE:
        pos = _position(t.position) if t.position else xosc.WorldPosition(0, 0, 0)
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.DistanceCondition(t.value, rule, pos, freespace=False), subject)

    if t.kind == TriggerKind.SPEED:
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.SpeedCondition(t.value, rule), subject)

    if t.kind == TriggerKind.RELATIVE_SPEED:
        if not t.entity_ref:
            raise OSCBuildError("relative_speed 需要 entity_ref")
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.RelativeSpeedCondition(t.value, rule, t.entity_ref), subject)

    if t.kind == TriggerKind.REACH_POSITION:
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.ReachPositionCondition(_position(t.position), t.tolerance),
            subject)

    if t.kind == TriggerKind.COLLISION:
        if not t.entity_ref:
            raise OSCBuildError("collision 需要 entity_ref 作为碰撞目标")
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.CollisionCondition(xosc.EntityRef(t.entity_ref)), subject)

    if t.kind == TriggerKind.STANDSTILL:
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.StandStillCondition(t.value), subject)

    if t.kind == TriggerKind.TIME_TO_COLLISION:
        if not t.entity_ref:
            raise OSCBuildError("time_to_collision 需要 entity_ref")
        return xosc.EntityTrigger(
            name, t.delay, edge,
            xosc.TimeToCollisionCondition(
                t.value, rule, freespace=False,
                position=xosc.WorldPosition(0, 0, 0)), subject)

    raise OSCBuildError("未知触发类型: %s" % t.kind)


# =============================================================== 主入口
def build_scenario(spec: ScenarioSpec) -> xosc.Scenario:
    """ScenarioSpec -> xosc.Scenario（尚未落盘）。"""
    # 1. 实体
    entities = xosc.Entities()
    for e in spec.entities:
        entities.add_scenario_object(e.id, _make_entity(e))

    # 2. 初始化
    init = xosc.Init()
    for e in spec.entities:
        init.add_init_action(
            e.id, xosc.TeleportAction(_position(e.initial_position)))
        if e.initial_speed:
            init.add_init_action(
                e.id, xosc.AbsoluteSpeedAction(
                    e.initial_speed,
                    xosc.TransitionDynamics(
                        xosc.DynamicsShapes.step,
                        xosc.DynamicsDimension.time, 0.0)))

    # 3. 每个行为配一个触发条件
    #    触发条件按顺序分配给行为；不足则复用最后一个；无触发则 t=0
    default_entity = spec.entities[0].id
    triggers = list(spec.triggers)

    groups: List[xosc.ManeuverGroup] = []
    fallback = xosc.ValueTrigger(
        "start", 0, xosc.ConditionEdge.rising,
        xosc.SimulationTimeCondition(0.0, xosc.Rule.greaterThan))

    for i, b in enumerate(spec.behaviors):
        action = _make_action(b)
        ev = xosc.Event("%s_ev" % b.resolved_name(),
                        _priority(spec.priority, spec.osc_minor_version))
        ev.add_action("%s_act" % b.resolved_name(), action)

        if triggers:
            t = triggers[i] if i < len(triggers) else triggers[-1]
            ev.add_trigger(_make_trigger(t, default_entity))
        else:
            ev.add_trigger(fallback)

        man = xosc.Maneuver("%s_man" % b.resolved_name())
        man.add_event(ev)

        mg = xosc.ManeuverGroup("mg_%s" % b.resolved_name())
        mg.add_actor(b.entity_ref)
        mg.add_maneuver(man)
        groups.append(mg)

    # 4. Story / Act
    sb = xosc.StoryBoard(
        init,
        xosc.ValueTrigger(
            "stop", 0, xosc.ConditionEdge.rising,
            xosc.SimulationTimeCondition(spec.duration, xosc.Rule.greaterThan),
            triggeringpoint="stop"))

    if groups:
        act = xosc.Act("act", xosc.ValueTrigger(
            "start", 0, xosc.ConditionEdge.rising,
            xosc.SimulationTimeCondition(0.0, xosc.Rule.greaterThan)))
        for mg in groups:
            act.add_maneuver_group(mg)
        story = xosc.Story("story")
        story.add_act(act)
        sb.add_story(story)

    # 5. 路网
    rn = xosc.RoadNetwork(scene=spec.xodr_file) if spec.xodr_file \
        else xosc.RoadNetwork()

    return xosc.Scenario(
        spec.name, spec.author, xosc.ParameterDeclarations(),
        entities, sb, rn, xosc.Catalog(),
        osc_minor_version=spec.osc_minor_version)


def write_scenario(spec: ScenarioSpec, path: str) -> str:
    """编译并写出 .xosc，返回文件路径。"""
    sc = build_scenario(spec)
    sc.write_xml(path)
    return path


def validate_roundtrip(path: str) -> Dict[str, object]:
    """结构校验：用 scenariogeneration 自己的 parser 回读。

    库不打包 ASAM XSD，且官方 XSD 直链不可得，因此以
    ParseOpenScenario 成功解析作为"结构合法"的判据。
    """
    try:
        sc = xosc.ParseOpenScenario(path)
    except Exception as e:
        return {"valid": False, "error": "%s: %s" % (type(e).__name__, e)}

    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    hdr = root.find("FileHeader")
    ents = root.findall(".//Entities/ScenarioObject")
    mans = root.findall(".//Maneuver")
    conds = root.findall(".//Condition")
    return {
        "valid": True,
        "version": "%s.%s" % (hdr.get("revMajor"), hdr.get("revMinor")),
        "entities": len(ents),
        "entity_names": [e.get("name") for e in ents],
        "maneuvers": len(mans),
        "conditions": len(conds),
        "size_bytes": __import__("os").path.getsize(path),
    }
