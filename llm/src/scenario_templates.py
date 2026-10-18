"""
scenario_templates.py - 参数化场景模板库

设计原则
--------
原项目已有 17 个 scenario_* 函数（main_ai.py:1389-3058），它们直接往 CARLA
世界 spawn actor，参数硬编码在代码里，场景关掉即失、无法批量、无法复现。

本模块**不重写**这些函数，而是把它们验证过的场景语义"数据化"：
每个模板根据参数产出一份 ScenarioSpec，进而可导出 .xosc、可采样、可复现。

参数来源标注了原函数行号，便于对照与后续回灌 CARLA。
"""
from __future__ import annotations

import random
from typing import Any, Callable, Dict, List

from scenario_ir import (
    BehaviorKind, BehaviorSpec, BoundingBoxSpec, ConditionEdge,
    DynamicsDimension, DynamicsShape, EntityKind, EntitySpec, ParamRange,
    PedestrianCategory, PositionKind, PositionSpec, Rule, ScenarioSpec,
    TemplateSpec, TriggerKind, TriggerSpec, VehicleCategory,
)

# =====================================================================
# 公共工具
# =====================================================================
LANE_R = "-1"    # 右车道（OpenDRIVE 负号）
LANE_L = "-2"    # 左侧相邻车道
LANE_ONCOMING = "1"  # 对向车道（正号）


def _veh(eid: str, s: float, speed: float = 10.0, lane: str = LANE_R,
         blueprint: str = None) -> EntitySpec:
    return EntitySpec(id=eid, kind=EntityKind.VEHICLE,
                      vehicle_category=VehicleCategory.CAR,
                      initial_speed=speed,
                      initial_position=PositionSpec.lane(s=s, lane_id=lane),
                      carla_blueprint=blueprint or "vehicle.tesla.model3")


def _ped(eid: str, x: float, y: float, speed: float = 1.4) -> EntitySpec:
    return EntitySpec(id=eid, kind=EntityKind.PEDESTRIAN,
                      pedestrian_category=PedestrianCategory.PEDESTRIAN,
                      initial_speed=speed,
                      # 行人包围盒（约 0.6 × 0.6 × 1.8 m），
                      # 不能用车辆默认值
                      bounding_box=BoundingBoxSpec(length=0.6, width=0.6,
                                                   height=1.8, x=0.0, y=0.0, z=0.9),
                      initial_position=PositionSpec.world(x, y, 0.0, 0.0),
                      carla_blueprint="walker.pedestrian.0001")


def _speed(eid: str, target: float, shape: DynamicsShape = DynamicsShape.LINEAR,
           value: float = 2.0, ref: str = None) -> BehaviorSpec:
    """构造速度行为。

    ref 不为空时表示**相对速度**：目标是"参照实体速度 + target"。
    """
    return BehaviorSpec(kind=BehaviorKind.SPEED, entity_ref=eid,
                        target_speed=target, dynamics_shape=shape,
                        dynamics_value=value,
                        relative_speed=bool(ref),
                        speed_entity_ref=ref)


def _lane_change(eid: str, delta: int, value: float = 1.6) -> BehaviorSpec:
    return BehaviorSpec(kind=BehaviorKind.LANE_CHANGE, entity_ref=eid,
                        lane_change=delta, dynamics_value=value)


def _follow(eid: str, ref: str, distance: float = 10.0) -> BehaviorSpec:
    """纵向跟驰：保持与 ref 的距离。"""
    return BehaviorSpec(kind=BehaviorKind.LONGITUDINAL_DISTANCE, entity_ref=eid,
                        distance_entity_ref=ref, distance=distance)


def _offset(eid: str, target: float,
            shape: DynamicsShape = DynamicsShape.LINEAR) -> BehaviorSpec:
    """车道内横向偏移。"""
    return BehaviorSpec(kind=BehaviorKind.LANE_OFFSET, entity_ref=eid,
                        lane_offset_target=target, dynamics_shape=shape)


# =====================================================================
# 模板 1：危险切入  （源自 scenario_cut_in, main_ai.py:2594）
#   原硬编码：cutter 在前方 12.0m，3.0s 后切入，steer=-0.5，throttle=0.45，时长 1.6s
# =====================================================================
def build_cut_in(p: Dict[str, Any]) -> ScenarioSpec:
    direction = p.get("direction", "left")
    gap = float(p.get("gap", 12.0))
    delay = float(p.get("delay", 3.0))
    ego_speed = float(p.get("ego_speed", 12.0))
    cutter_speed = float(p.get("cutter_speed", 13.0))
    duration = float(p.get("duration", 30.0))

    # direction 决定 cutter 起始车道与变道方向
    if direction == "left":
        cutter_lane, delta = LANE_L, +1
    else:
        cutter_lane, delta = LANE_R, -1

    return ScenarioSpec(
        name="cut_in_%s" % direction,
        description="邻道车在 %.1fs 后以 %.1fm 间距切入本车道" % (delay, gap),
        entities=[_veh("ego", 50.0, ego_speed, LANE_R),
                  _veh("cutter", 50.0 + gap, cutter_speed, cutter_lane),
                  _veh("follower", 50.0 - max(gap, 15.0), ego_speed, LANE_R),
                  _ped("ped", 0.0, 8.0, 1.4)],
        behaviors=[_speed("ego", ego_speed, DynamicsShape.STEP, 0.0),
                   _lane_change("cutter", delta, 1.6),
                   _follow("follower", "ego", 12.0),
                   _offset("cutter", 0.0, DynamicsShape.LINEAR),
                   _speed("ped", 1.4, DynamicsShape.STEP, 0.0)],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.TIME, value=delay,
                        edge=ConditionEdge.RISING),
            TriggerSpec(kind=TriggerKind.TIME_HEADWAY, rule=Rule.LESS_THAN,
                        value=1.5, entity_ref="cutter", triggering_entity="ego"),
            TriggerSpec(kind=TriggerKind.SPEED, rule=Rule.GREATER_THAN,
                        value=ego_speed * 0.5, triggering_entity="ego"),
            TriggerSpec(kind=TriggerKind.REACH_POSITION, tolerance=2.0,
                        position=PositionSpec.lane(s=50.0 + gap * 2, lane_id=LANE_R),
                        triggering_entity="ego"),
        ],
        duration=duration, template="cut_in",
        params={"direction": direction, "gap": gap, "delay": delay},
        tags=["危险", "变道", "cut-in"],
    )


# =====================================================================
# 模板 2：前车急刹  （源自 scenario_lead_vehicle mode='brake', main_ai.py:2543）
#   原硬编码：distance=25.0，4.0s 后刹车，持续 4.0s
# =====================================================================
def build_lead_brake(p: Dict[str, Any]) -> ScenarioSpec:
    distance = float(p.get("distance", 25.0))
    brake_delay = float(p.get("brake_delay", 4.0))
    brake_speed = float(p.get("brake_speed", 2.0))
    ego_speed = float(p.get("ego_speed", 13.0))
    lead_speed = float(p.get("lead_speed", 13.0))
    duration = float(p.get("duration", 25.0))

    return ScenarioSpec(
        name="lead_brake",
        description="前车在 %.1fs 后由 %.1f 减至 %.1f m/s" % (
            brake_delay, lead_speed, brake_speed),
        entities=[_veh("ego", 50.0, ego_speed, LANE_R),
                  _veh("lead", 50.0 + distance, lead_speed, LANE_R),
                  _veh("follower", 50.0 - 30.0, ego_speed, LANE_R)],
        behaviors=[_speed("lead", lead_speed, DynamicsShape.STEP, 0.0),
                   _speed("lead", brake_speed, DynamicsShape.LINEAR, 2.5),
                   _follow("ego", "lead", 15.0),
                   _follow("follower", "ego", 18.0)],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.TIME, value=brake_delay,
                        edge=ConditionEdge.RISING),
            # ego 用跟驰距离触发制动（原实现是硬编码 sleep）
            TriggerSpec(kind=TriggerKind.TIME_HEADWAY, rule=Rule.LESS_THAN,
                        value=1.2, entity_ref="lead", triggering_entity="ego"),
            TriggerSpec(kind=TriggerKind.STANDSTILL, value=1.0,
                        triggering_entity="lead"),
            TriggerSpec(kind=TriggerKind.SPEED, rule=Rule.LESS_THAN,
                        value=brake_speed + 1.0, triggering_entity="lead"),
        ],
        duration=duration, template="lead_brake",
        params={"distance": distance, "brake_delay": brake_delay},
        tags=["追尾", "AEB", "前车"],
    )


# =====================================================================
# 模板 3：前车静止  （源自 scenario_lead_vehicle mode='stationary'）
# =====================================================================
def build_lead_stationary(p: Dict[str, Any]) -> ScenarioSpec:
    distance = float(p.get("distance", 40.0))
    ego_speed = float(p.get("ego_speed", 14.0))
    duration = float(p.get("duration", 20.0))

    return ScenarioSpec(
        name="lead_stationary",
        description="同车道 %.1fm 处静止障碍车" % distance,
        entities=[_veh("ego", 50.0, ego_speed, LANE_R),
                  _veh("lead", 50.0 + distance, 0.0, LANE_R)],
        behaviors=[_speed("ego", 0.0, DynamicsShape.LINEAR, 3.0)],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN,
                        value=max(distance * 0.6, 10.0),
                        position=PositionSpec.lane(s=50.0 + distance, lane_id=LANE_R),
                        triggering_entity="ego"),
        ],
        duration=duration, template="lead_stationary",
        params={"distance": distance},
        tags=["静止障碍", "前车"],
    )


# =====================================================================
# 模板 4：路口危险横穿  （源自 scenario_crossing_hazard, main_ai.py:2735）
#   原硬编码：ego 前方 30.0m，行人速度 1.6 m/s，需 ≥3 臂路口
# =====================================================================
def build_crossing_hazard(p: Dict[str, Any]) -> ScenarioSpec:
    crosser = p.get("crosser", "pedestrian")
    ego_speed = float(p.get("ego_speed", 10.0))
    ped_speed = float(p.get("ped_speed", 1.6))
    ego_offset = float(p.get("ego_offset", 30.0))
    lateral = float(p.get("lateral", 6.0))
    duration = float(p.get("duration", 25.0))

    if crosser == "pedestrian":
        cross_entity = _ped("crosser", 0.0, lateral, ped_speed)
        cross_beh = _speed("crosser", ped_speed, DynamicsShape.STEP, 0.0)
    else:
        cat = VehicleCategory.BICYCLE if crosser == "bicycle" else VehicleCategory.CAR
        cross_entity = EntitySpec(
            id="crosser", kind=EntityKind.VEHICLE, vehicle_category=cat,
            initial_speed=ped_speed if crosser == "bicycle" else 8.0,
            initial_position=PositionSpec.world(0.0, lateral, 0.0, -90.0))
        cross_beh = _speed("crosser", ped_speed if crosser == "bicycle" else 8.0,
                           DynamicsShape.STEP, 0.0)

    return ScenarioSpec(
        name="crossing_hazard_%s" % crosser,
        description="%s 从侧向 %.1fm 处横穿本车路径" % (crosser, lateral),
        entities=[_veh("ego", -ego_offset, ego_speed, LANE_R), cross_entity],
        behaviors=[_speed("ego", ego_speed, DynamicsShape.STEP, 0.0), cross_beh],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            # 横穿者先动
            TriggerSpec(kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN,
                        value=25.0,
                        position=PositionSpec.world(0.0, lateral, 0.0, 0.0),
                        triggering_entity="ego"),
            # ego 制动
            TriggerSpec(kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN,
                        value=12.0,
                        position=PositionSpec.world(0.0, lateral, 0.0, 0.0),
                        triggering_entity="ego"),
        ],
        duration=duration, template="crossing_hazard",
        params={"crosser": crosser, "lateral": lateral},
        tags=["横穿", "VRU", "路口"],
    )


# =====================================================================
# 模板 5：低重叠碰撞  （源自 scenario_low_overlap, main_ai.py:2823）
#   原硬编码：offset_ratio=0.35，横向偏移 = lane_width*(0.5-offset_ratio)
# =====================================================================
def build_low_overlap(p: Dict[str, Any]) -> ScenarioSpec:
    offset_ratio = float(p.get("offset_ratio", 0.35))
    lane_width = float(p.get("lane_width", 3.5))
    gap = float(p.get("gap", 20.0))
    ego_speed = float(p.get("ego_speed", 12.0))
    target_speed = float(p.get("target_speed", 8.0))
    duration = float(p.get("duration", 25.0))

    # 横向偏移：目标车向本车道一侧贴线
    shift = lane_width * (0.5 - offset_ratio)

    target = EntitySpec(
        id="target", kind=EntityKind.VEHICLE, vehicle_category=VehicleCategory.CAR,
        initial_speed=target_speed,
        initial_position=PositionSpec(
            kind=PositionKind.LANE, s=50.0 + gap, lane_id=LANE_L,
            lane_offset=shift))

    return ScenarioSpec(
        name="low_overlap",
        description="目标车贴线 %.2fm（重叠率 %.0f%%）" % (shift, offset_ratio * 100),
        entities=[_veh("ego", 50.0, ego_speed, LANE_R), target,
                  _ped("ped", 0.0, 9.0, 1.4)],
        behaviors=[_speed("ego", ego_speed, DynamicsShape.STEP, 0.0),
                   _offset("target", shift, DynamicsShape.LINEAR),
                   _follow("target", "ego", -max(gap, 10.0)),
                   _speed("ped", 1.4, DynamicsShape.STEP, 0.0)],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN,
                        value=max(gap * 0.5, 8.0),
                        position=PositionSpec.lane(s=50.0 + gap, lane_id=LANE_L),
                        triggering_entity="ego"),
            TriggerSpec(kind=TriggerKind.SPEED, rule=Rule.GREATER_THAN,
                        value=ego_speed * 0.6, triggering_entity="ego"),
            TriggerSpec(kind=TriggerKind.REACH_POSITION, tolerance=1.5,
                        position=PositionSpec.world(0.0, 9.0, 0.0, 0.0),
                        triggering_entity="ped"),
        ],
        duration=duration, template="low_overlap",
        params={"offset_ratio": offset_ratio, "gap": gap},
        tags=["碰撞", "低重叠", "偏置"],
    )


# =====================================================================
# 模板 6：逆行  （源自 scenario_wrong_way, main_ai.py:2922）
#   原硬编码：speed=8.0，对向车道 spawn 并旋转 180°
# =====================================================================
def build_wrong_way(p: Dict[str, Any]) -> ScenarioSpec:
    speed = float(p.get("speed", 8.0))
    ego_speed = float(p.get("ego_speed", 12.0))
    gap = float(p.get("gap", 60.0))
    duration = float(p.get("duration", 25.0))

    wrong = EntitySpec(
        id="wrong_way", kind=EntityKind.VEHICLE,
        vehicle_category=VehicleCategory.CAR, initial_speed=speed,
        initial_position=PositionSpec(
            kind=PositionKind.LANE, s=50.0 + gap, lane_id=LANE_ONCOMING, h=180.0))

    return ScenarioSpec(
        name="wrong_way",
        description="对向车道车辆以 %.1f m/s 逆行接近" % speed,
        entities=[_veh("ego", 50.0, ego_speed, LANE_R), wrong],
        behaviors=[_speed("ego", ego_speed, DynamicsShape.STEP, 0.0),
                   _speed("wrong_way", speed, DynamicsShape.STEP, 0.0)],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN,
                        value=max(gap * 0.5, 20.0),
                        position=PositionSpec.lane(s=50.0 + gap, lane_id=LANE_ONCOMING),
                        triggering_entity="ego"),
        ],
        duration=duration, template="wrong_way",
        params={"speed": speed, "gap": gap},
        tags=["逆行", "对向", "危险"],
    )


# =====================================================================
# 模板 7：无保护左转  （源自 scenario_unprotected_turn, main_ai.py:3007）
# =====================================================================
def build_unprotected_turn(p: Dict[str, Any]) -> ScenarioSpec:
    ego_speed = float(p.get("ego_speed", 8.0))
    oncoming_speed = float(p.get("oncoming_speed", 12.0))
    oncoming_gap = float(p.get("oncoming_gap", 50.0))
    duration = float(p.get("duration", 30.0))

    return ScenarioSpec(
        name="unprotected_turn",
        description="无保护左转，对向来车距 %.1fm" % oncoming_gap,
        entities=[
            _veh("ego", 50.0, ego_speed, LANE_R),
            EntitySpec(id="oncoming", kind=EntityKind.VEHICLE,
                       vehicle_category=VehicleCategory.CAR,
                       initial_speed=oncoming_speed,
                       initial_position=PositionSpec(
                           kind=PositionKind.LANE, s=50.0 + oncoming_gap,
                           lane_id=LANE_ONCOMING, h=180.0)),
            _ped("ped", 0.0, 8.0, 1.4),
        ],
        behaviors=[_speed("ego", ego_speed, DynamicsShape.STEP, 0.0),
                   _lane_change("ego", +1, 3.0),
                   _speed("oncoming", oncoming_speed, DynamicsShape.STEP, 0.0),
                   _speed("ped", 1.4, DynamicsShape.STEP, 0.0)],
        triggers=[
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.TIME, value=5.0, edge=ConditionEdge.RISING),
            TriggerSpec(kind=TriggerKind.TIME, value=0.0),
            TriggerSpec(kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN, value=15.0,
                        position=PositionSpec.world(0.0, 8.0, 0.0, 0.0),
                        triggering_entity="ego"),
        ],
        duration=duration, template="unprotected_turn",
        params={"oncoming_gap": oncoming_gap, "oncoming_speed": oncoming_speed},
        tags=["左转", "无保护", "交叉口"],
    )


# =====================================================================
# 模板 8：信号交叉口冲突  （源自 scenario_junction_light, main_ai.py:1785）
# =====================================================================
def build_junction_light(p: Dict[str, Any]) -> ScenarioSpec:
    vehicle_count = int(p.get("vehicle_count", 4))
    ego_speed = float(p.get("ego_speed", 10.0))
    cross_speed = float(p.get("cross_speed", 9.0))
    duration = float(p.get("duration", 30.0))

    entities = [_veh("ego", 50.0, ego_speed, LANE_R),
                _veh("follower", 50.0 - 25.0, ego_speed, LANE_R)]
    behaviors = [_speed("ego", ego_speed, DynamicsShape.STEP, 0.0),
                 _follow("follower", "ego", 16.0)]
    triggers = [TriggerSpec(kind=TriggerKind.TIME, value=0.0),
                TriggerSpec(kind=TriggerKind.SPEED, rule=Rule.LESS_THAN,
                            value=ego_speed * 0.3, triggering_entity="ego")]

    # 横向来车：world 位置铺开，跨过 ego 路径
    for i in range(max(vehicle_count - 1, 1)):
        eid = "cross_%d" % (i + 1)
        y = 12.0 + i * 12.0
        entities.append(EntitySpec(
            id=eid, kind=EntityKind.VEHICLE, vehicle_category=VehicleCategory.CAR,
            initial_speed=cross_speed,
            initial_position=PositionSpec.world(-30.0, y, 0.0, 90.0)))
        behaviors.append(_speed(eid, cross_speed, DynamicsShape.STEP, 0.0))
        triggers.append(TriggerSpec(
            kind=TriggerKind.DISTANCE, rule=Rule.LESS_THAN, value=25.0,
            position=PositionSpec.world(-30.0, y, 0.0, 0.0),
            triggering_entity="ego"))

    # 停车等待信号 + 到达停车线
    triggers.append(TriggerSpec(
        kind=TriggerKind.STANDSTILL, value=2.0, triggering_entity="ego"))
    triggers.append(TriggerSpec(
        kind=TriggerKind.REACH_POSITION, tolerance=2.0,
        position=PositionSpec.lane(s=70.0, lane_id=LANE_R),
        triggering_entity="ego"))

    return ScenarioSpec(
        name="junction_light",
        description="信号交叉口，%d 辆横向来车" % max(vehicle_count - 1, 1),
        entities=entities, behaviors=behaviors, triggers=triggers,
        duration=duration, template="junction_light",
        params={"vehicle_count": vehicle_count},
        tags=["交叉口", "信号灯", "冲突"],
    )


# =====================================================================
# 模板 9：近距跟驰（TTC 触发减速）
#   自车以明显高于前车的速度接近，当 TTC 低于阈值时减速到前车速度。
#   用来验证 TimeToCollisionCondition + 相对位置语义。
# =====================================================================
def build_close_follow(p: Dict[str, Any]) -> ScenarioSpec:
    ego_speed = float(p.get("ego_speed", 16.0))
    lead_speed = float(p.get("lead_speed", 6.0))
    gap = float(p.get("gap", 35.0))
    ttc = float(p.get("ttc", 3.0))
    brake_time = float(p.get("brake_time", 3.0))
    duration = float(p.get("duration", 30.0))

    return ScenarioSpec(
        name="close_follow",
        description="自车 %.1fm/s 接近 %.1fm/s 前车，TTC 低于 %.1fs 时减速"
                    % (ego_speed, lead_speed, ttc),
        entities=[_veh("ego", 50.0, ego_speed, LANE_R),
                  _veh("lead", 50.0 + gap, lead_speed, LANE_R)],
        # behaviors[i] 与 triggers[i] 按下标配对
        behaviors=[_speed("ego", lead_speed, DynamicsShape.LINEAR, brake_time)],
        triggers=[TriggerSpec(kind=TriggerKind.TIME_TO_COLLISION, value=ttc,
                              rule=Rule.LESS_THAN, entity_ref="lead",
                              triggering_entity="ego", name="ttc_brake")],
        duration=duration, template="close_follow",
        params={"ego_speed": ego_speed, "lead_speed": lead_speed,
                "gap": gap, "ttc": ttc},
        tags=["追尾", "TTC", "跟驰"],
    )


# =====================================================================
# 模板 10：超车
#   接近慢车 -> 向左变道 -> 以相对速度超越 -> 回到原车道。
#   用来验证 ReachPositionCondition + 相对速度动作 + 连续变道。
# =====================================================================
def build_overtake(p: Dict[str, Any]) -> ScenarioSpec:
    ego_speed = float(p.get("ego_speed", 14.0))
    lead_speed = float(p.get("lead_speed", 7.0))
    gap = float(p.get("gap", 40.0))
    delta = float(p.get("delta", 4.0))       # 超越时相对前车的速度差
    lc_time = float(p.get("lc_time", 1.8))
    duration = float(p.get("duration", 35.0))

    return ScenarioSpec(
        name="overtake",
        description="接近 %.1fm 前方的慢车后向左变道，以 +%.1fm/s 超越后回到原车道"
                    % (gap, delta),
        entities=[_veh("ego", 50.0, ego_speed, LANE_R),
                  _veh("lead", 50.0 + gap, lead_speed, LANE_R),
                  _veh("oncoming", 50.0 + gap + 80.0, 10.0, LANE_ONCOMING)],
        behaviors=[
            # 1) 接近到 25m 内向左变道
            _lane_change("ego", +1, lc_time),
            # 2) 相对前车 +delta 超车（相对速度动作）
            _speed("ego", delta, DynamicsShape.LINEAR, 4.0, ref="lead"),
            # 3) 拉开到 30m 后回到原车道
            _lane_change("ego", -1, lc_time),
        ],
        triggers=[
            TriggerSpec(kind=TriggerKind.DISTANCE, value=25.0,
                        rule=Rule.LESS_THAN,
                        position=PositionSpec.lane(s=50.0 + gap, lane_id=LANE_R),
                        triggering_entity="ego", name="approach"),
            TriggerSpec(kind=TriggerKind.TIME, value=1.0,
                        rule=Rule.GREATER_THAN,
                        triggering_entity="ego", name="speed_up"),
            TriggerSpec(kind=TriggerKind.REACH_POSITION, tolerance=3.0,
                        position=PositionSpec.lane(s=50.0 + gap + 30.0,
                                                   lane_id=LANE_L),
                        triggering_entity="ego", name="return"),
        ],
        duration=duration, template="overtake",
        params={"ego_speed": ego_speed, "lead_speed": lead_speed,
                "gap": gap, "delta": delta},
        tags=["超车", "变道", "相对速度"],
    )


# =====================================================================
# 模板 11：紧急避让（轨迹跟随）
#   前方本车道有静止障碍，自车按一条 S 形轨迹换道绕行。
#   用 FollowTrajectoryAction 表达任意形状的避让路径。
# =====================================================================
def build_evasive_swerve(p: Dict[str, Any]) -> ScenarioSpec:
    ego_speed = float(p.get("ego_speed", 12.0))
    obstacle_gap = float(p.get("obstacle_gap", 45.0))
    lateral = float(p.get("lateral", 3.6))       # 横向避让量（约一个车道）
    swerve_len = float(p.get("swerve_len", 30.0))
    duration = float(p.get("duration", 30.0))

    s0 = 50.0
    s_obs = s0 + obstacle_gap
    # 右车道中心线的横向坐标。straight 路网（lane_width=3.5、2 车道）实测约 1.8m，
    # 这里跟着 lane_width 走，默认参数下即 1.75。
    y0 = float(p.get("lane_center_y", 1.75))
    # 五个关键点：直行 -> 开始避让 -> 越过障碍 -> 回正 -> 继续直行
    pts = [
        [0.0, s0, y0],
        [obstacle_gap / ego_speed * 0.35, s0 + swerve_len * 0.4, y0 + lateral],
        [obstacle_gap / ego_speed * 0.65, s_obs, y0 + lateral],
        [obstacle_gap / ego_speed * 0.90, s_obs + swerve_len * 0.5, y0],
        [obstacle_gap / ego_speed * 1.30, s_obs + swerve_len * 1.4, y0],
    ]

    return ScenarioSpec(
        name="evasive_swerve",
        description="前方 %.1fm 有静止障碍，自车按 S 形轨迹换道绕行"
                    % obstacle_gap,
        entities=[_veh("ego", s0, ego_speed, LANE_R),
                  _veh("obstacle", s_obs, 0.0, LANE_R),
                  _veh("oncoming", s0 + 60.0, 9.0, LANE_ONCOMING)],
        behaviors=[
            BehaviorSpec(kind=BehaviorKind.FOLLOW_TRAJECTORY, entity_ref="ego",
                         trajectory_points=pts),
            _speed("oncoming", 9.0, DynamicsShape.STEP, 0.0),
        ],
        triggers=[
            # 轨迹第一点就是车辆初始位置，所以要尽早触发；
            # 延迟太久车会开过起点，前瞻点就落到车后了。
            TriggerSpec(kind=TriggerKind.TIME, value=0.0,
                        rule=Rule.GREATER_THAN,
                        triggering_entity="ego", name="swerve_start"),
            TriggerSpec(kind=TriggerKind.TIME, value=0.0,
                        rule=Rule.GREATER_THAN,
                        triggering_entity="oncoming", name="oncoming_start"),
        ],
        duration=duration, template="evasive_swerve",
        params={"ego_speed": ego_speed, "obstacle_gap": obstacle_gap,
                "lateral": lateral},
        tags=["避让", "轨迹跟随", "紧急"],
    )


# =====================================================================
# 注册表
# =====================================================================
_BUILDERS: Dict[str, Callable[[Dict[str, Any]], ScenarioSpec]] = {
    "cut_in": build_cut_in,
    "lead_brake": build_lead_brake,
    "lead_stationary": build_lead_stationary,
    "crossing_hazard": build_crossing_hazard,
    "low_overlap": build_low_overlap,
    "wrong_way": build_wrong_way,
    "unprotected_turn": build_unprotected_turn,
    "junction_light": build_junction_light,
    "close_follow": build_close_follow,
    "overtake": build_overtake,
    "evasive_swerve": build_evasive_swerve,
}

TEMPLATES: Dict[str, TemplateSpec] = {
    "cut_in": TemplateSpec(
        name="cut_in", title="危险切入",
        description="邻道车突然切入本车道前方（源自 scenario_cut_in）",
        carla_function="scenario_cut_in", tags=["危险", "变道"],
        params={
            "direction": ParamRange(kind="choice", choices=["left", "right"]),
            "gap": ParamRange(kind="uniform", low=8.0, high=25.0, unit="m"),
            "delay": ParamRange(kind="uniform", low=1.5, high=6.0, unit="s"),
            "ego_speed": ParamRange(kind="uniform", low=8.0, high=18.0, unit="m/s"),
            "cutter_speed": ParamRange(kind="uniform", low=8.0, high=20.0, unit="m/s"),
        }),
    "lead_brake": TemplateSpec(
        name="lead_brake", title="前车急刹",
        description="同车道前车急减速（源自 scenario_lead_vehicle mode=brake）",
        carla_function="scenario_lead_vehicle", tags=["追尾", "AEB"],
        params={
            "distance": ParamRange(kind="uniform", low=15.0, high=45.0, unit="m"),
            "brake_delay": ParamRange(kind="uniform", low=2.0, high=8.0, unit="s"),
            "brake_speed": ParamRange(kind="uniform", low=0.0, high=5.0, unit="m/s"),
            "ego_speed": ParamRange(kind="uniform", low=8.0, high=20.0, unit="m/s"),
        }),
    "lead_stationary": TemplateSpec(
        name="lead_stationary", title="前车静止",
        description="同车道静止障碍车（源自 scenario_lead_vehicle mode=stationary）",
        carla_function="scenario_lead_vehicle", tags=["静止障碍"],
        params={
            "distance": ParamRange(kind="uniform", low=25.0, high=70.0, unit="m"),
            "ego_speed": ParamRange(kind="uniform", low=8.0, high=20.0, unit="m/s"),
        }),
    "crossing_hazard": TemplateSpec(
        name="crossing_hazard", title="路口危险横穿",
        description="人/车从侧向横穿本车路径（源自 scenario_crossing_hazard）",
        carla_function="scenario_crossing_hazard", tags=["横穿", "VRU"],
        params={
            "crosser": ParamRange(kind="choice",
                                  choices=["pedestrian", "bicycle", "vehicle"]),
            "lateral": ParamRange(kind="uniform", low=4.0, high=10.0, unit="m"),
            "ego_speed": ParamRange(kind="uniform", low=6.0, high=15.0, unit="m/s"),
            "ped_speed": ParamRange(kind="uniform", low=1.0, high=2.5, unit="m/s"),
        }),
    "low_overlap": TemplateSpec(
        name="low_overlap", title="低重叠碰撞",
        description="目标车贴线行驶，部分侵入本车道（源自 scenario_low_overlap）",
        carla_function="scenario_low_overlap", tags=["碰撞", "偏置"],
        params={
            "offset_ratio": ParamRange(kind="uniform", low=0.15, high=0.5),
            "gap": ParamRange(kind="uniform", low=12.0, high=35.0, unit="m"),
            "lane_width": ParamRange(kind="fixed", value=3.5, unit="m"),
        }),
    "wrong_way": TemplateSpec(
        name="wrong_way", title="逆行",
        description="对向车道出现逆行车辆（源自 scenario_wrong_way）",
        carla_function="scenario_wrong_way", tags=["逆行", "危险"],
        params={
            "speed": ParamRange(kind="uniform", low=4.0, high=15.0, unit="m/s"),
            "gap": ParamRange(kind="uniform", low=30.0, high=100.0, unit="m"),
            "ego_speed": ParamRange(kind="uniform", low=8.0, high=18.0, unit="m/s"),
        }),
    "unprotected_turn": TemplateSpec(
        name="unprotected_turn", title="无保护左转",
        description="无左转保护信号下穿越对向车流（源自 scenario_unprotected_turn）",
        carla_function="scenario_unprotected_turn", tags=["左转", "交叉口"],
        params={
            "oncoming_gap": ParamRange(kind="uniform", low=25.0, high=80.0, unit="m"),
            "oncoming_speed": ParamRange(kind="uniform", low=8.0, high=16.0, unit="m/s"),
            "ego_speed": ParamRange(kind="uniform", low=5.0, high=12.0, unit="m/s"),
        }),
    "junction_light": TemplateSpec(
        name="junction_light", title="信号交叉口冲突",
        description="信号交叉口横向来车冲突（源自 scenario_junction_light）",
        carla_function="scenario_junction_light", tags=["交叉口", "信号灯"],
        params={
            "vehicle_count": ParamRange(kind="int", low=2, high=6),
            "ego_speed": ParamRange(kind="uniform", low=6.0, high=14.0, unit="m/s"),
            "cross_speed": ParamRange(kind="uniform", low=6.0, high=14.0, unit="m/s"),
        }),
    "close_follow": TemplateSpec(
        name="close_follow", title="近距跟驰",
        description="高速接近慢车，TTC 低于阈值时减速跟驰",
        carla_function="scenario_lead_vehicle", tags=["追尾", "TTC"],
        params={
            "ego_speed": ParamRange(kind="uniform", low=14.0, high=22.0, unit="m/s"),
            "lead_speed": ParamRange(kind="uniform", low=3.0, high=9.0, unit="m/s"),
            "gap": ParamRange(kind="uniform", low=25.0, high=50.0, unit="m"),
            "ttc": ParamRange(kind="uniform", low=2.0, high=4.5, unit="s"),
        }),
    "evasive_swerve": TemplateSpec(
        name="evasive_swerve", title="紧急避让",
        description="前方静止障碍，自车按 S 形轨迹换道绕行",
        carla_function="scenario_low_overlap", tags=["避让", "轨迹跟随"],
        params={
            "ego_speed": ParamRange(kind="uniform", low=10.0, high=16.0, unit="m/s"),
            "obstacle_gap": ParamRange(kind="uniform", low=35.0, high=60.0, unit="m"),
            "lateral": ParamRange(kind="uniform", low=3.2, high=4.0, unit="m"),
        }),
    "overtake": TemplateSpec(
        name="overtake", title="超车",
        description="接近慢车后向左变道，相对速度超越后回到原车道",
        carla_function="scenario_cut_in", tags=["超车", "变道"],
        params={
            "ego_speed": ParamRange(kind="uniform", low=12.0, high=18.0, unit="m/s"),
            "lead_speed": ParamRange(kind="uniform", low=5.0, high=10.0, unit="m/s"),
            "gap": ParamRange(kind="uniform", low=30.0, high=55.0, unit="m"),
            "delta": ParamRange(kind="uniform", low=3.0, high=6.0, unit="m/s"),
        }),
}


# =====================================================================
# 公共 API
# =====================================================================
def list_templates() -> List[Dict[str, Any]]:
    out = []
    for name, t in TEMPLATES.items():
        out.append({
            "name": name, "title": t.title, "description": t.description,
            "tags": t.tags, "carla_function": t.carla_function,
            "params": {k: r.model_dump() for k, r in t.params.items()},
            "defaults": t.defaults(),
        })
    return out


def build(template: str, params: Dict[str, Any] = None,
          seed: int = None) -> ScenarioSpec:
    """按模板生成一个场景。

    未提供的参数用模板默认值补齐；填充缺省 params 里的 duration/name 等。
    """
    if template not in _BUILDERS:
        raise KeyError("未知模板 %r，可用: %s"
                       % (template, sorted(_BUILDERS)))
    tpl = TEMPLATES[template]
    full = tpl.defaults()
    full.update(params or {})

    spec = _BUILDERS[template](full)
    spec.seed = seed
    spec.params = {k: v for k, v in full.items() if k in tpl.params}
    spec.template = template
    return spec


def sample(template: str, n: int, seed: int = None,
           rng: random.Random = None) -> List[ScenarioSpec]:
    """对同一模板采样 n 个参数化场景。同 seed 可复现。"""
    if template not in TEMPLATES:
        raise KeyError("未知模板 %r" % template)
    if rng is None:
        rng = random.Random(seed if seed is not None else 0)

    tpl = TEMPLATES[template]
    out = []
    for i in range(n):
        params = tpl.sample_params(rng)
        spec = build(template, params,
                     seed=(seed + i) if seed is not None else None)
        out.append(spec)
    return out
