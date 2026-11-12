"""场景工具链回归脚本。

默认做"离线回归"：把全部模板编译成 .xosc，校验结构、回读播放器解析结果，
并断言 unsupported 为空。加 --carla 时再连 CARLA 逐个播放（需要 CarlaUE4 已启动）。

用法：
    python llm/tools/run_regression.py
    python llm/tools/run_regression.py --carla --duration 6
    python llm/tools/run_regression.py --carla --templates lead_brake,signal_phase
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", "src"))
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from scenario_ir import (BehaviorKind, BehaviorSpec, EntitySpec, PositionSpec,
                         ScenarioSpec, TriggerKind, TriggerSpec)
from scenario_templates import TEMPLATES, build
from osc_builder import validate_roundtrip, write_scenario
from xosc_player import parse_xosc

# 红绿灯模板依赖地图自带信号灯，自建 xodr 路网里没有，
# 实跑时改用 run_traffic_light_demo（内置地图）验证。
SKIP_ON_XODR = {"signal_phase"}


# 手工构造的 OSC：覆盖条件组 AND/OR、Condition@delay、参数声明、
# ManeuverGroup 多 Actor、Sequence 嵌套与 RelativeTargetSpeed(factor)。
CONFORMANCE_XOSC = """<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="1" date="2026-11-12"
              description="player conformance" author="regression"/>
  <ParameterDeclarations>
    <ParameterDeclaration name="brake_target" parameterType="double" value="3.0"/>
  </ParameterDeclarations>
  <Entities>
    <ScenarioObject name="ego"><Vehicle name="v1" vehicleCategory="car"/></ScenarioObject>
    <ScenarioObject name="lead"><Vehicle name="v2" vehicleCategory="car"/></ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <Private entityRef="ego">
          <PrivateAction><TeleportAction><Position>
            <LanePosition roadId="1" laneId="-1" s="10" offset="0"/>
          </Position></TeleportAction></PrivateAction>
          <PrivateAction><LongitudinalAction><SpeedAction>
            <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
            <SpeedActionTarget><AbsoluteTargetSpeed value="10"/></SpeedActionTarget>
          </SpeedAction></LongitudinalAction></PrivateAction>
        </Private>
        <Private entityRef="lead">
          <PrivateAction><TeleportAction><Position>
            <LanePosition roadId="1" laneId="-1" s="40" offset="0"/>
          </Position></TeleportAction></PrivateAction>
          <PrivateAction><LongitudinalAction><SpeedAction>
            <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
            <SpeedActionTarget><AbsoluteTargetSpeed value="8"/></SpeedActionTarget>
          </SpeedAction></LongitudinalAction></PrivateAction>
        </Private>
      </Actions>
    </Init>
    <Story name="story"><Act name="act"><Sequence name="seq" numberOfExecutions="1">
      <ManeuverGroup name="mg" maximumExecutionCount="1">
        <Actors selectTriggeringEntities="false">
          <EntityRef entityRef="ego"/>
          <EntityRef entityRef="lead"/>
        </Actors>
        <Maneuver name="m">
          <Event name="follow_lead" priority="overwrite">
            <Action><PrivateAction><LongitudinalAction><SpeedAction>
              <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
              <SpeedActionTarget>
                <RelativeTargetSpeed entityRef="lead" value="1.2"
                                     speedTargetValueType="factor"/>
              </SpeedActionTarget>
            </SpeedAction></LongitudinalAction></PrivateAction></Action>
            <StartTrigger>
              <ConditionGroup>
                <Condition name="c_speed" delay="2" conditionEdge="rising">
                  <ByEntityCondition>
                    <TriggeringEntities triggeringEntitiesRule="any">
                      <EntityRef entityRef="lead"/>
                    </TriggeringEntities>
                    <EntityCondition>
                      <SpeedCondition value="5" rule="lessThan" entityRef="lead"/>
                    </EntityCondition>
                  </ByEntityCondition>
                </Condition>
                <Condition name="c_still" conditionEdge="rising">
                  <ByEntityCondition>
                    <TriggeringEntities triggeringEntitiesRule="any">
                      <EntityRef entityRef="lead"/>
                    </TriggeringEntities>
                    <EntityCondition><StandStillCondition duration="1.5"/></EntityCondition>
                  </ByEntityCondition>
                </Condition>
              </ConditionGroup>
              <ConditionGroup>
                <Condition name="c_traveled" conditionEdge="rising">
                  <ByEntityCondition>
                    <TriggeringEntities triggeringEntitiesRule="any">
                      <EntityRef entityRef="ego"/>
                    </TriggeringEntities>
                    <EntityCondition><TraveledDistanceCondition value="20"/></EntityCondition>
                  </ByEntityCondition>
                </Condition>
              </ConditionGroup>
            </StartTrigger>
          </Event>
          <Event name="ramp_up" priority="overwrite">
            <Action><PrivateAction><LongitudinalAction><SpeedAction>
              <SpeedActionDynamics dynamicsShape="cubic" value="2" dynamicsDimension="time"/>
              <SpeedActionTarget><AbsoluteTargetSpeed value="12"/></SpeedActionTarget>
            </SpeedAction></LongitudinalAction></PrivateAction></Action>
            <StartTrigger>
              <ConditionGroup>
                <Condition name="c_up" conditionEdge="rising">
                  <ByValueCondition><SimulationTimeCondition value="8" rule="greaterThan"/></ByValueCondition>
                </Condition>
              </ConditionGroup>
            </StartTrigger>
          </Event>
          <Event name="follow_limited" priority="overwrite">
            <Action><PrivateAction><LongitudinalAction>
              <LongitudinalDistanceAction entityRef="lead" distance="12"
                                          maxAcceleration="1.5" maxDeceleration="4"/>
            </LongitudinalAction></PrivateAction></Action>
            <StartTrigger>
              <ConditionGroup>
                <Condition name="c_follow" conditionEdge="rising">
                  <ByValueCondition><SimulationTimeCondition value="10" rule="greaterThan"/></ByValueCondition>
                </Condition>
              </ConditionGroup>
            </StartTrigger>
          </Event>
          <Event name="slow_down" priority="overwrite">
            <Action><PrivateAction><LongitudinalAction><SpeedAction>
              <SpeedActionDynamics dynamicsShape="linear" value="1" dynamicsDimension="rate"/>
              <SpeedActionTarget>
                <AbsoluteTargetSpeed value="$brake_target"/>
              </SpeedActionTarget>
            </SpeedAction></LongitudinalAction></PrivateAction></Action>
            <StartTrigger>
              <ConditionGroup>
                <Condition name="c_time" conditionEdge="rising">
                  <ByValueCondition><SimulationTimeCondition value="6" rule="greaterThan"/></ByValueCondition>
                </Condition>
              </ConditionGroup>
            </StartTrigger>
          </Event>
        </Maneuver>
      </ManeuverGroup>
    </Sequence></Act></Story>
    <StopTrigger>
      <ConditionGroup>
        <Condition name="stop_time" conditionEdge="rising">
          <ByValueCondition><SimulationTimeCondition value="20" rule="greaterThan"/></ByValueCondition>
        </Condition>
      </ConditionGroup>
      <ConditionGroup>
        <Condition name="stop_still" conditionEdge="rising">
          <ByEntityCondition>
            <TriggeringEntities triggeringEntitiesRule="any">
              <EntityRef entityRef="ego"/>
            </TriggeringEntities>
            <EntityCondition><StandStillCondition duration="1"/></EntityCondition>
          </ByEntityCondition>
        </Condition>
      </ConditionGroup>
    </StopTrigger>
  </Storyboard>
</OpenSCENARIO>
"""


# StopTrigger 提前结束的实跑用例：1s 后刹停，静止满 1s 即结束（计划 8s）
STOP_TRIGGER_XOSC = """<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="1" date="2026-11-12"
              description="stop trigger demo" author="regression"/>
  <Entities>
    <ScenarioObject name="ego"><Vehicle name="v1" vehicleCategory="car"/></ScenarioObject>
  </Entities>
  <Storyboard>
    <Init><Actions><Private entityRef="ego">
      <PrivateAction><TeleportAction><Position>
        <LanePosition roadId="1" laneId="-1" s="10" offset="0"/>
      </Position></TeleportAction></PrivateAction>
      <PrivateAction><LongitudinalAction><SpeedAction>
        <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
        <SpeedActionTarget><AbsoluteTargetSpeed value="8"/></SpeedActionTarget>
      </SpeedAction></LongitudinalAction></PrivateAction>
    </Private></Actions></Init>
    <Story name="s"><Act name="a"><ManeuverGroup name="mg" maximumExecutionCount="1">
      <Actors selectTriggeringEntities="false">
        <EntityRef entityRef="ego"/>
      </Actors>
      <Maneuver name="m">
        <Event name="brake" priority="overwrite">
          <Action><PrivateAction><LongitudinalAction><SpeedAction>
            <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
            <SpeedActionTarget><AbsoluteTargetSpeed value="0"/></SpeedActionTarget>
          </SpeedAction></LongitudinalAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup>
            <Condition name="c_brake" conditionEdge="rising">
              <ByValueCondition><SimulationTimeCondition value="1" rule="greaterThan"/></ByValueCondition>
            </Condition>
          </ConditionGroup></StartTrigger>
        </Event>
      </Maneuver>
    </ManeuverGroup></Act></Story>
    <StopTrigger><ConditionGroup>
      <Condition name="c_still" conditionEdge="rising">
        <ByEntityCondition>
          <TriggeringEntities triggeringEntitiesRule="any">
            <EntityRef entityRef="ego"/>
          </TriggeringEntities>
          <EntityCondition><StandStillCondition duration="1"/></EntityCondition>
        </ByEntityCondition>
      </Condition>
    </ConditionGroup></StopTrigger>
  </Storyboard>
</OpenSCENARIO>
"""


def _synthetic_spec() -> ScenarioSpec:
    """覆盖全部新增条件的合成场景，确保新功能不回退。"""
    entities = [
        EntitySpec(id="ego", initial_position=PositionSpec.lane(50.0),
                   initial_speed=10.0),
        EntitySpec(id="lead", initial_position=PositionSpec.lane(70.0),
                   initial_speed=8.0),
    ]
    triggers = [
        TriggerSpec(kind=TriggerKind.ACCELERATION, value=2.0),
        TriggerSpec(kind=TriggerKind.OFFROAD, value=0.5),
        TriggerSpec(kind=TriggerKind.END_OF_ROAD, value=0.5),
        TriggerSpec(kind=TriggerKind.TRAVELED_DISTANCE, value=30.0),
        TriggerSpec(kind=TriggerKind.RELATIVE_DISTANCE, entity_ref="lead",
                    value=15.0),
        TriggerSpec(kind=TriggerKind.RELATIVE_ANGLE, entity_ref="lead",
                    value=10.0, tolerance=2.0),
        TriggerSpec(kind=TriggerKind.TRAFFIC_SIGNAL, signal_phase="red"),
    ]
    behaviors = [
        BehaviorSpec(kind=BehaviorKind.SPEED, entity_ref="ego", target_speed=5.0)
        for _ in triggers
    ]
    return ScenarioSpec(name="new_conditions", entities=entities,
                        behaviors=behaviors, triggers=triggers, duration=20.0)


def offline_checks(out_dir: str) -> int:
    failures = 0
    names = sorted(TEMPLATES)
    print("== 离线回归：%d 个模板 + 新增条件 ==\n" % len(names))
    for name in names:
        path = os.path.join(out_dir, "%s.xosc" % name)
        try:
            spec = build(name, seed=1)
            write_scenario(spec, path)
            v = validate_roundtrip(path)
            if not v.get("valid"):
                print("❌ %-18s 结构校验失败: %s" % (name, v.get("error")))
                failures += 1
                continue
            sc = parse_xosc(path)
            if sc.unsupported:
                print("❌ %-18s 有未执行项: %s" % (name, sc.unsupported[:3]))
                failures += 1
                continue
            if not sc.events:
                print("❌ %-18s 没有可识别的事件" % name)
                failures += 1
                continue
            print("✅ %-18s OSC %s  实体 %d  事件 %d  未执行 0"
                  % (name, v.get("version"), len(sc.entities), len(sc.events)))
        except Exception as e:
            print("❌ %-18s 异常 %s: %s" % (name, type(e).__name__, e))
            traceback.print_exc()
            failures += 1

    # 新增条件合成场景
    try:
        path = os.path.join(out_dir, "new_conditions.xosc")
        write_scenario(_synthetic_spec(), path)
        v = validate_roundtrip(path)
        sc = parse_xosc(path)
        kinds = {ev.trigger.kind for ev in sc.events}
        expect = {"acceleration", "offroad", "end_of_road", "traveled_distance",
                  "relative_distance", "relative_angle"}
        missing = expect - kinds
        if not v.get("valid") or sc.unsupported or missing:
            print("❌ 新增条件覆盖不全: valid=%s unsupported=%s missing=%s"
                  % (v.get("valid"), sc.unsupported, sorted(missing)))
            failures += 1
        else:
            print("✅ %-18s OSC %s  新增条件 %d 种全部可解析"
                  % ("new_conditions", v.get("version"), len(expect)))
    except Exception as e:
        print("❌ 新增条件场景异常 %s: %s" % (type(e).__name__, e))
        failures += 1

    # 信号灯动作 + 条件
    try:
        spec = build("signal_phase", seed=1)
        path = os.path.join(out_dir, "signal_phase.xosc")
        write_scenario(spec, path)
        sc = parse_xosc(path)
        has_cond = any(ev.trigger.kind == "traffic_signal" for ev in sc.events)
        has_act = any(a.kind == "traffic_light"
                      for ev in sc.events for a in ev.actions)
        if not (has_cond and has_act):
            print("❌ signal_phase 缺少信号灯条件/动作: cond=%s act=%s"
                  % (has_cond, has_act))
            failures += 1
        else:
            print("✅ %-18s 信号灯条件 + 信号灯动作均可解析" % "signal_phase")
    except Exception as e:
        print("❌ signal_phase 异常 %s: %s" % (type(e).__name__, e))
        failures += 1

    # OSC 语义一致性（参照 esmini）：条件组 AND/OR、delay、参数、多 Actor、相对速度
    try:
        path = os.path.join(out_dir, "conformance.xosc")
        with open(path, "w", encoding="utf-8") as f:
            f.write(CONFORMANCE_XOSC)
        sc = parse_xosc(path)
        problems = []
        evs = {e.name: e for e in sc.events}
        expect_names = {"follow_lead[ego]", "follow_lead[lead]",
                        "slow_down[ego]", "slow_down[lead]",
                        "ramp_up[ego]", "ramp_up[lead]",
                        "follow_limited[ego]", "follow_limited[lead]"}
        if set(evs) != expect_names:
            problems.append("多 Actor 未展开: %s" % sorted(evs))

        fl = evs.get("follow_lead[ego]")
        if fl is None:
            problems.append("缺少 follow_lead[ego]")
        else:
            if fl.trigger.kind != "any" or len(fl.trigger.children) != 2:
                problems.append("ConditionGroup 未按 OR 组合: kind=%s groups=%d"
                                % (fl.trigger.kind, len(fl.trigger.children)))
            leaves = fl.trigger.leaves()
            kinds = {c.kind for c in leaves}
            if kinds != {"speed", "standstill", "traveled_distance"}:
                problems.append("组内 AND 叶子异常: %s" % sorted(kinds))
            sp = next((c for c in leaves if c.kind == "speed"), None)
            if sp is None or abs(sp.delay - 2.0) > 1e-6:
                problems.append("Condition@delay 未解析")
            if sp is None or sp.triggering_entity != "lead":
                problems.append("TriggeringEntities 未生效")
            if not fl.actions or fl.actions[0].relative_mode != "factor":
                problems.append("RelativeTargetSpeed factor 未识别")

        sd = evs.get("slow_down[ego]")
        if sd is None or not sd.actions or abs(sd.actions[0].value - 3.0) > 1e-6:
            problems.append("参数声明 $brake_target 未替换")
        if sd is None or not sd.actions or sd.actions[0].dynamics_dimension != "rate":
            problems.append("SpeedActionDynamics rate 未解析")

        ru = evs.get("ramp_up[ego]")
        if ru is None or not ru.actions or ru.actions[0].shape != "cubic":
            problems.append("cubic 速度曲线未解析")

        fo = evs.get("follow_limited[ego]")
        if (fo is None or not fo.actions
                or abs(fo.actions[0].max_accel - 1.5) > 1e-6
                or abs(fo.actions[0].max_decel - 4.0) > 1e-6):
            problems.append("跟驰加减速上限未解析")

        if sc.stop_trigger is None or sc.stop_trigger.kind != "any":
            problems.append("StopTrigger 条件组未解析")
        else:
            kinds_stop = {c.kind for c in sc.stop_trigger.leaves()}
            if kinds_stop != {"time", "standstill"}:
                problems.append("StopTrigger 条件异常: %s" % sorted(kinds_stop))
        if abs(sc.stop_time - 20.0) > 1e-6:
            problems.append("StopTrigger 时间异常: %.1f" % sc.stop_time)
        if sc.unsupported:
            problems.append("有未执行项: %s" % sc.unsupported[:2])

        if problems:
            print("❌ %-18s %s" % ("conformance", "；".join(problems)))
            failures += 1
        else:
            print("✅ %-18s 条件组 AND/OR、delay、参数、多 Actor、相对速度、"
                  "StopTrigger、速度曲线与跟驰上限全部生效" % "conformance")
    except Exception as e:
        print("❌ %-18s 异常 %s: %s" % ("conformance", type(e).__name__, e))
        traceback.print_exc()
        failures += 1

    return failures


def carla_checks(names, duration: float, host: str, port: int) -> int:
    import carla  # noqa: F401
    import carla_runner as CR
    from road_builder import write
    from xosc_player import play

    failures = 0
    out = os.path.join(tempfile.gettempdir(), "xosc_regression")
    os.makedirs(out, exist_ok=True)
    xodr_path = os.path.join(out, "straight.xodr")
    write("straight", xodr_path)

    client = CR.connect(host, port)
    world = CR.load_xodr_world(client, xodr_path)
    CR.clear_actors(world)

    for name in names:
        if name in SKIP_ON_XODR:
            print("⏭ %-18s 需要带红绿灯的内置地图，改用 run_traffic_light_demo 验证"
                  % name)
            continue
        path = os.path.join(out, "%s.xosc" % name)
        try:
            write_scenario(build(name, seed=1), path)
            sc = parse_xosc(path)
            res = play(world, sc, duration=duration, capture_at=[],
                       out_dir="", verbose=False)
            ok = (res["spawned"] == len(sc.entities)
                  and not res["unsupported"] and not res["errors"])
            flag = "✅" if ok else "❌"
            print("%s %-18s 布景 %d/%d  触发 %d/%d  错误 %d"
                  % (flag, name, res["spawned"], len(sc.entities),
                     res["events_fired"], res["events_total"],
                     len(res["errors"])))
            if not ok:
                failures += 1
                for e in res["errors"][:3]:
                    print("     - %s" % e)
        except Exception as e:
            print("❌ %-18s 异常 %s: %s" % (name, type(e).__name__, e))
            failures += 1
        CR.clear_actors(world)

    # StopTrigger 提前结束
    try:
        p = os.path.join(out, "stop_trigger.xosc")
        with open(p, "w", encoding="utf-8") as f:
            f.write(STOP_TRIGGER_XOSC)
        sc = parse_xosc(p)
        res = play(world, sc, duration=8.0, capture_at=[],
                   out_dir="", verbose=False)
        ended = res.get("ended_at", -1)
        ok = res.get("stop_reason") == "stop_trigger" and 0 < ended < 6.0
        print("%s %-18s 实际 %.2fs（计划 8.0s）  stop_reason=%s"
              % ("✅" if ok else "❌", "stop_trigger", ended,
                 res.get("stop_reason")))
        if not ok:
            failures += 1
            for e in res.get("errors", [])[:3]:
                print("     - %s" % e)
    except Exception as e:
        print("❌ %-18s 异常 %s: %s" % ("stop_trigger", type(e).__name__, e))
        failures += 1
    CR.clear_actors(world)

    return failures


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--carla", action="store_true", help="连 CARLA 实跑")
    ap.add_argument("--duration", type=float, default=6.0,
                    help="实跑时长（秒）")
    ap.add_argument("--templates", default="",
                    help="只跑指定模板，逗号分隔")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    out_dir = os.path.join(HERE, "..", "output", "regression")
    os.makedirs(out_dir, exist_ok=True)

    failures = offline_checks(out_dir)
    if args.carla:
        names = ([n.strip() for n in args.templates.split(",") if n.strip()]
                 or sorted(TEMPLATES))
        print("\n== CARLA 实跑：%d 个模板，每个 %.1fs ==" % (len(names),
                                                          args.duration))
        failures += carla_checks(names, args.duration, args.host, args.port)

    print("\n%s" % ("✅ 全部通过" if failures == 0 else "❌ 失败 %d 项" % failures))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
