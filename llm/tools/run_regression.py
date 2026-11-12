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
