"""
scenario_validator.py - 双层场景校验器

原项目没有任何校验：generate_openscenario 手拼 XML 后直接落盘，
既不验证结构合法性，也无法确认产物是否符合生成意图。

本模块提供两层校验：

  结构层  ParseOpenScenario 往返解析
          库不打包 ASAM XSD，官方 XSD 直链也不可得，因此以
          "scenariogeneration 能成功解析回来"作为结构合法判据。

  语义层  回读 XML 统计实体/行为/条件，与预期 ScenarioSpec 断言一致
          （防"生成了合法但不符合意图的场景"）
"""
from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional

from scenariogeneration import xosc

from scenario_ir import BehaviorKind, ScenarioSpec, TriggerKind, EntityKind


# =====================================================================
# 结构层
# =====================================================================
def check_structure(path: str) -> Dict[str, Any]:
    """用 scenariogeneration 自己的 parser 回读，失败即结构不合法。"""
    if not os.path.exists(path):
        return {"ok": False, "errors": ["文件不存在: %s" % path]}

    errors: List[str] = []
    try:
        xosc.ParseOpenScenario(path)
    except Exception as e:
        errors.append("解析失败 %s: %s" % (type(e).__name__, e))

    # 基础 XML 合法性
    try:
        root = ET.parse(path).getroot()
    except Exception as e:
        return {"ok": False, "errors": errors + ["XML 解析失败: %s" % e]}

    if root.tag != "OpenSCENARIO":
        errors.append("根元素不是 OpenSCENARIO: %s" % root.tag)

    hdr = root.find("FileHeader")
    if hdr is None:
        errors.append("缺少 FileHeader")
    else:
        for attr in ("revMajor", "revMinor"):
            if not hdr.get(attr):
                errors.append("FileHeader 缺少 %s" % attr)

    if root.find("Entities") is None:
        errors.append("缺少 Entities")
    if root.find("Storyboard") is None:
        errors.append("缺少 Storyboard")

    return {"ok": not errors, "errors": errors}


# =====================================================================
# 语义层
# =====================================================================
def read_stats(path: str) -> Dict[str, Any]:
    """从 .xosc 回读可统计的结构信息。

    真实层次（scenariogeneration 0.16.7 输出）：
        Action > PrivateAction > LongitudinalAction > SpeedAction
        Action > PrivateAction > LateralAction > LaneChangeAction
        Init   > Actions > Private[entityRef] > PrivateAction > ...
    因此统计动作时需要穿透 PrivateAction 的包装元素。
    """
    root = ET.parse(path).getroot()

    entities = root.findall(".//Entities/ScenarioObject")
    names = [e.get("name") for e in entities]

    # 包装层：这些标签只是分类容器，真正的动作在其子元素
    WRAPPERS = {"LongitudinalAction", "LateralAction", "VerticalAction",
                "SynchronizeAction", "TeleportAction", "RoutingAction"}

    def leaf_actions(node):
        """从 PrivateAction 里找出实际动作标签。"""
        out = []
        for child in node:
            if child.tag == "TeleportAction":
                out.append("TeleportAction")
            elif child.tag in WRAPPERS and len(child):
                for deep in child:
                    out.append(deep.tag)
            else:
                out.append(child.tag)
        return out

    # 机动中的动作（含变道、变速等）
    actions: Dict[str, int] = {}
    for pa in root.findall(".//Action/PrivateAction"):
        for tag in leaf_actions(pa):
            actions[tag] = actions.get(tag, 0) + 1

    # 初始化动作
    init_actions = 0
    for pa in root.findall(".//Init/Actions/Private/PrivateAction"):
        init_actions += 1

    # 条件类型：穿透 ByValueCondition / ByEntityCondition
    conds: Dict[str, int] = {}
    for c in root.findall(".//Condition"):
        found = []
        for sub in c:
            if sub.tag in ("ByValueCondition", "ByEntityCondition"):
                for deep in sub:
                    if deep.tag == "TriggeringEntities":
                        continue
                    found.append(deep.tag)
            else:
                found.append(sub.tag)
        for tag in found:
            conds[tag] = conds.get(tag, 0) + 1

    maneuvers = root.findall(".//Maneuver")
    stories = root.findall(".//Story")
    acts = root.findall(".//Act")

    hdr = root.find("FileHeader")
    return {
        "version": "%s.%s" % (hdr.get("revMajor"), hdr.get("revMinor")),
        "entity_count": len(entities),
        "entity_names": names,
        "maneuver_count": len(maneuvers),
        "story_count": len(stories),
        "act_count": len(acts),
        "init_action_count": init_actions,
        "action_types": sorted(actions),
        "action_counts": actions,
        "condition_types": sorted(conds),
        "condition_count": sum(conds.values()),
        "size_bytes": os.path.getsize(path),
    }


# 期望值 → XML 动作标签 的映射（用于断言）
_ACTION_TAG = {
    BehaviorKind.SPEED: "SpeedAction",
    BehaviorKind.LANE_CHANGE: "LaneChangeAction",
    BehaviorKind.LANE_OFFSET: "LaneOffsetAction",
    BehaviorKind.LONGITUDINAL_DISTANCE: "LongitudinalDistanceAction",
    BehaviorKind.LATERAL_DISTANCE: "LateralDistanceAction",
    BehaviorKind.FOLLOW_TRAJECTORY: "FollowTrajectoryAction",
}


def check_semantics(path: str, spec: ScenarioSpec) -> Dict[str, Any]:
    """断言产物与 ScenarioSpec 的意图一致。"""
    errors: List[str] = []
    warnings: List[str] = []
    stats = read_stats(path)

    # 1. 实体数量与名字
    if stats["entity_count"] != len(spec.entities):
        errors.append("实体数不符: 文件 %d, 预期 %d"
                      % (stats["entity_count"], len(spec.entities)))
    expected_names = sorted(e.id for e in spec.entities)
    if sorted(stats["entity_names"]) != expected_names:
        errors.append("实体名不符: 文件 %s, 预期 %s"
                      % (sorted(stats["entity_names"]), expected_names))

    # 2. 行为数量：每个行为产生一个 Maneuver
    if stats["maneuver_count"] != len(spec.behaviors):
        errors.append("行为数不符: 文件 %d, 预期 %d"
                      % (stats["maneuver_count"], len(spec.behaviors)))

    # 3. 每个行为的动作类型是否出现在文件中
    present = set(stats["action_types"])
    for b in spec.behaviors:
        tag = _ACTION_TAG.get(b.kind)
        if tag and tag not in present:
            errors.append("行为 %s(%s) 的动作 %s 未出现在文件中"
                          % (b.resolved_name(), b.kind.value, tag))

    # 4. 初始化动作应覆盖所有实体
    if stats["init_action_count"] < len(spec.entities):
        errors.append("初始化动作不足: %d < 实体数 %d"
                      % (stats["init_action_count"], len(spec.entities)))

    # 5. OSC 版本与预期一致
    want = "1.%d" % spec.osc_minor_version
    if stats["version"] != want:
        errors.append("版本不符: 文件 %s, 预期 %s" % (stats["version"], want))

    # 6. 触发条件至少覆盖行为数
    if stats["condition_count"] < len(spec.behaviors):
        warnings.append("条件数 %d 少于行为数 %d（行为可能共用触发）"
                        % (stats["condition_count"], len(spec.behaviors)))

    return {"ok": not errors, "errors": errors, "warnings": warnings,
            "stats": stats}


# =====================================================================
# 合并入口
# =====================================================================
def validate(path: str, spec: Optional[ScenarioSpec] = None,
             strict: bool = False) -> Dict[str, Any]:
    """双层校验。

    spec 为 None 时只做结构层校验，但仍然回读并返回统计信息
    （否则调用方拿到的 stats 会是 None）。
    strict=True 时把 warning 也视为失败。
    """
    s = check_structure(path)
    out: Dict[str, Any] = {"path": path, "structure": s,
                           "semantics": None, "ok": s["ok"],
                           "errors": list(s["errors"]), "warnings": []}

    # 结构合法时先回读统计（与是否提供 spec 无关）
    stats: Optional[Dict[str, Any]] = None
    if s["ok"]:
        try:
            stats = read_stats(path)
            out["stats"] = stats
        except Exception as e:
            out["errors"].append("回读统计失败: %s" % e)
            out["ok"] = False

    if out["ok"] and spec is not None:
        sem = check_semantics(path, spec)
        out["semantics"] = sem
        out["errors"] += sem["errors"]
        out["warnings"] += sem["warnings"]
        out["ok"] = sem["ok"]
        out["stats"] = sem.get("stats") or stats

    if strict and out["warnings"]:
        out["ok"] = False

    return out


def format_report(r: Dict[str, Any]) -> str:
    if r["ok"]:
        st = r.get("stats") or {}
        return "[OK] %s  OSC=%s 实体=%s 行为=%s 条件=%s %sB" % (
            os.path.basename(r["path"]), st.get("version"),
            st.get("entity_count"), st.get("maneuver_count"),
            st.get("condition_count"), st.get("size_bytes"))
    lines = ["[FAIL] %s" % os.path.basename(r["path"])]
    for e in r["errors"]:
        lines.append("       - %s" % e)
    for w in r["warnings"]:
        lines.append("       ! %s" % w)
    return "\n".join(lines)
