# -*- coding: utf-8 -*-
"""自然语言 → 场景模板的语义路由层。

MCP 的调用方通常是大模型，本身具备语义理解能力；这一层提供的是
**确定性、可测试的兜底**：把中英文口语描述映射到模板 + 参数，
并明确给出未识别的槽位，方便调用方追问。

设计原则与 ChatScene 一致：可执行性优先。LLM 不生成代码，只产出
{template, params}，真正可执行性由模板库保证。
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

# 每个模板的口语说法与示例。别名越具体，匹配权重越高。
INTENTS: Dict[str, Dict[str, Any]] = {
    "cut_in": {
        "aliases": ["加塞", "强行并线", "强行变道", "突然切入", "别车", "插队",
                    "cut in", "cut-in", "cutin"],
        "examples": ["旁边车道的车突然加塞到我前面", "右侧车辆强行并线逼我刹车"],
    },
    "lead_brake": {
        "aliases": ["前车急刹", "前车突然急刹", "前车急停", "前车突然刹车",
                    "前车猛踩刹车", "突然急刹", "急刹", "急停", "急减速",
                    "追尾风险"],
        "examples": ["高速上前车突然急刹车", "跟车时前车毫无预兆地急停"],
    },
    "lead_stationary": {
        "aliases": ["前车静止", "静止障碍", "抛锚", "故障车", "停着的车",
                    "前方静止车辆", "静止目标", "停在路上"],
        "examples": ["前方车道停着一辆抛锚的车", "夜里前方有个静止的故障车"],
    },
    "crossing_hazard": {
        "aliases": ["横穿", "鬼探头", "行人窜出", "行人突然出现", "电动车横穿",
                    "自行车横穿", "侧向来车", "横穿马路"],
        "examples": ["行人从路边突然窜出横穿马路", "公交车遮挡后一辆电动车横穿过来"],
    },
    "low_overlap": {
        "aliases": ["低重叠", "贴线行驶", "偏置碰撞", "擦边", "部分侵入",
                    "压线行驶", "小重叠"],
        "examples": ["对向车贴着中心线行驶，只有小部分重叠", "目标车压线侵入我的车道"],
    },
    "wrong_way": {
        "aliases": ["逆行", "逆向行驶", "走错方向", "对向车道来车", "迎面来车",
                    "反向行驶"],
        "examples": ["本车道前方出现一辆逆行的车", "对向车道有车逆向朝我开过来"],
    },
    "unprotected_turn": {
        "aliases": ["无保护左转", "无保护转向", "左转抢行", "路口让行",
                    "横穿对向车流", "抢行左转", "路口左转", "左转让行",
                    "对面直行", "对向直行"],
        "examples": ["没有左转灯的路口，我要左转但对面直行车流不断",
                     "路口无保护左转时和对面直行车抢道"],
    },
    "merge_turn": {
        "aliases": ["汇入", "并入口", "右转汇入", "切入车流", "匝道汇入",
                    "并入主路"],
        "examples": ["从匝道汇入主路时切入后车前方", "转弯汇入车流时距离后车太近"],
    },
    "dense_lane_change": {
        "aliases": ["连续变道", "频繁变道", "密集变道", "连续并线", "连变两条",
                    "多次变道"],
        "examples": ["车辆在拥堵路段连续变道", "高速上连续两次向左变道"],
    },
    "junction_light": {
        "aliases": ["信号灯路口", "红绿灯路口", "闯红灯", "路口横向来车",
                    "信号交叉口冲突", "十字路口冲突"],
        "examples": ["信号灯路口横向有车闯红灯冲过来", "十字路口绿灯起步时侧向来车"],
    },
    "close_follow": {
        "aliases": ["近距离跟驰", "跟车太近", "贴尾", "近距离跟车", "车距过小",
                    "跟车过近"],
        "examples": ["高速上跟车距离太近，前车一减速就要追尾",
                     "拥堵时后车贴得很近"],
    },
    "evasive_swerve": {
        "aliases": ["紧急避让", "绕行", "急打方向", "避让静止障碍", "紧急变道避让",
                    "S 形避让", "s形避让"],
        "examples": ["前方突然出现静止障碍，紧急打方向绕行",
                     "为了躲开路上的障碍物猛打方向"],
    },
    "overtake": {
        "aliases": ["超车", "借道超车", "对向超车", "超越慢车", "逆向超车"],
        "examples": ["借对向车道超越前方慢车", "超车时对向有车驶来"],
    },
    "signal_phase": {
        "aliases": ["红绿灯", "信号配时", "绿灯起步", "黄灯", "红灯停车",
                    "信号灯变化"],
        "examples": ["绿灯刚亮时前车起步很慢", "黄灯亮起时该加速还是停车"],
    },
    "cut_in_then_brake": {
        "aliases": ["加塞后急刹", "加塞后急停", "切入后刹车", "别车后急停",
                    "并线后急刹", "加塞又刹车", "切入后急减速", "加塞然后急刹"],
        "examples": ["旁边车加塞到我前面后立刻急刹", "被别车后前车马上踩死刹车"],
    },
}

# 组合规则：多个关键词同时出现时给指定模板加权，避免被单个宽泛别名带偏。
_BOOSTS = [
    (("加塞", "急刹"), "cut_in_then_brake", 8.0),
    (("加塞", "急停"), "cut_in_then_brake", 8.0),
    (("加塞", "刹车"), "cut_in_then_brake", 8.0),
    (("并线", "急刹"), "cut_in_then_brake", 8.0),
    (("前车", "急刹"), "lead_brake", 6.0),
    (("前车", "急停"), "lead_brake", 6.0),
    (("逆行",), "wrong_way", 4.0),
    (("闯红灯",), "junction_light", 6.0),
    (("左转",), "unprotected_turn", 3.0),
    (("汇入",), "merge_turn", 3.0),
]

# 模板依赖的地图（与 TemplateSpec.required_map 保持一致，避免循环导入时拿不到）
_REQUIRED_MAP = {
    "unprotected_turn": "Town10HD_Opt",
    "merge_turn": "Town10HD_Opt",
}

_PUNCT = re.compile(r"[\s，。,.!?！？、；;：:（）()\[\]【】\"'“”‘’《》<>/\\|~`@#$%^&*_+=-]+")
_CN_NUM = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
           "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def _norm(text: str) -> str:
    return _PUNCT.sub("", (text or "").lower())


def _to_num(s: str) -> Optional[float]:
    s = (s or "").strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        pass
    if s in _CN_NUM:
        return float(_CN_NUM[s])
    if len(s) == 2 and s[0] == "十" and s[1] in _CN_NUM:
        return float(10 + _CN_NUM[s[1]])
    if len(s) == 2 and s[1] == "十" and s[0] in _CN_NUM:
        return float(_CN_NUM[s[0]] * 10)
    if len(s) == 3 and s[1] == "十" and s[0] in _CN_NUM and s[2] in _CN_NUM:
        return float(_CN_NUM[s[0]] * 10 + _CN_NUM[s[2]])
    return None


def _bigrams(text: str) -> set:
    return {text[i:i + 2] for i in range(len(text) - 1)}


def _overlap(a: str, b: str) -> float:
    ba, bb = _bigrams(a), _bigrams(b)
    if not ba or not bb:
        return 0.0
    return len(ba & bb) / float(len(ba))


# ---------------------------------------------------------------------------
# 槽位抽取
# ---------------------------------------------------------------------------
_SPEED_KMH = re.compile(r"(\d+(?:\.\d+)?|[零一二两三四五六七八九十]{1,3})\s*"
                        r"(?:km/h|kmh|公里/小时|公里每小时|千米每小时|千米/小时|迈|码)")
_SPEED_MS = re.compile(r"(\d+(?:\.\d+)?|[零一二两三四五六七八九十]{1,3})\s*"
                       r"(?:m/s|米每秒|米/秒)")
_DIST = re.compile(r"(\d+(?:\.\d+)?|[零一二两三四五六七八九十]{1,3})\s*(?:米|m\b|公尺)")
_TIME = re.compile(r"(\d+(?:\.\d+)?|[零一二两三四五六七八九十]{1,3})\s*(?:秒|s\b|sec)")
_COUNT = re.compile(r"(\d+|[零一二两三四五六七八九十]{1,3})\s*(?:辆|台|个)")


def _mask(text: str, spans: List[Tuple[int, int]]) -> str:
    chars = list(text)
    for a, b in spans:
        for i in range(a, b):
            chars[i] = " "
    return "".join(chars)


def parse_slots(text: str) -> Dict[str, Any]:
    """从一句中文/英文描述里抽取可用的数值与枚举槽位。"""
    raw = text or ""
    slots: Dict[str, Any] = {}
    spans: List[Tuple[int, int]] = []

    m = _SPEED_KMH.search(raw)
    if m:
        v = _to_num(m.group(1))
        if v is not None:
            slots["speed"] = round(v / 3.6, 2)
            slots["speed_text"] = m.group(0)
        spans.append(m.span())
    else:
        m = _SPEED_MS.search(raw)
        if m:
            v = _to_num(m.group(1))
            if v is not None:
                slots["speed"] = v
                slots["speed_text"] = m.group(0)
            spans.append(m.span())

    masked = _mask(raw, spans)
    m = _DIST.search(masked)
    if m:
        v = _to_num(m.group(1))
        if v is not None:
            slots["distance"] = v
            slots["distance_text"] = m.group(0)
        spans.append(m.span())

    masked = _mask(raw, spans)
    m = _TIME.search(masked)
    if m:
        v = _to_num(m.group(1))
        if v is not None:
            slots["time"] = v
            slots["time_text"] = m.group(0)
        spans.append(m.span())

    m = _COUNT.search(raw)
    if m:
        v = _to_num(m.group(1))
        if v is not None:
            slots["count"] = int(v)

    n = _norm(raw)
    if "左" in n or "left" in n:
        slots["direction"] = "left"
    elif "右" in n or "right" in n:
        slots["direction"] = "right"
    if "对向" in n or "迎面" in n or "逆行" in n:
        slots["oncoming"] = True
    if "横向" in n or "侧向" in n:
        slots["lateral"] = True

    if "行人" in n or "路人" in n:
        slots["actor"] = "pedestrian"
    elif "自行车" in n or "单车" in n:
        slots["actor"] = "bicycle"
    elif any(k in n for k in ("电动车", "摩托", "车辆", "汽车", "轿车", "货车", "卡车")):
        slots["actor"] = "vehicle"

    return slots


# ---------------------------------------------------------------------------
# 模板匹配
# ---------------------------------------------------------------------------
def _score_one(query: str, name: str) -> Tuple[float, List[str]]:
    info = INTENTS.get(name, {})
    text = _norm(query)
    score = 0.0
    hits: List[str] = []
    for alias in info.get("aliases", []):
        a = _norm(alias)
        if a and a in text:
            score += len(a) * 1.2
            hits.append(alias)
    for ex in info.get("examples", []):
        score += 1.5 * _overlap(text, _norm(ex))
    for kws, target, weight in _BOOSTS:
        if target == name and all(_norm(k) in text for k in kws):
            score += weight
    return score, hits


def find_scenarios(query: str, top_k: int = 3) -> List[Dict[str, Any]]:
    """按口语描述给模板打分排序，返回候选列表。"""
    rows = []
    for name in INTENTS:
        score, hits = _score_one(query, name)
        rows.append({"name": name, "score": round(score, 2),
                     "confidence": round(min(1.0, score / 10.0), 2),
                     "matched": hits})
    rows.sort(key=lambda r: r["score"], reverse=True)
    return [r for r in rows[:max(1, int(top_k))]]


# ---------------------------------------------------------------------------
# 参数填充
# ---------------------------------------------------------------------------
def _clamp(value: float, pr: Any) -> Any:
    if pr.kind in ("uniform", "int"):
        lo = pr.low if pr.low is not None else value
        hi = pr.high if pr.high is not None else value
        value = max(lo, min(hi, value))
        if pr.kind == "int":
            value = int(round(value))
        return value
    if pr.kind == "fixed":
        return pr.value
    return value


def fill_params(tpl: Any, slots: Dict[str, Any]) -> Dict[str, Any]:
    """把槽位映射到模板参数，并按参数范围夹紧。"""
    out: Dict[str, Any] = {}
    for name, pr in tpl.params.items():
        low = name.lower()
        if "speed" in low and "speed" in slots:
            out[name] = _clamp(slots["speed"], pr)
        elif low == "direction" and "direction" in slots:
            choices = pr.choices or []
            if slots["direction"] in choices:
                out[name] = slots["direction"]
        elif low == "crosser" and "actor" in slots:
            choices = pr.choices or []
            if slots["actor"] in choices:
                out[name] = slots["actor"]
        elif ("gap" in low or "distance" in low) and "distance" in slots:
            out[name] = _clamp(slots["distance"], pr)
        elif ("delay" in low or low == "ttc" or "period" in low) and "time" in slots:
            out[name] = _clamp(slots["time"], pr)
        elif ("count" in low or low == "vehicle_count") and "count" in slots:
            out[name] = _clamp(slots["count"], pr)
    return out


def plan_scenario(description: str) -> Dict[str, Any]:
    """把一句自然语言规划成 {template, params}，必要时给出追问。"""
    slots = parse_slots(description)
    ranked = find_scenarios(description, top_k=3)
    top = ranked[0] if ranked else None
    second = ranked[1] if len(ranked) > 1 else None

    if not top or (top["score"] < 2.5 and not top["matched"]):
        return {
            "matched": False, "needs_clarification": True,
            "query": description, "slots": slots,
            "question": "没识别出具体场景类型。能补充是前车急刹、邻道加塞、"
                        "横穿、逆行，还是路口冲突吗？",
            "alternatives": [r["name"] for r in ranked if r["score"] > 0],
        }

    confidence = top["confidence"]
    ratio = 0.0
    if second and top["score"] > 0:
        ratio = second["score"] / top["score"]
    ambiguous = bool(second and ratio > 0.75 and top["score"] < 6.0)
    if ambiguous:
        confidence *= 0.7

    try:
        from scenario_templates import TEMPLATES
        tpl = TEMPLATES[top["name"]]
        params = fill_params(tpl, slots)
        title = tpl.title
        required_map = tpl.required_map
        param_space = {k: {"kind": v.kind, "low": v.low, "high": v.high,
                           "unit": v.unit, "choices": v.choices}
                       for k, v in tpl.params.items()}
    except Exception:
        params, title, required_map, param_space = {}, top["name"], None, {}

    question = None
    if ambiguous:
        question = ("更像「%s」还是「%s」？可以补充一句更具体的行为，例如谁先动、"
                    "距离多远。" % (top["name"], second["name"]))

    return {
        "matched": True,
        "needs_clarification": ambiguous,
        "query": description,
        "template": top["name"],
        "title": title,
        "confidence": round(confidence, 2),
        "matched_terms": top["matched"],
        "params": params,
        "param_space": param_space,
        "required_map": required_map or _REQUIRED_MAP.get(top["name"]),
        "slots": slots,
        "question": question,
        "alternatives": [{"template": r["name"], "score": r["score"]}
                         for r in ranked[1:]],
    }
