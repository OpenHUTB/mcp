# -*- coding: utf-8 -*-
"""统一的意图召回与打分。

场景模板、快捷按钮文案、视角切换原先各写一套匹配逻辑，口径不一致。
这里提供一套打分器：别名命中 + 示例句相似度 + 组合词加权 + 否定过滤 +
近似匹配兜底，输出 top-k、置信度与处置建议。
"""
from __future__ import annotations

from difflib import SequenceMatcher
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from text_norm import expand, negated, normalize
except ImportError:  # 包内导入
    from .text_norm import expand, negated, normalize  # type: ignore

# 命中别名的基础权重：按长度计，越具体越可信
ALIAS_WEIGHT = 1.2
EXAMPLE_WEIGHT = 1.5
FUZZY_WEIGHT = 0.8
FUZZY_RATIO = 0.75


def _bigrams(text: str) -> set:
    return {text[i:i + 2] for i in range(len(text) - 1)}


def _overlap(a: str, b: str) -> float:
    ba, bb = _bigrams(a), _bigrams(b)
    if not ba or not bb:
        return 0.0
    return len(ba & bb) / float(len(ba))


def _fuzzy_hit(alias: str, text: str) -> bool:
    """别名较长且拼写略有差异时，用最长公共片段兜底。"""
    if len(alias) < 3 or alias in text:
        return False
    match = SequenceMatcher(None, alias, text).find_longest_match(
        0, len(alias), 0, len(text))
    return match.size >= max(3, int(len(alias) * FUZZY_RATIO))


def score_entry(query: str, entry: Dict[str, Any],
                boosts: Sequence[Tuple[Sequence[str], float]] = ()
                ) -> Dict[str, Any]:
    """给单个候选打分。boosts 形如 [((词1, 词2), 权重), ...]。"""
    text = expand(query)
    score = 0.0
    hits: List[str] = []
    ignored: List[str] = []

    for alias in entry.get("aliases", []) + entry.get("keywords", []):
        key = normalize(alias)
        if not key:
            continue
        idx = text.find(key)
        if idx >= 0:
            if negated(text, idx):
                ignored.append(alias)
                continue
            score += len(key) * ALIAS_WEIGHT
            hits.append(alias)
        elif _fuzzy_hit(key, text):
            score += len(key) * FUZZY_WEIGHT
            hits.append(alias + "~")

    for example in entry.get("examples", []):
        score += EXAMPLE_WEIGHT * _overlap(text, normalize(example))

    name = entry.get("name")
    for terms, weight in list(boosts) + list(entry.get("boosts") or ()):
        if all(normalize(t) in text for t in terms):
            score += weight
            hits.extend(terms)

    return {"name": name, "score": round(score, 2), "matched": hits,
            "ignored": ignored}


def route(query: str, catalog: Iterable[Dict[str, Any]], top_k: int = 3,
          boosts: Sequence[Tuple[Sequence[str], float]] = ()
          ) -> List[Dict[str, Any]]:
    """对候选集合打分排序，返回带置信度的 top-k。"""
    rows = []
    for entry in catalog:
        row = score_entry(query, entry, boosts)
        row["confidence"] = round(min(1.0, row["score"] / 10.0), 2)
        rows.append(row)
    rows.sort(key=lambda r: (r["score"], len(r["matched"])), reverse=True)
    return rows[:max(1, int(top_k))]


def decide(rows: Sequence[Dict[str, Any]], ambiguous: bool = False) -> Dict[str, Any]:
    """按置信度给出处置建议：execute / choose / clarify。

    - execute：直接执行，回显理解结果
    - choose：给出 2~3 个候选让用户点选
    - clarify：追问一句，避免落到 others
    """
    top = rows[0] if rows else None
    if not top or top["score"] < 2.5:
        return {"action": "clarify", "top": top,
                "reason": "没有命中的模板或关键词"}
    if ambiguous or top["confidence"] < 0.35:
        return {"action": "choose", "top": top,
                "reason": "存在相近候选，置信度不足"}
    return {"action": "execute", "top": top, "reason": "置信度足够"}


def best_name(query: str, catalog: Iterable[Dict[str, Any]],
              boosts: Sequence[Tuple[Sequence[str], float]] = ()
              ) -> Optional[str]:
    """只取最优候选名（用于快捷文案这类“必须有唯一结果”的入口）。"""
    rows = route(query, catalog, top_k=1, boosts=boosts)
    if rows and rows[0]["score"] > 0:
        return rows[0]["name"]
    return None
