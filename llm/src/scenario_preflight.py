# -*- coding: utf-8 -*-
"""执行前的场景参数校验。

此前参数写错（拼写、越界、类型不符）会一路传到 CARLA 才暴露，
出图看起来“识别对了但画面是错的”。这里在生成/播放前把问题挡下来，
并给出可用取值，便于调用方或用户自行修正。
"""
from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional


def _templates(templates: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if templates is not None:
        return templates
    try:
        from scenario_templates import TEMPLATES
    except ImportError:  # 包内导入
        from .scenario_templates import TEMPLATES  # type: ignore
    return TEMPLATES


def _issue(code: str, message: str, suggestion: str = "") -> Dict[str, str]:
    return {"code": code, "message": message, "suggestion": suggestion}


def check_params(template: str, params: Dict[str, Any],
                 templates: Optional[Dict[str, Any]] = None) -> List[Dict[str, str]]:
    """校验参数名与取值是否落在模板声明的范围内。"""
    table = _templates(templates)
    tpl = table.get(template)
    if tpl is None:
        return [_issue("unknown_template", "未知模板 `%s`" % template,
                       "可用模板：%s" % ", ".join(sorted(table)))]

    issues: List[Dict[str, str]] = []
    for key, value in (params or {}).items():
        pr = tpl.params.get(key)
        if pr is None:
            issues.append(_issue(
                "unknown_param", "模板 `%s` 没有参数 `%s`" % (template, key),
                "可用参数：%s" % ", ".join(sorted(tpl.params))))
            continue
        if pr.kind == "fixed":
            if pr.value is not None and value != pr.value:
                issues.append(_issue("fixed_param",
                                     "参数 `%s` 固定为 %r" % (key, pr.value),
                                     "请改为 %r" % (pr.value,)))
            continue
        if pr.choices:
            if value not in pr.choices:
                issues.append(_issue("choice_out_of_range",
                                     "参数 `%s` 取值 %r 不在可选范围内" % (key, value),
                                     "可选：%s" % ", ".join(map(str, pr.choices))))
            continue
        try:
            num = float(value)
        except (TypeError, ValueError):
            issues.append(_issue("not_numeric",
                                 "参数 `%s` 需要数值，收到 %r" % (key, value),
                                 "示例：%s" % (pr.low if pr.low is not None else 0)))
            continue
        lo, hi = pr.low, pr.high
        if (lo is not None and num < lo) or (hi is not None and num > hi):
            issues.append(_issue("out_of_range",
                                 "参数 `%s`=%s 超出范围 [%s, %s]"
                                 % (key, value, lo, hi),
                                 "建议取值 %.2f ~ %.2f%s"
                                 % (lo if lo is not None else num,
                                    hi if hi is not None else num,
                                    " " + pr.unit if pr.unit else "")))
    return issues


def check_map(template: str, map_name: str,
              templates: Optional[Dict[str, Any]] = None) -> List[Dict[str, str]]:
    """模板依赖特定地图时，提前给出明确提示。"""
    tpl = _templates(templates).get(template)
    need = getattr(tpl, "required_map", None) if tpl is not None else None
    if not need:
        return []
    current = (map_name or "").rsplit("/", 1)[-1]
    if current.lower() == need.lower():
        return []
    return [_issue("map_mismatch",
                   "模板 `%s` 使用真实路口几何，必须在 `%s` 地图上运行"
                   % (template, need),
                   "请传 map_name=\"%s\"（服务端已在该图时直接复用，不会切图）"
                   % need)]


def format_issues(issues: Iterable[Dict[str, str]]) -> str:
    lines = []
    for item in issues:
        line = "❌ " + item["message"]
        if item.get("suggestion"):
            line += "\n   建议：" + item["suggestion"]
        lines.append(line)
    return "\n".join(lines)
