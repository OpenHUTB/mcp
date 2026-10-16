"""
scenario_batch.py - 批量场景生成器

把「一个模板 → 一个场景」扩展为「一个模板/多个模板 → 一批场景 + 可追溯清单」。

原项目的 generate_sumo_network / generate_openscenario 每次只能产出一个结果，
且返回值是给 LLM 看的字符串，无法被程序二次消费 —— 因此无法批量、无法追踪、
无法复现。本模块解决这一点：

    output/scenarios/<batch_id>/
        <scenario_name>.xosc     × N
        manifest.json            每个场景的参数、seed、指纹、校验结果
        summary.json             批量统计（模板分布、参数分布、耗时）
"""
from __future__ import annotations

import json
import os
import platform
import random
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence

from scenario_ir import ScenarioSpec
from scenario_templates import TEMPLATES, build, sample
from osc_builder import write_scenario
from scenario_validator import validate, format_report


def _batch_id(template: str, seed: Optional[int]) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = "seed%s" % seed if seed is not None else "noseed"
    return "%s_%s_%s" % (template, ts, tag)


def generate_batch(
    template: str,
    n: int,
    out_root: str,
    seed: Optional[int] = None,
    batch_id: Optional[str] = None,
    validate_each: bool = True,
    strict: bool = False,
    verbose: bool = True,
) -> Dict[str, Any]:
    """按模板批量生成 n 个场景。

    返回 manifest dict（同时已写入磁盘）。
    """
    if template not in TEMPLATES:
        raise KeyError("未知模板 %r，可用: %s" % (template, sorted(TEMPLATES)))
    if n <= 0:
        raise ValueError("n 必须为正整数")

    bid = batch_id or _batch_id(template, seed)
    out_dir = os.path.join(out_root, bid)
    os.makedirs(out_dir, exist_ok=True)

    if verbose:
        print("[batch] 模板=%s n=%d seed=%s" % (template, n, seed))
        print("[batch] 输出: %s" % out_dir)

    t0 = time.time()
    rng = random.Random(seed if seed is not None else 0)
    tpl = TEMPLATES[template]

    records: List[Dict[str, Any]] = []
    used_names: Dict[str, int] = {}
    ok = failed = 0

    for i in range(n):
        # 1. 采样参数（seed 决定，可复现）
        params = tpl.sample_params(rng)
        spec = build(template, params,
                     seed=(seed + i) if seed is not None else None)

        # 2. 文件名去重
        base = spec.name or template
        used_names[base] = used_names.get(base, 0) + 1
        idx = used_names[base]
        fname = "%s_%03d.xosc" % (base, idx)
        path = os.path.join(out_dir, fname)

        rec: Dict[str, Any] = {
            "index": i,
            "file": fname,
            "name": spec.name,
            "template": template,
            "seed": spec.seed,
            "fingerprint": spec.fingerprint(),
            "params": {k: (round(v, 6) if isinstance(v, float) else v)
                       for k, v in params.items()},
            "stats": spec.stats(),
            "valid": None,
            "errors": [],
        }

        # 3. 编译
        try:
            write_scenario(spec, path)
        except Exception as e:
            rec["valid"] = False
            rec["errors"] = ["编译失败 %s: %s" % (type(e).__name__, e)]
            failed += 1
            records.append(rec)
            if verbose:
                print("  [%3d/%d] FAIL %s" % (i + 1, n, rec["errors"][0]))
            continue

        # 4. 校验
        if validate_each:
            r = validate(path, spec, strict=strict)
            rec["valid"] = r["ok"]
            rec["errors"] = r["errors"]
            if r.get("stats"):
                rec["file_stats"] = r["stats"]
            if r["ok"]:
                ok += 1
            else:
                failed += 1
            if verbose and (not r["ok"] or (i + 1) % 10 == 0 or i == 0):
                print("  [%3d/%d] %s" % (i + 1, n, format_report(r)))
        else:
            rec["valid"] = None
            ok += 1
            if verbose and ((i + 1) % 10 == 0 or i == 0):
                print("  [%3d/%d] 已生成 %s" % (i + 1, n, fname))

        records.append(rec)

    elapsed = time.time() - t0

    # 5. 指纹去重统计
    fps = [r["fingerprint"] for r in records if r.get("fingerprint")]
    dup = len(fps) - len(set(fps))

    manifest = {
        "batch_id": bid,
        "template": template,
        "template_title": TEMPLATES[template].title,
        "count": n,
        "seed": seed,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "out_dir": os.path.abspath(out_dir),
        "generated": len(records),
        "valid": ok,
        "failed": failed,
        "duplicate_fingerprints": dup,
        "elapsed_seconds": round(elapsed, 3),
        "per_scenario_seconds": round(elapsed / max(n, 1), 4),
        "scenarios": records,
    }

    # 6. summary
    param_values: Dict[str, List[Any]] = {}
    for r in records:
        for k, v in (r.get("params") or {}).items():
            if isinstance(v, (int, float)):
                param_values.setdefault(k, []).append(v)

    def _stat(vals):
        if not vals:
            return None
        return {"min": min(vals), "max": max(vals),
                "mean": round(sum(vals) / len(vals), 4), "n": len(vals)}

    action_types, condition_types = set(), set()
    for r in records:
        fs = r.get("file_stats") or {}
        action_types |= set(fs.get("action_types") or [])
        condition_types |= set(fs.get("condition_types") or [])

    summary = {
        "batch_id": bid,
        "template": template,
        "count": n,
        "valid": ok,
        "failed": failed,
        "duplicate_fingerprints": dup,
        "elapsed_seconds": round(elapsed, 3),
        "param_distribution": {k: _stat(v) for k, v in param_values.items()},
        "action_types": sorted(action_types),
        "condition_types": sorted(condition_types),
        "template_params": {k: r.model_dump() for k, r in
                            TEMPLATES[template].params.items()},
    }

    mpath = os.path.join(out_dir, "manifest.json")
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    spath = os.path.join(out_dir, "summary.json")
    with open(spath, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    manifest["manifest_path"] = mpath
    manifest["summary_path"] = spath

    if verbose:
        print("[batch] 完成: 成功 %d / 失败 %d, 耗时 %.2fs (%.0f 个/秒)"
              % (ok, failed, elapsed, n / elapsed if elapsed else 0))
        if dup:
            print("[batch] 警告: %d 个场景指纹重复" % dup)
        print("[batch] manifest: %s" % mpath)

    return manifest


def generate_mixed(
    plan: Dict[str, int],
    out_root: str,
    seed: Optional[int] = None,
    batch_id: Optional[str] = None,
    verbose: bool = True,
    **kw,
) -> Dict[str, Any]:
    """多模板混合批量生成。plan 形如 {"cut_in": 20, "lead_brake": 10}。"""
    if not plan:
        raise ValueError("plan 不能为空")
    total = sum(plan.values())
    bid = batch_id or ("mixed_%s_seed%s" % (
        datetime.now().strftime("%Y%m%d_%H%M%S"),
        seed if seed is not None else "none"))
    out_dir = os.path.join(out_root, bid)
    os.makedirs(out_dir, exist_ok=True)

    if verbose:
        print("[mixed] 计划: %s  总计 %d" % (plan, total))

    t0 = time.time()
    all_records: List[Dict[str, Any]] = []
    per_template: Dict[str, Any] = {}
    ok = failed = 0
    offset = 0

    for tname, cnt in plan.items():
        sub_seed = None if seed is None else seed + offset
        m = generate_batch(tname, cnt, out_dir, seed=sub_seed,
                           batch_id=tname, validate_each=True, verbose=False, **kw)
        per_template[tname] = {
            "count": cnt, "valid": m["valid"], "failed": m["failed"],
            "elapsed_seconds": m["elapsed_seconds"],
        }
        ok += m["valid"]
        failed += m["failed"]
        for r in m["scenarios"]:
            r2 = dict(r)
            r2["file"] = os.path.join(tname, r["file"])
            all_records.append(r2)
        offset += cnt

    elapsed = time.time() - t0
    fps = [r["fingerprint"] for r in all_records if r.get("fingerprint")]
    manifest = {
        "batch_id": bid,
        "kind": "mixed",
        "plan": plan,
        "count": total,
        "seed": seed,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "out_dir": os.path.abspath(out_dir),
        "valid": ok, "failed": failed,
        "duplicate_fingerprints": len(fps) - len(set(fps)),
        "elapsed_seconds": round(elapsed, 3),
        "per_scenario_seconds": round(elapsed / max(total, 1), 4),
        "per_template": per_template,
        "scenarios": all_records,
    }
    mpath = os.path.join(out_dir, "manifest.json")
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    manifest["manifest_path"] = mpath

    if verbose:
        print("[mixed] 完成: 成功 %d / 失败 %d, 耗时 %.2fs" % (ok, failed, elapsed))
        print("[mixed] manifest: %s" % mpath)
    return manifest


def verify_manifest(manifest_path: str) -> Dict[str, Any]:
    """独立复核：按 manifest 重新校验每个产物。"""
    with open(manifest_path, "r", encoding="utf-8") as f:
        man = json.load(f)

    base = os.path.dirname(manifest_path)
    bad = []
    for r in man.get("scenarios", []):
        p = os.path.join(base, r["file"])
        if not os.path.exists(p):
            bad.append((r["file"], "文件缺失"))
            continue
        v = validate(p, None)
        if not v["ok"]:
            bad.append((r["file"], "; ".join(v["errors"])))

    return {"manifest": manifest_path, "total": len(man.get("scenarios", [])),
            "rechecked_ok": len(man.get("scenarios", [])) - len(bad),
            "bad": bad}
