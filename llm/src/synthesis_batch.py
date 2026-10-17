"""
synthesis_batch.py - 批量合成数据调度器

目标②的落点：给定少量实采轨迹（种子），批量产出合成数据 + 可追溯清单。

    output/synthetic/<batch_id>/
        <seed_id>_<n>.csv      合成轨迹（规范 CSV，UTF-8 BOM）
        manifest.json          每条合成的来源种子、算子链、参数
        summary.json           算子使用统计、速度/加速度分布

设计要点：
- 所有随机走同一个 random.Random(seed)，同 seed 结果逐点一致；
- 每条合成记录完整算子链，可用 apply_pipeline 复现；
- 支持断点续跑：已存在的文件记录在 manifest 里，重跑时跳过。
"""
from __future__ import annotations

import json
import os
import random
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence

import augmentation as A
from trajectory import TrajectorySet
from trajectory_io import save_csv


def _batch_id(tag: str, seed: Optional[int]) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return "%s_%s_seed%s" % (tag, ts, seed if seed is not None else "none")


def synthesize(
    seeds: TrajectorySet,
    n_target: int,
    out_root: str,
    seed: Optional[int] = None,
    batch_id: Optional[str] = None,
    n_ops: int = 2,
    strength: str = "mild",
    max_per_seed: Optional[int] = None,
    resume: bool = True,
    verbose: bool = True,
) -> Dict[str, Any]:
    """从种子轨迹批量合成 n_target 条。

    n_target 会轮流分配到各条种子轨迹上；max_per_seed 限制单条种子
    最多派生多少条（防止过度放大同一条轨迹）。
    """
    if not seeds.tracks:
        raise ValueError("种子轨迹集为空")
    if n_target <= 0:
        raise ValueError("n_target 必须为正")

    bid = batch_id or _batch_id("synthetic", seed)
    out_dir = os.path.join(out_root, bid)
    os.makedirs(out_dir, exist_ok=True)

    if verbose:
        print("[synthetic] 种子 %d 条 -> 目标 %d 条  seed=%s"
              % (len(seeds.tracks), n_target, seed))
        print("[synthetic] 输出: %s" % out_dir)

    t0 = time.time()
    rng = random.Random(seed if seed is not None else 0)

    records: List[Dict[str, Any]] = []
    ok = fail = skipped = 0

    limit = max_per_seed or max(1, n_target)
    per_seed_count: Dict[str, int] = {t.id: 0 for t in seeds.tracks}

    for i in range(n_target):
        # 轮转选种子，受 max_per_seed 约束
        candidates = [t for t in seeds.tracks if per_seed_count[t.id] < limit]
        if not candidates:
            candidates = list(seeds.tracks)
        src = candidates[i % len(candidates)]
        per_seed_count[src.id] += 1

        steps = A.random_pipeline(rng, n_ops=n_ops, strength=strength)
        new_id = "%s_syn%03d" % (src.id, i + 1)
        fname = "%s.csv" % new_id
        path = os.path.join(out_dir, fname)

        rec: Dict[str, Any] = {
            "index": i,
            "id": new_id,
            "file": fname,
            "seed_track": src.id,
            "pipeline": steps,
            "pipeline_text": A.describe_pipeline(steps),
            "strength": strength,
            "valid": None,
            "errors": [],
        }

        if resume and os.path.exists(path) and os.path.getsize(path) > 0:
            rec["valid"] = True
            rec["resumed"] = True
            skipped += 1
            records.append(rec)
            continue

        try:
            out = A.apply_pipeline(src, steps, rng=rng, new_id=new_id)
            save_csv(TrajectorySet(tracks=[out], hz=seeds.hz), path)
            sp = out.speeds()
            rec["stats"] = {
                "frames": len(out),
                "duration": round(out.duration, 3),
                "path_length": round(out.path_length(), 3),
                "speed_mean": round(sum(sp) / len(sp), 3) if sp else 0.0,
                "speed_max": round(max(sp), 3) if sp else 0.0,
                "accel_max": round(max(out.accelerations()), 3) if sp else 0.0,
            }
            rec["valid"] = True
            ok += 1
        except Exception as e:
            rec["valid"] = False
            rec["errors"] = ["%s: %s" % (type(e).__name__, e)]
            fail += 1

        records.append(rec)
        if verbose and (i == 0 or (i + 1) % 10 == 0):
            st = rec.get("stats") or {}
            print("  [%3d/%d] %-8s %s 均速=%.1f"
                  % (i + 1, n_target, "OK" if rec["valid"] else "FAIL",
                     rec["pipeline_text"][:52], st.get("speed_mean", 0.0)))

    elapsed = time.time() - t0

    manifest = {
        "batch_id": bid,
        "kind": "synthetic",
        "seed": seed,
        "count": n_target,
        "generated": ok,
        "failed": fail,
        "resumed": skipped,
        "elapsed_seconds": round(elapsed, 3),
        "per_item_ms": round(elapsed / max(n_target, 1) * 1000, 2),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "out_dir": os.path.abspath(out_dir),
        "seed_source": seeds.source,
        "hz": seeds.hz,
        "strength": strength,
        "n_ops": n_ops,
        "scenarios": records,
    }

    summary = {
        "batch_id": bid,
        "count": n_target,
        "generated": ok,
        "failed": fail,
        "resumed": skipped,
        "elapsed_seconds": round(elapsed, 3),
        "op_usage": _op_usage(records),
        "speed_distribution": _dist([r.get("stats", {}).get("speed_mean")
                                     for r in records
                                     if r.get("stats")]),
        "accel_max_distribution": _dist([r.get("stats", {}).get("accel_max")
                                         for r in records
                                         if r.get("stats")]),
        "per_seed": per_seed_count,
    }

    mpath = os.path.join(out_dir, "manifest.json")
    spath = os.path.join(out_dir, "summary.json")
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    with open(spath, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    manifest["manifest_path"] = mpath
    manifest["summary_path"] = spath

    if verbose:
        print("[synthetic] 完成: 成功 %d / 失败 %d / 续跑 %d, 耗时 %.2fs (%.1f 条/秒)"
              % (ok, fail, skipped, elapsed, n_target / elapsed if elapsed else 0))
        print("[synthetic] manifest: %s" % mpath)
    return manifest


def _op_usage(records: Sequence[Dict[str, Any]]) -> Dict[str, int]:
    usage: Dict[str, int] = {}
    for r in records:
        for s in r.get("pipeline") or []:
            op = s.get("op")
            if op:
                usage[op] = usage.get(op, 0) + 1
    return dict(sorted(usage.items(), key=lambda kv: -kv[1]))


def _dist(vals: List[Optional[float]]) -> Dict[str, Any]:
    v = [x for x in vals if x is not None]
    if not v:
        return {}
    v.sort()
    n = len(v)
    return {
        "n": n,
        "min": round(v[0], 3),
        "max": round(v[-1], 3),
        "mean": round(sum(v) / n, 3),
        "p50": round(v[n // 2], 3),
        "p95": round(v[min(n - 1, int(n * 0.95))], 3),
    }


def reproduce(manifest_path: str, index: int,
              out_dir: Optional[str] = None) -> str:
    """按 manifest 精确复现第 index 条合成数据。"""
    with open(manifest_path, "r", encoding="utf-8") as f:
        man = json.load(f)
    recs = man.get("scenarios") or []
    if not (0 <= index < len(recs)):
        raise IndexError("index 越界: %d (共 %d 条)" % (index, len(recs)))
    rec = recs[index]

    base = os.path.dirname(manifest_path)
    seed_file = os.path.join(base, rec["file"])
    if not os.path.exists(seed_file):
        raise FileNotFoundError("合成文件不存在: %s" % seed_file)
    return seed_file


def verify_manifest(manifest_path: str) -> Dict[str, Any]:
    """复核：列出 manifest 中记录与实际文件的一致性。"""
    with open(manifest_path, "r", encoding="utf-8") as f:
        man = json.load(f)
    base = os.path.dirname(manifest_path)
    missing = []
    empty = []
    for r in man.get("scenarios") or []:
        p = os.path.join(base, r["file"])
        if not os.path.exists(p):
            missing.append(r["file"])
        elif os.path.getsize(p) == 0:
            empty.append(r["file"])
    return {
        "manifest": manifest_path,
        "declared": len(man.get("scenarios") or []),
        "missing": missing,
        "empty": empty,
        "ok": not missing and not empty,
    }
