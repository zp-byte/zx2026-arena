#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/tip_matrix_post.py — ② 枝梢时窗矩阵归因（2026-09-18）。

碰撞 → 最近障碍归属（2D）：树干面 / 枝段（点-线段距离 + 参数 t，t≥0.85 判
_TIP）/ fence/curb AABB。枝形随 run_seed 变（br_rng=森林种子×run_seed）——
按 cell 的 seed 分组 sed run_seed 后实例化 Scene，逐组归因。

J_hiso 教训对账口径：A/B 两臂的碰撞类分布（trunk/branch_TIP/branch_mid/
fence/curb）必须分开列——B 臂若把碰撞从 branch_TIP 挪到别的类=风险再分布，
即使 Σcol 降也判杀。

用法: python3 tools/tip_matrix_post.py <matrix_outbase>
"""
import glob
import math
import os
import re
import sys

import yaml

WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, WS + "/tools")
sys.path.insert(0, WS + "/src/zx2026_common/scripts")
from matrix_run import CFG, set_top  # noqa: E402
from zx2026_common.scene import Scene  # noqa: E402

TIP_T = 0.85


def seg_dist_t(px, py, sx, sy, ex, ey):
    """点到 2D 线段距离 + 参数 t∈[0,1]（0=起点 1=末端=梢）。"""
    dx, dy = ex - sx, ey - sy
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(px - sx, py - sy), 0.0
    t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / l2))
    cx, cy = sx + t * dx, sy + t * dy
    return math.hypot(px - cx, py - cy), t


def nearest_obstacle(sc, x, y):
    """(cls, label, dist)——2D 最近障碍分类归属。"""
    best = ("?", "?", 1e9)
    for i, ob in enumerate(sc.obstacles):
        cands = []
        if ob.kind == "tree":
            d = math.hypot(x - ob.cx, y - ob.cy) - ob.trunk_r
            cands.append(("trunk", "tree#%d(%.1f,%.1f)" % (i, ob.cx, ob.cy), d))
            for j, seg in enumerate(ob.branches or []):
                sx, sy = seg[0], seg[1]
                ex, ey = seg[3], seg[4]
                r = seg[6] if len(seg) > 6 else 0.015
                d, t = seg_dist_t(x, y, sx, sy, ex, ey)
                d -= r
                lab = "tree#%d seg#%d t=%.2f%s" % (
                    i, j, t, "_TIP" if t >= TIP_T else "")
                cands.append(("branch_TIP" if t >= TIP_T else "branch_mid",
                              lab, d))
        else:
            lo, hi = ob.lo, ob.hi
            dx = max(lo[0] - x, 0, x - hi[0])
            dy = max(lo[1] - y, 0, y - hi[1])
            d = math.hypot(dx, dy)
            cands.append((ob.kind, "%s@(%s)" % (
                ob.kind, ",".join("%.1f" % v for v in lo[:2])), d))
        for (cls, lab, d) in cands:
            if d < best[2]:
                best = (cls, lab, d)
    return best


def parse_cell(cdir):
    """cell 目录 → dict（verdict/score/col/碰撞去重列表/run_seed）。"""
    out = dict(tag=os.path.basename(cdir), verdict="?", score=-1, col=-1,
               seed=None, cols=[])
    try:
        d = yaml.safe_load(open(os.path.join(cdir, "nav_metrics.yaml")))
        out["seed"] = d.get("run_seed")
        out["col"] = int(sum(v.get("collisions", 0) for v in d["drones"].values()))
    except Exception:
        pass
    try:
        txt = open(os.path.join(cdir, "verify.txt"), errors="replace").read()
        if "VERDICT: PASS" in txt:
            out["verdict"] = "PASS"
        elif "VERDICT" in txt:
            out["verdict"] = "FAIL"
    except OSError:
        pass
    try:
        sy = yaml.safe_load(open(os.path.join(cdir, "score.yaml")))
        t = sy.get("total")
        if isinstance(t, (int, float)):
            out["score"] = int(t)
    except Exception:
        pass
    seen = set()
    try:
        for ln in open(os.path.join(cdir, "evidence.txt"), errors="replace"):
            m = re.search(r"drone (\d+) collision marker at "
                          r"\((-?\d+\.\d+),(-?\d+\.\d+)\)", ln)
            if not m:
                continue
            key = (m.group(1), round(float(m.group(2)), 1),
                   round(float(m.group(3)), 1))
            if key in seen:
                continue
            seen.add(key)
            out["cols"].append((int(m.group(1)), float(m.group(2)),
                                float(m.group(3))))
    except OSError:
        pass
    return out


def main():
    base = sys.argv[1]
    cells = [c for c in sorted(glob.glob(base + "_*"))
             if os.path.isdir(c) and not c.endswith("__contaminated")]
    parsed = [parse_cell(c) for c in cells]

    print("== per-cell ==")
    for p in parsed:
        print("%-28s seed=%-3s %-4s score=%-4d col=%d  n_ev=%d" % (
            p["tag"], p["seed"], p["verdict"], p["score"], p["col"],
            len(p["cols"])))

    # 按 seed 分组实例化 Scene（枝形随 run_seed 变）
    by_seed = {}
    for p in parsed:
        if p["seed"] is not None and p["cols"]:
            by_seed.setdefault(int(p["seed"]), []).append(p)

    hist = {}
    for seed, group in sorted(by_seed.items()):
        set_top(CFG, "run_seed", seed)
        sc = Scene()
        for p in group:
            for (dr, x, y) in p["cols"]:
                cls, lab, d = nearest_obstacle(sc, x, y)
                arm = p["tag"].split("_r")[0]
                hist.setdefault(arm, {}).setdefault(cls, []).append(
                    "%s %s d=%.3f" % (p["tag"], lab, d))

    print("\n== 归因直方图（臂 × 类）==")
    for arm in sorted(hist):
        total = sum(len(v) for v in hist[arm].values())
        parts = ", ".join("%s=%d" % (c, len(v))
                          for c, v in sorted(hist[arm].items(),
                                             key=lambda kv: -len(kv[1])))
        print("%-8s n=%d: %s" % (arm, total, parts))
        for c in ("branch_TIP", "branch_mid"):
            for ln in hist[arm].get(c, []):
                print("    [%s] %s" % (c, ln))

    print("\n== 臂汇总（J 对账：类分布不许挪移）==")
    for arm in sorted(hist):
        cls_n = {c: len(v) for c, v in hist[arm].items()}
        print("%-8s %s" % (arm, cls_n))


if __name__ == "__main__":
    main()
