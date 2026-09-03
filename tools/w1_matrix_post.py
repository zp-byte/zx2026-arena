#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w1_matrix_post.py — W1 矩阵后处理：同 seed 配对 + 机制证据聚合。

输入: matrix_w1_<ts> 目录（w1_matrix_run.py 产出：matrix_results.csv +
      各 cell evidence.txt）。
输出:
  1. 同 seed 配对表（A/B/C/D 逐 seed：verdict/score/done/col/stuck/minclr）
  2. 每臂均值 + delta vs A
  3. 机制证据: VIA-SLOT 槽位数、quiet-window 数、sep/swarm guard hit 数、
     close=（接触法向闭合速度）逐臂中位数/最大值
判读纪律: A/B 只按同 seed 配对判；FAIL-cell 的 nav_metrics 可能是陈旧文件
（collector 未定稿）——以 verify.txt 与 CSV 行为准，机制证据以 evidence.txt 为准。
用法: python3 tools/w1_matrix_post.py <matrix_w1_dir>
"""
import csv
import glob
import os
import re
import statistics
import sys

WS = os.path.expanduser("~/zx2026_arena_ws")


def load_rows(d):
    p = os.path.join(d, "matrix_results.csv")
    rows = []
    with open(p) as f:
        for r in csv.DictReader(f):
            for k in ("seed", "score", "done_t", "col", "stuck_s", "stuck_n",
                      "flips", "dist", "minclr"):
                r[k] = float(r[k]) if r[k] not in ("", None) else -1.0
            rows.append(r)
    return rows


def load_evidence(d):
    """cell 名 → dict(via_slots, via_slot_fallback, quiet_win, de_eng, sep_hits,
    swarm_hits, closes, clrs, slots)。"""
    out = {}
    for ev in sorted(glob.glob(os.path.join(d, "*", "evidence.txt"))):
        cell = os.path.basename(os.path.dirname(ev))
        txt = open(ev, encoding="utf-8", errors="replace").read()
        slots = re.findall(r"VIA-SLOT \(([-\d.]+),([-\d.]+)\) clr=([-\d.]+)", txt)
        out[cell] = dict(
            via_slots=len(slots),
            via_slot_fallback=len(re.findall(r"via_slots 无可行槽位集", txt)),
            quiet_win=len(re.findall(r"swarm quiet window", txt)),
            de_eng=len(re.findall(r"DEAD-END escape engaged", txt)),
            de_done=len(re.findall(r"dead-end escape done", txt)),
            sep_hits=len(re.findall(r"sep_guard hit", txt)),
            swarm_hits=len(re.findall(r"swarm_guard hit", txt)),
            closes=[float(x) for x in re.findall(r"close=([-\d.]+)", txt)],
            clrs=[float(x) for x in re.findall(r"clr=([-\d.]+)", txt)],
            slots=slots,
        )
    return out


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else sorted(
        glob.glob(os.path.join(WS, "run_logs", "matrix_w1_*")), key=os.path.getmtime)[-1]
    print("matrix dir:", d)
    rows = load_rows(d)
    ev = load_evidence(d)
    tags = sorted({r["tag"] for r in rows})
    seeds = sorted({int(r["seed"]) for r in rows})

    # ---- 1. 同 seed 配对 ----
    print("\n===== 同 seed 配对（判读以此为准） =====")
    print("%-5s %-8s %5s %6s %5s %7s %7s" %
          ("seed", "arm", "v", "score", "col", "stuck_s", "minclr"))
    for s in seeds:
        for r in [x for x in rows if int(x["seed"]) == s]:
            print("%-5d %-8s %5s %6.0f %5.0f %7.1f %7.3f" %
                  (s, r["tag"], r["verdict"][:1], r["score"], r["col"],
                   r["stuck_s"], r["minclr"]))

    # ---- 2. 每臂均值 + delta vs A ----
    print("\n===== 每臂均值（delta vs A_base） =====")
    base = {}
    for tag in tags:
        arm = [r for r in rows if r["tag"] == tag]
        n = len(arm)
        base[tag] = {k: sum(r[k] for r in arm) / n for k in
                     ("score", "done_t", "col", "stuck_s", "minclr")}
        base[tag]["pass"] = sum(1 for r in arm if r["verdict"] == "PASS")
        base[tag]["n"] = n
    hd = ("%-8s %2s %5s %7s %7s %7s %7s" %
          ("arm", "n", "pass", "score", "done", "col", "minclr"))
    print(hd)
    for tag in tags:
        b = base[tag]
        print("%-8s %2d %5d %7.1f %7.0f %7.1f %7.3f" %
              (tag, b["n"], b["pass"], b["score"], b["done_t"], b["col"],
               b["minclr"]))
    for tag in tags[1:]:
        b, a = base[tag], base[tags[0]]
        print("  %-8s d_score=%+6.1f d_done=%+6.0f d_col=%+5.1f d_stuck=%+6.1f" %
              (tag, b["score"] - a["score"], b["done_t"] - a["done_t"],
               b["col"] - a["col"], b["stuck_s"] - a["stuck_s"]))

    # ---- 3. 机制证据 ----
    print("\n===== 机制证据（evidence.txt 采自 ~/.ros/log per-node 日志） =====")
    print("%-18s %9s %7s %6s %6s %9s %10s %10s" %
          ("cell", "VIA-SLOT", "fb", "quiet", "de_eng", "sep/swarm", "close_med",
           "close_max"))
    for cell in sorted(ev):
        e = ev[cell]
        cm = statistics.median(e["closes"]) if e["closes"] else float("nan")
        cx = max(e["closes"]) if e["closes"] else float("nan")
        print("%-18s %9d %7d %6d %6d %9s %10.2f %10.2f" %
              (cell, e["via_slots"], e["via_slot_fallback"], e["quiet_win"],
               e["de_eng"], "%d/%d" % (e["sep_hits"], e["swarm_hits"]),
               cm, cx))
    # VIA-SLOT 期望 6/机；quiet-window 期望 ≈ de_done（旗开臂）
    print("\n期望形态: B/C/D 臂 VIA-SLOT≈6（旗开）且 A 臂=0；"
          "swarm_guard hit 仅 D 臂>0；quiet-window 仅 C/D 臂>0 且 ≈ de_done。")


if __name__ == "__main__":
    main()
