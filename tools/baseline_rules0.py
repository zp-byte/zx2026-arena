#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/baseline_rules0.py — 规则对齐前基线冻结（P0）。

现行默认旗栈（滚动基线：tc/wind/veto/pc/nstop/hi 关，
de/swq/rm/bc/soa 开，inflation 0.4）× seeds 42/43/45。
产出 run_logs/baseline_rules0_<ts>/ 供 P8 回归对照。

用法: python3 tools/baseline_rules0.py
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matrix_run as mr  # noqa: E402

SEEDS = (42, 43, 45)
FLAGS = dict(tc=False, wind=False, veto=False, pc=False, nstop=False,
             hi=False, de=True, rm=True, bc=True, soa=True, infl=0.4)
CELLS = [dict(tag="base", seed=s, **FLAGS) for s in SEEDS]
COLS = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
        "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
        "timeout"]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(mr.WS, "run_logs", "baseline_rules0_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = mr.CFG + ".baseline_bak"
    shutil.copy(mr.CFG, backup)
    csv_path = os.path.join(outroot, "baseline_results.csv")
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%02d_%s_seed%d" % (k, cell["tag"], cell["seed"])
            print("[%d/%d] %s running..." % (k + 1, len(CELLS), name),
                  flush=True)
            row = mr.run_cell(cell, os.path.join(outroot, name))
            rows.append(row)
            with open(csv_path, "a") as f:
                if k == 0:
                    f.write(",".join(COLS) + "\n")
                f.write(",".join(str(row[c]) for c in COLS) + "\n")
            print("    -> %s score=%s done=%ss col=%s stuck=%s minclr=%s (%ss)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["minclr"], row["runtime_s"]),
                  flush=True)
    finally:
        shutil.copy(backup, mr.CFG)
        os.remove(backup)

    print("\n===== BASELINE SUMMARY =====", flush=True)
    good = [r for r in rows if r["verdict"] == "PASS"]
    mean = (lambda key: (sum(float(r[key]) for r in good) / len(good))
            if good else -1)
    print("n=%d pass=%d score=%.1f done=%.0fs col=%.1f stuck=%.1fs "
          "minclr=%.3f flips=%.1f"
          % (len(rows), len(good), mean("score"), mean("done_t"),
             mean("col"), mean("stuck_s"), mean("minclr"), mean("flips")),
          flush=True)
    print("baseline csv: %s" % csv_path, flush=True)


if __name__ == "__main__":
    main()
