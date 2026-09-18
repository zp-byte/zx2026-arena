#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/padhover_mine.py — W4 pad-hover 立案取证（2026-09-18）。

形态：drone 到 pad 段后悬停不落/落地迟滞（landed_n<6、TOUCHDOWN 缺失、
返航时长异常）。对矩阵 cell 批量对账：
  - score.yaml：landed_n / 逐机 landed / retired / first_correct；
  - ~/.ros/log/<run_dir>/executor_*-stdout.log：drone N return start /
    TOUCHDOWN 的 sim 时间戳 → 逐机返航时长（还 w2 工具债：返航时长解析器）；
  - hover/retire/UNAUTHORIZED 行全文采集。

判读基准（w2 A_base_s43 实测）：正常返航 45-50s；>90s（2×）或 TOUCHDOWN
缺失=pad 段异常候选。

用法: python3 tools/padhover_mine.py <matrix_outbase> [<outbase2> ...]
"""
import glob
import os
import re
import sys

import yaml

WS = "/home/ubuntu/zx2026_arena_ws"
ROS_LOG = os.path.expanduser("~/.ros/log")

TS_RE = re.compile(r"\[(\d+)\.(\d+)\]")
RET_RE = re.compile(r"drone (\d+) return start")
TD_RE = re.compile(r"drone (\d+) TOUCHDOWN z=(-?\d+\.?\d*)")
RCB_RE = re.compile(r"drone (\d+) crossed zone, RECON scan begin")
RCL_RE = re.compile(r"drone (\d+) RECON LOCK platform")
FLAG_RE = re.compile(r"hover|retire|UNAUTHORIZED|LANDED|lock", re.I)


def ts_of(line):
    m = TS_RE.search(line)
    return float(m.group(1)) + float(m.group(2)) / 1e6 if m else None


def mine_cell(cdir):
    out = dict(tag=os.path.basename(cdir), landed_n=None, total=None,
               drones={}, flag_lines=[])
    try:
        sy = yaml.safe_load(open(os.path.join(cdir, "score.yaml")))
        out["landed_n"] = sy.get("landed_n")
        out["total"] = sy.get("total")
        for k, v in (sy.get("scores") or {}).items():
            out["drones"][int(k)] = dict(landed=v.get("landed"),
                                         retired=v.get("retired"))
    except Exception:
        pass
    run_dir = None
    try:
        with open(os.path.join(cdir, "evidence.txt"), errors="replace") as f:
            for ln in f:
                m = re.match(r"run_dir=(\S+)", ln)
                if m:
                    run_dir = os.path.join(ROS_LOG, m.group(1))
                    break
    except OSError:
        pass
    if not run_dir or not os.path.isdir(run_dir):
        out["err"] = "no run_dir"
        return out
    ret, td, rcb, rcl = {}, {}, {}, {}
    t0 = None
    for fp in glob.glob(os.path.join(run_dir, "executor_*-stdout.log")):
        try:
            for ln in open(fp, encoding="utf-8", errors="replace"):
                t = ts_of(ln)
                if t is not None and (t0 is None or t < t0):
                    t0 = t
                m = RET_RE.search(ln)
                if m:
                    d = int(m.group(1))
                    if d not in ret or (t and t < ret[d]):
                        ret[d] = t
                    continue
                m = TD_RE.search(ln)
                if m:
                    d = int(m.group(1))
                    if d not in td or (t and t > td[d]):
                        td[d] = (t, float(m.group(2)))
                    continue
                m = RCB_RE.search(ln)
                if m:
                    d = int(m.group(1))
                    if d not in rcb or (t and t < rcb[d]):
                        rcb[d] = t
                    continue
                m = RCL_RE.search(ln)
                if m:
                    d = int(m.group(1))
                    if d not in rcl or (t and t > rcl[d]):
                        rcl[d] = t
                    continue
                if FLAG_RE.search(ln) and "drone" in ln:
                    out["flag_lines"].append(
                        os.path.basename(fp) + " | " + ln.strip()[:150])
        except OSError:
            continue
    # 任务终点 sim 时刻 = t0 + done_t（verify.txt 的 DONE @ Ns）
    done_t = None
    try:
        vtxt = open(os.path.join(cdir, "verify.txt"), errors="replace").read()
        m = re.search(r"-- state -> DONE @ (\d+)s", vtxt)
        if m:
            done_t = int(m.group(1))
    except OSError:
        pass
    t_end = (t0 + done_t) if (t0 is not None and done_t is not None) else None
    out["t_end"] = t_end
    for d in sorted(set(ret) | set(td)):
        r, t = ret.get(d), td.get(d, (None, None))[0]
        dur = (t - r) if (r is not None and t is not None) else None
        out["drones"].setdefault(d, {})
        out["drones"][d]["ret_s"] = round(dur, 1) if dur is not None else None
        out["drones"][d]["no_td"] = t is None and r is not None
        # LOCK→return-start 延迟、RECON 游走时长、return start 相对任务终点余量
        if d in rcl and d in ret:
            out["drones"][d]["sched_s"] = round(ret[d] - rcl[d], 1)
        if d in rcb and d in rcl:
            out["drones"][d]["recon_s"] = round(rcl[d] - rcb[d], 1)
        if t_end is not None and d in ret:
            out["drones"][d]["slack_s"] = round(t_end - ret[d], 1)
    return out


def main():
    bases = sys.argv[1:]
    long_ret, no_td, not_landed = [], [], []
    long_recon, low_slack = [], []
    for base in bases:
        cells = sorted(glob.glob(base + "_*"))
        for cdir in cells:
            if not os.path.isdir(cdir) or cdir.endswith("__contaminated"):
                continue
            c = mine_cell(cdir)
            print("== %s landed_n=%s total=%s" % (c["tag"], c["landed_n"],
                                                  c["total"]))
            for d, v in sorted(c["drones"].items()):
                ret_s = v.get("ret_s")
                flag = ""
                if v.get("retired"):
                    flag += " RETIRED"
                if v.get("landed") is False:
                    flag += " NOT_LANDED"
                    not_landed.append((c["tag"], d))
                if v.get("no_td"):
                    flag += " NO_TOUCHDOWN"
                    no_td.append((c["tag"], d))
                if ret_s is not None and ret_s > 90.0:
                    flag += " LONG_RET"
                    long_ret.append((c["tag"], d, ret_s))
                recon_s = v.get("recon_s")
                slack_s = v.get("slack_s")
                if recon_s is not None and recon_s > 80.0:
                    flag += " LONG_RECON"
                    long_recon.append((c["tag"], d, recon_s))
                if slack_s is not None and slack_s < 70.0:
                    flag += " LOW_SLACK"
                    low_slack.append((c["tag"], d, slack_s))
                print("   d%d ret=%-6s recon=%-6s sched=%-5s slack=%-6s "
                      "landed=%-5s%s" % (
                          d, ret_s, recon_s, v.get("sched_s"), slack_s,
                          v.get("landed"), flag))
            if c["flag_lines"]:
                for ln in c["flag_lines"][:4]:
                    print("   ~ %s" % ln)
                if len(c["flag_lines"]) > 4:
                    print("   ~ ... +%d more" % (len(c["flag_lines"]) - 4))
    print("\n== 汇总 ==")
    print("LONG_RET(>90s):     %s" % long_ret)
    print("NO_TOUCHDOWN:       %s" % no_td)
    print("NOT_LANDED:         %s" % not_landed)
    print("LONG_RECON(>80s):   %s" % long_recon)
    print("LOW_SLACK(<70s):    %s" % low_slack)


if __name__ == "__main__":
    main()
