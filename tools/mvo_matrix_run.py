#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/mvo_matrix_run.py — W6 VO-lite 机间速度预测避撞 A/B 矩阵（2026-09-07）。

立案：d2/d3 返程对头互撞（sim 127.0s，机间 1.04m、最近树 2.44m——树接触
不可能）；分离纯位置反应窗 ~0.5s 不够汇聚闭合。W6 = TCPA/DCPA 预判 +
相对速度垂直切向偏转（vo_deflect 纯函数 + _apply_vo_avoid 增量层，默认关）。

基线 = 滚动基线（w1 D_full 形全开 + sign_fix 代码层默认，显式落盘）：
via_slots / rescue_quiet / obs_guard / sep_obs_guard / de+hyst2.5 /
rescue_mutex / bounce_corridor 全开；tc/wind/comms/veto/pc/dam/nstop/hi
全关；inflation 0.4。臂（每 seed 交错 X→V，防时段漂移混淆臂间差异）：
  X_base  vo_avoid off（= 当前默认栈）
  V_vo    vo_avoid on
seeds 42,43,45,46,47（机间事件稀疏，5 seed 补功效；42 = 事故 seed）。

主判据：col / idcol（COLLIDED inter-drone 行数）同 seed 配对 +
min_dclr（全程最小机间距，W6 新指标）+ vohits（VO 触发遥测行数，
2s 节流行=活跃度下界，法证采样）。证据从 ~/.ros/log 最新 run 目录采
（banner vo= 回显 / vo_avoid: 触发行 / _guard hit 分桶 / COLLIDED）。

复用 matrix_run.py 补丁机器与指标解析（不跑其 _run_once——它把
sep_obs_guard 强制回 false，写于翻默认前已陈旧）。

用法:
  python3 tools/mvo_matrix_run.py smoke           # Phase 0：X+V seed42 双 cell + 证据门
  python3 tools/mvo_matrix_run.py rest <outdir>   # Phase 1：其余 8 cells 续跑
  python3 tools/mvo_matrix_run.py all             # 全 10 cells（跳过门）
"""
import glob
import os
import re
import shutil
import subprocess
import sys
import time

import yaml

WS = os.path.expanduser("~/zx2026_arena_ws")


def _time_limit_s():
    """P6：时限从 competition_rules.yaml 读取（超时=时限+300）。"""
    try:
        with open(WS + "/src/zx2026_common/config/competition_rules.yaml") as f:
            import yaml
            return float(yaml.safe_load(f).get("time_limit_s", 600.0))
    except Exception:
        return 600.0

sys.path.insert(0, os.path.join(WS, "tools"))
from matrix_run import CFG, VERIFY_TXT, SMOKE_LOG, set_in_block, \
    set_key_in_block, set_top, parse_metrics

ROS_LOG = os.path.expanduser("~/.ros/log")

CELL_FLAGS = [
    ("X_base", dict(vo=False)),
    ("V_vo", dict(vo=True)),
]
SEEDS = (42, 43, 45, 46, 47)
CELLS = [dict(tag=t, seed=s, **f) for s in SEEDS for (t, f) in CELL_FLAGS]

EVIDENCE_PATS = [
    "closed_loop=",            # banner（含 soa=/swg=/swq=/vo= 回显）
    "vo_avoid:",               # W6 VO 触发遥测（d/dcpa/tcpa/push 法证采样）
    "_guard hit",              # 分离/群集/VO guard 触发（分桶遥测）
    "collision marker at",     # nav_node 碰撞标记行（含 close=/clr=）
    "COLLIDED inter-drone",    # world_node 互撞仲裁（本案主信号）
    "COLLIDED obstacle",       # world_node 树撞（防 VO 改道挤树的位移信号）
    "DEAD-END escape engaged",
    "dead-end escape done",
    "goal-seal",
    "STALL",                   # 停滞看门狗（活锁形态自证）
]


def patch_cell(cell):
    """显式落盘全部旗（滚动基线 + W6）：补丁跨 cell 累积，缺省也必须写。"""
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 滚动基线 = 当前默认栈（w1 D_full 形） ----
    set_in_block(CFG, "temporal_consistency", "false")
    set_in_block(CFG, "wind", "false")
    set_in_block(CFG, "comms", "false")
    set_in_block(CFG, "deadend_escape", "true")
    set_key_in_block(CFG, "deadend_escape", "exit_hyst_s", 2.5)
    set_in_block(CFG, "veto_gate", "false")
    set_in_block(CFG, "point_cache", "false")
    set_in_block(CFG, "drift_aware_margin", "false")
    set_key_in_block(CFG, "near_stop_zone", "enabled", "false")
    set_key_in_block(CFG, "nav", "inflation", 0.4)
    set_in_block(CFG, "rescue_mutex", "true")
    set_in_block(CFG, "bounce_corridor", "true")
    set_in_block(CFG, "sep_obs_guard", "true")
    set_in_block(CFG, "hotspot_inflate", "false")
    set_key_in_block(CFG, "via_slots", "enabled", "true")
    set_key_in_block(CFG, "rescue_quiet", "enabled", "true")
    set_key_in_block(CFG, "obs_guard", "enabled", "true")
    # ---- W6 ----
    set_in_block(CFG, "vo_avoid", "true" if cell.get("vo") else "false")


def parse_dclear(yml_path):
    """W6 新指标：飞行段最小机间距（对邻居真值 3D），跨机取 min。"""
    d = yaml.safe_load(open(yml_path)) or {}
    best = 1e9
    for v in (d.get("drones") or {}).values():
        x = float(v.get("min_drone_clear_m", 1e9))
        if 0 < x < 900:        # 999=从未见过邻居哨兵，900 留余量，均不参与
            best = min(best, x)
    return round(best, 3) if best < 1e9 else -1


def ev_count(ev_text, pattern):
    """rosout.log 单源计数。

    同一 rospy 日志行落在节点 .log、-stdout.log、rosout.log 三副本——
    全量扫描会把每条 vo_avoid/COLLIDED 计 2-3 次（vohits 虚高 3×、
    单次互撞 idcol 记 2-3）。rosout.log 聚合全部节点日志且每事件恰一行，
    为计数正典；缺失时退化全量（事件计数宁可粗不可零）。
    """
    ros, alll = [], []
    for ln in ev_text.splitlines():
        if " | " not in ln:
            continue
        base, _, content = ln.partition(" | ")
        alll.append(content)
        if base == "rosout.log":
            ros.append(content)
    src = ros if ros else alll
    return sum(1 for c in src if pattern in c)


def capture_evidence(outdir):
    """从 ~/.ros/log 最新 run 目录采证据行 → <outdir>/evidence.txt。"""
    dirs = [d for d in glob.glob(os.path.join(ROS_LOG, "*"))
            if os.path.isdir(d) and os.path.basename(d) != "latest"]
    if not dirs:
        open(os.path.join(outdir, "evidence.txt"), "w").write("(no ros log dir)")
        return "(none)"
    run_dir = max(dirs, key=os.path.getmtime)
    lines = []
    for fp in sorted(glob.glob(os.path.join(run_dir, "*.log"))):
        try:
            txt = open(fp, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        base = os.path.basename(fp)
        for ln in txt.splitlines():
            if any(p in ln for p in EVIDENCE_PATS):
                lines.append("%s | %s" % (base, ln.strip()))
    with open(os.path.join(outdir, "evidence.txt"), "w", encoding="utf-8") as f:
        f.write("run_dir=%s\n" % os.path.basename(run_dir))
        f.write("\n".join(lines) + "\n")
    return os.path.basename(run_dir)


def _run_once(cell, outdir):
    """跑一次 run_verify.sh 并收集结果 + 证据；返回 (row, sleep_drift_s)。"""
    tag, seed = cell["tag"], cell["seed"]
    # 清理上一 cell 可能残留的节点（防 collector 双实例交错写 CSV）
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    patch_cell(cell)

    t0 = time.time()
    m0 = time.monotonic()
    timed_out = False
    try:
        subprocess.run(["bash", WS + "/run_verify.sh"],
                       capture_output=True, text=True,
                       timeout=_time_limit_s() + 300.0)
    except subprocess.TimeoutExpired:
        timed_out = True
        subprocess.run(["pkill", "-f", "rosmaster.*11411"], capture_output=True)
        subprocess.run(["pkill", "-f", "zx2026_all.launch"], capture_output=True)
        time.sleep(2)
    dt = time.time() - t0
    drift = dt - (time.monotonic() - m0)

    row = dict(tag=tag, seed=seed, verdict="FAIL", score=-1, done_t=-1,
               col=-1, idcol=0, min_dclr=-1, vohits=0, metrics_stale=False,
               stuck_s=-1, stuck_n=-1, flips=-1, dist=-1, minclr=-1,
               runtime_s=round(dt), timeout=timed_out, sleep_s=round(drift, 1))
    try:
        txt = open(VERIFY_TXT).read()
        if "VERDICT: PASS" in txt:
            row["verdict"] = "PASS"
        m = re.search(r"score_summary: total=(\d+)", txt)
        if m:
            row["score"] = int(m.group(1))
        m = re.search(r"-- state -> DONE @ (\d+)s", txt)
        if m:
            row["done_t"] = int(m.group(1))
    except OSError:
        pass

    ymls = sorted(glob.glob(WS + "/run_logs/nav_metrics_2*.yaml"),
                  key=os.path.getmtime)
    # [审查修复] 陈旧 yaml 三重门槛：FAIL/超时 cell 不写定稿 yaml（collector
    # 仅 state==DONE 落盘），glob mtime 最新会拿到上一 cell（同 seed 另一臂）
    # 的数值——跨臂毒化 col/minclr/min_dclr 主判据（matrix_post 存在本身即
    # 此污染的历史铁证）。门槛：本 cell 启动后新写 + run_seed 归属 + DONE。
    # 任一不满足保留 -1 哨兵并打 metrics_stale 标记，不入判据。
    if ymls:
        fresh = None
        try:
            y = yaml.safe_load(open(ymls[-1])) or {}
            if (os.path.getmtime(ymls[-1]) > t0
                    and int(y.get("run_seed", -1)) == cell["seed"]
                    and y.get("state") == "DONE"):
                fresh = ymls[-1]
        except (OSError, ValueError, TypeError):
            fresh = None
        if fresh is None:
            row["metrics_stale"] = True
            print("    !! metrics yaml 陈旧/不属于本 cell——判据字段保留 -1",
                  flush=True)
        else:
            try:
                row.update(parse_metrics(fresh))
                row["min_dclr"] = parse_dclear(fresh)
            except Exception as e:
                print("metrics parse fail:", e, flush=True)

    os.makedirs(outdir, exist_ok=True)
    for src, dst in ((VERIFY_TXT, "verify.txt"), (SMOKE_LOG, "smoke.log")):
        try:
            shutil.copy(src, os.path.join(outdir, dst))
        except OSError:
            pass
    if ymls:
        # 陈旧 yaml 改名归档（nav_metrics_stale.yaml）——行虽已打
        # metrics_stale 旗，同名拷贝仍会污染法证档的跨臂对账
        dst = "nav_metrics_stale.yaml" if row["metrics_stale"] \
            else "nav_metrics.yaml"
        shutil.copy(ymls[-1], os.path.join(outdir, dst))
    scores = glob.glob("/tmp/zx2026_score_*.yaml")
    if scores:
        shutil.copy(max(scores, key=os.path.getmtime),
                    os.path.join(outdir, "score.yaml"))
    run_dir = capture_evidence(outdir)
    row["run_dir"] = run_dir
    # W6 证据计数（rosout.log 单源去重，见 ev_count）
    try:
        ev = open(os.path.join(outdir, "evidence.txt"),
                  encoding="utf-8", errors="replace").read()
        row["idcol"] = ev_count(ev, "COLLIDED inter-drone")
        row["vohits"] = ev_count(ev, "vo_avoid: drone")
    except OSError:
        pass
    return row, drift


def run_cell(cell, outdir, rerun_on_sleep=True):
    """跑一个 cell；检出睡眠/挂死污染则留档并自动重跑一次。"""
    row, drift = _run_once(cell, outdir)
    # 挂死污染签名：verify 无 total（rosmaster 残留 → 起飞空转 ~456s，指标
    # 逐字节继承上一 cell）。非 timeout 的 score=-1 一律按污染重跑一次。
    contaminated = drift > 30.0 or (row["score"] == -1 and not row["timeout"])
    if contaminated and rerun_on_sleep:
        why = ("wall-monotonic 漂移 %.0fs" % drift) if drift > 30.0 \
            else "verify 无 total（rosmaster 残留挂死）"
        print("    !! %s —— cell 污染：留档 __contaminated，自动重跑" % why,
              flush=True)
        shutil.rmtree(outdir + "__contaminated", ignore_errors=True)
        try:
            os.rename(outdir, outdir + "__contaminated")
        except OSError:
            pass
        row, drift = _run_once(cell, outdir)
    row["sleep_s"] = round(drift, 1)
    return row


COLS = ["tag", "seed", "verdict", "score", "done_t", "col", "idcol",
        "min_dclr", "vohits", "stuck_s", "stuck_n", "flips", "dist",
        "minclr", "runtime_s", "sleep_s", "timeout", "metrics_stale"]


def append_row(csv_path, row, write_header):
    with open(csv_path, "a") as f:
        if write_header:
            f.write(",".join(COLS) + "\n")
        f.write(",".join(str(row[c]) for c in COLS) + "\n")


def summarize(rows):
    print("\n===== SUMMARY (per arm) =====", flush=True)
    for tag in ("X_base", "V_vo"):
        arm = [r for r in rows if r["tag"] == tag]
        if not arm:
            continue
        n = len(arm)
        # 指标均值只计非陈旧行（metrics_stale 行判据字段是 -1 哨兵）
        good = [r for r in arm if str(r.get("metrics_stale")) not in
                ("True", "true")]
        mean = lambda k: (sum(r[k] for r in good) / len(good)) if good else -1
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        dclr = [r["min_dclr"] for r in good if r["min_dclr"] > 0]
        print("%-6s n=%d pass=%d score=%.1f done=%.0fs col=%.1f idcol=%d "
              "min_dclr=%s stuck=%.1fs minclr=%.3f vohits=%d%s"
              % (tag, n, passes, mean("score"), mean("done_t"), mean("col"),
                 sum(int(r["idcol"]) for r in arm),
                 ("min=%.3f" % min(dclr)) if dclr else "n/a",
                 mean("stuck_s"), mean("minclr"),
                 sum(int(r["vohits"]) for r in arm),
                 (" (stale=%d 行不入均值)" % (n - len(good)))
                 if n - len(good) else ""), flush=True)
    print("\n===== 同 seed 配对（X→V 主判据）=====", flush=True)
    for s in SEEDS:
        seg = [r for r in rows if r["seed"] == s]
        print("seed %d: %s" % (s, " | ".join(
            "%s col=%s idcol=%s dclr=%s vohits=%s %s"
            % (r["tag"], r["col"], r["idcol"], r["min_dclr"], r["vohits"],
               r["verdict"]) for r in seg)), flush=True)


def _banner(ev):
    return re.search(r"tc=(\w+) de=(\w+) veto=(\w+) metrics=(\w+) "
                     r"soa=(\w+) swg=(\w+) swq=(\w+) vo=(\w+)", ev)


def smoke_gate(outroot):
    """Phase 0 证据门：双 cell banner 全 8 旗回显 + X 臂零 VO 行 + V 臂接线观察。

    banner 八旗全断言（不只 vo）——patch_cell 落盘的任何锚点在未来 yaml
    改版后静默 no-op（P3 ^ 锚点缩进失配同型），双臂同污时配对无法暴露，
    全旗回显是最廉价的防线。
    """
    ok, msgs = True, []
    ev_x = open(os.path.join(outroot, "00_X_base_seed42",
                             "evidence.txt")).read()
    ev_v = open(os.path.join(outroot, "01_V_vo_seed42",
                             "evidence.txt")).read()
    # 期望：tc/de/veto/metrics/soa/swg/swq = 滚动基线固定值，vo 随臂
    want = ("False", "True", "False", "True", "True", "True", "True")
    for name, ev, want_vo in (("X_base", ev_x, "False"),
                              ("V_vo", ev_v, "True")):
        m = _banner(ev)
        if not m:
            ok = False
            msgs.append("FAIL: %s banner (tc=/de=/veto=/metrics=/soa=/swg=/"
                        "swq=/vo=) not found" % name)
            continue
        msgs.append("%s banner: %s" % (name, m.group(0)))
        got = tuple(m.groups())
        if got != want + (want_vo,):
            ok = False
            msgs.append("FAIL: %s 旗错（期望 tc=F de=T veto=F metrics=T "
                        "soa=T swg=T swq=T vo=%s）" % (name, want_vo))
    # X 臂关旗必须零触发（接线反向验证）
    n_vo_x = ev_count(ev_x, "vo_avoid: drone")
    if n_vo_x:
        ok = False
        msgs.append("FAIL: X_base 关旗却有 %d 行 vo_avoid 触发" % n_vo_x)
    else:
        msgs.append("X_base vo_avoid 零触发（关旗干净）OK")
    # V 臂观察项（不设硬门——方差纪律；0 触发对照 min_dclr 人判）
    n_vo_v = ev_count(ev_v, "vo_avoid: drone")
    n_id_v = ev_count(ev_v, "COLLIDED inter-drone")
    n_id_x = ev_count(ev_x, "COLLIDED inter-drone")
    msgs.append("V_vo vo_avoid 触发行=%d%s" % (
        n_vo_v, "" if n_vo_v else "  (WARNING: 0——若 d2/d3 汇聚仍现则接线可疑,"
        "否则方差; 对照 min_dclr)"))
    msgs.append("COLLIDED inter-drone: X=%d V=%d（本案主信号，rosout 单源）"
                % (n_id_x, n_id_v))
    n_grd = ev_count(ev_v, "_guard hit")
    msgs.append("V_vo guard hits=%d（分桶含 tag=vo，纯遥测）" % n_grd)
    return ok, msgs


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    if mode == "smoke":
        ts = time.strftime("%Y%m%d_%H%M%S")
        outroot = os.path.join(WS, "run_logs", "matrix_mvo_" + ts)
        os.makedirs(outroot, exist_ok=True)
        backup = CFG + ".matrix_bak"
        shutil.copy(CFG, backup)
        csv_path = os.path.join(outroot, "matrix_results.csv")
        try:
            for idx in (0, 1):        # 00_X_base_seed42, 01_V_vo_seed42
                cell = CELLS[idx]
                name = "%02d_%s_seed%d" % (idx, cell["tag"], cell["seed"])
                print("[smoke] %s running..." % name, flush=True)
                row = run_cell(cell, os.path.join(outroot, name))
                append_row(csv_path, row, idx == 0)
                print("    -> %s score=%s col=%s idcol=%s dclr=%s vohits=%s"
                      % (row["verdict"], row["score"], row["col"],
                         row["idcol"], row["min_dclr"], row["vohits"]),
                      flush=True)
        finally:
            shutil.copy(backup, CFG)
            os.remove(backup)
        ok, msgs = smoke_gate(outroot)
        print("\n===== PHASE 0 GATE =====", flush=True)
        for m in msgs:
            print("  " + m, flush=True)
        print("GATE: %s" % ("PASS" if ok else "FAIL"), flush=True)
        print("outdir: %s" % outroot, flush=True)
        if not ok:
            sys.exit(2)
        print("next: python3 tools/mvo_matrix_run.py rest %s" % outroot,
              flush=True)
        return

    if mode == "rest":
        outroot = sys.argv[2]
        csv_path = os.path.join(outroot, "matrix_results.csv")
        done = set()
        if os.path.exists(csv_path):
            with open(csv_path) as f:
                for r in csv_rows(f):
                    done.add((r[0], int(float(r[1]))))
        todo = [c for c in CELLS if (c["tag"], c["seed"]) not in done]
        print("rest: %d cells todo in %s" % (len(todo), outroot), flush=True)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        outroot = os.path.join(WS, "run_logs", "matrix_mvo_" + ts)
        todo = list(CELLS)

    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".matrix_bak"
    shutil.copy(CFG, backup)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    try:
        for k, cell in enumerate(todo):
            name = "%02d_%s_seed%d" % (CELLS.index(cell), cell["tag"],
                                       cell["seed"])
            print("[%d/%d] %s running..." % (k + 1, len(todo), name), flush=True)
            row = run_cell(cell, os.path.join(outdir_cell(outroot, name)))
            append_row(csv_path, row, not os.path.exists(csv_path))
            print("    -> %s score=%s done=%ss col=%s idcol=%s dclr=%s "
                  "vohits=%s (%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["idcol"], row["min_dclr"], row["vohits"],
                     row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)
    # 汇总读全量 CSV（含 smoke 行）
    all_rows = []
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            for r in csv_rows(f):
                all_rows.append(dict(zip(COLS, r[:len(COLS)])))
        for r in all_rows:
            r["seed"] = int(float(r["seed"]))   # CSV 反序列化是 str——配对前必须转
            for k in ("score", "done_t", "col", "idcol", "min_dclr", "vohits",
                      "stuck_s", "stuck_n", "flips", "dist", "minclr"):
                r[k] = float(r[k])
    summarize(all_rows)
    print("results csv: %s" % csv_path, flush=True)


def outdir_cell(outroot, name):
    p = os.path.join(outroot, name)
    os.makedirs(p, exist_ok=True)
    return p


def csv_rows(f):
    import csv as _csv
    for r in _csv.reader(f):
        if len(r) >= len(COLS) and r[0] != "tag":
            yield r


if __name__ == "__main__":
    main()
