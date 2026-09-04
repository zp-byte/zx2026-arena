#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/fleet_monitor.py — 机群后台实时监测（纯被动，零 sim 影响）。

姿态对齐 nav_metrics_node.py 先例：全局单实例、匿名节点、不发布任何话题、
不写 nav_metrics_* 文件（避免与收集器双重写入截行）。

数据面（全话题，双后端同名）:
  /clock                                    sim 时间
  /zx2026/state /fleet_ready /score_summary /phase_timeout
  /zx2026/score/<i>                         per-drone 比分（可选，import 失败自动跳过）
  /drone_<i>/odom        20Hz               真实位姿+速度
  /drone_<i>/mission/phase                  IDLE..DONE/FAILED (latched)
  /drone_<i>/mission/crossed_zone|at_drop   latched Bool
  /drone_<i>/planning/goal                  当前导航目标
  /drone_<i>/nav_metrics  5Hz                11 floats: 0=col 1=stuck_s 4=flips
                                             6=min_clear 9=speed 10=clear (999 cap)
  /drone_<i>/collision                      事件 Bool (latched, 晚加入的陈旧 True
                                             由 run 重置逻辑清除)
  /rosout                                   事件流（FAILED 原因/VIA-SLOT/过带/
                                            STALL/DEAD-END/GOAL-SEAL/quiet/guard/
                                            碰撞坐标/重试 —— 这些只落日志）

用法（另开一个 WSL 终端，或 tmux 里）:
  source /opt/ros/noetic/setup.bash && source ~/zx2026_arena_ws/devel/setup.bash
  python3 tools/fleet_monitor.py                     # 实时面板（ANSI 刷屏）
  python3 tools/fleet_monitor.py --noline            # 不清屏，适合 tee 进文件
  python3 tools/fleet_monitor.py --once              # 单帧快照后退出（脚本用）
  python3 tools/fleet_monitor.py --record /tmp/fm.jsonl   # 同步落 JSONL 存证
  可选 --rate HZ（默认 1.0）

master 生命周期归 runner 所有（run_verify.sh 起止都 pkill 11411）：
master 消失 = 本轮结束，监测器打印提示并退出；下轮重新拉起即可。
"""
import argparse
import collections
import json
import os
import re
import sys
import threading
import time

# 本机环境钩子会把 ROS_MASTER_URI 带成默认 11311——监测器必须赢回 11411
# （用户显式指向其它 master 时以用户为准）。
_ROMU = os.environ.get("ROS_MASTER_URI", "").rstrip("/")
if _ROMU in ("", "http://localhost:11311", "http://127.0.0.1:11311"):
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11411"
os.environ.setdefault("ROS_HOSTNAME", "127.0.0.1")

import rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock, Log  # noqa: E402
from std_msgs.msg import Bool, Float32MultiArray, String  # noqa: E402

try:
    from zx2026_common.msg import Score  # noqa: E402
except ImportError:
    Score = None

DRONES = 6
CAP999 = 999.0
LEG_ALARM = 90.0   # 单腿超阈告警（秒）；--leg-alarm 可调

# /rosout 事件分类（顺序即优先级；全 ASCII，来源见各 node 源码）
PATTERNS = [
    (re.compile(r"drone (\d) FAILED: (.+)"), "FAILED"),
    (re.compile(r"drone (\d) COLLISION #(\d+)"), "COL"),           # gz 后端
    (re.compile(r"drone (\d) collision marker at \(([-\d.]+),([-\d.]+)\)"), "COL"),  # py
    (re.compile(r"drone (\d) recovered from collision \(count=(\d+)\)"), "col_rec"),
    (re.compile(r"drone (\d) frozen \(>max_collisions"), "FROZEN"),
    (re.compile(r"drone (\d) STALL diag"), "STALL"),
    (re.compile(r"drone (\d) DEAD-END escape engaged"), "DE"),
    (re.compile(r"drone (\d) GOAL-SEAL"), "GS"),
    (re.compile(r"drone (\d) swarm quiet window open"), "quiet"),
    (re.compile(r"drone (\d) (sep|swarm)_guard hit n=(\d+)"), "guard"),
    (re.compile(r"drone (\d) VIA-SLOT \(([-\d.]+),([-\d.]+)\) clr=([-\d.]+)"), "slot"),
    (re.compile(r"drone (\d) heading to (via slot|crossing zone) \(([-\d.]+), ([-\d.]+)\)"), "goto"),
    (re.compile(r"drone (\d) crossed zone"), "crossed"),
    (re.compile(r"drone (\d) at pad, descending to hover"), "at_pad"),
    (re.compile(r"drone (\d) match timeout, retry (\d)"), "retry"),
    (re.compile(r"drone (\d) type MATCH confirmed"), "match"),
    (re.compile(r"drone (\d) HOVER_TIMEOUT"), "hover_to"),
    (re.compile(r"drone (\d) releasing type (\w+)"), "drop"),
    (re.compile(r"drone (\d) via_slots infeasible"), "fallback"),
    (re.compile(r"drone (\d) mission DONE"), "done"),
]

# 一轮的重置触发：stage controller 刚进入初始相位
RESET_STATES = ("P1_INIT", "P2_WAIT")


def fmt1(v):
    """999 哨兵/None -> '--'；其余一位小数。"""
    if v is None or v >= CAP999:
        return "--"
    return "%.1f" % v


class FleetState(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.sim_t = None
        self.stage = "-"
        self.ready = None
        self.score_summary = ""
        self.phase_timeout = False
        self.events = collections.deque(maxlen=40)   # (sim_t, wall, text)
        self.drones = {i: self._fresh() for i in range(DRONES)}

    @staticmethod
    def _fresh():
        return {
            "phase": "-", "x": None, "y": None, "v": None, "goal": None,
            "col": None, "stuck": None, "flips": None, "minclr": None,
            "clr": None, "crossed": False, "at_drop": False, "match": "-",
            "score": None, "last_event": "-", "leg_t0": None, "leg_wall": None,
            "ev_col": 0, "ev_stall": 0, "ev_de": 0, "ev_gs": 0,
            "ev_quiet": 0, "ev_retry": 0, "ev_fb": 0,
        }

    def reset_run(self):
        with self.lock:
            self.events.clear()
            self.drones = {i: self._fresh() for i in range(DRONES)}
            self.phase_timeout = False

    def leg_touch(self, i, sim_t):
        """P4 长尾观测：腿起点打点（腿事件/相位变化时调用）。"""
        with self.lock:
            d = self.drones[min(max(i, 0), DRONES - 1)]
            d["leg_t0"] = sim_t
            d["leg_wall"] = time.time()


FS = FleetState()


def on_clock(m):
    FS.sim_t = m.clock.to_sec()


def on_state(m):
    if m.data in RESET_STATES:
        FS.reset_run()
    FS.stage = m.data


def on_ready(m):
    FS.ready = bool(m.data)


def on_score_summary(m):
    FS.score_summary = m.data


def on_phase_timeout(m):
    FS.phase_timeout = bool(m.data)


def _mk_drone_cb(idx, key):
    def cb(m):
        d = FS.drones[idx]
        with FS.lock:
            if key == "phase":
                if d["phase"] != m.data:   # 相位变化 = 新腿起点
                    d["leg_t0"], d["leg_wall"] = FS.sim_t, time.time()
                d["phase"] = m.data
            elif key == "crossed":
                d["crossed"] = bool(m.data)
            elif key == "at_drop":
                d["at_drop"] = bool(m.data)
            elif key == "odom":
                p = m.pose.pose.position
                t = m.twist.twist.linear
                d["x"], d["y"] = p.x, p.y
                d["v"] = (t.x * t.x + t.y * t.y + t.z * t.z) ** 0.5
            elif key == "goal":
                d["goal"] = (m.pose.position.x, m.pose.position.y)
            elif key == "metrics":
                a = m.data
                if len(a) >= 11:
                    d["col"], d["stuck"] = int(a[0]), a[1]
                    d["flips"], d["minclr"] = int(a[4]), a[6]
                    d["clr"] = a[10]
            elif key == "collision":
                if m.data:  # True=事件; latched 陈旧 True 由 reset_run 清
                    d["ev_col"] += 1
                    d["last_event"] = "COL-event"
    return cb


def on_match(idx):
    def cb(m):
        with FS.lock:
            FS.drones[idx]["match"] = m.data
    return cb


def on_score(idx):
    def cb(m):
        with FS.lock:
            FS.drones[idx]["score"] = getattr(m, "score", None)
    return cb


def on_rosout(m):
    text = m.msg
    for pat, kind in PATTERNS:
        mm = pat.search(text)
        if not mm:
            continue
        i = int(mm.group(1))
        i = min(max(i, 0), DRONES - 1)
        detail = kind
        with FS.lock:
            d = FS.drones[i]
            if kind == "FAILED":
                detail = "FAILED:%s" % mm.group(2)
            elif kind == "slot":
                detail = "slot(%.1f,%.1f) clr=%s" % (
                    float(mm.group(2)), float(mm.group(3)), mm.group(4))
            elif kind == "goto":
                detail = "goto %s(%.1f,%.1f)" % (
                    "slot" if mm.group(2).startswith("via") else "zone",
                    float(mm.group(3)), float(mm.group(4)))
            elif kind == "COL":
                d["ev_col"] += 1
                if len(mm.groups()) == 3:   # py: (i, x, y)
                    detail = "COL (%s,%s)" % (mm.group(2), mm.group(3))
                else:                        # gz: (i, n)
                    detail = "COL#%s" % mm.group(2)
            elif kind == "retry":
                d["ev_retry"] = int(mm.group(2))
            elif kind == "guard":
                pass
            elif kind == "drop":
                detail = "drop %s" % mm.group(2)
            elif kind == "STALL":
                d["ev_stall"] += 1
            elif kind == "DE":
                d["ev_de"] += 1
            elif kind == "GS":
                d["ev_gs"] += 1
            elif kind == "quiet":
                d["ev_quiet"] += 1
            elif kind == "fallback":
                d["ev_fb"] += 1
            elif kind == "done":
                detail = "DONE"
            elif kind == "crossed":
                detail = "crossed zone"
            if kind in ("goto", "slot", "crossed", "at_pad", "drop",
                        "FAILED", "done"):   # 腿级事件 = 新腿起点
                d["leg_t0"], d["leg_wall"] = FS.sim_t, time.time()
            d["last_event"] = detail
            FS.events.append((FS.sim_t, time.strftime("%H:%M:%S"),
                              "d%d %s" % (i, detail)))
        break


def snapshot():
    with FS.lock:
        ds = []
        for i in range(DRONES):
            d = dict(FS.drones[i])
            if d["leg_t0"] is not None and FS.sim_t is not None \
                    and FS.sim_t >= d["leg_t0"]:
                d["leg_t"] = FS.sim_t - d["leg_t0"]
            elif d["leg_wall"] is not None:
                d["leg_t"] = time.time() - d["leg_wall"]
            else:
                d["leg_t"] = None
            ds.append({"i": i, **d})
        return {
            "sim_t": FS.sim_t,
            "wall": time.strftime("%H:%M:%S"),
            "stage": FS.stage,
            "ready": FS.ready,
            "phase_timeout": FS.phase_timeout,
            "score_summary": FS.score_summary,
            "drones": ds,
            "events": list(FS.events)[-10:],
        }


def render(s):
    lines = []
    head = "zx2026 FLEET  sim_t=%s wall=%s stage=%s ready=%s timeout=%s" % (
        "%.1f" % s["sim_t"] if s["sim_t"] is not None else "--",
        s["wall"], s["stage"],
        "-" if s["ready"] is None else ("Y" if s["ready"] else "N"),
        "Y" if s["phase_timeout"] else "-")
    if s["score_summary"]:
        head += "  |  " + s["score_summary"]
    lines.append("== " + head + " ==")
    lines.append("%-3s %-15s %-16s %-5s %-16s %-4s %-6s %-5s %-7s %s" % (
        "d#", "phase", "pos(x,y)", "v", "goal(x,y)", "col", "stuck",
        "clr", "minclr", "last event / counters"))
    for d in s["drones"]:
        pos = "(%6.1f,%6.1f)" % (d["x"], d["y"]) if d["x"] is not None else "(--,--)"
        goal = ("(%6.1f,%5.1f)" % d["goal"]) if d["goal"] else "(--)"
        v = "%.2f" % d["v"] if d["v"] is not None else "--"
        lt = "--" if d["leg_t"] is None else "%.0f" % d["leg_t"]
        if d["leg_t"] is not None and d["leg_t"] > LEG_ALARM:
            lt = "!" + lt          # P4 长尾告警：单腿超时（慢蹭看门狗测不到）
        cnt = "legT %s | col%d stall%d de%d gs%d q%d r%d fb%d" % (
            lt, d["ev_col"], d["ev_stall"], d["ev_de"], d["ev_gs"],
            d["ev_quiet"], d["ev_retry"], d["ev_fb"])
        if d["score"] is not None:
            cnt = "score %s | %s" % (d["score"], cnt)
        lines.append("%-3d %-15s %-16s %-5s %-16s %-4s %-6s %-5s %-7s %s" % (
            d["i"], d["phase"], pos, v, goal,
            "--" if d["col"] is None else d["col"],
            fmt1(d["stuck"]), fmt1(d["clr"]), fmt1(d["minclr"]),
            d["last_event"] + "  [" + cnt + "]"))
    lines.append("-- events (newest last) --")
    for sim_t, wall, text in s["events"]:
        ts = "%.1f" % sim_t if sim_t is not None else "--"
        lines.append("  %8s  %s  %s" % (ts, wall, text))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--leg-alarm", type=float, default=90.0,
                    help="单腿超过该秒数在面板标 '!'（P4 长尾观测）")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--noline", action="store_true",
                    help="不清屏（tee 进文件时用）")
    ap.add_argument("--record", metavar="FILE", help="每帧追加 JSONL")
    args = ap.parse_args()
    global LEG_ALARM
    LEG_ALARM = args.leg_alarm

    # disable_rostime: 本工具不使用 rospy 时间（循环走墙钟，sim_t 取自 /clock
    # 载荷），跳过 use_sim_time 下等首个 /clock 的初始化窗口。
    rospy.init_node("fleet_monitor", anonymous=True, disable_rosout=True,
                    disable_rostime=True)

    rospy.Subscriber("/clock", Clock, on_clock, queue_size=2)
    rospy.Subscriber("/zx2026/state", String, on_state, queue_size=2)
    rospy.Subscriber("/zx2026/fleet_ready", Bool, on_ready, queue_size=2)
    rospy.Subscriber("/zx2026/score_summary", String, on_score_summary, queue_size=2)
    rospy.Subscriber("/zx2026/phase_timeout", Bool, on_phase_timeout, queue_size=2)
    rospy.Subscriber("/rosout", Log, on_rosout, queue_size=200)
    for i in range(DRONES):
        for key, typ, topic in (
                ("phase", String, "/drone_%d/mission/phase" % i),
                ("crossed", Bool, "/drone_%d/mission/crossed_zone" % i),
                ("at_drop", Bool, "/drone_%d/mission/at_drop" % i),
                ("odom", Odometry, "/drone_%d/odom" % i),
                ("goal", PoseStamped, "/drone_%d/planning/goal" % i),
                ("metrics", Float32MultiArray, "/drone_%d/nav_metrics" % i),
                ("collision", Bool, "/drone_%d/collision" % i)):
            rospy.Subscriber(topic, typ, _mk_drone_cb(i, key), queue_size=5)
        rospy.Subscriber("/drone_%d/match/result" % i, String,
                         on_match(i), queue_size=5)
        if Score is not None:
            rospy.Subscriber("/zx2026/score/%d" % i, Score, on_score(i), queue_size=2)

    rec = open(args.record, "a", encoding="ascii") if args.record else None

    if args.once:
        time.sleep(2.0)   # 让 latched 话题先灌进来（注册在负载下可慢）
        block = render(snapshot())
        print(block)
        sys.exit(0)

    period = 1.0 / max(args.rate, 0.1)
    while not rospy.is_shutdown():
        s = snapshot()
        block = render(s)
        if args.noline:
            print(block + "\n", flush=True)
        else:
            sys.stdout.write("\033[2J\033[H" + block + "\n")
            sys.stdout.flush()
        if rec:
            rec.write(json.dumps(s) + "\n")
            rec.flush()
        time.sleep(period)


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, OSError) as e:
        print("master gone / interrupted (end of run?) -- monitor exit: %r" % e)
    except SystemExit:
        raise
