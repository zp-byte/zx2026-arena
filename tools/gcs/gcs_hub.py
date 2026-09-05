#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gcs_hub.py — 非凸α 地面站聚合枢纽（GCS v0 第 1 步）。

职责：
  1. TCP server 收纳全部 agent 的 JSON 行遥测流
  2. 看门狗（数据年龄判死活，不信 TCP）：LINK LOST / STALL / PLANNER DEAD
  3. 全量遥测 + 事件落 JSONL（法证日志，落地后可复盘）
  4. ANSI 六格仪表（--view dash）或纯日志模式（--view log，后台常开）

告警判据（全部仿真法证武器移植）：
  LINK LOST     遥测年龄 > link_lost_s（默认 3s）
  STALL         phase 活跃 且 speed<stall_vel 持续 > stall_s（仿真同款判据）
  PLANNER DEAD  phase 活跃 且 liveness 流年龄 > plan_dead_s

用法：
  python3 gcs_hub.py --view dash            # 前台仪表
  python3 gcs_hub.py --view log             # 后台纯日志
"""
import argparse
import json
import socketserver
import threading
import time
from collections import deque

ACTIVE_PHASES = {"TAKEOFF", "EXECUTE", "RETURN", "GOTO", "MISSION"}
C_DIM, C_RED, C_YEL, C_GRN, C_RST = "\033[2m", "\033[31m", "\033[33m", \
    "\033[32m", "\033[0m"


class Hub(object):
    def __init__(self, ids, link_lost_s, stall_vel, stall_s, plan_dead_s,
                 logpath, view, status_path=None):
        self.ids = [str(i) for i in ids]
        self.link_lost_s = link_lost_s
        self.stall_vel = stall_vel
        self.stall_s = stall_s
        self.plan_dead_s = plan_dead_s
        self.view = view
        self.lock = threading.Lock()
        self.st = {i: {"data": None, "rx_t": 0.0, "stall_t0": None,
                       "stall_on": False, "lost_on": True,
                       "pdead_t0": None, "pdead_on": False}
                   for i in self.ids}
        self.conns = 0
        self.stage = None  # 全局阶段机（/zx2026/state，agent 上报）
        self.events = deque(maxlen=400)
        self.logf = open(logpath, "a", encoding="utf-8")
        self.logpath = logpath
        self.status_path = status_path

    # ---- 事件与日志 --------------------------------------------------------
    def event(self, did, kind, detail):
        rec = {"t": round(time.time(), 3), "drone": did, "event": kind,
               "detail": detail}
        self.events.append(rec)
        self.logf.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.logf.flush()
        print("[EVT] %s d%s %s %s" % (time.strftime("%H:%M:%S",
                                                    time.localtime(rec["t"])),
                                      did, kind, detail), flush=True)

    # ---- 遥测入库 ----------------------------------------------------------
    def on_msg(self, obj):
        now = time.time()
        recs = []
        with self.lock:
            if obj.get("stage") is not None:
                self.stage = obj["stage"]
            for did, d in obj.get("drones", {}).items():
                if did not in self.st:
                    continue
                st = self.st[did]
                st["data"] = d
                st["rx_t"] = now
                recs.append({"t": round(now, 3), "drone": did,
                             "event": "TELEM", "d": d})
        for r in recs:
            self.logf.write(json.dumps(r, ensure_ascii=False) + "\n")
        if recs:
            self.logf.flush()

    # ---- 看门狗（1Hz）------------------------------------------------------
    def watchdog(self):
        while True:
            now = time.time()
            with self.lock:
                for did in self.ids:
                    st = self.st[did]
                    d = st["data"]
                    age = now - st["rx_t"] if st["rx_t"] else 1e9
                    # LINK LOST：边沿触发，恢复时也报（成对法证）
                    if age > self.link_lost_s and not st["lost_on"]:
                        st["lost_on"] = True
                        self.event(did, "LINK_LOST", "age=%.1fs" % age)
                    elif age <= self.link_lost_s and st["lost_on"] and \
                            st["rx_t"] > 0:
                        st["lost_on"] = False
                        self.event(did, "LINK_RECOVER", "age=%.1fs" % age)
                    if d is None or st["lost_on"]:
                        st["stall_t0"] = st["pdead_t0"] = None
                        st["stall_on"] = st["pdead_on"] = False
                        continue
                    phase = d.get("phase")
                    active = phase in ACTIVE_PHASES
                    speed = float(d.get("speed") or 0.0)
                    # STALL：规划健康+机体不动（W3/W5 同款签名）
                    if active and speed < self.stall_vel:
                        if st["stall_t0"] is None:
                            st["stall_t0"] = now
                        elif now - st["stall_t0"] > self.stall_s and \
                                not st["stall_on"]:
                            st["stall_on"] = True
                            pos = d.get("pos")
                            self.event(did, "STALL",
                                       "phase=%s v=%.3f pos=%s dur>=%ss"
                                       % (phase, speed,
                                          tuple(round(c, 2) for c in pos)
                                          if pos else "?", self.stall_s))
                    else:
                        if st["stall_on"]:
                            self.event(did, "STALL_CLEAR",
                                       "v=%.3f" % speed)
                        st["stall_t0"] = None
                        st["stall_on"] = False
                    # PLANNER DEAD：任务活跃但轨迹/规划流断
                    plan_age = d.get("plan_age", -1.0)
                    if active and (plan_age < 0 or plan_age >
                                   self.plan_dead_s):
                        if st["pdead_t0"] is None:
                            st["pdead_t0"] = now
                        elif now - st["pdead_t0"] > self.plan_dead_s and \
                                not st["pdead_on"]:
                            st["pdead_on"] = True
                            self.event(did, "PLANNER_DEAD",
                                       "plan_age=%.1fs phase=%s"
                                       % (plan_age, phase))
                    else:
                        if st["pdead_on"]:
                            self.event(did, "PLANNER_OK",
                                       "plan_age=%.1fs" % plan_age)
                        st["pdead_t0"] = None
                        st["pdead_on"] = False
            time.sleep(1.0)

    # ---- 状态快照文件（1Hz 原子写，ops/第 3 步面板的数据源） ---------------
    # 地面站编排进程不进 ROS 图——从 hub 快照文件读遥测，铁律 ① 的延伸。
    def status_writer(self):
        while True:
            now = time.time()
            snap = {"hub_ts": round(now, 3), "stage": self.stage,
                    "conns": self.conns, "drones": {}}
            with self.lock:
                for did in self.ids:
                    st = self.st[did]
                    d = st["data"]
                    age = now - st["rx_t"] if st["rx_t"] else None
                    snap["drones"][did] = {
                        "age": round(age, 2) if age is not None else None,
                        "phase": d.get("phase") if d else None,
                        "pos": d.get("pos") if d else None,
                        "speed": d.get("speed") if d else None,
                        "plan_age": d.get("plan_age") if d else None,
                        "bat": d.get("bat") if d else None,
                        "fc": d.get("fc") if d else None,
                        "connected": d.get("connected") if d else None,
                        "lost_on": st["lost_on"], "stall_on": st["stall_on"],
                        "pdead_on": st["pdead_on"],
                    }
                evs = list(self.events)[-50:]
            snap["events"] = evs
            try:
                tmp = self.status_path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(snap, f, ensure_ascii=False)
                import os
                os.replace(tmp, self.status_path)
            except Exception:
                pass
            time.sleep(1.0)

    # ---- ANSI 六格仪表（2Hz）----------------------------------------------
    def dash(self):
        while True:
            now = time.time()
            lines = []
            alarms = []
            with self.lock:
                hdr = ("== fei tu A GCS hub  %s  conns=%d  log=%s =="
                       % (time.strftime("%H:%M:%S"), self.conns,
                          self.logpath.rsplit("/", 1)[-1]))
                lines.append(hdr)
                for did in self.ids:
                    st = self.st[did]
                    d = st["data"]
                    age = now - st["rx_t"] if st["rx_t"] else None
                    if d is None or age is None or age > self.link_lost_s:
                        col, bat, ph = C_RED, "--", "NO LINK"
                        pos, spd, pa = "--", "--", "--"
                        if not st["lost_on"]:
                            col = C_DIM
                    else:
                        col = C_GRN if age < self.link_lost_s else C_RED
                        bat = ("%.0f%%" % d["bat"]) if d.get("bat") is \
                            not None else "--"
                        ph = d.get("phase") or "--"
                        pos = ("(%.1f,%.1f,%.1f)" % tuple(d["pos"])) \
                            if d.get("pos") else "--"
                        spd = "%.2f" % (d.get("speed") or 0.0)
                        pa = "%.1f" % d.get("plan_age", -1.0) \
                            if d.get("plan_age", -1) >= 0 else "never"
                    if st["stall_on"]:
                        col, _ = C_RED, alarms.append(
                            "d%s STALL" % did)
                    elif st["pdead_on"]:
                        col, _ = C_YEL, alarms.append(
                            "d%s PLANNER_DEAD" % did)
                    lines.append(" d%-2s %s|%-8s|bat %-4s|%-9s pos %-16s "
                                 "v %-5s plan %-6s%s"
                                 % (did, col, ph[:8], bat, "LINK",
                                    pos, spd, pa, C_RST))
                if alarms:
                    lines.append(" ALARM: " + " | ".join(alarms))
                lines.append(" ---- events ----")
                for ev in list(self.events)[-8:]:
                    lines.append(" %s d%s %s %s"
                                 % (time.strftime("%H:%M:%S",
                                                  time.localtime(ev["t"])),
                                    ev["drone"], ev["event"], ev["detail"]))
            print("\033[H\033[J" + "\n".join(lines), flush=True)
            time.sleep(0.5)


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        hub = self.server.hub
        with hub.lock:
            hub.conns += 1
        try:
            for line in self.rfile:
                line = line.strip()
                if not line:
                    continue
                try:
                    hub.on_msg(json.loads(line.decode("ascii")))
                except Exception:
                    continue
        finally:
            with hub.lock:
                hub.conns -= 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="0,1,2,3,4,5")
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--view", choices=["dash", "log"], default="dash")
    ap.add_argument("--link-lost", type=float, default=3.0)
    ap.add_argument("--stall-vel", type=float, default=0.05)
    ap.add_argument("--stall-s", type=float, default=5.0)
    ap.add_argument("--plan-dead", type=float, default=5.0)
    ap.add_argument("--logdir", default="/home/ubuntu/zx2026_arena_ws/run_logs")
    ap.add_argument("--status-file", default=None,
                    help="1Hz 快照 JSON（默认 <logdir>/gcs_status.json）")
    args = ap.parse_args()
    logpath = "%s/gcs_telem_%s.jsonl" % (args.logdir,
                                         time.strftime("%Y%m%d_%H%M%S"))
    status_path = args.status_file or "%s/gcs_status.json" % args.logdir
    hub = Hub(args.ids.split(","), args.link_lost, args.stall_vel,
              args.stall_s, args.plan_dead, logpath, args.view,
              status_path=status_path)
    srv = socketserver.ThreadingTCPServer(("0.0.0.0", args.port), Handler)
    srv.hub = hub
    srv.allow_reuse_address = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=hub.watchdog, daemon=True).start()
    threading.Thread(target=hub.status_writer, daemon=True).start()
    print("[hub] up port=%d ids=%s log=%s status=%s"
          % (args.port, args.ids, logpath, status_path), flush=True)
    if args.view == "dash":
        print("\033[2J", end="", flush=True)
        hub.dash()
    else:
        while True:
            time.sleep(60)


if __name__ == "__main__":
    main()
