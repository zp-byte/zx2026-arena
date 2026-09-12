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
import math
import socket
import socketserver
import threading
import time
from collections import deque

ACTIVE_PHASES = {"TAKEOFF", "EXECUTE", "RETURN", "GOTO", "MISSION"}
C_DIM, C_RED, C_YEL, C_GRN, C_RST = "\033[2m", "\033[31m", "\033[33m", \
    "\033[32m", "\033[0m"


def _num(v):
    """分值字段防御：TCP 透传来的畸形数据不炸 dash 主线程。"""
    return v if isinstance(v, (int, float)) else 0


class Hub(object):
    def __init__(self, ids, link_lost_s, stall_vel, stall_s, plan_dead_s,
                 logpath, view, status_path=None, bat_min=None,
                 bat_low_s=5.0, fence=None, scene_drops=None,
                 scene_pads=None, jump_thresh=3.0):
        self.ids = [str(i) for i in ids]
        self.link_lost_s = link_lost_s
        self.stall_vel = stall_vel
        self.stall_s = stall_s
        self.plan_dead_s = plan_dead_s
        self.view = view
        self.bat_min = bat_min        # None=关（仿真无电量）；real 建议开
        self.bat_low_s = bat_low_s
        self.fence = fence            # FR-3.5 {"x":(lo,hi),...} 或 None
        self.scene_drops = scene_drops  # FR-2.8 {drop_point_id:(x,y,z)} 或 None
        self.scene_pads = scene_pads  # FR-2.8 [(x,y),...] 起降 pad 或 None
        self.jump_thresh = jump_thresh  # FR-3.6 估计器跳变阈值(m)
        self.lock = threading.Lock()
        self.st = {i: {"data": None, "rx_t": 0.0, "stall_t0": None,
                       "stall_on": False, "lost_on": True,
                       "pdead_t0": None, "pdead_on": False,
                       "bat_t0": None, "bat_on": False, "grid": None,
                       "score": None, "mission": None,
                       # FR-3.4 FC 模式切换/OFFBOARD 丢失告警
                       "prev_fc": None, "offboard_on": False,
                       # FR-1.7 碰撞告警（sticky：碰撞即终结）
                       "coll_prev": 0, "coll_on": False,
                       # FR-3.5 围栏越界告警（sticky，恢复成对）
                       "fence_on": False,
                       # FR-3.6 估计器跳变（单次事件）
                       "prev_pos": None,
                       # FR-2.8 真机阶段推断
                       "ever_air": False, "inferred_phase": None}
                   for i in self.ids}
        self.conns = 0
        self.stage = None  # 全局阶段机（/zx2026/state，agent 上报）
        self.stage_t0 = None  # FR-1.6 阶段切换时刻（面板倒计时基准）
        self.tasks = {}          # did -> 任务进度事件（task_update，agent 透传）
        self.score_total = None  # 赛末总分 dict（score_summary 解析，agent 透传）
        self.events = deque(maxlen=400)
        self.logf = open(logpath, "a", encoding="utf-8")
        self.logpath = logpath
        self.status_path = status_path
        self.last_swfail = 0.0  # 写盘失败告警节流

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
            if obj.get("stage") is not None and obj["stage"] != self.stage:
                self.stage_t0 = now   # FR-1.6 倒计时基准：阶段切换时刻
                self.stage = obj["stage"]
            # 建图栅格 / 比分 / 任务指派（独立顶层键，1Hz 捎带）：
            # 只存最新，不进遥测 JSONL
            for did, g in (obj.get("grids") or {}).items():
                if did in self.st:
                    self.st[did]["grid"] = g
            for did, s in (obj.get("scores") or {}).items():
                # 入库形状校验：非 dict / score 非数值=畸形，丢弃不覆盖旧值
                if did in self.st and isinstance(s, dict) \
                        and isinstance(s.get("score"), (int, float)):
                    self.st[did]["score"] = s
            for did, m in (obj.get("missions") or {}).items():
                if did in self.st:
                    self.st[did]["mission"] = m
            for did, t in (obj.get("tasks") or {}).items():
                if did in self.st:
                    self.tasks[did] = t
            if obj.get("score_total"):
                self.score_total = obj["score_total"]
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

    # ---- FR-2.8 真机阶段推断 -----------------------------------------------
    def _infer_phase(self, d, pos, st):
        """无 phase 话题时以航迹邻近推断腿推进（替代 phase 供操作员盯屏）。

        判据：起降 pad 邻域 + 投放点邻域 + z 高度 + drop_done 标志。
        粗糙但给操作员"这架到哪一步了"的直觉，不用于控制。
        投放点取 st["mission"] 的参考指派（real 由 points.yaml 内部持有，
        mission 缺失时只按 pad/高度粗推）。
        """
        px, py, pz = pos[0], pos[1], pos[2]
        near_pad = False
        for pad in self.scene_pads:
            if math.hypot(px - pad[0], py - pad[1]) < 2.0:
                near_pad = True
                break
        near_drop = False
        drop_pt = (st.get("mission") or {}).get("drop")
        if drop_pt is not None and self.scene_drops:
            dp = self.scene_drops.get(int(drop_pt))
            if dp is not None and math.hypot(px - dp[0], py - dp[1]) < 2.0:
                near_drop = True
        if pz >= 0.5:
            st["ever_air"] = True
        if pz < 0.3 and near_pad:
            return "DONE" if st["ever_air"] else "IDLE"
        if pz >= 0.5:
            if near_drop:
                return "AT_DROP"
            if d.get("drop_done"):
                return "RETURN"
            if near_pad:
                return "TAKEOFF"
            return "EXECUTE"
        return "EXECUTE"

    # ---- 看门狗（1Hz）------------------------------------------------------
    def watchdog(self):
        while True:
            now = time.time()
            evs = []   # 事件在锁内只收集，锁外再落盘/打印（不持锁做 I/O）
            with self.lock:
                for did in self.ids:
                    st = self.st[did]
                    d = st["data"]
                    age = now - st["rx_t"] if st["rx_t"] else 1e9
                    # LINK LOST：边沿触发，恢复时也报（成对法证）
                    if age > self.link_lost_s and not st["lost_on"]:
                        st["lost_on"] = True
                        evs.append((did, "LINK_LOST", "age=%.1fs" % age))
                    elif age <= self.link_lost_s and st["lost_on"] and \
                            st["rx_t"] > 0:
                        st["lost_on"] = False
                        evs.append((did, "LINK_RECOVER", "age=%.1fs" % age))
                    if d is None or st["lost_on"]:
                        st["stall_t0"] = st["pdead_t0"] = None
                        st["stall_on"] = st["pdead_on"] = False
                        st["bat_t0"] = None
                        st["bat_on"] = False
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
                            evs.append((did, "STALL",
                                        "phase=%s v=%.3f pos=%s dur>=%ss"
                                        % (phase, speed,
                                           tuple(round(c, 2) for c in pos)
                                           if pos else "?", self.stall_s)))
                    else:
                        if st["stall_on"]:
                            evs.append((did, "STALL_CLEAR", "v=%.3f" % speed))
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
                            evs.append((did, "PLANNER_DEAD",
                                        "plan_age=%.1fs phase=%s"
                                        % (plan_age, phase)))
                    else:
                        if st["pdead_on"]:
                            evs.append((did, "PLANNER_OK",
                                        "plan_age=%.1fs" % plan_age))
                        st["pdead_t0"] = None
                        st["pdead_on"] = False
                    # LOW BAT（real 模式；sim 电量 null 自动跳过）：
                    # 持续低于门限才触发（瞬时压降不误报），恢复成对
                    bat = d.get("bat")
                    if self.bat_min is not None and bat is not None \
                            and bat < self.bat_min:
                        if st["bat_t0"] is None:
                            st["bat_t0"] = now
                        elif now - st["bat_t0"] > self.bat_low_s and \
                                not st["bat_on"]:
                            st["bat_on"] = True
                            evs.append((did, "LOW_BAT",
                                        "bat=%.0f%% < %.0f%%"
                                        % (bat, self.bat_min)))
                    else:
                        if st["bat_on"]:
                            evs.append((did, "BAT_OK", "bat=%.0f%%"
                                        % (bat if bat is not None else -1)))
                        st["bat_t0"] = None
                        st["bat_on"] = False
                    # FR-3.4 FC 模式切换 / OFFBOARD 丢失（real；sim fc=null 跳过）。
                    # OFFBOARD 丢失=人工接管或失控
                    fc = d.get("fc")
                    if fc is not None:
                        if st["prev_fc"] is not None and fc != st["prev_fc"]:
                            evs.append((did, "FC_MODE",
                                        "%s→%s" % (st["prev_fc"], fc)))
                        if active and fc != "OFFBOARD" \
                                and not st["offboard_on"]:
                            st["offboard_on"] = True
                            evs.append((did, "OFFBOARD_LOST", "mode=%s" % fc))
                        elif fc == "OFFBOARD" and st["offboard_on"]:
                            st["offboard_on"] = False
                            evs.append((did, "OFFBOARD_OK", "mode=OFFBOARD"))
                        st["prev_fc"] = fc
                    # FR-1.7 碰撞告警（nav_metrics 捎带，count 递增即触发，
                    # sticky 不清除——碰撞即终结，一级告警）
                    coll = d.get("coll")
                    if coll is not None and coll > st["coll_prev"]:
                        st["coll_on"] = True
                        evs.append((did, "COLLISION",
                                    "count=%d min_clr=%s"
                                    % (coll, d.get("min_clr"))))
                    st["coll_prev"] = coll or 0
                    # FR-3.5 围栏越界告警（sticky，恢复成对）
                    pos = d.get("pos")
                    if self.fence is not None and pos is not None:
                        fx, fy, fz = self.fence["x"], self.fence["y"], \
                            self.fence["z"]
                        outside = (pos[0] < fx[0] or pos[0] > fx[1]
                                   or pos[1] < fy[0] or pos[1] > fy[1]
                                   or pos[2] < fz[0] or pos[2] > fz[1])
                        if outside and not st["fence_on"]:
                            st["fence_on"] = True
                            evs.append((did, "FENCE_BREACH",
                                        "pos=(%.1f,%.1f,%.1f)" % tuple(pos)))
                        elif not outside and st["fence_on"]:
                            st["fence_on"] = False
                            evs.append((did, "FENCE_OK",
                                        "pos=(%.1f,%.1f,%.1f)" % tuple(pos)))
                    # FR-3.6 估计器跳变检测（VINS 输出位置突跳，单次事件）
                    if pos is not None and st["prev_pos"] is not None:
                        jump = math.sqrt(
                            (pos[0] - st["prev_pos"][0]) ** 2 +
                            (pos[1] - st["prev_pos"][1]) ** 2 +
                            (pos[2] - st["prev_pos"][2]) ** 2)
                        if jump > self.jump_thresh:
                            evs.append(
                                (did, "ESTIMATOR_JUMP",
                                 "(%.1f,%.1f,%.1f)→(%.1f,%.1f,%.1f) Δ=%.1fm"
                                 % (st["prev_pos"][0], st["prev_pos"][1],
                                    st["prev_pos"][2], pos[0], pos[1], pos[2],
                                    jump)))
                    if pos is not None:
                        st["prev_pos"] = pos
                    # FR-2.8 真机阶段推断（phase=null 时以航迹邻近推断腿推进）
                    if phase is None and self.scene_pads is not None \
                            and pos is not None:
                        st["inferred_phase"] = self._infer_phase(d, pos, st)
                    elif phase is not None:
                        st["inferred_phase"] = None
            for did, kind, detail in evs:
                self.event(did, kind, detail)
            time.sleep(1.0)

    # ---- 状态快照文件（1Hz 原子写，ops/第 3 步面板的数据源） ---------------
    # 地面站编排进程不进 ROS 图——从 hub 快照文件读遥测，铁律 ① 的延伸。
    def status_writer(self):
        while True:
            now = time.time()
            snap = {"hub_ts": round(now, 3), "stage": self.stage,
                    "stage_t0": self.stage_t0,
                    "conns": self.conns, "drones": {}}
            with self.lock:
                linked = 0
                for did in self.ids:
                    st = self.st[did]
                    d = st["data"]
                    age = now - st["rx_t"] if st["rx_t"] else None
                    if age is not None and age <= self.link_lost_s:
                        linked += 1
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
                        "pdead_on": st["pdead_on"], "bat_on": st["bat_on"],
                        # FR-3.4/1.7/3.5 告警态
                        "offboard_on": st["offboard_on"],
                        "coll_on": st["coll_on"], "fence_on": st["fence_on"],
                        # FR-1.7/1.8/1.9 数据透出（agent 捎带→面板）
                        "coll": d.get("coll") if d else None,
                        "min_clr": d.get("min_clr") if d else None,
                        "matched": d.get("matched") if d else None,
                        "drop_done": d.get("drop_done") if d else False,
                        "first_drop_t": d.get("first_drop_t") if d else None,
                        "match_progress": d.get("match_progress") if d
                        else None,
                        # FR-2.8 推断相位
                        "inferred_phase": st["inferred_phase"],
                    }
                evs = list(self.events)[-50:]
                grids = {did: self.st[did]["grid"] for did in self.ids
                         if self.st[did]["grid"] is not None}
                scores = {did: self.st[did]["score"] for did in self.ids
                          if self.st[did]["score"] is not None}
                missions = {did: self.st[did]["mission"]
                            for did in self.ids
                            if self.st[did]["mission"] is not None}
                tasks = dict(self.tasks)
                total = self.score_total
            snap["linked"] = linked   # 按数据年龄算的活链路数（比 TCP conns 真实）
            snap["events"] = evs
            snap["grids"] = grids
            snap["scores"] = scores
            snap["missions"] = missions
            if tasks:
                snap["tasks"] = tasks
            if total:
                snap["score_total"] = total
            try:
                tmp = self.status_path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(snap, f, ensure_ascii=False)
                import os
                os.replace(tmp, self.status_path)
            except Exception as e:
                # 写盘失败必须可见（"石化 status 文件"教训的代码层闭环）：
                # 节流 10s 一条进事件流/日志，绝不静默
                if now - self.last_swfail > 10.0:
                    self.last_swfail = now
                    try:
                        self.event("hub", "STATUS_WRITE_FAIL", str(e)[:120])
                    except Exception:
                        print("[hub] status write FAIL: %s" % e, flush=True)
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
                    if st["bat_on"]:
                        alarms.append("d%s LOW_BAT" % did)
                    if st["offboard_on"]:
                        col, _ = C_RED, alarms.append(
                            "d%s OFFBOARD_LOST" % did)
                    if st["coll_on"]:
                        col, _ = C_RED, alarms.append("d%s COLLISION" % did)
                    if st["fence_on"]:
                        col, _ = C_RED, alarms.append("d%s FENCE_BREACH" % did)
                    lines.append(" d%-2s %s|%-8s|bat %-4s|%-9s pos %-16s "
                                 "v %-5s plan %-6s%s"
                                 % (did, col, ph[:8], bat, "LINK",
                                    pos, spd, pa, C_RST))
                if alarms:
                    lines.append(" ALARM: " + " | ".join(alarms))
                sc = [(did, self.st[did]["score"]) for did in self.ids
                      if self.st[did]["score"]]
                if sc:
                    tot = sum(_num((s or {}).get("score")) for _, s in sc)
                    cor = sum(_num((s or {}).get("correct")) for _, s in sc)
                    wrg = sum(_num((s or {}).get("wrong")) for _, s in sc)
                    lines.append(" SCORE total=%d correct=%d wrong=%d  %s"
                                 % (tot, cor, wrg,
                                    " ".join("d%s:%s" % (d, (s or {})
                                                        .get("score"))
                                             for d, s in sc)))
                lines.append(" ---- events ----")
                for ev in list(self.events)[-8:]:
                    lines.append(" %s d%s %s %s"
                                 % (time.strftime("%H:%M:%S",
                                                  time.localtime(ev["t"])),
                                    ev["drone"], ev["event"], ev["detail"]))
            print("\033[H\033[J" + "\n".join(lines), flush=True)
            time.sleep(0.5)


class Handler(socketserver.StreamRequestHandler):
    def setup(self):
        # TCP keepalive：半开连接（agent 断电/拔线无 FIN）由内核探活回收，
        # 否则 handler 线程与 conns 计数虚高。
        # 注意用 self.request——self.connection 要父类 setup() 才存在；
        # 且异常必须兜宽（AttributeError 漏网=每个连接建立即死，E2E 实弹抓到）
        try:
            sock = self.request
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 30)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        except Exception:
            pass
        super().setup()

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


class Srv(socketserver.ThreadingTCPServer):
    # allow_reuse_address 必须是类属性（bind 前读取）——曾写成实例属性在
    # bind 之后才设=无效，hub 快速重启偶发 EADDRINUSE
    allow_reuse_address = True
    daemon_threads = True


def _parse_scene(path, fence_margin=0.3):
    """FR-3.5/2.8：从 scene_topology.yaml 解析围栏内边界、投放点、起降 pad。

    返回 (fence, drops, pads)；解析失败返回 (None, None, None)。
    """
    import yaml
    try:
        with open(path, encoding="utf-8") as f:
            sc = yaml.safe_load(f)
    except Exception:
        return None, None, None
    # 围栏内边界：取所有 kind=fence 的 AABB 内侧求并集，内缩 margin 防贴栏误报
    fences = [o for o in sc.get("static_obstacles", [])
              if o.get("kind") == "fence"]
    if fences:
        x_lo = min(float(o["lo"][0]) for o in fences)
        x_hi = max(float(o["hi"][0]) for o in fences)
        y_lo = min(float(o["lo"][1]) for o in fences)
        y_hi = max(float(o["hi"][1]) for o in fences)
        z_hi = float(sc.get("venue", {}).get("size", [0, 0, 6])[2])
        fence = {"x": (x_lo + fence_margin, x_hi - fence_margin),
                 "y": (y_lo + fence_margin, y_hi - fence_margin),
                 "z": (0.0, z_hi - fence_margin)}
    else:
        fence = None
    # 投放点 id→xyz
    drops = {}
    for dp in sc.get("drop_points", []):
        drops[int(dp["id"])] = tuple(float(c) for c in dp["xyz"])
    # 起降 pad 坐标
    pads = []
    for z in sc.get("zones", []):
        if z.get("kind") == "pad" and z.get("pad_origin"):
            ox, oy = float(z["pad_origin"][0]), float(z["pad_origin"][1])
            sx, sy = float(z["pad_spacing"][0]), float(z["pad_spacing"][1])
            for k in range(int(z.get("pads", 6))):
                pads.append((ox + k * sx, oy + k * sy))
    return fence, drops, pads


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
    ap.add_argument("--bat-min", type=float, default=None,
                    help="LOW_BAT 门限（%%，缺省=关；真机建议 30）")
    ap.add_argument("--bat-low-s", type=float, default=5.0)
    ap.add_argument("--scene", default=None,
                    help="scene_topology.yaml 路径→启用围栏越界(FR-3.5)+"
                         "阶段推断(FR-2.8)；缺省=关")
    ap.add_argument("--fence-margin", type=float, default=0.3,
                    help="围栏内缩余量(m)")
    ap.add_argument("--jump-thresh", type=float, default=3.0,
                    help="估计器跳变阈值(m)，FR-3.6")
    args = ap.parse_args()
    logpath = "%s/gcs_telem_%s.jsonl" % (args.logdir,
                                         time.strftime("%Y%m%d_%H%M%S"))
    status_path = args.status_file or "%s/gcs_status.json" % args.logdir
    fence, drops, pads = None, None, None
    if args.scene:
        fence, drops, pads = _parse_scene(args.scene, args.fence_margin)
    hub = Hub(args.ids.split(","), args.link_lost, args.stall_vel,
              args.stall_s, args.plan_dead, logpath, args.view,
              status_path=status_path, bat_min=args.bat_min,
              bat_low_s=args.bat_low_s, fence=fence, scene_drops=drops,
              scene_pads=pads, jump_thresh=args.jump_thresh)
    srv = Srv(("0.0.0.0", args.port), Handler)
    srv.hub = hub
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
