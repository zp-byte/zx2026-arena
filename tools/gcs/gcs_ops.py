#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gcs_ops.py — 非凸α 地面站操作编排器（GCS v0 第 2 步，mode 感知）。

健康门（全部从 hub 状态快照判据，本进程不进 ROS 图——铁律①延伸）：
  sim  模式（mode: sim，默认）：
    PREFLIGHT   全部 agent 遥测新鲜（age < link_lost_s）
    POSITIONING 全部机 odom 有限值 且 静止（|v| < max_vel）
    AUTONOMY    自主栈活着（sim: 相位流在发；real: 规划流 plan_age 新鲜）
    READY       前序门全绿 + 阶段机处于起飞前状态（P2_WAIT）→ 允许 TRIGGER
  real 模式（mode: real）在 POSITIONING 与 AUTONOMY 之间加两门：
    POWER       全部机 bat >= bat_min（电量门，real 缺省 30%）
    FC          全部机 mavros connected（FC 门）

mode: real 的附加保护：
  start 流程 = 四门绿 → 逐机错峰 takeoff → 离地确认(z >= airborne_z)
              → trigger（模板为 null 则跳过=任务自启）→ 盯飞
  start/takeoff/back/land 必须显式 --yes（面板两段确认后代为传参）；
  panic 豁免——急停链路永不设确认障碍。

命令层（模板全在 profile，仿真=rosservice，真机=ssh，本文件不含一处地址）：
  status / preflight / start(TRIGGER) / takeoff / back / land / panic / debug
  错峰起飞 stagger_s + 单机重试 retries；panic=land-all 逐机必达不中止，
  结果逐机汇报（未降落者列出+给人工介入提示，rc=1）。
  debug=真机 ssh 调试接口（ops.debug_ssh 通道 + ops.debug_cmds 命名只读命令，
  面板 DEBUG 窗同源）；preflight 真机先跑 ops.probe 探针（不阻断门）。
  real start 附加保护：空中(z>=airborne_z)拒绝重复 start（--force 才放行）；
  离地确认失败自动对已离地机 land 回滚。

用法：
  python3 gcs_ops.py --profile profile_sim.yaml preflight
  python3 gcs_ops.py --profile profile_sim.yaml start
  python3 gcs_ops.py --profile profile_real_lio.yaml panic
"""
import argparse
import json
import math
import os
import shlex
import subprocess
import time

import yaml

C_DIM, C_RED, C_YEL, C_GRN, C_RST = "\033[2m", "\033[31m", "\033[33m", \
    "\033[32m", "\033[0m"
STAGES = ["PREFLIGHT", "POSITIONING", "AUTONOMY", "READY"]
STAGES_REAL = ["PREFLIGHT", "POSITIONING", "POWER", "FC", "AUTONOMY",
               "READY"]


class Ops(object):
    def __init__(self, profile_path):
        with open(profile_path, encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)
        self.ids = [str(i) for i in self.cfg["ids"]]
        ops = self.cfg.get("ops", {}) or {}
        self.status_file = ops.get("status_file")
        self.shell_prefix = ops.get("shell_prefix", "")
        self.cmds = ops.get("cmds", {}) or {}
        self.stagger_s = float(ops.get("stagger_s", 2.0))
        self.retries = int(ops.get("retries", 2))
        g = ops.get("gates", {}) or {}
        self.link_lost_s = float(g.get("link_lost_s", 3.0))
        self.max_vel = float(g.get("max_vel", 0.10))
        self.plan_dead_s = float(g.get("plan_dead_s", 5.0))
        self.autonomy_mode = g.get("autonomy", "mission")
        self.pre_start_stages = g.get("pre_start_stages", ["P2_WAIT"])
        # mode 一级开关：sim（默认）/ real —— real 加 POWER/FC 门、
        # start 起飞先行、危险命令 --yes
        self.mode = str(self.cfg.get("mode", "sim")).lower()
        self.stages = list(STAGES_REAL if self.mode == "real" else STAGES)
        self.bat_min = g.get("bat_min")
        if self.mode == "real" and self.bat_min is None:
            self.bat_min = 30.0  # real 缺省电量门（POWER）
        self.airborne_z = float(g.get("airborne_z", 0.5))
        self.airborne_timeout_s = float(g.get("airborne_timeout_s", 60.0))
        self.execute_timeout_s = float(ops.get("execute_timeout_s", 300.0))
        self.terminal_phases = set(ops.get("terminal_phases",
                                           ["DONE", "FAILED"]))
        # 真机调试接口（ssh）：debug_ssh=通道模板({id})，debug_cmds=命名
        # 远端命令（约定只读；{args} 可选参数槽），probe=preflight 探针名单
        self.debug_ssh = ops.get("debug_ssh")
        self.debug_cmds = ops.get("debug_cmds", {}) or {}
        self.probe = ops.get("probe", []) or []
        self.last_results = []  # 最近一次 dispatch 的 (id, ok) 明细
        logdir = ops.get("logdir", "/home/ubuntu/zx2026_arena_ws/run_logs")
        self.logpath = "%s/gcs_ops_%s.jsonl" % (
            logdir, time.strftime("%Y%m%d_%H%M%S"))
        self.logf = open(self.logpath, "a", encoding="utf-8")
        self.stage_topic = (self.cfg.get("topics", {}) or {}).get("stage")

    # ---- 日志与快照 --------------------------------------------------------
    def log(self, kind, **kw):
        rec = {"t": round(time.time(), 3), "event": kind}
        rec.update(kw)
        self.logf.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.logf.flush()

    def read_status(self):
        try:
            with open(self.status_file, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    # ---- 四段健康门 --------------------------------------------------------
    def gates(self, snap):
        """返回 (每段(ok, 原因) 有序字典, 当前未过段名)。snap 可为 None。"""
        res = {}
        drones = (snap or {}).get("drones", {})
        stage = (snap or {}).get("stage")
        # PREFLIGHT：遥测新鲜
        bad = [i for i in self.ids
               if drones.get(i, {}).get("age") is None
               or drones[i]["age"] > self.link_lost_s]
        res["PREFLIGHT"] = (not bad,
                            "linked" if not bad else "no/fresh telem: %s"
                            % ",".join(bad))
        # POSITIONING：odom 有限 + 静止
        bad = []
        for i in self.ids:
            d = drones.get(i) or {}
            pos, v = d.get("pos"), d.get("speed")
            if not pos or not all(math.isfinite(c) for c in pos):
                bad.append("%s odom?" % i)
            elif v is None or v > self.max_vel:
                bad.append("%s v=%.2f" % (i, v if v is not None else -1))
        res["POSITIONING"] = (not bad, "settled" if not bad
                              else "; ".join(bad[:3]))
        # POWER：电量门（仅 real；sim 电量 null 无此门）
        if self.mode == "real":
            bad = []
            for i in self.ids:
                bat = (drones.get(i) or {}).get("bat")
                if bat is None or bat < self.bat_min:
                    bad.append("%s bat=%s" % (i, "--" if bat is None
                                              else "%.0f%%" % bat))
            res["POWER"] = (not bad, "ok" if not bad
                            else "; ".join(bad[:3]))
            # FC：mavros 连接门（仅 real）
            bad = [i for i in self.ids
                   if (drones.get(i) or {}).get("connected") is not True]
            res["FC"] = (not bad, "mavros ok" if not bad
                         else "fc not connected: %s" % ",".join(bad))
        # AUTONOMY：自主栈活着
        bad = []
        for i in self.ids:
            d = drones.get(i) or {}
            if self.autonomy_mode == "plan":
                pa = d.get("plan_age")
                if pa is None or pa < 0 or pa > self.plan_dead_s:
                    bad.append("%s plan_age=%s" % (i, pa))
            else:  # mission：相位流在发
                if d.get("phase") is None:
                    bad.append("%s phase?" % i)
        res["AUTONOMY"] = (not bad, "alive(%s)" % self.autonomy_mode
                           if not bad else "; ".join(bad[:3]))
        # READY：前序门全绿 + 起飞前阶段
        okpre = all(res[s][0] for s in self.stages[:-1])
        if not okpre:
            res["READY"] = (False, "gates above")
        elif self.stage_topic is None:
            res["READY"] = (True, "stage topic n/a (TODO 真机)")
        elif stage in self.pre_start_stages:
            res["READY"] = (True, "stage=%s" % stage)
        else:
            res["READY"] = (False, "stage=%s not in %s"
                            % (stage, self.pre_start_stages))
        cur = next((s for s in self.stages if not res[s][0]), "READY")
        return res, cur

    def print_table(self, snap, res, cur):
        drones = (snap or {}).get("drones", {})
        print("== GCS OPS  %s  stage=%s conns=%s ==" % (
            time.strftime("%H:%M:%S"), (snap or {}).get("stage") or "?",
            (snap or {}).get("conns", "?")))
        for i in self.ids:
            d = drones.get(i) or {}
            age, ph = d.get("age"), d.get("phase") or "--"
            pos = ("(%.1f,%.1f,%.1f)" % tuple(d["pos"])) if d.get("pos") \
                else "--"
            v = d.get("speed")
            pa = d.get("plan_age")
            col = C_GRN if age is not None and age <= self.link_lost_s \
                else C_RED
            print(" d%-2s %sage %-5s|%-9s|%-16s v %-5s plan %-6s%s"
                  % (i, col, "%.1f" % age if age is not None else "--",
                     ph[:8], pos,
                     "%.2f" % v if v is not None else "--",
                     "%.1f" % pa if pa is not None and pa >= 0 else "never",
                     C_RST))
        for s in self.stages:
            ok, why = res[s]
            mark = C_GRN + "[OK]" + C_RST if ok else C_YEL + "[--]" + C_RST
            print(" GATE %-11s %s %s" % (s, mark, why))
        print(" STAGE -> %s" % cur)

    def preflight(self, timeout_s):
        if self.mode == "real" and self.probe:
            self._probe_real()
        t0 = time.time()
        while True:
            snap = self.read_status()
            res, cur = self.gates(snap)
            print("\033[H\033[J", end="")
            self.print_table(snap, res, cur)
            if cur == "READY":
                print(" ALL GATES GREEN — cleared to `start`.")
                self.log("PREFLIGHT_OK")
                return 0
            if time.time() - t0 > timeout_s:
                print(" PREFLIGHT TIMEOUT (%.0fs) at %s" % (timeout_s, cur))
                self.log("PREFLIGHT_TIMEOUT", stage=cur)
                return 1
            time.sleep(1.0)

    # ---- 命令层 ------------------------------------------------------------
    def _run(self, cmd, did=None, timeout=15.0):
        # 用 replace 不用 .format：命令串（debug 拼装/模板）可能含字面花括号
        # （awk '{print $1}' 等），.format 直接 KeyError；也杜绝把已组装好的
        # 调试命令二次 format（{args} 里再有 {id} 被吞的暗坑）
        full = self.shell_prefix + cmd.replace(
            "{id}", did if did is not None else "")
        try:
            p = subprocess.run(
                ["bash", "-c", full], capture_output=True, text=True,
                timeout=timeout)
            return p.returncode, (p.stdout or "")[-300:], \
                (p.stderr or "")[-300:]
        except Exception as e:
            return -1, "", str(e)

    def dispatch(self, action, ids=None):
        """模板含 {id}=逐机错峰+重试；不含=全局一次。返回全成与否。

        last_results 三态：None=未执行（模板缺失早退——调用方据实报
        「未送达」而非假成功）；[(None, ok)]=全局模板一次广播；
        [(id, ok), ...]=逐机明细。
        """
        self.last_results = None
        tmpl = self.cmds.get(action)
        if tmpl is None and action == "panic":
            tmpl = self.cmds.get("land")  # panic 走 panic→land 模板链
        if tmpl is None:
            print(" [%s] not configured in this profile (sim: "
                  "stage_controller auto-drives; real: fill ops.cmds.%s)"
                  % (action, action))
            return False
        if "{id}" not in tmpl:
            rc, out, err = self._run(tmpl)
            ok = rc == 0
            self.last_results = [(None, ok)]
            print(" [%s] rc=%d %s" % (action, rc, (out or err).strip()
                                      .replace("\n", " ")[:120]))
            self.log("CMD", action=action, rc=rc, out=out.strip()[:200],
                     err=err.strip()[:200])
            return ok
        ids = ids if ids is not None else self.ids
        all_ok = True
        self.last_results = []
        for n, i in enumerate(ids):
            if n:
                time.sleep(self.stagger_s)
            ok = False
            for attempt in range(1, self.retries + 1):
                rc, out, err = self._run(tmpl, did=i)
                self.log("CMD", action=action, drone=i, attempt=attempt,
                         rc=rc, out=out.strip()[:200], err=err.strip()[:200])
                if rc == 0:
                    ok = True
                    break
                print("  d%s %s attempt %d/%d rc=%d %s"
                      % (i, action, attempt, self.retries, rc,
                         (err or out).strip().replace("\n", " ")[:100]))
            mark = C_GRN + "OK" + C_RST if ok else C_RED + "FAIL" + C_RST
            print(" d%s %s %s" % (i, action, mark))
            self.last_results.append((i, ok))
            all_ok = all_ok and ok
        return all_ok

    # ---- 离地确认（real 起飞先行流程）---------------------------------------
    @staticmethod
    def _z_of(drones, i):
        p = (drones.get(i) or {}).get("pos")
        if p and len(p) > 2 and all(math.isfinite(c) for c in p):
            return p[2]
        return None

    def _airborne_ids(self, drones=None):
        """z >= airborne_z 的机列表（数据源=hub 快照；误发保护/回滚共用）。"""
        if drones is None:
            drones = (self.read_status() or {}).get("drones", {})
        return [i for i in self.ids
                if (self._z_of(drones, i) or -1e9) >= self.airborne_z]

    def wait_airborne(self):
        """全部机 z >= airborne_z 才算离地；超时=失败（宁可不起飞）。"""
        print(" airborne confirm: z >= %.1fm, timeout %.0fs"
              % (self.airborne_z, self.airborne_timeout_s))
        t0 = time.time()
        while True:
            drones = (self.read_status() or {}).get("drones", {})
            up = self._airborne_ids(drones)
            ph = " ".join("%s:%s" % (i, "%.1f" % self._z_of(drones, i)
                                     if self._z_of(drones, i) is not None
                                     else "--")
                          for i in self.ids)
            print("  [%3.0fs] z %s (%d/%d up)"
                  % (time.time() - t0, ph, len(up), len(self.ids)))
            if len(up) == len(self.ids):
                print(" AIRBORNE — all %d drones up." % len(self.ids))
                self.log("AIRBORNE_OK")
                return True
            if time.time() - t0 > self.airborne_timeout_s:
                self.log("AIRBORNE_TIMEOUT", up=up)
                return False
            time.sleep(1.0)

    # ---- start = TRIGGER + 盯飞 --------------------------------------------
    def start(self, force, monitor_timeout):
        snap = self.read_status()
        res, cur = self.gates(snap)
        self.print_table(snap, res, cur)
        if cur != "READY" and not force:
            print(" NOT READY (%s) — preflight first, or --force." % cur)
            return 1
        self.log("START", force=force, mode=self.mode)
        if self.mode == "real":
            # 真机：起飞先行 → 离地确认 → 再触发任务（sim 直接 trigger）
            flying = self._airborne_ids()
            if flying and not force:
                print(" REFUSED: d%s 已在空中(z>=%.1fm) — 疑似任务中途重复 "
                      "start（READY 门在真机无 stage 话题时放行，此处兜底）；"
                      "续飞用 back/land，确要重发加 --force"
                      % (",".join(flying), self.airborne_z))
                self.log("START_REFUSED_AIRBORNE", flying=flying)
                return 1
            print(" REAL MODE: staggered takeoff first.")
            if not self.dispatch("takeoff"):
                print(" takeoff dispatch failed — abort before trigger.")
                self.log("TAKEOFF_FAIL")
                return 1
            if not self.wait_airborne():
                # 回滚：已离地的机就地召回（不给半空机群触任务）
                up = self._airborne_ids()
                self.log("AIRBORNE_FAIL", up=up)
                if up:
                    print(" airborne confirm FAILED — 已离地 d%s 就地 land "
                          "回滚（任务不触发）" % ",".join(up))
                    self.log("AIRBORNE_ROLLBACK", up=up)
                    if not self.dispatch("land", ids=up):
                        # 回滚失败必须喊人——滞留空中且日志假成功最不可接受
                        res = self.last_results
                        if res is None:
                            print(" ROLLBACK FAILED — land 模板未配置，d%s "
                                  "仍在空中：立即人工介入（遥控器 land / "
                                  "panic）" % ",".join(up))
                            self.log("ROLLBACK_FAIL", reason="no-land-tmpl",
                                     up=up)
                        else:
                            rb = [str(i) for i, r in res if not r]
                            print(" ROLLBACK FAILED — d%s land 未确认，仍在"
                                  "空中：立即人工介入（遥控器 land / panic）"
                                  % (",".join(rb) or "全部"))
                            self.log("ROLLBACK_FAIL", failed=rb)
                else:
                    print(" airborne confirm FAILED — mission NOT triggered.")
                return 1
        if self.cmds.get("trigger") is None:
            # 真机 profile trigger 待补/任务自启：跳过触发，直接盯飞
            print(" trigger not configured — assume auto-start after "
                  "takeoff.")
            self.log("TRIGGER_SKIP")
        elif not self.dispatch("trigger"):
            print(" trigger failed — mission NOT started.")
            self.log("TRIGGER_FAIL")
            return 1
        print(" monitoring (timeout %.0fs)..." % monitor_timeout)
        t0 = time.time()
        seen_ev = set()
        last_line = 0.0
        while True:
            snap = self.read_status()
            if snap:
                for ev in snap.get("events", []):
                    key = (ev.get("t"), ev.get("drone"), ev.get("event"))
                    if key not in seen_ev:
                        seen_ev.add(key)
                        if ev.get("event") != "TELEM":
                            print(" [EVT] d%s %s %s"
                                  % (ev.get("drone"), ev.get("event"),
                                     ev.get("detail")))
                drones = snap.get("drones", {})
                done = [i for i in self.ids
                        if (drones.get(i) or {}).get("phase")
                        in self.terminal_phases]
                now = time.time()
                if now - last_line >= 5.0:
                    last_line = now
                    ph = " ".join(
                        "%s:%s" % (i, (drones.get(i) or {})
                                   .get("phase") or "?")
                        for i in self.ids)
                    sc = snap.get("scores") or {}
                    tot = sum(int((s or {}).get("score") or 0)
                              for s in sc.values())
                    print(" [%3.0fs] stage=%s %s (%d/%d terminal) score=%s"
                          % (now - t0, snap.get("stage") or "?", ph,
                             len(done), len(self.ids),
                             tot if sc else "--"))
                if len(done) == len(self.ids):
                    el = time.time() - t0
                    fails = [i for i in self.ids
                             if drones[i].get("phase") == "FAILED"]
                    aborts = [i for i in self.ids
                              if drones[i].get("phase") == "ABORT"]
                    print(" ALL TERMINAL in %.0fs — DONE=%d ABORT=%s "
                          "FAILED=%s"
                          % (el, len(self.ids) - len(fails) - len(aborts),
                             ",".join(aborts) or "0",
                             ",".join(fails) or "0"))
                    sc = snap.get("scores") or {}
                    if sc:
                        tot = sum(int((s or {}).get("score") or 0)
                                  for s in sc.values())
                        cor = sum(int((s or {}).get("correct") or 0)
                                  for s in sc.values())
                        wrg = sum(int((s or {}).get("wrong") or 0)
                                  for s in sc.values())
                        print(" SCORE total=%d correct=%d wrong=%d | %s"
                              % (tot, cor, wrg,
                                 " ".join("d%s:%s" % (d, (s or {}).get("score"))
                                          for d, s in sorted(sc.items()))))
                    self.log("MISSION_END", elapsed_s=round(el, 1),
                             aborted=aborts, failed=fails,
                             score=sc or None)
                    return 0 if not fails else 2
            if time.time() - t0 > monitor_timeout:
                print(" MONITOR TIMEOUT — mission still running, "
                      "keep hub dash on.")
                self.log("MONITOR_TIMEOUT")
                return 3
            time.sleep(1.0)

    # ---- status 一发 -------------------------------------------------------
    def status(self):
        snap = self.read_status()
        if not snap:
            print(" no status file: %s (hub up?)" % self.status_file)
            return 1
        res, cur = self.gates(snap)
        self.print_table(snap, res, cur)
        return 0

    # ---- 真机调试接口（ssh）-------------------------------------------------
    def debug_cli(self, name, cmd_args, ids=None):
        """ops debug <名> [args]：跑 profile ops.debug_cmds 里的命名 ssh 命令。

        无参列出可用命令；结果只打印+落 JSONL（DBG 记录），约定只读。
        """
        if not self.debug_ssh:
            print(" profile 未配置 ops.debug_ssh — 真机调试接口不可用"
                  "（sim profile 无此段）")
            return 2
        if not name or name == "list":
            print(" debug cmds: %s" % " ".join(sorted(self.debug_cmds)))
            print(" 用法: gcs_ops.py --profile <p> debug <名> [args] "
                  "[--ids 0,2]")
            return 0
        cmd = self.debug_cmds.get(name)
        if cmd is None:
            print(" 未知调试命令 '%s' — 可用: %s"
                  % (name, " ".join(sorted(self.debug_cmds))))
            return 2
        if "{args}" in cmd and not cmd_args:
            print(" 调试命令 '%s' 需要参数 {args}" % name)
            return 2
        # ids 类型归一：CLI 传逗号串，面板传 list——历史版只当字符串
        # .split()，DEBUG 窗每次 RUN 必 AttributeError（审查确认 HIGH）
        ids = ids.split(",") if isinstance(ids, str) else (ids or self.ids)
        all_ok = True
        for i in ids:
            # replace 而非 .format：debug_cmds 若含字面花括号（awk '{...}'）
            # .format 直接 KeyError；命令串组装完毕后不再二次格式化
            full = "%s %s" % (
                self.debug_ssh.replace("{id}", str(i)),
                shlex.quote(cmd.replace("{args}", cmd_args or "")))
            rc, out, err = self._run(full, timeout=30.0)
            mark = C_GRN + "OK" + C_RST if rc == 0 \
                else C_RED + "FAIL rc=%d" % rc + C_RST
            body = (out or err).strip()
            # 多行原样回显（ros/node/proc/res 压平截断 200 字符=不可读）
            print(" d%s %s" % (i, mark))
            if body:
                print(body[:2000])
            self.log("DBG", name=name, drone=i, rc=rc, out=body[:300])
            all_ok = all_ok and rc == 0
        return 0 if all_ok else 1

    def _probe_real(self):
        """联调勘误探针（profile ops.probe=debug_cmds 名单，不阻断门）。

        把 TODO 清单变成启动自检：ssh 免密/目录大小写/远端 ROS 榛活，
        失败即列修复提示——不靠人记勘误清单。
        """
        print(" REAL PROBES (联调勘误探针，不阻断门):")
        for name in self.probe:
            if name not in self.debug_cmds:
                print("  %-6s 未定义于 ops.debug_cmds — 跳过" % name)
                continue
            rows = []
            for i in self.ids:
                full = "%s %s" % (
                    self.debug_ssh.replace("{id}", str(i)),
                    shlex.quote(self.debug_cmds[name]))
                rc, out, err = self._run(full, timeout=30.0)
                rows.append("d%s:%s" % (i, "OK" if rc == 0 else
                                        "FAIL rc=%d %s"
                                        % (rc, (err or out).strip()[:40])))
            self.log("PROBE", name=name, detail="; ".join(rows))
            print("  %-6s %s" % (name, "  ".join(rows)))
        print("  ↑ 失败修复: ssh-copy-id 发公钥; ls -d ~/Diff* 核目录大小写;"
              " 现场核实 IP 网段")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", required=True)
    ap.add_argument("action",
                    choices=["status", "preflight", "start", "takeoff",
                             "back", "land", "panic", "debug"])
    ap.add_argument("debug_name", nargs="?", default=None,
                    help="debug 子命令名（无参=list 可用命令）")
    ap.add_argument("debug_args", nargs="?", default=None,
                    help="debug 命令的 {args} 参数")
    ap.add_argument("--ids", default=None, help="逗号分隔，缺省=全队")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--yes", action="store_true",
                    help="real 模式危险命令确认（面板两段确认后代传）")
    ap.add_argument("--preflight-timeout", type=float, default=90.0)
    ap.add_argument("--monitor-timeout", type=float, default=None)
    args = ap.parse_args()
    ops = Ops(args.profile)
    # real 模式保护：危险命令必须 --yes；panic 豁免（急停不设障碍）
    REAL_DANGER = {"start", "takeoff", "back", "land"}
    if ops.mode == "real":
        print("== REAL MODE (真机) profile=%s ==" % args.profile)
        if args.action in REAL_DANGER and not args.yes:
            print(" REFUSED: real 模式 `%s` 必须显式 --yes "
                  "(CLI 直发；面板走两段确认后代传 --yes)" % args.action)
            raise SystemExit(2)
    if args.action == "panic":
        # land-all：逐机必达（模板带 {id} 时单机失败不中止）；
        # 结果必须汇报——急停链路"静默失败/假成功"是最不可接受的失败
        ids = args.ids.split(",") if args.ids else ops.ids
        print(" PANIC LAND ALL ids=%s" % ",".join(ids))
        ops.log("PANIC", ids=ids)
        ok = ops.dispatch("panic", ids=ids)
        res = ops.last_results
        if res is None:
            # 模板缺失=一条命令都没发——绝不能报 COMPLETE
            print(" PANIC NOT SENT — panic/land 模板未配置，急停链路未送达！")
            print(" → 立即人工介入: 遥控器逐机手动 land")
            ops.log("PANIC_NOT_SENT")
            raise SystemExit(2)
        if any(i is None for i, _ in res):
            # 全局模板：一次广播，无逐机确认语义——不得冒充"逐机送达"
            if ok:
                print(" PANIC SENT — 全局急停广播已发（无逐机确认语义，"
                      "人工盯降落）")
                ops.log("PANIC_SENT_GLOBAL")
            else:
                print(" PANIC FAILED — 全局急停广播发送失败，立即人工介入！")
                ops.log("PANIC_GLOBAL_FAIL")
            raise SystemExit(0 if ok else 1)
        failed = [str(i) for i, r in res if not r]
        if failed:
            print(" PANIC INCOMPLETE — 未确认降落: d%s" % ",".join(failed))
            print(" → 立即人工介入: 遥控器手动 land；或单机重发 "
                  "`gcs_ops.py ... panic --ids %s`；真机另可用 DEBUG 窗 "
                  "ros 命令核查 /px4ctrl/takeoff_land 流" % ",".join(failed))
            ops.log("PANIC_INCOMPLETE", failed=failed)
        else:
            print(" PANIC COMPLETE — %d/%d 机降落指令确认送达"
                  % (len(ids), len(ids)))
            ops.log("PANIC_COMPLETE", ids=ids)
        raise SystemExit(0 if ok else 1)
    if args.action == "preflight":
        raise SystemExit(ops.preflight(args.preflight_timeout))
    if args.action == "start":
        mt = args.monitor_timeout or (ops.execute_timeout_s + 60.0)
        raise SystemExit(ops.start(args.force, mt))
    if args.action == "status":
        raise SystemExit(ops.status())
    if args.action == "debug":
        raise SystemExit(ops.debug_cli(args.debug_name, args.debug_args,
                                       ids=args.ids))
    # takeoff / back / land：逐机命令
    ids = args.ids.split(",") if args.ids else ops.ids
    ok = ops.dispatch(args.action, ids=ids)
    ops.log("DISPATCH", action=args.action, ids=ids, ok=ok)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
