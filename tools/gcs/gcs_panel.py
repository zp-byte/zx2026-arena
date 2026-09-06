#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gcs_panel.py — 非凸α 地面站全功能面板（GCS v0 第 3 步）。

数据源：hub 的 1Hz 状态快照文件（铁律①——面板进程不进 ROS 图）。
命令层：复用 gcs_ops.Ops（同 profile 模板/门限/错峰/重试），子进程在
工作线程执行，stdout 经 Qt 信号流进日志窗，UI 不阻塞。

布局：
  头幅   SIM 仿真模式(蓝) / REAL 真机模式(红)——mode 一级开关的 UI 锚
  顶栏   阶段机 | 连接数 | 快照年龄
  门条   sim 四灯 / real 六灯(加 POWER+FC)，随 profile 动态
  六格   每机卡片: 相位/位置/速度/电量/规划年龄/链路年龄 + 告警徽章
  按钮排 START / TAKEOFF / BACK / LAND + PANIC(双击确认, 5s 解除)
         空模板按钮禁用置灰（不可达命令不给点）；real 全按钮两段确认
  日志窗 事件流 + 命令输出

用法：
  python3 gcs_panel.py --profile profile_sim.yaml
  QT_QPA_PLATFORM=offscreen python3 gcs_panel.py --profile ... --selftest
"""
import os
import sys
import time

from PySide6.QtCore import QObject, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPalette, QPixmap
from PySide6.QtWidgets import (QApplication, QDialog, QFrame, QGridLayout,
                               QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                               QMessageBox, QPlainTextEdit, QPushButton,
                               QVBoxLayout, QWidget)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gcs_ops import STAGES, Ops  # noqa: E402

GRN, RED, YEL, DIM, RST = "#2ecc71", "#e74c3c", "#f39c12", "#555555", "#dddddd"
CARD_BG = "#20242a"; PANEL_BG = "#12151a"; TXT = "#e8eaed"


def qss():
    return """
    QMainWindow, QWidget { background: %s; color: %s; font-size: 13px; }
    QFrame#card { border: 2px solid %s; border-radius: 8px;
                  background: %s; }
    QLabel#big   { font-size: 20px; font-weight: bold; }
    QLabel#phase { font-size: 16px; font-weight: bold; }
    QPushButton  { padding: 8px 14px; border: 1px solid %s;
                   border-radius: 6px; background: #2a2f37; color: %s; }
    QPushButton:disabled { color: #666; }
    QPushButton#panic  { background: %s; color: white; font-weight: bold;
                         font-size: 16px; border: 2px solid #ff6b6b; }
    QPushButton#panicArmed { background: #ff1f1f; color: white; }
    QLabel#modeSim  { background: #1f4e79; color: white; font-weight: bold;
                      font-size: 15px; border-radius: 6px; }
    QLabel#modeReal { background: #8e1f1f; color: white; font-weight: bold;
                      font-size: 15px; border-radius: 6px; }
    QPlainTextEdit { background: #0d1013; color: #9fd49f;
                     font-family: Monospace; font-size: 12px; }
    """ % (PANEL_BG, TXT, DIM, CARD_BG, DIM, TXT, RED)


class SignalStream(QObject):
    """把子进程 dispatch 的 print 输出实时搬进日志窗。"""
    text = Signal(str)

    def write(self, s):
        if s and s.strip():
            self.text.emit(s.rstrip("\n"))

    def flush(self):
        pass


class Worker(QThread):
    done = Signal(bool, str)

    def __init__(self, fn, parent=None):
        super().__init__(parent)
        self.fn = fn

    def run(self):
        try:
            ok = self.fn()
        except Exception as e:  # 兜底: 工作线程不许带崩 UI
            ok = False
            print("worker exception: %s" % e)
        self.done.emit(bool(ok), "")


class Card(QFrame):
    """单机卡片：状态字段 + 告警徽章，set_state 全量刷。"""

    def __init__(self, did):
        super().__init__()
        self.setObjectName("card")
        v = QVBoxLayout(self)
        v.setContentsMargins(8, 6, 8, 6)
        v.setSpacing(2)
        self.title = QLabel("d%s" % did)
        self.title.setObjectName("big")
        self.phase = QLabel("--")
        self.phase.setObjectName("phase")
        self.info = QLabel("pos --  v --")
        self.info.setWordWrap(True)
        self.meta = QLabel("bat --  plan --  age --")
        self.meta.setWordWrap(True)
        self.alarm = QLabel("")
        for w in (self.title, self.phase, self.info, self.meta, self.alarm):
            v.addWidget(w)
        self.did = did

    def set_state(self, d, link_ok, stall_on, pdead_on, bat_on=False):
        # 数据过期=NO LINK（与 hub dash 同语义：不信 TCP/旧数据）
        ph = (d or {}).get("phase") if link_ok else None
        self.phase.setText(str(ph) if ph else "NO LINK")
        self.phase.setStyleSheet(
            "color: %s;" % (GRN if ph else RED))
        if d and d.get("pos"):
            p = d["pos"]
            self.info.setText("pos (%.1f, %.1f, %.1f)  v %.2f"
                              % (p[0], p[1], p[2], d.get("speed") or 0.0))
        else:
            self.info.setText("pos --  v --")
        bat = d.get("bat") if d else None
        pa = d.get("plan_age") if d else None
        age = d.get("age") if d else None
        self.meta.setText("bat %-4s plan %-6s age %-5s"
                          % ("%.0f%%" % bat if bat is not None else "--",
                             "%.1f" % pa if pa is not None and pa >= 0
                             else "never",
                             "%.1f" % age if age is not None else "--"))
        badges = []
        if not link_ok:
            badges.append("LINK")
        if stall_on:
            badges.append("STALL")
        if pdead_on:
            badges.append("PLANNER")
        if bat_on:
            badges.append("BAT")
        self.alarm.setText(" ".join("[%-7s]" % b for b in badges))
        self.alarm.setStyleSheet("color: %s;" % (RED if badges else DIM))
        border = RED if not link_ok else (YEL if badges else GRN)
        self.setStyleSheet("QFrame#card { border: 2px solid %s; "
                           "border-radius: 8px; background: %s; }"
                           % (border, CARD_BG))


class MapDialog(QDialog):
    """六机自建栅格地图窗（第 4 步 ③）：快照 grids → QImage 渲染 + 机位叠加。

    各机栅格同处 world 系（原点/分辨率随格自带），全部机位置画到每张图上；
    占用=橙、空闲=底色、机位=绿。500ms 跟随面板轮询刷新。
    """

    OCC = bytes((230, 126, 34))   # 占用格
    FREE = bytes((16, 19, 26))    # 空闲格（近面板底色）
    POS = bytes((46, 204, 113))   # 机位

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.setWindowTitle("fei tu A GCS - occupancy maps")
        g = QGridLayout(self)
        self.lbls = {}
        for k, did in enumerate(panel.ops.ids):
            lbl = QLabel("d%s: no map" % did)
            lbl.setMinimumSize(240, 210)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("border: 1px solid #555555; "
                              "background: #0d1013; color: #888888;")
            self.lbls[did] = lbl
            g.addWidget(lbl, k // 3, k % 3)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def refresh(self):
        grids = self.panel.grids
        drones = self.panel.last_drones
        for did, lbl in self.lbls.items():
            gd = grids.get(did)
            if not gd:
                continue
            img = self._render(gd, drones)
            pm = QPixmap.fromImage(img.scaled(
                lbl.width() - 4, lbl.height() - 4, Qt.KeepAspectRatio,
                Qt.FastTransformation))
            lbl.setPixmap(pm)

    @classmethod
    def _render(cls, g, drones):
        import base64
        w, h, res = int(g["w"]), int(g["h"]), g["res"]
        x0, y0 = g["x0"], g["y0"]
        raw = base64.b64decode(g["rle"])
        buf = bytearray(cls.FREE * (w * h))
        idx = 0
        for k in range(0, len(raw) - 1, 2):
            run = raw[k + 1]
            if raw[k]:
                for m in range(run):
                    c, r = (idx + m) % w, (idx + m) // w
                    o = ((h - 1 - r) * w + c) * 3   # 行翻转：栅格原点在左下
                    buf[o:o + 3] = cls.OCC
            idx += run
        for d in drones.values():
            p = (d or {}).get("pos")
            if not p:
                continue
            ix = int((p[0] - x0) / res)
            iy = int((p[1] - y0) / res)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    xx, yy = ix + dx, iy + dy
                    if 0 <= xx < w and 0 <= yy < h:
                        o = ((h - 1 - yy) * w + xx) * 3
                        buf[o:o + 3] = cls.POS
        return QImage(bytes(buf), w, h, w * 3, QImage.Format_RGB888)


class Panel(QMainWindow):
    def __init__(self, ops, poll_ms=500, selftest=False):
        super().__init__()
        self.ops = ops
        self.selftest = selftest
        self.busy = False
        self._fired = False  # selftest 自动 START 只发一次（实例级）
        self.panic_armed_t = 0.0
        self.armed = None    # real 两段确认：当前 armed 的命令名
        self.armed_t = 0.0
        self.evt_seen = set()
        self.real = (getattr(ops, "mode", "sim") == "real")
        self.setWindowTitle("fei tu A GCS [%s]  %s"
                            % ("REAL" if self.real else "SIM",
                               ops.cfg.get("profile", "?")))
        self.resize(980, 720)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # 模式头幅（红=真机 / 蓝=仿真）——mode 一级开关的 UI 锚
        self.mode_lbl = QLabel(
            "REAL 真机模式 — 起飞先行 + 两段确认 + 低电告警"
            if self.real else
            "SIM 仿真模式 — 触发式启动（TAKEOFF/BACK/LAND 由阶段机自驱）")
        self.mode_lbl.setAlignment(Qt.AlignCenter)
        self.mode_lbl.setFixedHeight(28)
        self.mode_lbl.setObjectName("modeReal" if self.real else "modeSim")
        root.addWidget(self.mode_lbl)

        # 顶栏
        top = QHBoxLayout()
        self.stage_lbl = QLabel("stage --")
        self.stage_lbl.setObjectName("big")
        self.conns_lbl = QLabel("conns --")
        self.snap_lbl = QLabel("snap --")
        top.addWidget(self.stage_lbl)
        top.addStretch(1)
        top.addWidget(self.conns_lbl)
        top.addWidget(self.snap_lbl)
        root.addLayout(top)

        # 门条（sim 四灯 / real 六灯，随 profile 动态）
        gates_box = QHBoxLayout()
        self.gate_lbls = {}
        for s in ops.stages:
            lbl = QLabel(s)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(180)
            self.gate_lbls[s] = lbl
            gates_box.addWidget(lbl)
        root.addLayout(gates_box)

        # 六格卡片
        grid = QGridLayout()
        self.cards = {}
        for i, did in enumerate(ops.ids):
            c = Card(did)
            self.cards[did] = c
            grid.addWidget(c, i // 3, i % 3)
        root.addLayout(grid)

        # 地图窗状态（第 4 步 ③：建图上屏数据源 = poll 存的最新快照）
        self.grids = {}
        self.last_drones = {}
        self._map_dlg = None

        # 指令按钮排（空模板=禁用置灰：不可达命令不给点）
        btns = QHBoxLayout()
        self.btns = {}
        self.btn_text = {}
        self.btn_avail = {}
        for name, text in (("trigger", "START"),
                           ("takeoff", "TAKEOFF"),
                           ("back", "BACK"),
                           ("land", "LAND")):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, n=name: self.on_cmd(n))
            self.btns[name] = b
            self.btn_text[name] = text
            self.btn_avail[name] = ops.cmds.get(name) is not None
            if not self.btn_avail[name]:
                b.setEnabled(False)
                b.setToolTip("profile 未配置 ops.cmds.%s — 不可达，禁用"
                             % name)
            btns.addWidget(b)
        self.panic = QPushButton("PANIC LAND")
        self.panic.setObjectName("panic")
        self.panic.clicked.connect(self.on_panic)
        if ops.cmds.get("panic") is None and \
                ops.cmds.get("land") is None:
            self.panic.setEnabled(False)
            self.panic.setToolTip("profile 未配置 panic/land — 禁用")
        btns.addWidget(self.panic)
        self.map_btn = QPushButton("MAP")
        self.map_btn.setToolTip("六机自建栅格地图（view-only，不依赖命令模板）")
        self.map_btn.clicked.connect(self.toggle_map)
        btns.addWidget(self.map_btn)
        root.addLayout(btns)

        # 日志窗
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(800)
        root.addWidget(self.log)

        # 子进程 stdout 流
        self.stream = SignalStream()
        self.stream.text.connect(self.append)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(poll_ms)
        self.poll()

    # ---- 工具 --------------------------------------------------------------
    def append(self, s):
        self.log.appendPlainText(s)

    def _set_gates(self, res):
        for s in self.ops.stages:
            ok, why = res[s]
            lbl = self.gate_lbls[s]
            lbl.setText("%s\n%s" % (s, "OK" if ok else why[:18]))
            color = GRN if ok else (YEL if s != "READY" else RED)
            lbl.setStyleSheet("border: 2px solid %s; border-radius: 6px; "
                              "padding: 4px; color: %s;" % (color, color))

    def _set_buttons(self, enabled):
        for name, b in self.btns.items():
            b.setEnabled(enabled and self.btn_avail.get(name, False))
        if enabled:
            self._disarm_panic()

    def toggle_map(self):
        if self._map_dlg is None or not self._map_dlg.isVisible():
            self._map_dlg = MapDialog(self)
            self._map_dlg.show()
        else:
            self._map_dlg.raise_()
            self._map_dlg.activateWindow()

    # ---- 500ms 轮询快照 -----------------------------------------------------
    def poll(self):
        snap = self.ops.read_status()
        if snap is None:
            self.snap_lbl.setText("snap: NO FILE (hub up?)")
            return
        now = time.time()
        hub_age = now - snap.get("hub_ts", 0)
        self.snap_lbl.setText("snap age %.1fs" % hub_age)
        self.stage_lbl.setText("stage %s" % (snap.get("stage") or "--"))
        self.conns_lbl.setText("conns %s" % snap.get("conns", "--"))
        drones = snap.get("drones", {})
        self.last_drones = drones
        self.grids = snap.get("grids") or {}
        for did, card in self.cards.items():
            d = drones.get(did)
            age = (d or {}).get("age")
            link_ok = age is not None and age <= self.ops.link_lost_s
            card.set_state(d, link_ok,
                           bool((d or {}).get("stall_on")),
                           bool((d or {}).get("pdead_on")),
                           bool((d or {}).get("bat_on")))
        res, _cur = self.ops.gates(snap)
        self._set_gates(res)
        # 事件流增量（去重：hub events 尾 50 条）
        for ev in snap.get("events", []):
            key = (ev.get("t"), ev.get("drone"), ev.get("event"))
            if key in self.evt_seen or ev.get("event") == "TELEM":
                continue
            self.evt_seen.add(key)
            self.append("[EVT] d%s %s %s" % (ev.get("drone"),
                                             ev.get("event"),
                                             ev.get("detail")))
        # selftest：READY 后自动 START，全部终态退出
        if self.selftest:
            self._selftest_tick(snap, res)

    # ---- selftest 自动流 ---------------------------------------------------
    def _selftest_tick(self, snap, res):
        if self.busy:
            return
        done = [did for did in self.ops.ids
                if ((snap.get("drones", {}).get(did) or {}).get("phase")
                    in self.ops.terminal_phases)]
        if len(done) == len(self.ops.ids):
            self.append("SELFTEST: all terminal (%d/%d) — exit 0"
                        % (len(done), len(self.ops.ids)))
            self.ops.log("SELFTEST_PASS")
            self.timer.stop()
            QApplication.instance().quit()
            return
        if res["READY"][0] and not self._fired:
            self.append("SELFTEST: gates green -> auto START")
            self.ops.log("SELFTEST_AUTO_START")
            self.start_cmd("trigger")

    # ---- 命令 --------------------------------------------------------------
    def start_cmd(self, name, ids=None):
        if self.busy:
            return
        self.busy = True
        self._set_buttons(False)
        ops = self.ops

        def fn():
            import contextlib
            with contextlib.redirect_stdout(self.stream):
                return ops.dispatch(name, ids=ids)

        self.w = Worker(fn)
        self.w.done.connect(self.on_cmd_done)
        self.w.start()
        self.append("[CMD] %s dispatched" % name)

    # ---- real 两段确认 ------------------------------------------------------
    def on_cmd(self, name):
        if not self.real:
            self.start_cmd(name)
            return
        # REAL 模式：全部命令两段确认（arm 5s 内二次点击 → 弹窗确认）
        now = time.time()
        if self.armed != name or now - self.armed_t > 5.0:
            self._arm(name)
            return
        self._disarm()
        if QMessageBox.question(
                self, "REAL CONFIRM",
                "真机模式确认执行 %s？" % self.btn_text[name]) != \
                QMessageBox.Yes:
            self.append("[REAL] %s cancelled" % name)
            return
        if name == "trigger":
            self.start_flow()  # real START = 完整起飞先行流程
        else:
            self.start_cmd(name)

    def _arm(self, name):
        self.armed = name
        self.armed_t = time.time()
        b = self.btns[name]
        b.setText("CONFIRM %s ?" % self.btn_text[name])
        b.setStyleSheet("background: %s; color: white; font-weight: bold;"
                        % YEL)
        self.append("[REAL] %s armed — 5s 内再次点击确认" % name)
        QTimer.singleShot(5000, lambda: self._disarm(name))

    def _disarm(self, token=None):
        if token is not None and token != self.armed:
            return  # 已换新 arm，别误清新状态
        if self.armed is None:
            return
        b = self.btns.get(self.armed)
        if b is not None:
            b.setText(self.btn_text[self.armed])
            b.setStyleSheet("")
        self.armed = None
        self.armed_t = 0.0

    def start_flow(self):
        """real START = ops.start 全流程（门→错峰起飞→离地确认→触发→盯飞）。"""
        if self.busy:
            return
        self.busy = True
        self._set_buttons(False)
        ops = self.ops
        mt = ops.execute_timeout_s + 60.0

        def fn():
            import contextlib
            with contextlib.redirect_stdout(self.stream):
                return ops.start(False, mt)

        self.w = Worker(fn)
        self.w.done.connect(self.on_cmd_done)
        self.w.start()
        self.append("[START] real flow: gates->takeoff->airborne->trigger"
                    "->monitor")

    def on_cmd_done(self, ok, _msg):
        self.busy = False
        self._set_buttons(True)
        self._fired = True  # selftest 只自动触发一次
        self.append("[CMD] finished ok=%s" % ok)

    def on_panic(self):
        now = time.time()
        if now - self.panic_armed_t > 5.0:
            self.panic_armed_t = now
            self.panic.setObjectName("panicArmed")
            self.panic.setStyleSheet(
                "background: #ff1f1f; color: white; font-weight: bold;")
            self.append("[PANIC] armed — click again within 5s to "
                        "LAND ALL")
            QTimer.singleShot(5000, self._disarm_panic)
            return
        self._disarm_panic()
        if QMessageBox.question(
                self, "PANIC", "确认 PANIC LAND ALL？") != \
                QMessageBox.Yes:
            self.append("[PANIC] cancelled")
            return
        self.start_cmd("panic", ids=self.ops.ids)

    def _disarm_panic(self):
        self.panic_armed_t = 0.0
        self.panic.setStyleSheet(
            "background: %s; color: white; font-weight: bold; "
            "font-size: 16px; border: 2px solid #ff6b6b;" % RED)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", required=True)
    ap.add_argument("--poll-ms", type=int, default=500)
    ap.add_argument("--status-file", default=None,
                    help="覆盖 profile 的 ops.status_file")
    ap.add_argument("--selftest", action="store_true",
                    help="READY 自动 START，全部终态自动退出(离线 E2E)")
    args = ap.parse_args()
    ops = Ops(args.profile)
    if args.status_file:
        ops.status_file = args.status_file
    if os.environ.get("QT_QPA_PLATFORM") is None and \
            not os.environ.get("DISPLAY"):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    app = QApplication(sys.argv)
    app.setStyleSheet(qss())
    panel = Panel(ops, poll_ms=args.poll_ms, selftest=args.selftest)
    panel.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
