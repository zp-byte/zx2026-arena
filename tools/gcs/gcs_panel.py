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
from html import escape

from collections import deque

from PySide6.QtCore import QObject, QPointF, Qt, QThread, QTimer, Signal
from PySide6.QtGui import (QBrush, QColor, QFont, QImage, QPainter, QPalette,
                           QPixmap, QPolygonF, QPen)
from PySide6.QtWidgets import (QApplication, QComboBox, QDialog, QFrame,
                               QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                               QLineEdit, QMainWindow, QMessageBox,
                               QPlainTextEdit, QPushButton, QVBoxLayout,
                               QWidget)

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
            # 异常文本必须经 done 信号回 UI——print 只进真 stdout，
            # 面板用户永远看不见（ok=False 无原因）
            self.done.emit(False, "exception: %s: %s"
                           % (type(e).__name__, e))
            return
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

    def set_state(self, d, link_ok, stall_on, pdead_on, bat_on=False,
                  score=None):
        # 数据过期=NO LINK（与 hub dash 同语义：不信 TCP/旧数据）
        ph = (d or {}).get("phase") if link_ok else None
        # 退赛（rule_monitor 合规条款）整卡标红——S2 已剔除，操作员必须看见
        retired = bool(score and score.get("retire")) or ph == "RETIRED"
        self.phase.setText(str(ph) if ph else "NO LINK")
        self.phase.setStyleSheet(
            "color: %s;" % (RED if (retired or not ph) else GRN))
        if d and d.get("pos"):
            p = d["pos"]
            self.info.setText("pos (%.1f, %.1f, %.1f)  v %.2f"
                              % (p[0], p[1], p[2], d.get("speed") or 0.0))
        else:
            self.info.setText("pos --  v --")
        bat = d.get("bat") if d else None
        pa = d.get("plan_age") if d else None
        age = d.get("age") if d else None
        sc_v = score.get("score") if score else None
        self.meta.setText("bat %-4s plan %-6s age %-5s 分 %-4s"
                          % ("%.0f%%" % bat if bat is not None else "--",
                             "%.1f" % pa if pa is not None and pa >= 0
                             else "never",
                             "%.1f" % age if age is not None else "--",
                             str(sc_v) if sc_v is not None else "--"))
        badges = []
        if not link_ok:
            badges.append("LINK")
        if retired:
            badges.append("RETIRED")
        if stall_on:
            badges.append("STALL")
        if pdead_on:
            badges.append("PLANNER")
        if bat_on:
            badges.append("BAT")
        self.alarm.setText(" ".join("[%-7s]" % b for b in badges))
        self.alarm.setStyleSheet("color: %s;" % (RED if badges else DIM))
        border = RED if (not link_ok or retired) else (YEL if badges else GRN)
        self.setStyleSheet("QFrame#card { border: 2px solid %s; "
                           "border-radius: 8px; background: %s; }"
                           % (border, CARD_BG))


class _Thumb(QLabel):
    """缩略图标签：点击切换态势图高亮单机。"""

    def __init__(self, did, cb):
        super().__init__("d%s" % did)
        self.did = did
        self.cb = cb
        self.setMinimumSize(150, 128)
        self.setAlignment(Qt.AlignCenter)
        self._sel = False
        self._apply_css()

    def _apply_css(self):
        self.setStyleSheet(
            "border: 2px solid %s; background: #0d1013; color: #aaaaaa; "
            "font-size: 11px;" % ("#e67e22" if self._sel else "#555555"))

    def mousePressEvent(self, _ev):
        self.cb(self.did)

    def set_sel(self, sel):
        self._sel = sel
        self._apply_css()


class MapDialog(QDialog):
    """栅格态势图窗（第 4 步 ③ 增强版）。

    大图=六机栅格**合并**全局图（union bbox 自动裁剪覆盖区 + 5m 网格线 +
    分机颜色航向箭头/轨迹尾迹 + 比例尺 + 图例条），右侧缩略图点击高亮单机；
    RLE 三值（0 空闲/1 占用/2 未知）分离已探索空地与未探索区。500ms 刷新。
    """

    OCC = bytes((230, 126, 34))   # 占用格
    FREE = bytes((16, 19, 26))    # 已探索空闲格
    UNK = bytes((34, 40, 52))     # 未知/未探索格
    POS = bytes((46, 204, 113))   # 机位兜底色
    DCOL = {"0": (46, 204, 113), "1": (52, 152, 219), "2": (231, 76, 60),
            "3": (241, 196, 15), "4": (155, 89, 182), "5": (26, 188, 156)}

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.hl = None          # 高亮单机 did（点击缩略图切换）
        self.setWindowTitle("fei tu A GCS - situational map")
        self.resize(1150, 680)
        g = QGridLayout(self)
        g.setContentsMargins(6, 6, 6, 6)
        g.setSpacing(6)
        self.big = QLabel("no grid yet — occ_grid 在闭环活跃后发布（起飞+goal）")
        self.big.setAlignment(Qt.AlignCenter)
        self.big.setMinimumSize(660, 500)
        self.big.setStyleSheet("border: 1px solid #555555; "
                               "background: #0d1013; color: #888888;")
        g.addWidget(self.big, 0, 0, 6, 1)
        self.thumbs = {}
        for k, did in enumerate(panel.ops.ids):
            t = _Thumb(did, self._pick)
            self.thumbs[did] = t
            g.addWidget(t, k, 1)
        self.info = QLabel("")
        self.info.setTextFormat(Qt.RichText)
        g.addWidget(self.info, 6, 0, 1, 2)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def _pick(self, did):
        self.hl = None if self.hl == did else did
        for d, t in self.thumbs.items():
            t.set_sel(d == self.hl)

    # ---- 缩略图：单机栅格 1:1 + 全机彩色机位点 ------------------------------
    @classmethod
    def _thumb_img(cls, g, drones):
        import base64
        w, h, res = int(g["w"]), int(g["h"]), g["res"]
        x0, y0 = g["x0"], g["y0"]
        raw = base64.b64decode(g["rle"])
        buf = bytearray(cls.UNK * (w * h))
        idx = 0
        for k in range(0, len(raw) - 1, 2):
            v, run = raw[k], raw[k + 1]
            if v != 2:                       # 未知格跳过=保持底色
                col = cls.FREE if v == 0 else cls.OCC
                for m in range(run):
                    i = idx + m
                    c, r = i % w, i // w
                    o = ((h - 1 - r) * w + c) * 3   # 行翻转：原点在左下
                    buf[o:o + 3] = col
            idx += run
        for did, d in drones.items():
            p = (d or {}).get("pos")
            if not p:
                continue
            ix = int((p[0] - x0) / res)
            iy = int((p[1] - y0) / res)
            col = bytes(cls.DCOL.get(did, cls.POS))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    xx, yy = ix + dx, iy + dy
                    if 0 <= xx < w and 0 <= yy < h:
                        o = ((h - 1 - yy) * w + xx) * 3
                        buf[o:o + 3] = col
        return QImage(bytes(buf), w, h, w * 3, QImage.Format_RGB888)

    # ---- 大图：合并全局缓冲 → 裁剪 → 缩放 → 叠加层 --------------------------
    @classmethod
    def _merged(cls, grids):
        """六机栅格合并进 union 全局缓冲。

        返回 (buf, gw, gh, gx0, gy0, res, occ_bbox, occ_n)；occ_bbox 为
        (gc0, gr0, gc1, gr1) 全局格坐标（行自底起），无占用时 None。
        """
        if not grids:
            return None
        res = next(iter(grids.values()))["res"]
        gx0 = min(g["x0"] for g in grids.values())
        gy0 = min(g["y0"] for g in grids.values())
        gw = int(round((max(g["x0"] + g["w"] * res for g in grids.values())
                        - gx0) / res))
        gh = int(round((max(g["y0"] + g["h"] * res for g in grids.values())
                        - gy0) / res))
        if gw <= 0 or gh <= 0 or gw * gh > 4000000:
            return None
        buf = bytearray(cls.UNK * (gw * gh))
        occ_bbox, occ_n = None, 0
        import base64
        for g in grids.values():
            raw = base64.b64decode(g["rle"])
            w = int(g["w"])
            ox = int(round((g["x0"] - gx0) / res))
            oy = int(round((g["y0"] - gy0) / res))
            idx = 0
            for k in range(0, len(raw) - 1, 2):
                v, run = raw[k], raw[k + 1]
                if v != 2:
                    col = cls.FREE if v == 0 else cls.OCC
                    for m in range(run):
                        i = idx + m
                        gc, gr = ox + i % w, oy + i // w
                        if 0 <= gc < gw and 0 <= gr < gh:
                            o = ((gh - 1 - gr) * gw + gc) * 3
                            buf[o:o + 3] = col
                            if v == 1:
                                occ_n += 1
                                if occ_bbox is None:
                                    occ_bbox = [gc, gr, gc, gr]
                                else:
                                    b = occ_bbox
                                    b[0] = min(b[0], gc)
                                    b[1] = min(b[1], gr)
                                    b[2] = max(b[2], gc)
                                    b[3] = max(b[3], gr)
                idx += run
        return buf, gw, gh, gx0, gy0, res, occ_bbox, occ_n

    def refresh(self):
        grids = self.panel.grids
        drones = self.panel.last_drones
        trails = self.panel.trails
        # 缩略图
        for did, t in self.thumbs.items():
            gd = grids.get(did)
            if not gd:
                continue
            img = self._thumb_img(gd, drones)
            t.setPixmap(QPixmap.fromImage(img.scaled(
                t.width() - 8, t.height() - 20, Qt.KeepAspectRatio,
                Qt.FastTransformation)))
        # 大图
        m = self._merged(grids)
        if m is None:
            self.big.setPixmap(QPixmap())
            self.big.setText("no grid yet — occ_grid 在闭环活跃后发布（起飞+goal）")
            return
        buf, gw, gh, gx0, gy0, res, bbox, occ_n = m
        pts = []
        for d in drones.values():
            p = (d or {}).get("pos")
            if p:
                pts.append(((p[0] - gx0) / res, (p[1] - gy0) / res))
        for tr in trails.values():
            for q in tr:
                pts.append(((q[0] - gx0) / res, (q[1] - gy0) / res))
        # 裁剪框 = 占用 bbox ∪ 轨迹/机位，各留 6 格边，clamp
        if bbox:
            gc0, gr0, gc1, gr1 = bbox
        else:
            gc0, gr0, gc1, gr1 = gw - 1, gh - 1, 0, 0
        for c, r in pts:
            gc0, gc1 = min(gc0, int(c)), max(gc1, int(c))
            gr0, gr1 = min(gr0, int(r)), max(gr1, int(r))
        if gc1 < gc0 or gr1 < gr0:      # 全未知且无机位 → 不裁剪
            gc0, gr0, gc1, gr1 = 0, 0, gw - 1, gh - 1
        pad = 6
        gc0, gr0 = max(0, gc0 - pad), max(0, gr0 - pad)
        gc1, gr1 = min(gw - 1, gc1 + pad), min(gh - 1, gr1 + pad)
        cw, ch = gc1 - gc0 + 1, gr1 - gr0 + 1
        W = max(200, self.big.width() - 10)
        H = max(150, self.big.height() - 10)
        k = max(0.5, min(min(W / cw, H / ch), 14.0))     # px/格
        px_w, px_h = int(cw * k), int(ch * k)
        cbuf = bytearray()
        for rowtop in range(ch):
            gr = gr1 - rowtop
            s = ((gh - 1 - gr) * gw + gc0) * 3
            cbuf += buf[s:s + cw * 3]
        img = QImage(bytes(cbuf), cw, ch, cw * 3,
                     QImage.Format_RGB888).scaled(
            px_w, px_h, Qt.IgnoreAspectRatio, Qt.FastTransformation)
        p = QPainter(img)

        def w2p(wx, wy):
            return (((wx - gx0) / res - gc0 + 0.5) * k,
                    ((gr1 - (wy - gy0) / res) + 0.5) * k)

        # 5m 淡网格线
        p.setPen(QPen(QColor(255, 255, 255, 16), 1))
        step = 5.0 / res
        c = (gc0 // step + 1) * step
        while c <= gc1:
            x = (c - gc0) * k
            p.drawLine(int(x), 0, int(x), px_h)
            c += step
        r = (gr0 // step + 1) * step
        while r <= gr1:
            y = (gr1 - r) * k
            p.drawLine(0, int(y), px_w, int(y))
            r += step
        # 轨迹尾迹（高亮单机外淡化）
        import math
        for did, tr in trails.items():
            col = self.DCOL.get(did, (46, 204, 113))
            alpha = 80 if (self.hl and did != self.hl) else 255
            p.setPen(QPen(QColor(*col, alpha), 2 if not self.hl else 1))
            poly = [QPointF(*w2p(q[0], q[1])) for q in tr]
            if len(poly) > 1:
                p.drawPolyline(QPolygonF(poly))
        # 机位航向箭头 + 编号
        f = QFont()
        f.setPixelSize(max(10, int(3.2 * k)))
        p.setFont(f)
        for did, d in drones.items():
            dd = d or {}
            p0 = dd.get("pos")
            if not p0:
                continue
            x, y = w2p(p0[0], p0[1])
            col = self.DCOL.get(did, (46, 204, 113))
            alpha = 80 if (self.hl and did != self.hl) else 255
            yaw = math.radians(dd.get("yaw") or 0.0)
            L = max(10.0, 4.5 * k)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(*col, alpha)))
            p.drawPolygon(QPolygonF([
                QPointF(x + L * math.cos(yaw), y - L * math.sin(yaw)),
                QPointF(x + 0.45 * L * math.cos(yaw + 2.5),
                        y - 0.45 * L * math.sin(yaw + 2.5)),
                QPointF(x + 0.45 * L * math.cos(yaw - 2.5),
                        y - 0.45 * L * math.sin(yaw - 2.5))]))
            p.setPen(QPen(QColor(*col, alpha)))
            p.drawText(QPointF(x + 5, y - 5), "d%s" % did)
        # 比例尺（取 ≤150px 的最大整档）
        ppm = k / res
        Lm = next((v for v in (50, 20, 10, 5, 2, 1) if v * ppm <= 150), 1)
        bx, by = 10, px_h - 12
        p.setPen(QPen(QColor(240, 240, 240), 2))
        p.drawLine(bx, by, int(bx + Lm * ppm), by)
        p.drawLine(bx, by - 4, bx, by + 4)
        p.drawLine(int(bx + Lm * ppm), by - 4, int(bx + Lm * ppm), by + 4)
        p.drawText(QPointF(bx + Lm * ppm + 5, by + 4), "%dm" % Lm)
        p.end()
        self.big.setPixmap(QPixmap.fromImage(img))
        self.info.setText(
            '<span style="color:rgb(230,126,34)">■</span>占用 '
            '<span style="color:rgb(90,100,115)">■</span>空闲 '
            '<span style="color:rgb(34,40,52)">■</span>未知 &nbsp; '
            + " ".join('<span style="color:rgb(%d,%d,%d)">■d%s</span>'
                       % (self.DCOL.get(d, (46, 204, 113)) + (d,))
                       for d in self.panel.ops.ids)
            + " &nbsp;| res %.1fm · 合并 %dx%d 格 · 占用 %.1f%% · "
              "点击缩略图高亮单机"
            % (res, gw, gh, 100.0 * occ_n / max(1, gw * gh)))


class DebugDialog(QDialog):
    """真机 ssh 调试窗（真机调试接口的 UI 面）：选机+选命名调试命令
    （profile ops.debug_cmds），stdout 回显进文本窗。

    约定只读：debug_cmds 只放查看类命令（ros/hz/proc/res/log）；
    一切写操作走正式命令层（real 两段确认），调试窗不给写通道。
    """

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.running = False
        self.setWindowTitle("fei tu A GCS - vehicle debug (ssh)")
        self.resize(760, 480)
        v = QVBoxLayout(self)
        row = QHBoxLayout()
        self.combo_d = QComboBox()
        for did in panel.ops.ids:
            self.combo_d.addItem("d%s" % did, did)
        self.combo_c = QComboBox()
        for name in sorted(panel.ops.debug_cmds):
            self.combo_c.addItem(name)
        self.args_ed = QLineEdit()
        self.args_ed.setPlaceholderText("{args} 参数（无需则空）")
        self.run_b = QPushButton("RUN")
        self.run_b.clicked.connect(self.run)
        row.addWidget(self.combo_d)
        row.addWidget(self.combo_c)
        row.addWidget(self.args_ed, 1)
        row.addWidget(self.run_b)
        v.addLayout(row)
        self.hint = QLabel(
            "只读约定: debug_cmds 仅查看类命令；写操作走命令层（两段确认）。"
            "命令集在 profile ops.debug_cmds 配置。")
        self.hint.setStyleSheet("color: %s;" % YEL)
        self.hint.setWordWrap(True)
        v.addWidget(self.hint)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(800)
        v.addWidget(self.out, 1)
        self.stream = SignalStream()
        self.stream.text.connect(self.out.appendPlainText)

    def run(self):
        if self.running or not self.panel.ops.debug_ssh:
            return
        # 互斥：与面板命令共享 ops 实例和 redirect_stdout（进程级全局，
        # 并发=输出串流）——面板忙时不给开跑；反过来面板命令侧也拒绝
        # 调试运行中触发（PANIC 豁免，急停不排队）
        if self.panel.busy:
            self.out.appendPlainText("[DBG] 面板命令执行中 — 稍后再试")
            return
        if not self.combo_c.currentText():
            self.out.appendPlainText("[DBG] profile 未配置 ops.debug_cmds")
            return
        self.running = True
        self.run_b.setEnabled(False)
        # 调试运行期间占住面板忙位（PANIC 仍可用——急停永不设障）
        self.panel.busy = True
        self.panel._set_buttons(False)
        did = self.combo_d.currentData()
        name = self.combo_c.currentText()
        cargs = self.args_ed.text().strip() or None
        ops = self.panel.ops

        def fn():
            import contextlib
            with contextlib.redirect_stdout(self.stream):
                try:
                    # debug_cli 返回 int rc（0/1/2）——必须显式 ==0，
                    # bool(rc) 会把成功(0)报成 False、失败报成 True
                    return ops.debug_cli(name, cargs, ids=[str(did)]) == 0
                except Exception as e:
                    print("debug exception: %s: %s" % (type(e).__name__, e))
                    return False

        self.w = Worker(fn)
        self.w.done.connect(self._done)
        self.w.start()

    def _done(self, ok, msg):
        self.running = False
        self.run_b.setEnabled(True)
        self.panel.busy = False
        self.panel._set_buttons(True)
        if msg:
            self.out.appendPlainText("[DBG] %s" % msg)
        self.out.appendPlainText("[DBG] finished ok=%s" % ok)


class Panel(QMainWindow):
    def __init__(self, ops, poll_ms=500, selftest=False):
        super().__init__()
        self.ops = ops
        self.selftest = selftest
        self.busy = False
        self._workers = []  # 运行中/已完成 Worker 的强引用（防 GC 杀线程）
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

        # 比分/任务指派条（agent 采 scorekeeper latched 话题 → hub 快照；
        # real 无 scorekeeper 显示 --）
        self.score_lbl = QLabel("SCORE --")
        self.score_lbl.setTextFormat(Qt.RichText)
        self.score_lbl.setStyleSheet("color: %s;" % TXT)
        root.addWidget(self.score_lbl)

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
        self.trails = {}      # did -> deque[(x, y, t)] 轨迹尾迹（地图窗用）
        self._map_dlg = None
        self._dbg_dlg = None  # 真机 ssh 调试窗（真机专用，sim 置灰）

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
        self.debug_btn = QPushButton("DEBUG")
        self.debug_btn.setEnabled(ops.debug_ssh is not None)
        if ops.debug_ssh:
            self.debug_btn.setToolTip(
                "真机 ssh 调试窗（ops.debug_cmds 命名只读命令）")
        else:
            self.debug_btn.setToolTip(
                "profile 未配置 ops.debug_ssh — 真机专用，sim 置灰")
        self.debug_btn.clicked.connect(self.toggle_debug)
        btns.addWidget(self.debug_btn)
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

    def toggle_debug(self):
        if not self.ops.debug_ssh:
            self.append("[DEBUG] profile 未配置 ops.debug_ssh — 不可用")
            return
        if self._dbg_dlg is None or not self._dbg_dlg.isVisible():
            self._dbg_dlg = DebugDialog(self)
            self._dbg_dlg.show()
        else:
            self._dbg_dlg.raise_()
            self._dbg_dlg.activateWindow()

    # ---- 500ms 轮询快照 -----------------------------------------------------
    def _trails_add(self, drones, now):
        """轨迹采样：>15m 跳变=重启/重定位，清旧迹防跨场拉线。"""
        for did, d in drones.items():
            p = (d or {}).get("pos")
            if not p:
                continue
            tr = self.trails.setdefault(did, deque(maxlen=1500))
            if tr and abs(p[0] - tr[-1][0]) + abs(p[1] - tr[-1][1]) > 15.0:
                tr.clear()
            tr.append((p[0], p[1], now))

    @staticmethod
    def _si(v):
        """分值强转防御：畸形 hub 透传值炸 poll 槽=面板静默冻结。"""
        try:
            return int(v or 0)
        except (TypeError, ValueError):
            return 0

    def _set_score(self, sc, ms, tk=None, tot=None):
        """比分/任务指派条（比赛口径 S=S1+S2）：
        S1 n | S2 n | 落 k/6 | 总 n ‖ d0 A●红→P1 ✓ (10分) [退赛]。

        sc=scores ms=missions tk=task_update 事件 tot=赛末 summary（real 无
        scorekeeper 显示 --）。富文本——所有插值字段过 escape（hub/agent
        侧数据不上 HTML）。"""
        if not sc and not ms and not tk and not tot:
            self.score_lbl.setText("SCORE --  (sim: scorekeeper 触发后 latched)")
            return
        landed_n = sum(1 for t in (tk or {}).values()
                       if str((t or {}).get("state")) == "LANDED")
        if tot and "s2" in tot:
            parts = ['<b>S1 %d</b>' % self._si(tot.get("s1")),
                     '<b>S2 %d</b>' % self._si(tot.get("s2")),
                     '落 %d/%d' % (self._si(tot.get("landed")),
                                   len(self.ops.ids)),
                     '<b>总 %d</b>' % self._si(tot.get("total"))]
        else:
            # 赛末 summary 未出（比赛进行中）：S1/落地数走实时源
            s1 = sum(self._si((s or {}).get("s1", (s or {}).get("score")))
                     for s in sc.values())
            parts = ['S1 %d' % s1, 'S2 --',
                     '落 %d/%d' % (landed_n, len(self.ops.ids)), '总 --']
        for did in self.ops.ids:
            m = ms.get(did) or {}
            s = sc.get(did)
            t = (tk or {}).get(did) or {}
            if not m and not s and not t:
                continue
            # 指派段：d0 A●红→P1（箱色色卡来自 Mission.box_color）
            ptype = str(m.get("payload") or "")
            color = str(m.get("box_color") or "")
            cname = {"red": "红", "blue": "蓝", "yellow": "黄"}.get(color, "")
            ccol = {"red": RED, "blue": "#3498db",
                    "yellow": YEL}.get(color, DIM)
            seg = "d%s %s" % (did, escape(ptype[-1]) if ptype else "?")
            if cname:
                seg += '<span style="color:%s">●%s</span>' % (ccol, cname)
            seg += "→P%s" % escape(str(m.get("drop", "?")))
            st = str(t.get("state") or "")
            if st == "RETIRED" or (s or {}).get("retire"):
                seg += ' <span style="color:%s"><b>[退赛]</b></span>' % RED
            elif st == "LANDED":
                seg += ' <span style="color:%s">✓</span>' % GRN
            if s:
                seg += " <b>%d分</b>" % self._si(s.get("s1", s.get("score")))
            parts.append(seg)
        self.score_lbl.setText("  |  ".join(parts))

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
        sc = snap.get("scores") or {}
        ms = snap.get("missions") or {}
        self._set_score(sc, ms, snap.get("tasks") or {},
                        snap.get("score_total") or {})
        self._trails_add(drones, now)
        for did, card in self.cards.items():
            d = drones.get(did)
            age = (d or {}).get("age")
            link_ok = age is not None and age <= self.ops.link_lost_s
            card.set_state(d, link_ok,
                           bool((d or {}).get("stall_on")),
                           bool((d or {}).get("pdead_on")),
                           bool((d or {}).get("bat_on")),
                           score=sc.get(did))
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
        if self.busy and name != "panic":
            # 忙位拒绝必须可见（静默吞命令=操作员以为已下发）
            self.append("[CMD] %s REFUSED — 另一命令/调试运行中" % name)
            return
        # panic 豁免：命令/调试运行中急停照发（急停永不排队设障；
        # 并发 redirect_stdout 只乱打印归属，不影响下发）
        self.busy = True
        self._set_buttons(False)
        ops = self.ops

        def fn():
            import contextlib
            with contextlib.redirect_stdout(self.stream):
                return ops.dispatch(name, ids=ids)

        w = Worker(fn)
        self._workers.append(w)  # 保引用——覆盖 self.w 会被 GC 杀运行中线程
        w.done.connect(self.on_cmd_done)
        w.start()
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
            self.append("[START] REFUSED — 另一命令/调试运行中")
            return
        self.busy = True
        self._set_buttons(False)
        ops = self.ops
        mt = ops.execute_timeout_s + 60.0

        def fn():
            import contextlib
            with contextlib.redirect_stdout(self.stream):
                return ops.start(False, mt)

        w = Worker(fn)
        self._workers.append(w)  # 保引用——覆盖 self.w 会被 GC 杀运行中线程
        w.done.connect(self.on_cmd_done)
        w.start()
        self.append("[START] real flow: gates->takeoff->airborne->trigger"
                    "->monitor")

    def on_cmd_done(self, ok, msg):
        self.busy = False
        self._set_buttons(True)
        self._fired = True  # selftest 只自动触发一次
        if msg:
            self.append("[CMD] %s" % msg)
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
