#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w7_selftest.py — W7 冠撞修复三臂离线自测（C1/C2/C3 默认关基线保护）。

用法（WSL）：
  cd ~/zx2026_arena_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/w7_selftest.py

结果写 /tmp/w7_selftest.txt（UTF-8，规避 GBK 控制台）。

覆盖：
  A. tip_band_scan 前向锥扫描（C3 核心，nav_node 模块级纯函数）：锥内命中/
     锥外剔除/z 带剔除/距离上限/最近边距/指令反向
  B. C3 v_cap 数学：v_cap=(d_edge-margin)/react_t、地板/顶盖钳位（与
     _apply_tip_slow 同式契约）
  C. sim_settings 三旗默认关断言 + 非默认值翻转读回（mem6 教训：干测须含
     非默认值断言，防"默认漂移"与"旗子翻了没接上"）
  D. C2 hazard JSON 往返 + xy→格换算（_g_to_cell 逐字镜像：截断+钳位）+
     own-id/畸形包吞噬语义
"""
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/zx2026_arena_ws")
# 只插 arena_nav：nav_node 模块顶要 import zx2026_common.msg（devel 生成包），
# 与 vo_selftest/bh_selftest 同款结构。
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_nav", "scripts"))

try:
    from nav_node import tip_band_scan
except ImportError as e:
    sys.stderr.write("import nav_node 失败(%s)。\n请先: source /opt/ros/noetic/"
                     "setup.bash && source devel/setup.bash\n" % e)
    sys.exit(2)

import yaml

CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
OUT = "/tmp/w7_selftest.txt"

FAILS = []


def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    if not cond:
        FAILS.append(name)
    print("  [%s] %s %s" % (tag, name, detail))


def t_tip_scan():
    print("A. tip_band_scan 前向锥扫描")
    px, py = 0.0, 0.0
    ux, uy = 1.0, 0.0                       # 朝 +x
    cos35 = math.cos(math.radians(35.0))
    pts = [
        (1.5, 0.0, 2.0),    # 正前 1.5 → 命中
        (1.0, 0.5, 2.2),    # 夹角 26.6°<35° → 命中
        (1.0, 1.5, 2.0),    # 56.3° 锥外 → 剔除
        (2.0, 0.0, 1.2),    # z<1.5 → 剔除
        (2.0, 0.0, 3.2),    # z>3.0 → 剔除
        (3.0, 0.0, 2.0),    # 距离>engage_r 2.5 → 剔除
        (0.5, 0.0, 2.0),    # 正前 0.5 → 命中（最近）
        (-1.0, 0.0, 2.0),   # 正后方 180° → 剔除
    ]
    d, n = tip_band_scan(pts, px, py, ux, uy, 1.5, 3.0, cos35, 2.5)
    check("A1 锥内命中数=3", n == 3, "n=%d" % n)
    check("A2 最近边距 d_edge=0.5", d is not None and abs(d - 0.5) < 1e-9,
          "d=%s" % d)
    d, n = tip_band_scan([(-1.0, 0.0, 2.0)], px, py, ux, uy, 1.5, 3.0,
                         cos35, 2.5)
    check("A3 全锥外 → (None,0)", d is None and n == 0)
    d, n = tip_band_scan(pts, px, py, -1.0, 0.0, 1.5, 3.0, cos35, 2.5)
    # 反向指令下原"正后方"点 (-1,0,2.0) 变为锥内前方（点积门控正确语义），
    # 原前方点全部出局
    check("A4 指令反向 → 仅后方点入锥 (n=1,d=1.0)",
          n == 1 and d is not None and abs(d - 1.0) < 1e-9,
          "n=%d d=%s" % (n, d))
    # 锥半角边界感：30° < 35° 命中、40° > 35° 剔除
    d, n = tip_band_scan([(2.0, 1.0, 2.0)], px, py, ux, uy, 1.5, 3.0,
                         cos35, 2.5)   # 26.57°
    check("A5 26.6° 命中", n == 1)
    d, n = tip_band_scan([(1.0, 0.9, 2.0)], px, py, ux, uy, 1.5, 3.0,
                         cos35, 2.5)   # 41.99°
    check("A6 42.0° 锥外剔除", n == 0)


def t_tip_cap_math():
    print("B. C3 v_cap 数学（与 _apply_tip_slow 同式契约）")
    max_vel, floor_frac, margin, react_t = 1.8, 0.35, 0.5, 0.41

    def v_cap(d):
        return max(floor_frac * max_vel, min(max_vel, (d - margin) / react_t))

    check("B1 远距 2.5 → 顶盖 1.8", abs(v_cap(2.5) - 1.8) < 1e-9,
          "%.3f" % v_cap(2.5))
    check("B2 d=1.0 → (0.5)/0.41=1.22", abs(v_cap(1.0) - 0.5 / 0.41) < 1e-9,
          "%.3f" % v_cap(1.0))
    check("B3 d=0.6 → 0.244 → 地板 0.63", abs(v_cap(0.6) - 0.63) < 1e-9,
          "%.3f" % v_cap(0.6))
    check("B4 d=margin → 0 → 地板", abs(v_cap(0.5) - 0.63) < 1e-9)
    check("B5 d=0.9 → 0.976（逼近带内咬合）",
          abs(v_cap(0.9) - 0.4 / 0.41) < 1e-9, "%.3f" % v_cap(0.9))
    check("B6 地板=0.35×max_vel（与 vcap 地板同源）",
          abs(floor_frac * max_vel - 0.63) < 1e-9)


def t_settings():
    print("C. sim_settings 三旗默认断言 + 非默认翻转读回")
    with io.open(CFG, encoding="utf-8") as f:
        st = yaml.safe_load(f)
    cl = st["closed_loop"]
    ph = cl["post_hit_calm"]
    hz = cl["hazard_share"]
    ts = cl["branch_handling"]["tip_slow"]
    # 20260920 w7_matrix 过线翻默认：C1+C2 绑定开（B1 单开判毙不可拆），
    # C3 tip_slow 仍默认关（pilot 未跑）
    check("C1 post_hit_calm 默认开（矩阵过线）", ph["enabled"] is True)
    check("C2 hazard_share 默认开（与 C1 绑定）", hz["enabled"] is True)
    check("C3 tip_slow 默认关（pilot 未跑）", ts["enabled"] is False)
    check("C4 C1 参数 hold=3 vcap=0.8 vcap_s=2",
          ph["hold_s"] == 3.0 and ph["vcap"] == 0.8 and ph["vcap_s"] == 2.0)
    check("C5 C2 topic=/zx2026/hazard_cells",
          hz["topic"] == "/zx2026/hazard_cells")
    check("C6 C3 参数 react_t=0.41 margin=0.5 min_pts=2 cone=35",
          ts["react_t"] == 0.41 and ts["margin"] == 0.5
          and ts["min_pts"] == 2 and ts["cone_deg"] == 35)
    check("C7 C3 z_band=[0.7,2.6]（杨树林冠下巡航带）",
          ts["z_band"] == [0.70, 2.60])
    # 非默认值断言（mem6 教训）：翻旗后 yaml 侧读回翻转值——plumbing 双向可逆
    blob = io.open(CFG, encoding="utf-8").read()
    for key in ("post_hit_calm", "hazard_share"):
        flipped = blob.replace("%s:\n    enabled: true" % key,
                               "%s:\n    enabled: false" % key)
        st2 = yaml.safe_load(flipped)
        seg = st2["closed_loop"][key]
        check("C8 %s 翻回 false 读回 False" % key, seg["enabled"] is False)
    flipped = blob.replace("tip_slow:\n      enabled: false",
                           "tip_slow:\n      enabled: true")
    st2 = yaml.safe_load(flipped)
    check("C9 tip_slow 翻旗读回 true",
          st2["closed_loop"]["branch_handling"]["tip_slow"]["enabled"] is True)
    # 诚实化 20260923（作弊审查整改）：真值泄漏面翻默认断言
    vs = (st.get("mission", {}) or {}).get("via_slots", {}) or {}
    check("C10 via_slots source=cloud（选槽查自己点云，truth=god 臂）",
          vs.get("source") == "cloud")
    rules_p = os.path.join(WS, "src/zx2026_common/config/competition_rules.yaml")
    rules = yaml.safe_load(io.open(rules_p, encoding="utf-8"))
    check("C11 color_id source=camera（识别诚实化，truth=god 臂）",
          (rules.get("color_id", {}) or {}).get("source") == "camera")
    check("C12 flight_z_ceiling=4.5（行为封顶，距 OVER_HEIGHT 5.0 保 0.5m）",
          (rules.get("heights", {}) or {}).get("flight_z_ceiling") == 4.5)


def t_hazard():
    print("D. C2 hazard JSON 往返 + 格换算镜像")
    # _g_to_cell 逐字镜像（nav_node）：截断 + 钳位
    x0, y0, nx, ny, res = -10.0, -10.0, 40, 40, 0.5

    def g_to_cell(x, y):
        ix = int((x - x0) / res)
        iy = int((y - y0) / res)
        return (max(0, min(nx - 1, ix)), max(0, min(ny - 1, iy)))

    msg = json.dumps({"id": 2, "x": -4.76, "y": 7.88, "t": 126.0})
    d = json.loads(msg)
    cx, cy = g_to_cell(float(d["x"]), float(d["y"]))
    check("D1 往返+取格=(-4.76,7.88)→(10,35)", (cx, cy) == (10, 35),
          "(%d,%d)" % (cx, cy))
    check("D2 own-id 跳过判据", int(d["id"]) != 0)
    # 畸形包安全吞噬（坏 JSON / 缺 x / 缺 xy）
    bad = 0
    for s in ("{oops", '{"id":1}', '{"id":3,"y":1.0}'):
        try:
            dd = json.loads(s)
            g_to_cell(float(dd["x"]), float(dd["y"]))
        except Exception:
            bad += 1
    check("D3 畸形包吞噬=3", bad == 3)
    # 钳位语义：界外坐标→边缘格（镜像 nav 行为，供判读）
    cx, cy = g_to_cell(99.0, 99.0)
    check("D4 界外钳位到边缘格", (cx, cy) == (nx - 1, ny - 1),
          "(%d,%d)" % (cx, cy))


def t_branch_order():
    print("E. _de_tick 分支结构哨兵（W7 分支错位回归）")
    # 错位形态：engagement 主体挂进 HOLD-defer 分支 → B 臂 hold 内 20Hz
    # "engaged" 日志风暴+pick 空转、A 臂 engage 永久静默。正确序：engaged
    # 主体在扣压分支之前（源码序）。
    src = io.open(os.path.join(WS, "src", "arena_nav", "scripts",
                               "nav_node.py"), encoding="utf-8").read()
    i_engaged = src.find("DEAD-END escape engaged")
    # 源码里该日志串断行为两个相邻字面量（"...engage " + "deferred (...)"），
    # 连续搜索须截到断行点（mem6 教训：干测断言先对源码实测）
    i_defer = src.find("POST-HIT HOLD escape engage")
    check("E1 engaged 主体在 HOLD-defer 之前", 0 < i_engaged < i_defer,
          "engaged@%d defer@%d" % (i_engaged, i_defer))
    # 扣压分支内不得有 pick/状态初始化（风暴源头）；窗取到 _de_tick 函数尾
    i_next = src.find("def _de_dir_str", i_defer)
    tail = src[i_defer:i_next] if i_next > 0 else src[i_defer:i_defer + 300]
    check("E2 HOLD-defer 分支无 _de_pick_dir", "_de_pick_dir" not in tail)


def main():
    lines = []
    orig = sys.stdout
    buf = io.BytesIO()

    class Tee(object):
        def write(self, s):
            orig.write(s)
            buf.write(s.encode("utf-8"))

        def flush(self):
            orig.flush()

    sys.stdout = Tee()
    print("=== tools/w7_selftest.py — W7 冠撞修复三臂离线自测 ===")
    t_tip_scan()
    t_tip_cap_math()
    t_settings()
    t_hazard()
    t_branch_order()
    print("")
    if FAILS:
        print("结果: %d FAIL → %s" % (len(FAILS), ", ".join(FAILS)))
    else:
        print("结果: ALL PASS")
    sys.stdout = orig
    with io.open(OUT, "w", encoding="utf-8") as f:
        f.write(buf.getvalue().decode("utf-8"))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
