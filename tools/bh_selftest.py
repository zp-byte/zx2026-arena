#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/bh_selftest.py — 真机树枝避障 O1/O4 决策核心离线自测（O5 风摆膨胀
已随 20260912 判占线重设计退役）。

用法（WSL）：
  cd ~/zx2026_arena_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/bh_selftest.py

结果写 /tmp/bh_selftest.txt（UTF-8，规避 GBK 控制台）。

覆盖（nav_node 模块级纯函数 + 真实 Scene 几何集成）：
  A. bh_evidence_tier 判占分档：判占线=sway_dense_factor×T（近 6/远 12）——
     细枝稀疏回波（1-5 点）停留不确定档不封格，密集回波（≥判占线=树干/
     密枝簇签名）才判占；窗口遗忘/重观测刷新（20260912 分支矩阵定论：
     旧 T 线判占把细枝升 occ 闭合树间走廊 → B 臂 0/3 PASS，判占线移密度
     线——A2 即根因回归测试）
  A9 带下限>围栏顶 1.30 配置回归门（20260912_223444 矩阵根因：带 [0.8,2.0]
     把 1.30m 围栏环放进证据积分=实心墙判占 → pads→场内 2D A* 只能绕环外
     → OOB 全灭；下限抬 1.45 围栏/平台/桶全排除，A9 双查配置与常量防回退）
  B. bh_uncertain_t 走廊扫描：正中命中/侧向/后方/超距/全空
  C. 集成（真实 Scene + 模拟飞行）：无人机沿枝轴从枝梢侧飞向带枝树——
     C1 首个注入圈：注入格（稀疏签名载体）计数 ≤3 且 O4 走廊扫描 t>0
     C2 稀疏注入格接近期零判占（细枝单点/圈回波不封格——矩阵根因回归）
     C3 密集回波（同圈 6 点同格，树干/密枝簇签名）注入后枝格判占（A* 绕行）
     C4 无新观测 4 周期后窗口遗忘（"摆走的枝不再挡路"）
     C5 接近全程存在 t>0 的圈（O4 跨格限速持续生效）
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/zx2026_arena_ws")
# 只插 arena_nav：nav_node 模块顶要 import zx2026_common.msg（devel 生成包），
# 若把 src/zx2026_common/scripts 插到 sys.path 首位会遮蔽生成包（源包无 msg
# 子包）→ No module named 'zx2026_common.msg'（与 vo_selftest 同款结构）。
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_nav", "scripts"))

try:
    from nav_node import bh_evidence_tier, bh_uncertain_t
except ImportError as e:
    sys.stderr.write("import nav_node 失败(%s)。\n请先: source /opt/ros/noetic/"
                     "setup.bash && source devel/setup.bash\n" % e)
    sys.exit(2)

import numpy as np

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene

CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
OUT = "/tmp/bh_selftest.txt"

# 与 sim_settings branch_handling 同值（selftest 用常量，配置变更时此处同步）
BH_WINDOW_CYCLES = 3        # 1.5s / 0.5s 重建周期
BH_T_NEAR = 2.0
BH_T_FAR = 4.0
BH_TRUST_R = 5.0
BH_SWAY_DENSE = 3.0       # 判占密度倍数：occ 线 = dense×T（近 6/远 12）
BH_UNC_LAT = 0.15
BH_UNC_RAY = 4.0  # 与 sim_settings uncertain.ray 同步：4.0=枝首见距离(≈3.5m)内即进限速窗
RES = 0.5
LIDAR_AZ = 72
LIDAR_EL = [-0.5, -0.25, 0.0, 0.25, 0.5]
RANGE = 5.0
MAP_BAND = (1.45, 2.0)  # 下限=围栏顶 1.30+0.15（硬不变量，见 A9 与 sim_settings 注释）

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))


def patch_branches(value):
    with open(CFG, encoding="utf-8") as f:
        lines = f.readlines()
    blk_indent = None
    for i, ln in enumerate(lines):
        m = re.match(r"^(\s*)branches\s*:", ln)
        if m:
            blk_indent = len(m.group(1))
            continue
        if blk_indent is not None:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            if len(ln) - len(ln.lstrip()) <= blk_indent:
                break
            if re.match(r"enabled\s*:", s):
                lines[i] = re.sub(r"(enabled\s*:\s*)(false|true)",
                                  lambda mm: mm.group(1) + value, ln, count=1)
                break
    with open(CFG, "w", encoding="utf-8") as f:
        f.writelines(lines)


# ---- A. 判占分档 -----------------------------------------------------------
def test_tier():
    # 单格世界系坐标函数（偏移 -5：(10,10) → (0.25,0.25) 在 trust_r=5 内=近档；
    # (60,60) → (25.25,25.25) 远档）
    def g2w(ix, iy):
        return (-5.0 + (ix + 0.5) * RES, -5.0 + (iy + 0.5) * RES)

    # A1 一点不判占（近档，单次闪烁/噪声）
    ev = {(10, 10): [1.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A1 单点近档→unc 非 occ",
          (10, 10) in unc and (10, 10) not in occ,
          "unc=%s occ=%s" % (len(unc), len(occ)))
    # A2 两点近档不判占（20260912 矩阵根因回归：旧 T 线判占把细枝 2 次回波
    # 升 occ → 枝格+膨胀闭合树间走廊 → 北绕界外 OOB + 西墙滑扫）
    ev = {(10, 10): [2.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A2 两点近档→unc 非 occ（细枝不封格）",
          (10, 10) in unc and (10, 10) not in occ,
          "unc=%s occ=%s" % ((10, 10) in unc, (10, 10) in occ))
    # A3 五点近档不判占（判占线 dense×T_near=6 之下边界）
    ev = {(10, 10): [5.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A3 五点近档→unc（判占线之下）",
          (10, 10) in unc and (10, 10) not in occ,
          "unc=%s occ=%s" % ((10, 10) in unc, (10, 10) in occ))
    # A4 六点近档判占（判占线达成=树干/密枝簇签名）
    ev = {(10, 10): [6.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A4 六点近档→occ（判占线达成）",
          (10, 10) in occ and (10, 10) not in unc,
          "occ=%s" % ((10, 10) in occ))
    # A5 远档十一点不判占（远档线 dense×T_far=12 之下）
    ev = {(60, 60): [11.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A5 远档十一点→unc（远档线之下）",
          (60, 60) in unc and (60, 60) not in occ,
          "unc=%s occ=%s" % ((60, 60) in unc, (60, 60) in occ))
    # A6 远档十二点判占
    ev = {(60, 60): [12.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A6 远档十二点→occ",
          (60, 60) in occ and (60, 60) not in unc,
          "occ=%s" % ((60, 60) in occ))
    # A7 窗口遗忘：cycle0 观测，cycle4 重建 → 删（4-0>3）
    ev = {(10, 10): [2.0, 0]}
    bh_evidence_tier(ev, 4, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
                     BH_TRUST_R, BH_SWAY_DENSE, 100, 100, g2w,
                     np.array([0.0, 0.0]))
    check("A7 窗口遗忘", (10, 10) not in ev, "ev=%s" % ev)
    # A8 重观测刷新：cycle0 观测，cycle3 重观测，cycle4 重建 → 保留
    ev = {(10, 10): [2.0, 3]}
    bh_evidence_tier(ev, 4, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
                     BH_TRUST_R, BH_SWAY_DENSE, 100, 100, g2w,
                     np.array([0.0, 0.0]))
    check("A8 重观测刷新", (10, 10) in ev, "ev=%s" % ev)


# ---- A9. 带下限>围栏顶（配置回归门） ---------------------------------------
def test_band_fence_guard():
    """20260912_223444 矩阵根因回归门。

    bh 带下限必须 > 围栏顶 1.30（+0.10 裕度）：围栏环（西 x≈-18.6 跨
    y±15.5、南北沿 geofence 边、高 1.30）一旦进带即实心墙判占（密度线
    不救墙——墙回波远超任何判占线）→ pads（环外）→ 场内 2D A* 只能绕
    环外 → 必然 OOB 全灭（B 臂 0/3，六机 y≈±17.3 悬停锁退赛=围栏端点
    15.65+膨胀+栅格的自由通道）。带上限须盖住 cruise 1.8。配置与自测
    常量双查（MAP_BAND 漂移=复刻的 _on_cloud 带过滤与实机配置失同步）。
    """
    bh = cfg.load("sim_settings.yaml").get("closed_loop", {}).get(
        "branch_handling", {})
    band = bh.get("map_z_band", [])
    ok = (len(band) == 2 and band[0] > 1.30 + 0.10
          and band[1] >= 1.8 and tuple(band) == MAP_BAND)
    check("A9 带下限>围栏顶+常量同步", ok,
          "config=%s MAP_BAND=%s" % (band, list(MAP_BAND)))


# ---- B. 不确定走廊扫描 ------------------------------------------------------
def test_uncertain_scan():
    ny = nx = 20
    unc = np.zeros((ny, nx), dtype=bool)
    # 目标格 (10,10)，世界系 = (0,0)（g_x0=-5）
    def g2w(ix, iy):
        return (-5.0 + (ix + 0.5) * RES, -5.0 + (iy + 0.5) * RES)

    def g2c(x, y):
        ix = int((x + 5.0) / RES)
        iy = int((y + 5.0) / RES)
        return (max(0, min(nx - 1, ix)), max(0, min(ny - 1, iy)))

    # 注意 gocc_unc 布局为 [iy, ix]（行=iy，列=ix），g2c 返回 (ix, iy)。
    # B1 正中命中：格 ix=14, iy=10 中心 (2.25, 0.25)，走廊内 → unc[10,14]
    unc[10, 14] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B1 正中命中", t is not None and 2.0 <= t <= 2.5, "t=%s" % t)
    # B2 侧向过滤：格 ix=10, iy=11 中心 (0.25, 0.75)，lat=0.75 > corridor
    unc[:] = False
    unc[11, 10] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B2 侧向过滤", t is None, "t=%s" % t)
    # B3 后方过滤：格 ix=8, iy=10 中心 (-1.75, 0.25) → 沿轨 t_c=-1.75
    unc[:] = False
    unc[10, 8] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B3 后方过滤", t is None, "t=%s" % t)
    # B4 超距过滤（ray=4.0 外）：格 ix=18, iy=10 中心 (4.25, 0.25) > 4.0
    unc[:] = False
    unc[10, 18] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B4 超距过滤", t is None, "t=%s" % t)
    # B5 全空
    unc[:] = False
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B5 全空", t is None, "t=%s" % t)


# ---- C. 集成：模拟飞行（真实 Scene 几何 + 决策核心） -------------------------
def test_integration():
    patch_branches("true")
    try:
        scene = Scene()
    finally:
        patch_branches("false")
    v = scene.venue
    g_x0, g_y0 = -v["size"][0] / 2.0, -v["size"][1] / 2.0
    g_nx, g_ny = int(v["size"][0] / RES), int(v["size"][1] / RES)

    def g2c(x, y):
        ix = int((x - g_x0) / RES)
        iy = int((y - g_y0) / RES)
        return (max(0, min(g_nx - 1, ix)), max(0, min(g_ny - 1, iy)))

    def g2w(ix, iy):
        return (g_x0 + (ix + 0.5) * RES, g_y0 + (iy + 0.5) * RES)

    # 候选枝：枝梢（注入射线瞄准点）z∈[1.52,1.77]——≥1.52=梢部命中点
    # （z≥梢-0.082，枝半径+余量）稳进带 [1.45,2.0] 可被映射；≤1.77=
    # 枝面顶 ≤1.79 低于无人机 1.8m 巡航，机在枝上方飞过不共线相撞、
    # 方位角任意（飞行方向沿枝轴向）。20260912_223444 带下限 0.8→1.45
    # （围栏排除）后窗口随带上移（原 zmax∈[0.85,1.4]）。
    cand = []
    for ob in scene.obstacles:
        if ob.kind != "tree" or not ob.branches:
            continue
        for br in ob.branches:
            if 1.52 <= br[5] <= 1.77:
                cand.append((ob, br))
    if not cand:
        check("C 集成", False, "无满足 z 约束的枝（梢 z∈[1.52,1.77]）")
        return
    FLY_Z = 1.8
    # 试候选，双准则（z=FLY_Z 沿枝轴向 6 站位 tip-3.0..tip-0.5）：
    #  (a) nclear：点碰撞前缀（collides(...,0.0)=机点不埋入任何障碍）——净空
    #      不能是全球(0.35)：同树其他枝 z0∈[0.8,2.6] 均匀分布，横穿 1.8m
    #      高度概率 ~40%，64 条路径 0 条全球净空（实测），而飞行路径只是
    #      观测台，枝贴近/擦过路径恰是被测场景本体；
    #  (b) vis：对枝首直接射线可达链——实测 64 候选在 ≥1.5m 外 0% 可达
    #      （同树冠层遮挡=稀疏回波问题本体），贴到 tip-1.0~0.5m 才可见，
    #      且此时枝仰角 ~-44° 落在 72×5 lidar 的 el 网格（±14.3° 步进）外，
    #      真实模式采样率趋零 → 枝观测用"可达性门控注入"（见飞行循环）：
    #      每圈 1 点，等价 branch_selftest 3 实证的稀疏回波确定性替身。
    # 评分 (nclear, vc) 字典序，vc=飞行窗口内可见步数 ≥2 才录取（密集回波
    # 注入需要枝梢末位置可达，C3 判占的前提）。
    best = None
    for (ob, br) in cand:
        az = math.atan2(br[4] - br[1], br[3] - br[0])
        ux, uy = math.cos(az), math.sin(az)
        tip = (br[3], br[4], br[5])
        nclear = 0
        vis = []
        for k in range(6):
            px = tip[0] - ux * (3.0 - 0.5 * k)
            py = tip[1] - uy * (3.0 - 0.5 * k)
            if scene.collides((px, py, FLY_Z), 0.0):
                vis.append(False)
                break
            nclear += 1
            vx = tip[0] - px
            vy = tip[1] - py
            vz = tip[2] - FLY_Z
            vlen = math.hypot(vx, vy, vz)
            if vlen < 1e-9:
                vis.append(False)
                continue
            d = (vx / vlen, vy / vlen, vz / vlen)
            t = scene.raycast((px, py, FLY_Z), d, max_t=vlen + 0.05)
            ok = False
            if t is not None and t <= vlen + 0.05:
                p = (px + d[0] * t, py + d[1] * t, FLY_Z + d[2] * t)
                ok = geo.point_segment_dist(p, br[:3], br[3:6]) <= br[6] + 0.06
            vis.append(ok)
        # 可见步数按飞行循环实际到访的 6 站位计（N_STEPS=min(6,nclear)）：
        # 旧 min(5,·) 漏数 tip-0.5 末站——带下限抬 1.45 后枝梢贴近巡航
        # 高度、射线浅，可见性只在末两站（tip-1.0/0.5）出现 → 全被误拒
        # （20260912_223444 带修复回归首跑 C 集成 0 候选的根因）。
        flight = min(6, nclear)
        vc = sum(vis[:flight])
        if vc < 2 or nclear < 3:
            continue
        score = (nclear, vc)
        if best is None or score > best[0]:
            best = (score, ob, br, az, vis)
    if best is None:
        check("C 集成", False, "无 (nclear≥3, 可见≥2) 候选")
        return
    (nclear, vc), ob, br, az, vis = best
    ux, uy = math.cos(az), math.sin(az)
    yaw = az + math.pi  # 机头朝树（射线模式旋转，同 lidar_node odom_yaw）
    (sx, sy, sz, ex, ey, ez, brr) = br
    tip = (ex, ey, ez)
    N_STEPS = min(6, nclear)

    def is_branch_pt(pt):
        return geo.point_segment_dist(pt, (sx, sy, sz), (ex, ey, ez)) \
            <= brr + 0.02

    occ_ev = {}
    rebuild_n = 0
    phases = []
    # 阶段1 接近：枝梢 3.0m → 3.0-0.5*(N_STEPS-1) m，0.5m/步（0.5s 每步 →
    # ~1 m/s），z=FLY_Z。枝观测=可达性门控注入：每圈对枝首做真实 raycast，
    # 命中枝（dist≤brr+0.06）才注入 1 点（该命中点）——稀疏回波确定性替身
    # （72×5 el 网格步进 14.3°，2cm 枝的采样率趋零；稀疏签名由 branch_selftest
    # 3 独立实证）。环境格（树干/邻树/地面）仍全部来自真实 lidar 复刻。
    for step_i in range(N_STEPS):
        px = tip[0] - ux * (3.0 - 0.5 * step_i)
        py = tip[1] - uy * (3.0 - 0.5 * step_i)
        pos = (px, py, FLY_Z)
        pts = []
        for az_i in range(LIDAR_AZ):
            azr = yaw + az_i * (2.0 * math.pi / LIDAR_AZ)
            for el in LIDAR_EL:
                d = geo.normalize((math.cos(el) * math.cos(azr),
                                   math.cos(el) * math.sin(azr), math.sin(el)))
                t = scene.raycast(pos, d, max_t=RANGE)
                if t is not None:
                    pts.append((pos[0] + d[0] * t, pos[1] + d[1] * t,
                                pos[2] + d[2] * t))
        # 注入：对枝首直接射线（同选择阶段的 vis 判据）。命中点 z∈[梢z,1.8]
        # ⊂ 带 [1.45,2.0]（梢窗 [1.52,1.77] 保证）——注入格=稀疏签名载体
        # （1 点/圈；带下限抬 1.45 后枝贴巡航高度，直射 lidar 对近段枝格
        # 本身就密（实测 27 计数→occ，密目标语义正确），稀疏断言只看注入格）。
        inj_cell = None
        vx = tip[0] - px
        vy = tip[1] - py
        vz = tip[2] - FLY_Z
        vlen = math.hypot(vx, vy, vz)
        if vlen > 1e-9:
            d = (vx / vlen, vy / vlen, vz / vlen)
            t = scene.raycast(pos, d, max_t=vlen + 0.05)
            if t is not None and t <= vlen + 0.05:
                p = (px + d[0] * t, py + d[1] * t, FLY_Z + d[2] * t)
                if geo.point_segment_dist(p, br[:3], br[3:6]) <= brr + 0.06:
                    pts.append(p)
                    inj_cell = g2c(p[0], p[1])
        # _on_cloud 复刻（O1 分支：band + 证据计数），同时分类枝点/枝格
        b_pts = 0
        b_cells = set()
        for (x, y, z) in pts:
            if MAP_BAND[0] <= z <= MAP_BAND[1]:
                c = g2c(x, y)
                ev = occ_ev.get(c)
                if ev is None:
                    occ_ev[c] = [1.0, rebuild_n]
                else:
                    ev[0] += 1.0
                    ev[1] = rebuild_n
                if is_branch_pt((x, y, z)):
                    b_pts += 1
                    b_cells.add(c)
        # 重建（bh_evidence_tier 真函数）
        occ, unc = bh_evidence_tier(
            occ_ev, rebuild_n, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
            BH_TRUST_R, BH_SWAY_DENSE, g_nx, g_ny, g2w,
            np.array([px, py]))
        rebuild_n += 1
        # 走廊扫描（bh_uncertain_t 真函数，指令方向=朝树）
        gocc_unc = np.zeros((g_ny, g_nx), dtype=bool)
        for (ix, iy) in unc:
            gocc_unc[iy, ix] = True
        t_unc = bh_uncertain_t(px, py, ux, uy, gocc_unc, BH_UNC_RAY,
                               RES * 0.5, 0.35 + BH_UNC_LAT, g2c, g2w)
        phases.append((step_i, len(pts), len(occ), len(unc),
                       t_unc if t_unc is not None else -1.0,
                       b_pts, b_cells, set(occ) & b_cells,
                       {c: occ_ev[c][0] for c in b_cells},
                       inj_cell,
                       occ_ev[inj_cell][0] if inj_cell is not None
                       and inj_cell in occ_ev else None))

    # 阶段1.5 密集回波：末位置原地一圈，枝梢命中点同格注入 6 点（树干/
    # 密枝簇签名：近距大目标多波束同帧命中同格）——判占线 dense×T_near=6
    # 达成应判占（A* 绕行语义）。细枝稀疏单点/圈签名不封格由 C2 把关。
    dense_b_cells = set()
    vx = tip[0] - px
    vy = tip[1] - py
    vz = tip[2] - FLY_Z
    vlen = math.hypot(vx, vy, vz)
    if vlen > 1e-9:
        d = (vx / vlen, vy / vlen, vz / vlen)
        t = scene.raycast((px, py, FLY_Z), d, max_t=vlen + 0.05)
        if t is not None and t <= vlen + 0.05:
            p = (px + d[0] * t, py + d[1] * t, FLY_Z + d[2] * t)
            if geo.point_segment_dist(p, br[:3], br[3:6]) <= brr + 0.06:
                if MAP_BAND[0] <= p[2] <= MAP_BAND[1]:
                    c = g2c(p[0], p[1])
                    for _ in range(6):
                        ev = occ_ev.get(c)
                        if ev is None:
                            occ_ev[c] = [1.0, rebuild_n]
                        else:
                            ev[0] += 1.0
                            ev[1] = rebuild_n
                    dense_b_cells = {c}
    occ, unc = bh_evidence_tier(
        occ_ev, rebuild_n, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
        BH_TRUST_R, BH_SWAY_DENSE, g_nx, g_ny, g2w,
        np.array([px, py]))
    rebuild_n += 1
    dense_occ_hit = len(set(occ) & dense_b_cells)
    phases.append((N_STEPS, 6 * len(dense_b_cells), len(occ), len(unc),
                   -1.0, 6 * len(dense_b_cells), dense_b_cells,
                   set(occ) & dense_b_cells,
                   {c: occ_ev[c][0] for c in dense_b_cells},
                   next(iter(dense_b_cells)) if dense_b_cells else None,
                   occ_ev[next(iter(dense_b_cells))][0]
                   if dense_b_cells else None))

    # 阶段2 遗忘：无新观测，原地再跑 4 个重建周期（est 同最后位置）
    final_pos = (px, py)
    forget_trace = []
    for _ in range(4):
        occ, unc = bh_evidence_tier(
            occ_ev, rebuild_n, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
            BH_TRUST_R, BH_SWAY_DENSE, g_nx, g_ny, g2w,
            np.array(final_pos))
        forget_trace.append((len(occ_ev), len(occ), len(unc)))
        rebuild_n += 1

    # C1 首个注入圈：注入格（稀疏签名载体，1 点/圈）计数 ≤3 且 O4 走廊
    # 扫描 t>0（ray=4.0 ≥ 首见距离 → 稀疏枝首见即进限速窗）。容错：首见
    # 圈 t_unc 未命中时取次圈（注入格 cnt 仍在不确定档、t_c ≤ 4.0）验证
    # O4。注：直射 lidar 对贴近巡航高度的枝近段格本身就密（实测 27 计数
    # →occ，密目标语义正确），稀疏断言只看注入格。
    appr = [p for p in phases if p[0] < N_STEPS]
    first_b = next((p for p in appr if p[9] is not None), None)
    c1_ok = False
    c1_detail = "无注入观测"
    if first_b is not None:
        sparse = first_b[10] is not None and first_b[10] <= 3.0 + 1e-9
        o4_live = first_b[4] > 0.0
        if not o4_live:
            # 次圈兜底：注入格仍在不确定档，t_c ≤ 4.0 应命中
            nb = next((p for p in appr[first_b[0] + 1:] if p[9] is not None),
                      None)
            if nb is not None:
                o4_live = nb[4] > 0.0 and nb[10] is not None \
                    and nb[10] <= 2.0 + 1e-9
        c1_ok = sparse and o4_live
        c1_detail = "step=%d inj=%s cnt=%s t_unc=%.2f" % (
            first_b[0], first_b[9], first_b[10], first_b[4])
    check("C1 稀疏首见不判占+O4 命中", c1_ok, c1_detail)
    # C2 稀疏注入格接近期零判占：接近全程（未注入密集回波）注入格 ∉ occ
    # ——细枝单点/圈回波不封格（20260912 矩阵根因回归：旧 T 线判占把细枝
    # 升 occ 闭合树间走廊 → 北绕界外 OOB + 西墙滑扫，B 臂 0/3 PASS）。
    # 直射密格 occ 不在此断言（密目标语义，几何相依）。
    inj_occ = [p[0] for p in appr if p[9] is not None and p[9] in p[7]]
    check("C2 稀疏注入格接近期零判占", not inj_occ,
          "inj_occ_steps=%s" % inj_occ)
    # C3 密集回波判占：同圈 6 点同格（树干/密枝簇签名）→ 枝格判占（A* 绕行）
    dense_p = phases[-1] if phases and phases[-1][0] == N_STEPS else None
    d_ok = dense_p is not None and len(dense_p[7]) > 0 and len(dense_p[6]) > 0
    check("C3 密集回波→枝格判占", d_ok,
          "dense_cells=%s occ_hit=%s" %
          (len(dense_p[6]) if dense_p else 0,
           len(dense_p[7]) if dense_p else 0))
    # C4 无观测窗口遗忘：4 周期后 occ_ev 清空（"摆走的枝不再挡路"）
    tail = forget_trace[-1]
    forgotten = tail[0] == 0 and tail[1] == 0 and tail[2] == 0
    check("C4 无观测窗口遗忘", forgotten,
          "forget_trace=%s" % forget_trace)
    # C5 全程 O4：接近阶段存在 t_unc>0 的圈
    o4_hits = sum(1 for p in appr if p[4] > 0.0)
    check("C5 O4 扫描命中圈", o4_hits > 0, "o4_hits=%d/%d" % (o4_hits, len(appr)))


def main():
    test_tier()
    test_band_fence_guard()
    test_uncertain_scan()
    test_integration()
    npass = sum(1 for (_, ok, _) in RESULTS if ok)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("bh_selftest 结果 (%d/%d PASS)\n" % (npass, len(RESULTS)))
        for (name, ok, detail) in RESULTS:
            f.write("  [%s] %s%s\n" % ("PASS" if ok else "FAIL", name,
                                       (" — " + detail) if detail else ""))
    print("bh_selftest rc=%d (%d/%d)" % (0 if npass == len(RESULTS) else 1,
                                         npass, len(RESULTS)))
    sys.exit(0 if npass == len(RESULTS) else 1)


if __name__ == "__main__":
    main()
