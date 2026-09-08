#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""W6 VO-lite 机间避撞 —— 离线数学自测（不依赖 ROS 运行栈，只需可 import）。

用法（WSL）：
  cd ~/zx2026_arena_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/vo_selftest.py

覆盖四层：
  A. 纯函数判据：对头/交叉/远离/平行/超距/z带外/无速度/CPA外/时域外/重合
  B. 镜像互补不变量：任意对称二机场景，两机世界推力精确相反（各向己右，
     相对分离率 2×push）——这是无奇偶定向设计的核心可证性质
  C. 推力⊥相对速度：一阶不改闭合速率（dam v2 教训：贴脸减速=延长暴露）
  D. 离散二机积分仿真：对头+正交穿越两场景，指令=基线+推力+max_vel 钳制
     （镜像 nav_node 指令流：推力是每拍增量，不积分不记忆），
     断言全程最小机间距 > 2×机半径；并反证无 VO 时场景确有冲突
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_nav", "scripts"))

try:
    from nav_node import vo_deflect
except ImportError as e:  # 最常见：没 source ROS/devel 环境
    sys.stderr.write("import nav_node 失败(%s)。\n请先: source /opt/ros/noetic/"
                     "setup.bash && source devel/setup.bash\n" % e)
    sys.exit(2)

# ---- 与 sim_settings.yaml vo_avoid 默认值同源的测试参数 ----------------------
ENGAGE_R, SAFE_R, T_PRED = 3.5, 1.1, 3.0
Z_BAND = 0.5          # 与 swarm z_band 同款门控语义
DRONE_R = 0.35        # fleet.yaml 机半径；2×R=0.7 为硬底线
EPS = 1e-9

FAILS = []
NPASS = 0


def check(name, ok, detail=""):
    global NPASS
    if ok:
        NPASS += 1
        print("  [PASS] %s%s" % (name, ("  | " + detail) if detail else ""))
    else:
        FAILS.append(name)
        print("  [FAIL] %s%s" % (name, ("  | " + detail) if detail else ""))


def V(pa, va, pb, vb):
    """自机 pa 速度 va 对邻居 pb 速度 vb 算一次判据（nj/nv 直接喂三元组前两维语义）。"""
    return vo_deflect(pa[0], pa[1], pa[2], va[0], va[1], 0,
                      (pb[0], pb[1], pb[2]), (vb[0], vb[1]),
                      ENGAGE_R, SAFE_R, T_PRED, Z_BAND)


def mirror_push(pa, va, pb, vb):
    """对称二机各算一次，返回 (push_a, push_b)；任一静默返回 (None, None)。"""
    ra = V(pa, va, pb, vb)
    rb = V(pb, vb, pa, va)
    if ra is None or rb is None:
        return None, None
    return (ra[0], ra[1]), (rb[0], rb[1])


def test_a_judgements():
    print("A. 纯函数判据")
    # A1 对头冲突（d=3 在 engage 内，dcpa=0 最险形）：触发且 tcpa/dcpa 精确
    r = V((0, 0, 2), (1, 0, 0), (3, 0, 2), (-1, 0, 0))
    check("A1 对头触发", r is not None)
    if r is not None:
        _, _, score, tcpa, dcpa, d = r
        check("A1 tcpa=1.5", abs(tcpa - 1.5) < EPS, "tcpa=%.6f" % tcpa)
        check("A1 dcpa=0", dcpa < EPS, "dcpa=%.6f" % dcpa)
        check("A1 score∈(0,1]", 0.0 < score <= 1.0, "score=%.3f" % score)
    # A2 正在远离（前机更快拉開）：静默
    check("A2 远离静默",
          V((0, 0, 2), (1, 0, 0), (5, 0, 2), (2, 0, 0)) is None)
    # A3 平行同速（rv=0）：静默
    check("A3 平行静默",
          V((0, 0, 2), (1, 0, 0), (2, 0, 2), (1, 0, 0)) is None)
    # A4 z 带外（爬降段）：静默
    check("A4 z带外静默",
          V((0, 0, 2), (1, 0, 0), (2, 0, 2.8), (-1, 0, 0)) is None)
    # A5 超距（d=5>engage_r）：静默
    check("A5 超距静默",
          V((0, 0, 2), (1, 0, 0), (5, 0, 2), (-1, 0, 0)) is None)
    # A6 邻居速度未知：静默
    check("A6 无速度静默",
          vo_deflect(0, 0, 2, 1, 0, 0, (2, 0, 2), None,
                     ENGAGE_R, SAFE_R, T_PRED, Z_BAND) is None)
    # A7 CPA 在安全圈外（穿越但错得开，dcpa≈1.41）：静默
    check("A7 dcpa≈1.41≥safe_r 静默",
          V((0, 0, 2), (1, 0, 0), (2, 0, 2), (0, -1, 0)) is None)
    # A8 tcpa 超预判时域（慢速远距对头 tcpa=15）：静默
    check("A8 tcpa=15>t_pred 静默",
          V((0, 0, 2), (0.1, 0, 0), (3, 0, 2), (-0.1, 0, 0)) is None)
    # A9 位置重合退化：静默（防 0 除）
    check("A9 重合静默",
          V((1, 1, 2), (1, 0, 0), (1, 1, 2), (-1, 0, 0)) is None)
    # A10 侧偏穿越（dcpa=0.75√2≈1.061<safe_r）：触发
    r = V((0, 0, 2), (1, 0, 0), (1.5, 0, 2), (0, -1, 0))
    check("A10 侧偏穿越触发", r is not None)
    if r is not None:
        _, _, score, tcpa, dcpa, d = r
        check("A10 dcpa=0.75√2", abs(dcpa - 0.75 * math.sqrt(2)) < EPS,
              "dcpa=%.6f" % dcpa)
    # A11 score 单调性：dcpa 越小分越高（B 从 y=0 挪到 y=0.3 更贴 A 航线）
    r1 = V((0, 0, 2), (1, 0, 0), (1.5, 0, 2), (0, -1, 0))
    r2 = V((0, 0, 2), (1, 0, 0), (1.5, 0.3, 2), (0, -1, 0))
    ok = (r1 is not None and r2 is not None
          and r2[4] < r1[4] and r2[2] > r1[2])
    check("A11 dcpa小者分高", ok,
          "dcpa %.3f→%.3f, score %.3f→%.3f" % (
              r1[4] if r1 else -1, r2[4] if r2 else -1,
              r1[2] if r1 else -1, r2[2] if r2 else -1))


def test_b_mirror():
    print("B. 镜像互补不变量（两机世界推力精确相反）")
    scenes = [
        ("对头", (0, 0, 2), (1, 0, 0), (3, 0, 2), (-1, 0, 0)),
        ("对头带侧偏", (-1.6, -0.2, 2), (1, 0, 0), (1.6, 0.3, 2), (-1, 0, 0)),
        ("正交穿越", (-2, 0, 2), (1, 0, 0), (0, 2, 2), (0, -1, 0)),
        ("斜向汇聚", (0, 0, 2), (1, 0.2, 0), (2.5, 1.5, 2), (-0.8, -0.6, 0)),
        ("汇聚斜穿", (0, 0, 2), (1, 0, 0), (2.5, 1.2, 2), (-0.5, -1.0, 0)),
    ]
    for name, pa, va, pb, vb in scenes:
        push_a, push_b = mirror_push(pa, va, pb, vb)
        if push_a is None:
            check("B %s 该触发却静默" % name, False)
            continue
        dot = push_a[0] * push_b[0] + push_a[1] * push_b[1]
        mag_a = math.hypot(push_a[0], push_a[1])
        mag_b = math.hypot(push_b[0], push_b[1])
        # 精确相反 ⇔ 点积 = −|a||b|
        check("B %s 推力反向" % name, abs(dot + mag_a * mag_b) < EPS,
              "dot=%.2e  |a||b|=%.6f" % (dot, mag_a * mag_b))


def test_c_lateral():
    print("C. 推力⊥相对速度（一阶不减速）")
    scenes = [
        ((0, 0, 2), (1, 0, 0), (3, 0, 2), (-1, 0, 0)),
        ((0, 0, 2), (1, 0, 0), (1.5, 0, 2), (0, -1, 0)),
        ((-1.6, -0.2, 2), (1, 0, 0), (1.6, 0.3, 2), (-1, 0, 0)),
    ]
    for i, (pa, va, pb, vb) in enumerate(scenes):
        r = V(pa, va, pb, vb)
        if r is None:
            check("C%d 该触发却静默" % (i + 1), False)
            continue
        rvx, rvy = vb[0] - va[0], vb[1] - va[1]
        dot = r[0] * rvx + r[1] * rvy
        check("C%d push·rv≈0" % (i + 1), abs(dot) < EPS, "dot=%.2e" % dot)


def simulate(pa, pb, goal_a, goal_b,
             dt=0.02, t_end=10.0, max_vel=1.5, gain=1.2, max_push=1.0):
    """离散二机积分：指令=基线(奔目标巡航1m/s)+VO推力，|cmd|钳制 max_vel。

    镜像 nav_node 指令流：推力是每拍对当拍指令的增量（不积分、不记忆）；
    双方推力同拍用推前指令互相计算（在线实现两侧同帧独立算，等价并行）。
    """
    pa, pb = list(pa), list(pb)
    min_d = 1e9
    n = int(t_end / dt)
    for _ in range(n):
        # 基线指令：单位向目标 × 巡航 1 m/s（z 锁定）
        cmds = []
        for p, goal in ((pa, goal_a), (pb, goal_b)):
            ux, uy = goal[0] - p[0], goal[1] - p[1]
            nu = math.hypot(ux, uy) or 1.0
            cmds.append([ux / nu, uy / nu])
        cmd_a, cmd_b = cmds
        # 双方用推前指令互算 VO（并行语义）
        r_a = vo_deflect(pa[0], pa[1], pa[2], cmd_a[0], cmd_a[1], 0,
                         (pb[0], pb[1], pb[2]), (cmd_b[0], cmd_b[1]),
                         ENGAGE_R, SAFE_R, T_PRED, Z_BAND)
        r_b = vo_deflect(pb[0], pb[1], pb[2], cmd_b[0], cmd_b[1], 1,
                         (pa[0], pa[1], pa[2]), (cmd_a[0], cmd_a[1]),
                         ENGAGE_R, SAFE_R, T_PRED, Z_BAND)
        for cmd, r in ((cmd_a, r_a), (cmd_b, r_b)):
            if r is not None:
                push = min(gain * r[2], max_push)
                cmd[0] += r[0] * push
                cmd[1] += r[1] * push
        # max_vel 终钳制（镜像 nav_node 收尾）
        for p, cmd in ((pa, cmd_a), (pb, cmd_b)):
            m = math.hypot(cmd[0], cmd[1])
            if m > max_vel:
                cmd[0] *= max_vel / m
                cmd[1] *= max_vel / m
            p[0] += cmd[0] * dt
            p[1] += cmd[1] * dt
        d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        if d < min_d:
            min_d = d
    return min_d


def test_d_integration():
    print("D. 离散二机积分（硬底线 2×机半径=%.2f m）" % (2 * DRONE_R))
    # D1 对头带侧偏（d2/d3 事故形：返程对头汇聚、横向错位 0.5 m）
    d1 = simulate((-4.0, -0.25, 2), (4.0, 0.25, 2),
                  (10.0, -0.25, 2), (-10.0, 0.25, 2))
    check("D1 对头带侧偏 min_d>0.7", d1 > 2 * DRONE_R,
          "min_d=%.3f (safe_r=%.2f)" % (d1, SAFE_R))
    # D2 正交穿越（同时到原点，无 VO 时 min_d→0）
    d2 = simulate((-4.0, 0.0, 2), (0.0, 4.0, 2),
                  (4.0, 0.0, 2), (0.0, -4.0, 2))
    check("D2 正交穿越 min_d>0.7", d2 > 2 * DRONE_R,
          "min_d=%.3f (safe_r=%.2f)" % (d2, SAFE_R))
    # D3 反证：gain=0（等效关 VO）同一对头场景应撞穿 —— 证明场景真有冲突
    d3 = simulate((-4.0, -0.25, 2), (4.0, 0.25, 2),
                  (10.0, -0.25, 2), (-10.0, 0.25, 2), gain=0.0)
    check("D3 反证(无VO)确有冲突 min_d<0.7", d3 < 2 * DRONE_R,
          "min_d=%.3f" % d3)


def main():
    print("=== W6 vo_deflect 离线自测 ===")
    test_a_judgements()
    test_b_mirror()
    test_c_lateral()
    test_d_integration()
    print("=== 结果: PASS=%d FAIL=%d ===" % (NPASS, len(FAILS)))
    for f in FAILS:
        print("  FAIL:", f)
    if FAILS:
        sys.exit(1)
    print("ALL PASS")


if __name__ == "__main__":
    main()
