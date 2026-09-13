#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/branch_selftest.py — 仿真树枝层离线自测（不依赖 ROS 运行栈）。

用法（WSL）：
  cd ~/zx2026_arena_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/branch_selftest.py

结果写 /tmp/branch_selftest.txt（UTF-8，规避 GBK 控制台）。

覆盖（O1-O5 的仿真层地基）：
  0. 基线不变：branches.enabled=false 时所有树 branches=None（无枝场景
     Scene 几何与旧版逐位一致）
  1. 确定性：enabled=true 两次实例化，树枝元组逐位相同（同 run_seed）
  2. 生成合理：总枝数>0、每树 [1,per_tree_max]、枝参数在配置范围内
  3. 稀疏回波：72az×5el lidar 射线（同 lidar_node 模式），枝命中数>0
     且占比低（细枝特征，同真机 2cm 枝 @5m ~4% 量级复现）
  4. 硬碰撞：球(0.35) 置枝中点 collides=True；沿枝轴端点外 1.2m False
  5. A* 占用：collides_xy 枝 z 带内 True、带外(z>3.5) False、树干外侧点
     仅因枝判占（隔离树干混淆）
  6. 对照一致性：同 seed 无枝场景逐射线对照，有枝命中距离 ≤ 无枝
     （枝只会更近不会更远）
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "zx2026_common", "scripts"))

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene

WS = os.path.expanduser("~/zx2026_arena_ws")
CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
OUT = "/tmp/branch_selftest.txt"

LIDAR_AZ = 72
LIDAR_EL = [-0.5, -0.25, 0.0, 0.25, 0.5]
RANGE = 5.0

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))


def patch_branches(value):
    """branches.enabled 改 true/false（set_in_block 同款：块内首个 enabled）。"""
    import re
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


def lidar_hits(scene, pos):
    """复刻 lidar_node 射线模式（72az × 5el，yaw=0），返回 [(t, az, el),...]。"""
    hits = []
    for az_i in range(LIDAR_AZ):
        az = az_i * (2.0 * math.pi / LIDAR_AZ)
        for el in LIDAR_EL:
            d = geo.normalize((math.cos(el) * math.cos(az),
                               math.cos(el) * math.sin(az), math.sin(el)))
            t = scene.raycast(pos, d, max_t=RANGE)
            if t is not None:
                hits.append((t, az, el))
    return hits


def main():
    orig = cfg.load("sim_settings.yaml").get("branches", {}).get("enabled", False)
    try:
        # ---- 0 基线（显式关态；文件若被遗留脏状态先归位） ----
        patch_branches("false")
        s0 = Scene()
        n_trees = sum(1 for ob in s0.obstacles if ob.kind == "tree")
        n_wb = sum(1 for ob in s0.obstacles if ob.kind == "tree" and ob.branches)
        check("0 基线无枝", n_wb == 0, "trees=%d tree_with_branches=%d" % (n_trees, n_wb))
        if n_trees < 20:
            check("0b 森林规模", False, "trees=%d (<20, 场景异常)" % n_trees)
        else:
            check("0b 森林规模", True, "trees=%d" % n_trees)

        # ---- 开枝：确定性 / 生成合理 / 碰撞 / 占用 ----
        patch_branches("true")
        sA = Scene()
        sB = Scene()
        brA = [ob.branches for ob in sA.obstacles if ob.kind == "tree"]
        brB = [ob.branches for ob in sB.obstacles if ob.kind == "tree"]
        check("1 确定性", brA == brB,
              "trees=%d" % len(brA))
        tot = sum(len(b) for b in brA if b)
        per = [len(b) for b in brA if b]
        check("2 枝数", tot > 0 and all(0 < n <= 5 for n in per),
              "total=%d per_tree=%s" % (tot, sorted(set(per))))
        ok_r = True
        n_br = 0
        for b in brA:
            if not b:
                continue
            for (sx, sy, sz, ex, ey, ez, r) in b:
                n_br += 1
                if not (0.008 - 1e-9 <= r <= 0.022 + 1e-9):
                    ok_r = False
                ln = geo.dist((sx, sy, sz), (ex, ey, ez))
                if not (0.6 - 1e-6 <= ln <= 1.6 + 1e-6):
                    ok_r = False
        check("2b 枝参数范围", ok_r and n_br > 0, "branches=%d" % n_br)

        # 4 硬碰撞（选长枝 ln>=0.8，中点=球心）
        coll_ok = ctrl_ok = ctrl_skip = 0
        for ob in sA.obstacles:
            if ob.kind != "tree" or not ob.branches:
                continue
            for (sx, sy, sz, ex, ey, ez, br) in ob.branches:
                if geo.dist((sx, sy, sz), (ex, ey, ez)) < 0.8:
                    continue
                mid = ((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0)
                if sA.collides(mid, 0.35):
                    coll_ok += 1
                else:
                    check("4 中点碰撞", False, "tree%d br@%.2f,%.2f,%.2f" % (
                        ob.id, mid[0], mid[1], mid[2]))
                u = geo.normalize((ex - sx, ey - sy, ez - sz))
                probe = (ex + u[0] * 1.2, ey + u[1] * 1.2, ez + u[2] * 1.2)
                if sA.collides(probe, 0.35):
                    ctrl_skip += 1  # 可能贴近邻枝/树干，跳过对照组
                else:
                    ctrl_ok += 1
                if ctrl_ok >= 3:
                    break
            if ctrl_ok >= 3:
                break
        check("4 硬碰撞", coll_ok > 0 and ctrl_ok > 0,
              "mid_hit=%d ctrl_clear=%d ctrl_skip=%d" % (coll_ok, ctrl_ok, ctrl_skip))

        # 5 A* 占用（隔离树干：取树干外侧 trunk_r+0.45、沿枝方位角）
        occ_ok = occ_out = 0
        for ob in sA.obstacles:
            if ob.kind != "tree" or not ob.branches:
                continue
            for (sx, sy, sz, ex, ey, ez, br) in ob.branches:
                ln = geo.dist((sx, sy, sz), (ex, ey, ez))
                if ln < 0.6:
                    continue
                az = math.atan2(ey - sy, ex - sx)
                zmin, zmax = min(sz, ez), max(sz, ez)
                d0 = ob.trunk_r + 0.45
                x0 = ob.cx + math.cos(az) * d0
                y0 = ob.cy + math.sin(az) * d0
                in_band = sA.collides_xy(x0, y0, zmin, zmax, pad=0.0)
                out_band = sA.collides_xy(x0, y0, 3.5, 4.0, pad=0.0)
                if in_band and not out_band:
                    occ_ok += 1
                else:
                    check("5 占用", False, "tree%d br@%.2f,%.2f in=%s out=%s" % (
                        ob.id, x0, y0, in_band, out_band))
                if occ_ok >= 3:
                    break
            if occ_ok >= 3:
                break
        check("5 A* 占用", occ_ok > 0,
              "isolated_branch_occ=%d" % occ_ok)

        # ---- 3/6 稀疏回波 + 对照一致性（先切回无枝构建对照场景） ----
        patch_branches("false")
        sOff = Scene()
        poses = []
        for ob in sA.obstacles:
            if ob.kind != "tree" or not ob.branches:
                continue
            br = ob.branches[0]
            az = math.atan2(br[4] - br[1], br[3] - br[0])
            px = ob.cx + math.cos(az + math.pi) * 4.0
            py = ob.cy + math.sin(az + math.pi) * 4.0
            pz = 1.5
            pos = (px, py, pz)
            # 站位须净空（两场景同查，避免站在别的树里）
            if not sA.collides(pos, 0.0) and not sOff.collides(pos, 0.0):
                poses.append(pos)
            if len(poses) >= 8:
                break
        bhits = 0
        thits = 0
        anomalies = []
        for pos in poses:
            hA = lidar_hits(sA, pos)
            hO = lidar_hits(sOff, pos)
            tA = {(a, e): t for (t, a, e) in hA}
            tO = {(a, e): t for (t, a, e) in hO}
            for k, t in tA.items():
                if k in tO:
                    thits += 1
                    if t < tO[k] - 1e-9:
                        bhits += 1  # 有枝场景更近 → 枝命中
                    elif t > tO[k] + 1e-9:
                        anomalies.append((pos, k, t, tO[k]))
                else:
                    bhits += 1  # 有枝命中、无枝场景无命中（被枝挡在树干前）
        frac = bhits / max(thits + bhits, 1)
        check("6 对照一致性", len(anomalies) == 0,
              "anomalies=%d" % len(anomalies))
        check("3 稀疏回波", bhits > 0 and frac <= 0.15,
              "branch_hits=%d/%d frac=%.3f poses=%d" % (bhits, bhits + thits, frac, len(poses)))
    finally:
        patch_branches("true" if orig else "false")

    npass = sum(1 for (_, ok, _) in RESULTS if ok)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("branch_selftest 结果 (%d/%d PASS)\n" % (npass, len(RESULTS)))
        for (name, ok, detail) in RESULTS:
            f.write("  [%s] %s%s\n" % ("PASS" if ok else "FAIL", name,
                                       (" — " + detail) if detail else ""))
    rc = 0 if npass == len(RESULTS) else 1
    print("branch_selftest rc=%d (%d/%d)" % (rc, npass, len(RESULTS)))
    sys.exit(rc)


if __name__ == "__main__":
    main()
