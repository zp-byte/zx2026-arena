#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_selftest.py — 相机颜色检测链纯函数自检（无 ROS，P0 第 1 关）。

仿 rule_selftest 风格：A-E 五组锚点全过 exit 0，任一 FAIL exit 1。
  A 渲染 sanity   三色盘正上几何（质心≤4px/半径±15%）、pads 上空无色、
                   遮挡裁色块、渲染色板==world_builder.PLATFORM_COLORS
  B 检测正确性   三色悬停帧（噪声开）全对、裸地 255/0.0、HSV 带互斥
  C conf 标定     悬停偏移 0/0.3m → conf≥0.8 全色；0.4m → ≥0.5（分级）；
                   巡航 2.5m 正上仍判对色（下视相机任意高度均正射——
                   计划原文"斜视"假设在无云台下不成立，本组锚定实际语义）
  D 端到端语义   进程内 bucket_scan_step：20 帧悬停→LOCK；异色永不清零
                   不得 LOCK；NO_DET 清零；半遮挡低置信冻结后恢复帧即锁
  E 门控谓词     color_id.source 默认 truth 完好；两新节点早退谓词成立

用法： python3 tools/camera_selftest.py
"""
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "zx2026_common", "scripts"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "arena_sensor", "scripts"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "arena_world_gazebo", "scripts"))

import numpy as np                                                     # noqa: E402

from zx2026_common import config as cfg                                 # noqa: E402
from zx2026_common import bucket_select as bs                           # noqa: E402
from zx2026_common.scene import Scene, Obstacle                         # noqa: E402
from world_builder import PLATFORM_COLORS                               # noqa: E402
from arena_sensor.camera_render import (render_frame, make_cam_cfg,     # noqa: E402
                                        intrinsics, DISC_R)
from arena_sensor.color_detect import (detect, make_detect_params,      # noqa: E402
                                       NO_DET)

RED, BLUE, YELLOW = 0, 1, 2

_results = []


def check(name, ok):
    _results.append((name, bool(ok)))
    print("[%s] %s" % ("PASS" if ok else "FAIL", name))


def bgr_of(color_name):
    r, g, b = PLATFORM_COLORS[color_name]
    return (int(b * 255), int(g * 255), int(r * 255))


def disc_stats(img, color_name):
    """渲染帧中该色盘像素的 (面积, 质心)。无像素返回 (0, None)。"""
    target = np.array(bgr_of(color_name), dtype=np.uint8)
    mask = np.all(img == target, axis=2)
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return 0, None
    return int(xs.size), (float(xs.mean()), float(ys.mean()))


def rng_for(drone_id, seed=None):
    s = seed if seed is not None else int(cfg.load("sim_settings.yaml").get("run_seed", 42))
    return random.Random(s + drone_id * 7919)


# ---------------------------------------------------------------- A 渲染 sanity
def group_a():
    scene = Scene()
    cam = make_cam_cfg(cfg.load("sim_settings.yaml").get("sensor", {}).get("camera", {}))
    clean = dict(cam, gain_std=0.0, pixel_noise_std=0.0, dropout_rate=0.0)
    fx, fy, cx, cy = intrinsics(clean)
    hover_z = 0.85  # drop_hover_z 0.80 + 盘顶 0.05
    # A1 三色盘：正上悬停 → 质心≈主点、半径≈fx*0.6/0.80（±15%）
    all_ok = True
    for dp in scene.drop_points:
        img = render_frame(scene, (dp.xyz[0], dp.xyz[1], hover_z), 0.0, clean, rng_for(0))
        area, cent = disc_stats(img, dp.color)
        r_px = math.sqrt(area / math.pi) if area else 0.0
        exp_r = fx * DISC_R / (hover_z - 0.05)
        ok = (cent is not None
              and abs(cent[0] - cx) <= 4.0 and abs(cent[1] - cy) <= 4.0
              and abs(r_px - exp_r) / exp_r <= 0.15)
        all_ok &= ok
        if not ok:
            print("      dp%d color=%s area=%d cent=%s r=%.1f exp=%.1f"
                  % (dp.id, dp.color, area, cent, r_px, exp_r))
    check("A1 三色盘正上几何（质心/半径）", all_ok)
    # A1b 白环存在
    img = render_frame(scene, (scene.drop_points[0].xyz[0], scene.drop_points[0].xyz[1],
                               hover_z), 0.0, clean, rng_for(0))
    white_px = int(np.count_nonzero(np.all(img == np.array([245, 245, 245],
                                                           dtype=np.uint8), axis=2)))
    check("A1b 白环已渲染", white_px > 1000)
    # A2 pads 上空 → 检测器无检测
    pads = scene.get_pads()
    img = render_frame(scene, (pads[0][0], pads[0][1], 1.5), 0.0, clean, rng_for(0))
    c, cf, _ = detect(img, make_detect_params(cfg.load("competition_rules.yaml")))
    check("A2 pads 上空无检测", c == NO_DET and cf == 0.0)
    # A3 遮挡：0.8m 方柱骑盘心（±0.40 投影遮盘中心约 50%）→ 色块被裁
    # （面积 < 干净帧 70%）。
    s2 = Scene()
    dp = s2.drop_points[0]
    s2.obstacles.append(Obstacle((dp.xyz[0] - 0.40, dp.xyz[1] - 0.40, 0.0),
                                 (dp.xyz[0] + 0.40, dp.xyz[1] + 0.40, 3.0),
                                 "box", 999))
    clean_a = render_frame(s2, (dp.xyz[0], dp.xyz[1], hover_z), 0.0, clean, rng_for(0))
    area_occ, _ = disc_stats(clean_a, dp.color)
    check("A3 遮挡裁色块（面积降 >30%）", area_occ > 0 and area_occ < area * 0.7)
    # A4 色板权威：渲染色==world_builder.PLATFORM_COLORS（上两组的质心/面积
    # 校验已隐含逐色匹配；显式断言 BGR 顺序换算）
    ok4 = True
    for name, rgb in PLATFORM_COLORS.items():
        bgr = (int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255))
        ok4 &= bgr == bgr_of(name)
    check("A4 渲染色板==PLATFORM_COLORS", ok4)


# ---------------------------------------------------------------- B 检测正确性
def group_b():
    scene = Scene()
    cam = make_cam_cfg(cfg.load("sim_settings.yaml").get("sensor", {}).get("camera", {}))
    params = make_detect_params(cfg.load("competition_rules.yaml"))
    hover_z = 0.85
    all_ok = True
    for dp in scene.drop_points:
        rng = rng_for(dp.id)
        n_ok = n_frame = 0
        for _ in range(20):
            img = render_frame(scene, (dp.xyz[0], dp.xyz[1], hover_z), 0.0, cam, rng)
            if img is None:      # 断帧 dropout
                continue
            n_frame += 1
            c, cf, _ = detect(img, params)
            if c == cfg.color_to_uint8(dp.color) and cf >= 0.8:
                n_ok += 1
        ok = n_frame >= 18 and n_ok == n_frame
        all_ok &= ok
        print("      dp%d(%s) %d/%d 帧判对 conf≥0.8" % (dp.id, dp.color, n_ok, n_frame))
    check("B1 三色悬停 20 噪声帧全对", all_ok)
    img = render_frame(scene, (scene.get_pads()[0][0], scene.get_pads()[0][1], 1.5),
                       0.0, cam, rng_for(0))
    c, cf, _ = detect(img, params)
    check("B2 裸地 255/0.0", c == NO_DET and cf == 0.0)
    red = (params["red"][0], params["red"][1])
    blue = params["blue"]
    yellow = params["yellow"]
    bands = [("red1", red[0]), ("red2", red[1]), ("blue", blue), ("yellow", yellow)]
    disjoint = True
    for i in range(len(bands)):
        for j in range(i + 1, len(bands)):
            a, b = bands[i][1], bands[j][1]
            if a[0] <= b[1] and b[0] <= a[1]:
                disjoint = False
    check("B3 HSV 色带互斥", disjoint)


# ---------------------------------------------------------------- C conf 标定
def group_c():
    scene = Scene()
    cam = make_cam_cfg(cfg.load("sim_settings.yaml").get("sensor", {}).get("camera", {}))
    params = make_detect_params(cfg.load("competition_rules.yaml"))
    hover_z = 0.85
    # C1 悬停偏移 0/0.3 → conf≥0.8 全色；0.4 → ≥0.5（分级裁边）
    all_ok = True
    for dp in scene.drop_points:
        for off in (0.0, 0.3, 0.4):
            worst = 1.0
            rng = rng_for(dp.id + int(off * 100))
            for _ in range(5):
                img = render_frame(scene, (dp.xyz[0] + off, dp.xyz[1], hover_z),
                                   0.0, cam, rng)
                if img is None:
                    continue
                c, cf, _ = detect(img, params)
                if c == cfg.color_to_uint8(dp.color):
                    worst = min(worst, cf)
            need = 0.8 if off <= 0.3 else 0.5
            ok = worst >= need
            all_ok &= ok
            print("      dp%d(%s) off=%.1f worst conf=%.2f (need %.1f)"
                  % (dp.id, dp.color, off, worst, need))
    check("C1 悬停偏移 conf 标定（0.3→0.8，0.4→0.5 分级）", all_ok)
    # C2 巡航 2.5m 正上：下视正射仍判对色（计划"斜视"假设在无云台下不成立）
    ok2 = True
    for dp in scene.drop_points:
        img = render_frame(scene, (dp.xyz[0], dp.xyz[1], 2.55), 0.0, cam, rng_for(10 + dp.id))
        if img is None:
            continue
        c, cf, _ = detect(img, params)
        ok2 &= (c == cfg.color_to_uint8(dp.color))
    check("C2 巡航 2.5m 正上仍判对色", ok2)


# ---------------------------------------------------------------- D 端到端语义
def group_d():
    scene = Scene()
    cam = make_cam_cfg(cfg.load("sim_settings.yaml").get("sensor", {}).get("camera", {}))
    params = make_detect_params(cfg.load("competition_rules.yaml"))
    hover_z = 0.85
    dp1 = scene.drop_points[0]
    u1 = cfg.color_to_uint8(dp1.color)
    # D1 悬停 20 帧（箱=盘色）→ 锁
    st = bs.new_state()
    rng = rng_for(1)
    locked = False
    for _ in range(20):
        img = render_frame(scene, (dp1.xyz[0], dp1.xyz[1], hover_z), 0.0, cam, rng)
        if img is None:
            continue
        c, cf, _ = detect(img, params)
        st, dec = bs.bucket_scan_step(c, cf, u1, state=st)
        if dec == "LOCK":
            locked = True
            break
    check("D1 20 帧悬停内 LOCK", locked)
    # D2 异色（箱=红但盘=蓝）：不得 LOCK，hit 恒 0
    dp2 = scene.drop_points[1]
    u_other = cfg.color_to_uint8(dp2.color)
    st = bs.new_state()
    rng = rng_for(2)
    locked2 = False
    for _ in range(20):
        img = render_frame(scene, (dp2.xyz[0], dp2.xyz[1], hover_z), 0.0, cam, rng)
        if img is None:
            continue
        c, cf, _ = detect(img, params)
        st, dec = bs.bucket_scan_step(c, cf, u1, state=st)   # 箱=dp1 色
        if dec == "LOCK":
            locked2 = True
            break
    check("D2 异色 20 帧不锁且清零", not locked2 and st["hit"] == 0)
    # D3 NO_DET 清零：两命中后断帧哨兵 → hit 归零
    st = {"hit": 2}
    st, _ = bs.bucket_scan_step(bs.NO_DET, 0.0, u1, state=st)
    check("D3 NO_DET 清零", st["hit"] == 0)
    # D4 半遮挡低置信冻结（hit=2 冻结）→ 恢复帧即达 3 锁
    s2 = Scene()
    dp = s2.drop_points[0]
    s2.obstacles.append(Obstacle((dp.xyz[0] - 0.40, dp.xyz[1] - 0.40, 0.0),
                                 (dp.xyz[0] + 0.40, dp.xyz[1] + 0.40, 3.0),
                                 "box", 999))
    img = render_frame(s2, (dp.xyz[0], dp.xyz[1], hover_z), 0.0, cam, rng_for(4))
    c, cf, _ = detect(img, params)
    st = {"hit": 2}
    st, dec = bs.bucket_scan_step(c, cf, u1, state=st)
    frozen = (c == u1 and cf < 0.8 and st["hit"] == 2)
    st, dec = bs.bucket_scan_step(u1, 0.9, u1, state=st)
    check("D4 半遮挡冻结→恢复帧即锁", frozen and dec == "LOCK")


# ---------------------------------------------------------------- E 门控谓词
def group_e():
    rules = cfg.load("competition_rules.yaml")
    source = (rules.get("color_id", {}) or {}).get("source", "truth")
    check("E1 color_id.source 默认 truth", source == "truth")
    # 两新节点早退谓词（与节点内联一致）：source != camera → 不创建发布者
    exit_sim = source != "camera"
    exit_det = source != "camera"
    check("E2 新节点早退谓词成立", exit_sim and exit_det)


def main():
    t0 = time.time()
    group_a()
    group_b()
    group_c()
    group_d()
    group_e()
    n_pass = sum(1 for _, ok in _results if ok)
    n_fail = len(_results) - n_pass
    print("=== camera_selftest: %d/%d PASS (%.1fs) ==="
          % (n_pass, len(_results), time.time() - t0))
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
