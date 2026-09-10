# -*- coding: utf-8 -*-
"""arena_sensor::camera_render — 下视相机合成渲染（纯函数，无 ROS）。

color_id.source=camera 的图像源（2026-09-10 立项）。与 lidar_node 同一传感器
模型哲学：读 Scene 真值几何 + 真值位姿，产出带噪声的观测——这里是
sensor_msgs/Image 而非 PointCloud2。渲染与检测分离：本模块只管"拍"，
color_detect.py 只管"认"，检测器保持无真值（sim/real 同代码的诚实边界）。

实现：针孔投影（光轴垂直向下，u 沿机体右向、v 沿机体前向）+ 画家算法
剪影——地面 → 平台色盘/白环 → 遮挡物（树干圆柱/AABB 凸包）由远及近
cv2.fillPoly。半遮挡=色块被裁=分级 conf，正好喂 bucket_select 的冻结
语义（truth 检测器是二元遮挡门，这里是分级遮挡，遮挡集一致：树干+AABB、
树冠不入画）。色板唯一权威=world_builder.PLATFORM_COLORS（不复制 RGB）。

噪声模型（sim_settings.sensor.camera，每机独立种子 rng）：
  整帧丢失 dropout_rate → 返回 None（相机断帧）
  乘性曝光抖动 gain_std  → 每帧全图增益
  逐像素高斯 pixel_noise_std → np.random.RandomState(rng 派生，确定性)
"""
import math
import os
import sys

import numpy as np
import cv2

_WS = "/home/ubuntu/zx2026_arena_ws"
if os.path.join(_WS, "src/arena_world_gazebo/scripts") not in sys.path:
    sys.path.insert(0, os.path.join(_WS, "src/arena_world_gazebo/scripts"))
from world_builder import PLATFORM_COLORS  # noqa: E402  色板唯一权威（RGB 0-1）

# 平台几何口径（world_builder.drop_model 同值）
DISC_R = 0.60      # 色盘半径(m)
RIM_R = 0.68       # 白环外半径(m)
DISC_TOP_Z = 0.05  # 盘面顶高(m)

# 场景色（BGR；均避开检测 HSV 带——绿相 hue≈120-140 不落红/蓝/黄带，
# 低饱和灰棕 S<s_min 不成块）
GROUND_COLOR = (86, 112, 64)
TRUNK_COLOR = (52, 58, 64)
BUSH_COLOR = (58, 96, 48)
BOX_COLOR = (70, 70, 70)
CROWN_COLOR = (46, 84, 40)
WHITE = (245, 245, 245)

_CIRCLE_N = 24  # 圆周采样边数（投影多边形）


def make_cam_cfg(settings_camera):
    """sim_settings.sensor.camera 段 → 渲染参数 dict（缺省同 yaml 注释）。"""
    c = settings_camera or {}
    return {
        "width": int(c.get("width", 320)),
        "height": int(c.get("height", 240)),
        "vfov_deg": float(c.get("vfov_deg", 95.0)),
        "gain_std": float(c.get("gain_std", 0.06)),
        "pixel_noise_std": float(c.get("pixel_noise_std", 6.0)),
        "dropout_rate": float(c.get("dropout_rate", 0.02)),
        "render_crowns": bool(c.get("render_crowns", False)),
    }


def intrinsics(cam_cfg):
    """(fx, fy, cx, cy)：方形像素，光轴过图像中心。"""
    h = float(cam_cfg["height"])
    f = (h / 2.0) / math.tan(math.radians(cam_cfg["vfov_deg"]) / 2.0)
    return f, f, cam_cfg["width"] / 2.0, h / 2.0


def _circle_pts(cx_w, cy_w, z, r, n=_CIRCLE_N):
    ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([cx_w + r * np.cos(ang), cy_w + r * np.sin(ang),
                     np.full(n, z)], axis=1)


def _project(pts, pos, yaw, fx, fy, cx, cy):
    """世界系点 (N,3) → 像素 (N,2)；在相机上方/同高（dz<=0）的点置 NaN。"""
    dx = pts[:, 0] - pos[0]
    dy = pts[:, 1] - pos[1]
    dz = pos[2] - pts[:, 2]                     # 点在相机下方的垂直距离
    fwd_x, fwd_y = math.cos(yaw), math.sin(yaw)
    rgt_x, rgt_y = math.sin(yaw), -math.cos(yaw)
    u = cx + fx * (dx * rgt_x + dy * rgt_y) / dz
    v = cy + fy * (dx * fwd_x + dy * fwd_y) / dz
    out = np.stack([u, v], axis=1)
    out[dz <= 1e-6] = np.nan
    return out


def _in_frame(px, w, h, margin=60.0):
    ok = np.isfinite(px).all(axis=1)
    if not ok.any():
        return False
    p = px[ok]
    return bool((p[:, 0] > -margin).any() and (p[:, 0] < w + margin).any()
                and (p[:, 1] > -margin).any() and (p[:, 1] < h + margin).any())


def _fill(img, px, color):
    pts = px[~np.isnan(px).any(axis=1)].astype(np.int32)
    if pts.shape[0] < 3:
        return
    # 凸包：AABB 八角点/树干双圆环点序非环绕序，直接 fillPoly 会自交
    # （even-odd 只填一半）；凸包给出正确剪影。
    hull = cv2.convexHull(pts)
    cv2.fillPoly(img, [hull], color)


def _occluders(scene, render_crowns):
    """(远到近排序在调用方) → [(水平距, 投影点数组, 颜色)] 生成器。"""
    gz = scene.venue["ground_z"]
    for ob in scene.obstacles:
        if ob.kind == "tree":
            n = 16
            ring = _circle_pts(ob.cx, ob.cy, 0.0, ob.trunk_r, n)
            top = _circle_pts(ob.cx, ob.cy, gz + ob.trunk_h, ob.trunk_r, n)
            pts = np.concatenate([ring, top], axis=0)
            yield pts, TRUNK_COLOR
            if render_crowns and ob.crown_r:
                yield _circle_pts(ob.cx, ob.cy, ob.crown_z, ob.crown_r, 20), CROWN_COLOR
        else:
            lo, hi = ob.lo, ob.hi
            xs = (lo[0], hi[0])
            ys = (lo[1], hi[1])
            zs = (lo[2], hi[2])
            pts = np.array([[x, y, z] for x in xs for y in ys for z in zs])
            color = BUSH_COLOR if ob.kind == "bush" else BOX_COLOR
            yield pts, color


def render_frame(scene, pos, yaw, cam_cfg, rng):
    """渲染一帧下视图像。

    scene: zx2026_common.scene.Scene（真值几何）
    pos/yaw: 机体真值位姿（世界系）
    cam_cfg: make_cam_cfg 产物
    rng: random.Random（每机独立，种子=run_seed+id*7919）
    返回 (H,W,3) uint8 BGR；整帧丢失时返回 None。
    """
    if cam_cfg["dropout_rate"] > 0 and rng.random() < cam_cfg["dropout_rate"]:
        return None
    w, h = int(cam_cfg["width"]), int(cam_cfg["height"])
    fx, fy, cx, cy = intrinsics(cam_cfg)
    img = np.full((h, w, 3), GROUND_COLOR, np.uint8)

    gz = scene.venue["ground_z"]
    # 平台：白环盘打底 + 色盘覆盖（画家算法，先环后盘）
    for dp in scene.drop_points:
        z = gz + DISC_TOP_Z
        rim = _project(_circle_pts(dp.xyz[0], dp.xyz[1], z, RIM_R), pos, yaw, fx, fy, cx, cy)
        if _in_frame(rim, w, h):
            _fill(img, rim, WHITE)
        disc = _project(_circle_pts(dp.xyz[0], dp.xyz[1], z, DISC_R), pos, yaw, fx, fy, cx, cy)
        if _in_frame(disc, w, h):
            rgb = PLATFORM_COLORS.get(dp.color, (0.90, 0.90, 0.90))
            _fill(img, disc, (int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255)))

    # 遮挡物：由远及近覆盖（半遮挡=裁色块=分级 conf）
    occ = []
    for pts, color in _occluders(scene, cam_cfg["render_crowns"]):
        px = _project(pts, pos, yaw, fx, fy, cx, cy)
        if _in_frame(px, w, h):
            fin = px[~np.isnan(px).any(axis=1)]
            d = float(np.min((fin[:, 0] - cx) ** 2 + (fin[:, 1] - cy) ** 2))
            occ.append((d, px, color))
    occ.sort(key=lambda e: -e[0])
    for _, px, color in occ:
        _fill(img, px, color)

    # 噪声：乘性增益 + 逐像素高斯（RandomState 从 rng 派生，帧间确定性）
    out = img.astype(np.float32)
    if cam_cfg["gain_std"] > 0:
        out *= max(0.0, 1.0 + rng.gauss(0.0, cam_cfg["gain_std"]))
    if cam_cfg["pixel_noise_std"] > 0:
        rs = np.random.RandomState(rng.randint(0, 2 ** 31 - 1))
        out += rs.normal(0.0, cam_cfg["pixel_noise_std"], out.shape)
    np.clip(out, 0.0, 255.0, out=out)
    return out.astype(np.uint8)
