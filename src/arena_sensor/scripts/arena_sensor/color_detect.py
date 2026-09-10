# -*- coding: utf-8 -*-
"""arena_sensor::color_detect — 相机颜色检测（纯函数，无 ROS、无真值）。

color_id.source=camera 的检测算法（2026-09-10 立项）。只吃图像+配置参数，
不读 Scene/odom——这份文件在真机 Jetson 上原样运行（D435 RGB 只换话题）。

算法：HSV 域值分割（红双带 union 防 360° 回绕）→ 3×3 开运算去孤立噪点 →
最大连通域 → conf = w_circ*圆形度 + w_pur*色纯度（面积只作噪声地板门限，
不入 conf——真机规则高 0.5-0.6m 时 1.2m 盘溢出 D435 帧，面积/帧归一化
无意义；纯度/形状对距离与偏移不敏感，这正是"悬停漂移 0.3m 内 conf 稳"
的标定来源）。边缘情形按 bucket_select 语义输出：
  无足够大色块 → (NO_DET, 0.0, 255)（清零）
  色块破碎/半遮挡 → conf 低（冻结）
  完整盘面 → conf 高（可 LOCK）
"""
import math

import numpy as np
import cv2

NO_DET = 255  # 与 bucket_select / 真值检测器同哨兵


def make_detect_params(rules_cfg):
    """competition_rules.color_id.camera_detector 段 → 检测参数 dict。"""
    cd = (rules_cfg.get("color_id", {}) or {}).get("camera_detector", {}) or {}
    hsv = cd.get("hsv", {}) or {}
    conf = cd.get("conf", {}) or {}
    return {
        "min_blob_area_px": int(cd.get("min_blob_area_px", 40)),
        "red": ((int(hsv.get("red_lo1", 0)), int(hsv.get("red_hi1", 8))),
                (int(hsv.get("red_lo2", 172)), int(hsv.get("red_hi2", 179)))),
        "blue": (int(hsv.get("blue_lo", 100)), int(hsv.get("blue_hi", 115))),
        "yellow": (int(hsv.get("yellow_lo", 20)), int(hsv.get("yellow_hi", 35))),
        "s_min": int(hsv.get("s_min", 120)),
        "v_min": int(hsv.get("v_min", 80)),
        "w_circularity": float(conf.get("w_circularity", 0.6)),
        "w_purity": float(conf.get("w_purity", 0.4)),
        "w_area": float(conf.get("w_area", 0.0)),
        "area_floor_px": int(conf.get("area_floor_px", 40)),
    }


def _color_mask(hsv, lo, hi, s_min, v_min):
    return ((hsv[:, :, 0] >= lo) & (hsv[:, :, 0] <= hi)
            & (hsv[:, :, 1] >= s_min) & (hsv[:, :, 2] >= v_min))


def _largest_blob(mask):
    """最大连通域 → (area, 质心, 外接框, mask) 或 None。"""
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    cent = (float(cents[i][0]), float(cents[i][1]))
    box = (int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP]),
           int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT]))
    return area, cent, box, (labels == i)


def _circularity(blob_mask):
    contours, _ = cv2.findContours(blob_mask.astype(np.uint8),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    c = max(contours, key=cv2.contourArea)
    a = cv2.contourArea(c)
    p = cv2.arcLength(c, True)
    if a <= 0.0 or p <= 1e-6:
        return 0.0
    return min(1.0, 4.0 * math.pi * a / (p * p))


def _purity(blob_mask, area, cent):
    """色纯度：色块外接圆内命中像素占比。完整盘≈1；裁边/破碎<1。"""
    r = math.sqrt(max(area, 1) / math.pi)
    x0 = max(0, int(cent[0] - r) - 1)
    x1 = min(blob_mask.shape[1], int(cent[0] + r) + 2)
    y0 = max(0, int(cent[1] - r) - 1)
    y1 = min(blob_mask.shape[0], int(cent[1] + r) + 2)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    yy, xx = np.mgrid[y0:y1, x0:x1]
    inside = (xx - cent[0]) ** 2 + (yy - cent[1]) ** 2 <= r * r
    circle_area = math.pi * r * r
    hits = int(np.count_nonzero(blob_mask[y0:y1, x0:x1] & inside))
    return max(0.0, min(1.0, hits / circle_area))


def detect(img_bgr, params):
    """单帧检测。

    img_bgr: (H,W,3) uint8 BGR（调用方负责编码转换；D435 rgb8 先翻转）
    返回 (color_u8, conf, type_u8)；color_u8=NO_DET 时 conf=0.0、type=255。
    type_u8 不在本层解析（同色多类型歧义属配置层语义），恒返 NO_DET，
    由节点按 color_map 反查表回填。
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    s_min, v_min = params["s_min"], params["v_min"]
    candidates = (
        (0, _color_mask(hsv, params["red"][0][0], params["red"][0][1], s_min, v_min)
           | _color_mask(hsv, params["red"][1][0], params["red"][1][1], s_min, v_min)),
        (1, _color_mask(hsv, params["blue"][0], params["blue"][1], s_min, v_min)),
        (2, _color_mask(hsv, params["yellow"][0], params["yellow"][1], s_min, v_min)),
    )
    best_color, best = NO_DET, None
    for cu8, mask in candidates:
        blob = _largest_blob(mask)
        if blob is None:
            continue
        if best is None or blob[0] > best[0]:
            best_color, best = cu8, blob
    if best is None or best[0] < params["min_blob_area_px"]:
        return NO_DET, 0.0, NO_DET

    area, cent, _, blob_mask = best
    circ = _circularity(blob_mask)
    pur = _purity(blob_mask, area, cent)
    conf = params["w_circularity"] * circ + params["w_purity"] * pur
    if params["w_area"] > 0.0:
        conf += params["w_area"] * min(1.0, area / max(1.0, 4.0 * params["area_floor_px"]))
    conf = max(0.0, min(1.0, conf))
    return best_color, conf, NO_DET  # type_u8 由节点按反查表回填
