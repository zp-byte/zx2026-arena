# -*- coding: utf-8 -*-
"""arena_sensor 公共：感知模型参数读取、Tag 检测几何判定。"""
import math

from zx2026_common import config as cfg
from zx2026_common import geometry as geo


def load_tag_params():
    r = cfg.load("competition_rules.yaml")
    t = r.get("tag_detection", {})
    s = cfg.load("sim_settings.yaml").get("sensor", {}).get("tag", {})
    seed = int(cfg.load("sim_settings.yaml").get("run_seed", 42))
    return {
        "range": float(t.get("tag_range", 4.0)),
        "fov_deg": float(t.get("fov_deg", 60.0)),
        "occlusion": bool(t.get("occlusion", True)),
        "min_confidence": float(r.get("type_match", {}).get("min_confidence", 0.8)),
        # 噪声参数
        "range_noise_std": float(s.get("range_noise_std", 0.05)),
        "false_negative_rate": float(s.get("false_negative_rate", 0.05)),
        "rng_seed": seed,
    }


def detect_tag(scene, odom_pos, odom_yaw, params, rng=None):
    """对每个投放点做：距离 ≤ range、在 FOV 内、无遮挡。

    返回最近且可见的投放点 (drop_point, confidence)，无则 (None, 0.0)。
    若提供 rng (random.Random)，则叠加传感器噪声：漏检 + 置信度抖动。
    """
    fov_deg = float(params["fov_deg"])
    fov_skip = 1.5  # 正下方识别：水平距离 < 1.5m 视为已对准，跳过水平 FOV 判定
    range_noise_std = float(params.get("range_noise_std", 0.0))
    false_negative_rate = float(params.get("false_negative_rate", 0.0))
    best = None
    best_conf = 0.0
    for dp in scene.drop_points:
        d = geo.dist_xy(odom_pos, dp.xyz)
        if d > params["range"]:
            continue
        # 视角：投放点相对机体朝向角差（仅远距离巡航/进场时约束；正下方由下视相机覆盖）
        if d >= fov_skip:
            to_dp = (dp.xyz[0] - odom_pos[0], dp.xyz[1] - odom_pos[1])
            ang = math.atan2(to_dp[1], to_dp[0])
            diff = abs(normalize_angle(ang - odom_yaw))
            if diff > math.radians(fov_deg) / 2.0:
                continue
        # 遮挡：自无人机向投放点水平连线，检查是否有障碍挡住
        if params["occlusion"]:
            dirv = geo.normalize((dp.xyz[0] - odom_pos[0],
                                  dp.xyz[1] - odom_pos[1], 0.0))
            t = scene.raycast(odom_pos, dirv, max_t=d - 0.05, ground=False)
            if t is not None:
                continue
        # 漏检噪声：距离越远越容易漏检
        if rng is not None and false_negative_rate > 0:
            fn_prob = false_negative_rate * (d / params["range"])
            if rng.random() < fn_prob:
                continue
        # 置信度：正上方（≤1m）视为可靠命中；否则距离越近越高
        if d < 1.0:
            conf = 1.0
        else:
            conf = max(0.0, 1.0 - d / params["range"])
        # 置信度噪声：距离测量抖动
        if rng is not None and range_noise_std > 0:
            conf += rng.gauss(0.0, range_noise_std / params["range"])
            conf = max(0.0, min(1.0, conf))
        conf = min(conf, params["min_confidence"] + 0.2)
        if best is None or d < geo.dist_xy(odom_pos, best.xyz):
            best = dp
            best_conf = conf
    return best, best_conf


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
