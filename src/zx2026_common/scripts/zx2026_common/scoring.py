# -*- coding: utf-8 -*-
"""scoring.py — S1/S2 比赛口径计分纯函数（无 ROS 依赖）。

S1 投放分 60：每箱正确投放到对应补给点 +10，错平台/偏平台 0。
S2 完成分 40：时限内飞抵补给点并返回起降区降落的机数阶梯。
"""

VERDICT_CORRECT = "correct"
VERDICT_WRONG_COLOR = "wrong_color"
VERDICT_OFF_BUCKET = "off_bucket"


def judge_drop(release_xy, bucket_xy, bucket_color, box_color, tol_m=0.6):
    """释放时刻判平台（裁判口径，独立于机载 MATCH）。

    - correct:     箱色==平台色 且 释放水平距平台心 <= tol_m（平台径 1.2m 投影内）
    - wrong_color: 色不匹配（错平台）
    - off_bucket:  色匹配但水平偏出平台上空投影
    """
    dx = release_xy[0] - bucket_xy[0]
    dy = release_xy[1] - bucket_xy[1]
    in_proj = (dx * dx + dy * dy) <= tol_m * tol_m
    if bucket_color != box_color:
        return VERDICT_WRONG_COLOR
    return VERDICT_CORRECT if in_proj else VERDICT_OFF_BUCKET


def s1_team(drops, cap=60):
    """drops: 每机 verdict 列表 → 队级 S1（correct 数×10，封顶 cap）。"""
    n = sum(1 for v in drops if v == VERDICT_CORRECT)
    return min(10 * n, cap)


def s2_from_landed(n, ladder=None):
    """完成分阶梯（默认规则表 6→40/5→30/4→25/3→15/2→10/1→5/0→0）。"""
    if ladder is None:
        ladder = {6: 40, 5: 30, 4: 25, 3: 15, 2: 10, 1: 5, 0: 0}
    return ladder.get(int(n), 0)
