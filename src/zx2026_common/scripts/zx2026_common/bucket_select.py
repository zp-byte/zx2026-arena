# -*- coding: utf-8 -*-
"""bucket_select.py — RECON 扫描择点纯决策器（无 ROS 依赖）。

规则背景：3 个投送平台的颜色-补给类型对应赛前不告知，机载须逐平台
悬停侦察颜色，连续命中本机箱色才允许锁定投放目标。

判据（P8 自检 A 组锚点）：
- 命中：det==box 且 conf>=min_conf → hit+1；hit>=n_frames → LOCK
- 异色：det!=box → 清零（见过但不是我的，此平台排除）
- 漏检（NO_DET）：清零（观测断了，重数）
- 低置信：不清零也不计数（观测质量差，冻结现场）
"""
NO_DET = 255


def new_state():
    return {"hit": 0}


def bucket_scan_step(color_u8, conf, box_u8, min_conf=0.8, n_frames=3,
                     state=None):
    """一拍扫描决策。

    color_u8: 本拍检测颜色（NO_DET=无检测）
    conf:     本拍置信度 [0,1]
    box_u8:   本机箱色编码
    返回 (state, decision)：decision None=继续观测，"LOCK"=锁定当前平台。
    """
    if state is None:
        state = new_state()
    if color_u8 == NO_DET:
        state["hit"] = 0
        return state, None
    if conf < min_conf:
        return state, None                      # 低置信：不计不清
    if color_u8 != box_u8:
        state["hit"] = 0                        # 异色：排除此平台
        return state, None
    state["hit"] += 1
    if state["hit"] >= n_frames:
        return state, "LOCK"
    return state, None
