# -*- coding: utf-8 -*-
"""rules.py — 比赛规则合规判定纯函数（无 ROS 依赖）。

科目三退赛条款（rule_monitor_node 的判定核心，P8 自检 B 组锚点）：
- 飞出比赛区持续 >10s → 退赛
- 计划外落地（起降区外、未 armed）→ 退赛（防抖豁免碰撞弹起瞬态）
- 飞行高度 >5m 持续 → 违规
- 返航必须穿过树林（走廊多边形代理）
"""


def oob_tick(outside, accum_s, grace_s, dt=1.0):
    """出界计时一拍。返回 (accum_s, tripped)。inside 即回零。

    dt=每拍时长（rule_monitor 20Hz 传 0.05；缺省 1.0 兼容 1Hz 节拍）。
    """
    accum_s = accum_s + dt if outside else 0.0
    return accum_s, accum_s >= grace_s


def landing_verdict(z, contact_z, armed, in_pad_zone, grace_accum_s, grace_s,
                    dt=1.0):
    """落地判据一拍。返回 (accum_s, verdict)。

    verdict: "none" | "planned"（armed 且 pad 内=合法 touchdown）
           | "unauthorized"（pad 外贴地且未 armed，防抖到期）
    防抖：pad 外未 armed 贴地累计 grace_s 才判（豁免碰撞弹起瞬态）。
    """
    contact = z <= contact_z
    if contact and armed and in_pad_zone:
        return 0.0, "planned"
    if contact and not armed and not in_pad_zone:
        grace_accum_s += dt
        if grace_accum_s >= grace_s:
            return grace_accum_s, "unauthorized"
        return grace_accum_s, "none"
    return 0.0, "none"


def height_verdict(z, max_z, grace_accum_s, grace_s, dt=1.0):
    """高度违规一拍。返回 (accum_s, tripped)。低于上限即回零。"""
    accum_s = grace_accum_s + dt if z > max_z else 0.0
    return accum_s, accum_s >= grace_s


def _pip(poly, x, y):
    """射线法点在多边形内（与 geometry.point_in_poly_2d 同款，本地副本
    保持本模块零依赖）。"""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i][0], poly[i][1]
        xj, yj = poly[j][0], poly[j][1]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# 对外别名（rule_monitor 用；_pip 保持本模块内部实现自由）
point_in_poly = _pip


def corridor_hit(path_xy, poly):
    """返程轨迹是否命中走廊多边形（任一采样点入多边形即命中）。"""
    for (x, y) in path_xy:
        if _pip(poly, x, y):
            return True
    return False


def geofence_in_fence(geofence_poly, fence_aabb, margin=1e-6):
    """自检断言：geofence 必须落在围栏 AABB 内（顶点级检查）。

    fence_aabb=(xmin, xmax, ymin, ymax)；防两处几何失和再次发生
    （world_builder fence_e 18.6 vs yaml 22.5 教训）。
    """
    xmin, xmax, ymin, ymax = fence_aabb
    for (x, y) in geofence_poly:
        if not (xmin - margin <= x <= xmax + margin
                and ymin - margin <= y <= ymax + margin):
            return False
    return True
