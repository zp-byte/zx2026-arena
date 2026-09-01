# -*- coding: utf-8 -*-
"""纯几何工具：向量、多边形、求交。无 ROS 依赖，可独立测试。"""
import math


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm2(a):
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2]


def norm(a):
    return math.sqrt(norm2(a))


def dist(a, b):
    return norm(sub(a, b))


def dist_xy(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def point_in_poly_2d(p, poly):
    """射线法判断 2D 点在多边形内（poly: [(x,y),...]，闭合由函数保证）。"""
    x, y = p[0], p[1]
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xint = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < xint:
                inside = not inside
        j = i
    return inside


def ray_sphere(origin, direction, center, radius):
    """射线与球求交，返回最近 t（>=0），无交返回 None。方向需归一化。"""
    oc = sub(origin, center)
    b = dot(oc, direction)
    c = norm2(oc) - radius * radius
    disc = b * b - c
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    t1 = -b - sq
    if t1 >= 0:
        return t1
    t2 = -b + sq
    return t2 if t2 >= 0 else None


def ray_cylinder(origin, direction, center, radius, z0, z1):
    """射线与竖直圆柱（center=(cx,cy)，半径 radius，z∈[z0,z1]）求交。

    只求侧面（lidar 俯仰角有限，不射穿上下底）。返回最近 t（>=0）或 None。
    """
    ox, oy, oz = origin
    dx, dy, dz = direction
    a = dx * dx + dy * dy
    if a < 1e-12:
        # 近似竖直射线，忽略柱面
        return None
    fx = ox - center[0]
    fy = oy - center[1]
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    best = None
    for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
        if t < 0:
            continue
        z = oz + t * dz
        if z0 <= z <= z1:
            if best is None or t < best:
                best = t
    return best


def ray_aabb(origin, direction, lo, hi):
    """射线与轴对齐盒求交（lo/hi: 最小/最大角点），返回最近 t 或 None。"""
    tmin = 0.0
    tmax = float("inf")
    for i in range(3):
        if abs(direction[i]) < 1e-12:
            if origin[i] < lo[i] or origin[i] > hi[i]:
                return None
        else:
            ood = 1.0 / direction[i]
            t1 = (lo[i] - origin[i]) * ood
            t2 = (hi[i] - origin[i]) * ood
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
    return tmin if tmin >= 0 else (tmax if tmax >= 0 else None)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize(v):
    n = norm(v)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return scale(v, 1.0 / n)
