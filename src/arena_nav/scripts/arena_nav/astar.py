# -*- coding: utf-8 -*-
"""2.5D A*：在 (x, y) 栅格上搜索，z 视为常数飞行层。

占用判定复用 Scene.collides_xy(z_lo, z_hi)，可加 inflation 膨胀。
"""
import heapq

import numpy as np


class AStar:
    def __init__(self, scene, resolution=0.5, inflation=0.3,
                 x_range=(-35.0, 35.0), y_range=(-35.0, 25.0),
                 z_lo=1.0, z_hi=4.0):
        self.scene = scene
        self.res = resolution
        self.inflation = inflation
        self.z_lo = z_lo
        self.z_hi = z_hi
        self.x_range = x_range
        self.y_range = y_range
        self.nx = int((x_range[1] - x_range[0]) / resolution) + 1
        self.ny = int((y_range[1] - y_range[0]) / resolution) + 1
        self._occ_cache = {}

    def _occ(self, ix, iy):
        key = (ix, iy)
        if key not in self._occ_cache:
            x = self.x_range[0] + ix * self.res
            y = self.y_range[0] + iy * self.res
            self._occ_cache[key] = self.scene.collides_xy(
                x, y, self.z_lo, self.z_hi, pad=self.inflation)
        return self._occ_cache[key]

    def _to_index(self, x, y):
        ix = int(round((x - self.x_range[0]) / self.res))
        iy = int(round((y - self.y_range[0]) / self.res))
        return ix, iy

    def _to_xy(self, ix, iy):
        return (self.x_range[0] + ix * self.res,
                self.y_range[0] + iy * self.res)

    def _nearest_free(self, ix, iy):
        """从 (ix,iy) 起螺旋向外找最近的非占用格索引，找不到返回 None。"""
        if not self._occ(ix, iy):
            return (ix, iy)
        for r in range(1, max(self.nx, self.ny)):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if not (0 <= nx < self.nx and 0 <= ny < self.ny):
                        continue
                    if not self._occ(nx, ny):
                        return (nx, ny)
        return None

    def plan(self, start, goal):
        """返回世界系路径点 [(x,y),...]（含 start；目标被占时以最近可达点为终）。"""
        s = self._to_index(start[0], start[1])
        g = self._to_index(goal[0], goal[1])
        if self._occ(*s):
            # 起点被占（如从森林内部起飞），暂不寻路
            return [start]
        if self._occ(*g):
            # 目标被占：退化为最近可达点（如穿越区中心恰有树）
            alt = self._nearest_free(*g)
            if alt is None:
                return [start]
            g = alt

        open_heap = [(0.0, s)]
        came_from = {}
        g_cost = {s: 0.0}
        closed = set()
        found = False

        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == g:
                found = True
                break
            cx, cy = cur
            for dx, dy, cost in [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                                 (-1, -1, 1.414), (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414)]:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.nx and 0 <= ny < self.ny):
                    continue
                if self._occ(nx, ny):
                    continue
                ng = g_cost[cur] + cost
                if (nx, ny) not in g_cost or ng < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = ng
                    f = ng + self._heu(nx, ny, g)
                    came_from[(nx, ny)] = cur
                    heapq.heappush(open_heap, (f, (nx, ny)))

        if not found:
            # 起点可达但终点无法到达（理论不应发生）：退化为最近可达点
            alt = self._nearest_free(*g)
            if alt is None or alt == s:
                return [start]
            g = alt
            return self.plan(start, (self._to_xy(alt[0], alt[1])))

        # 回溯
        path = []
        node = g
        while node != s:
            path.append(node)
            node = came_from[node]
        path.append(s)
        path.reverse()
        return [self._to_xy(ix, iy) for (ix, iy) in path]

    def _heu(self, ix, iy, g):
        gx = self.x_range[0] + g[0] * self.res
        gy = self.y_range[0] + g[1] * self.res
        x = self.x_range[0] + ix * self.res
        y = self.y_range[0] + iy * self.res
        return abs(x - gx) + abs(y - gy)
