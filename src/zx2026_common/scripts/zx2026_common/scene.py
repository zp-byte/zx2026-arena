# -*- coding: utf-8 -*-
"""场景模型：解析 scene_topology.yaml + fleet.yaml，提供碰撞 / 射线 / 占用查询。

无 ROS 依赖（除配置文件路径），供 arena_world / arena_sensor / arena_nav /
arena_mission 共用同一份场景权威数据。
"""
import math
import random

from zx2026_common import config as cfg
from zx2026_common import geometry as geo


# ---------------------------------------------------------------------------
# 障碍物
# ---------------------------------------------------------------------------
class Obstacle:
    """轴对齐盒障碍（立木/灌木/路缘/围栏/标识柱的包围盒）。

    kind=="tree" 额外携带「圆柱树干 + 球冠」几何（lo/hi 保留作 AABB 兜底，
    供其余 kind 使用）。树干为硬碰撞体并参与 lidar / A* 占用；球冠仅视觉
    （rviz / gazebo），不参与碰撞、lidar 或占用。
    """

    __slots__ = ("lo", "hi", "kind", "id",
                 "cx", "cy", "trunk_r", "trunk_h", "crown_r", "crown_z")

    def __init__(self, lo, hi, kind="tree", oid=0):
        self.lo = lo
        self.hi = hi
        self.kind = kind
        self.id = oid
        # tree 专用几何字段（其余 kind 保持 None）
        self.cx = None
        self.cy = None
        self.trunk_r = None
        self.trunk_h = None
        self.crown_r = None
        self.crown_z = None


class Zone:
    __slots__ = ("id", "kind", "polygon", "pads", "ref")

    def __init__(self, zid, kind, polygon, pads=None, ref=None):
        self.id = zid
        self.kind = kind
        self.polygon = polygon  # [(x,y),...]
        self.pads = pads or []
        self.ref = ref

    def contains_2d(self, p):
        return geo.point_in_poly_2d(p, self.polygon)


class DropPoint:
    __slots__ = ("id", "xyz", "type_id", "marker")

    def __init__(self, did, xyz, type_id, marker):
        self.id = did
        self.xyz = tuple(xyz)
        self.type_id = type_id
        self.marker = marker


class DroneConfig:
    __slots__ = ("drone_id", "payload", "sensor_range")

    def __init__(self, drone_id, payload, sensor_range):
        self.drone_id = drone_id
        self.payload = payload
        self.sensor_range = sensor_range


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
class Scene:
    def __init__(self, scene_cfg=None, fleet_cfg=None):
        self.scene_cfg = scene_cfg if scene_cfg is not None else cfg.load("scene_topology.yaml")
        self.fleet_cfg = fleet_cfg if fleet_cfg is not None else cfg.load("fleet.yaml")

        v = self.scene_cfg["venue"]
        self.venue = {"size": tuple(v["size"]), "ground_z": float(v.get("ground_z", 0.0))}

        # zones（先解析 ref）
        raw_zones = list(self.scene_cfg["zones"])
        by_id = {}
        for z in raw_zones:
            kind = z.get("kind", "open_field")
            if z.get("kind") == "pad" and z.get("pads"):
                pads = self._make_pads(z)
            else:
                pads = None
            by_id[z["id"]] = Zone(z["id"], kind,
                                  [tuple(p) for p in z.get("polygon", [])],
                                  pads=pads, ref=z.get("ref"))
        for z in raw_zones:
            if z.get("ref"):
                target = by_id[z["ref"]]
                by_id[z["id"]].pads = target.pads
                by_id[z["id"]].polygon = target.polygon
        self.zones = by_id

        # 投放点
        self.drop_points = [
            DropPoint(d["id"], d["xyz"], d["type_id"], d["marker"])
            for d in self.scene_cfg.get("drop_points", [])
        ]
        self.payload_types = list(self.scene_cfg.get("payload_types", []))

        # 穿越区
        cz = self.scene_cfg.get("crossing_zone", {})
        self.crossing_zone = [tuple(p) for p in cz.get("polygon", [])]

        # 障碍物生成
        transit = self.zones.get("transit")
        self.obstacles = []
        self.rng = random.Random(self._forest_seed())
        self._gen_forest(transit)
        self._load_static_obstacles()

        # 无人机编队
        self.drone_radius = float(self.fleet_cfg.get("drone_radius", 0.35))
        self.drones = [
            DroneConfig(d["drone_id"], d["payload"], float(d.get("sensor_range", 5.0)))
            for d in self.fleet_cfg.get("drones", [])
        ]
        self.drone_count = len(self.drones)

    def _forest_seed(self):
        for z in self.scene_cfg["zones"]:
            if z.get("kind") == "forest" and z.get("forest"):
                return z["forest"].get("density_seed", 42)
        return 42

    def _make_pads(self, zone):
        n = int(zone["pads"])
        sx, sy = zone["pad_spacing"]
        ox, oy = zone["pad_origin"]
        pads = []
        cols = 3
        for i in range(n):
            c = i % cols
            r = i // cols
            pads.append((ox + c * sx, oy + r * sy))
        return pads

    def _gen_forest(self, transit):
        if transit is None or transit.kind != "forest":
            return
        f = None
        for z in self.scene_cfg["zones"]:
            if z.get("kind") == "forest":
                f = z.get("forest")
                break
        if not f:
            return

        generator = f.get("generator", "procedural")
        oid = 0
        clear_r = float(f.get("drop_clearance", 1.5))
        dps = [(d.xyz[0], d.xyz[1]) for d in self.drop_points]

        def near_drop(x, y):
            return any((x - dx) ** 2 + (y - dy) ** 2 < clear_r * clear_r
                       for dx, dy in dps)

        if generator == "static":
            # 静态树集：按配置规则生成确定性位置
            tree_sets = f.get("tree_sets", [])
            for ts in tree_sets:
                layout = ts.get("layout", "grid")
                count = int(ts.get("count", 0))
                trunk_r = ts.get("trunk_r", [0.16, 0.30])
                height = float(ts.get("height", 3.2))
                for i in range(count):
                    if layout == "grid":
                        cols = int(ts.get("cols", 7))
                        ox, oy = ts.get("origin", [0.0, 0.0])
                        sx, sy = ts.get("spacing", [4.1, 4.6])
                        c = i % cols
                        r = i // cols
                        bx = ox + c * sx
                        by = oy + r * sy
                    elif layout == "line":
                        sx, sy = ts.get("start", [0.0, 0.0])
                        ex, ey = ts.get("end", [1.0, 1.0])
                        t = (count - 1 - i) / max(count - 1, 1) if count > 1 else 0.5
                        bx = sx + t * (ex - sx)
                        by = sy + t * (ey - sy)
                    else:
                        continue
                    jitter = float(ts.get("jitter", 0.0))
                    if jitter > 0:
                        bx += self.rng.uniform(-jitter / 2.0, jitter / 2.0)
                        by += self.rng.uniform(-jitter / 2.0, jitter / 2.0)
                    if near_drop(bx, by):
                        continue
                    r = self.rng.uniform(*trunk_r)
                    h = height
                    crown_r = float(ts.get("crown_r", 1.2))
                    gz = self.venue["ground_z"]
                    lo = (bx - r, by - r, gz)
                    hi = (bx + r, by + r, gz + h)
                    ob = Obstacle(lo, hi, "tree", oid)
                    ob.cx = bx
                    ob.cy = by
                    ob.trunk_r = r
                    ob.trunk_h = h
                    ob.crown_r = crown_r
                    ob.crown_z = gz + h  # 球冠球心在树干顶，冠体延伸到 ~h+crown_r
                    self.obstacles.append(ob)
                    oid += 1
            return

        # procedural 旧逻辑（保留可回退）
        poly = transit.polygon
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        for _ in range(int(f.get("n_trees", 300))):
            x = self.rng.uniform(xmin, xmax)
            y = self.rng.uniform(ymin, ymax)
            if not transit.contains_2d((x, y)):
                continue
            if near_drop(x, y):
                continue
            r = self.rng.uniform(*f["trunk_r"])
            h = self.rng.uniform(*f["tree_h"])
            crown_r = float(f.get("crown_r", 1.2))
            gz = self.venue["ground_z"]
            lo = (x - r, y - r, gz)
            hi = (x + r, y + r, gz + h)
            ob = Obstacle(lo, hi, "tree", oid)
            ob.cx = x
            ob.cy = y
            ob.trunk_r = r
            ob.trunk_h = h
            ob.crown_r = crown_r
            ob.crown_z = gz + h
            self.obstacles.append(ob)
            oid += 1
        for _ in range(int(f.get("n_bushes", 120))):
            x = self.rng.uniform(xmin, xmax)
            y = self.rng.uniform(ymin, ymax)
            if not transit.contains_2d((x, y)):
                continue
            if near_drop(x, y):
                continue
            w = self.rng.uniform(0.3, 0.6)
            h = self.rng.uniform(*f["bush_h"])
            lo = (x - w / 2, y - w / 2, self.venue["ground_z"])
            hi = (x + w / 2, y + w / 2, self.venue["ground_z"] + h)
            self.obstacles.append(Obstacle(lo, hi, "bush", oid))
            oid += 1

    def _load_static_obstacles(self):
        for sob in self.scene_cfg.get("static_obstacles", []):
            lo = tuple(sob.get("lo", [0, 0, 0]))
            hi = tuple(sob.get("hi", [1, 1, 1]))
            kind = sob.get("kind", "box")
            oid = len(self.obstacles)
            self.obstacles.append(Obstacle(lo, hi, kind, oid))

    # ------------------------------------------------------------------ queries
    def get_pads(self):
        tz = self.zones.get("takeoff")
        return tz.pads if tz else []

    def drone_config(self, drone_id):
        for d in self.drones:
            if d.drone_id == drone_id:
                return d
        return None

    def in_crossing_zone(self, p):
        """判断二维点 p=(x,y) 是否在穿越区内。"""
        if not self.crossing_zone:
            return True
        return geo.point_in_poly_2d(p, self.crossing_zone)

    @property
    def crossing_zone_center(self):
        """返回穿越区多边形重心 (x,y)。"""
        if not self.crossing_zone:
            return (0.0, 0.0)
        poly = self.crossing_zone
        n = len(poly)
        cx = sum(p[0] for p in poly) / n
        cy = sum(p[1] for p in poly) / n
        return (cx, cy)

    def collides(self, pos, radius=None):
        """球(pos, radius) 与任一障碍或地面碰撞。"""
        radius = radius if radius is not None else self.drone_radius
        gz = self.venue["ground_z"]
        if pos[2] - radius < gz:
            return True
        for ob in self.obstacles:
            if ob.kind == "tree":
                # 树干：球 vs 竖直圆柱；球冠不参与硬碰撞
                if self._collides_trunk(pos, radius, ob):
                    return True
                continue
            lo, hi = ob.lo, ob.hi
            cx = geo.clamp(pos[0], lo[0], hi[0])
            cy = geo.clamp(pos[1], lo[1], hi[1])
            cz = geo.clamp(pos[2], lo[2], hi[2])
            if geo.norm2((pos[0] - cx, pos[1] - cy, pos[2] - cz)) <= radius * radius:
                return True
        return False

    def _collides_trunk(self, pos, radius, ob):
        """球(pos, radius) 是否与树的竖直树干相交。"""
        if geo.dist_xy(pos, (ob.cx, ob.cy)) > ob.trunk_r + radius:
            return False
        gz = self.venue["ground_z"]
        return pos[2] + radius >= gz and pos[2] - radius <= gz + ob.trunk_h

    def collides_xy(self, x, y, z_lo, z_hi, pad=0.0):
        """2D 单元(x,y) 在 z 带 [z_lo,z_hi] 内是否有障碍（A* 占用用）。"""
        for ob in self.obstacles:
            # 围栏/标识柱低于巡航高度，A* 不应被其阻挡
            if ob.kind in ("fence", "marker"):
                continue
            if ob.kind == "tree":
                gz = self.venue["ground_z"]
                dxy = geo.dist_xy((x, y), (ob.cx, ob.cy))
                # 树干圆盘（球冠仅视觉，不参与占用）
                if not (gz + ob.trunk_h < z_lo or gz > z_hi):
                    if dxy <= ob.trunk_r + pad:
                        return True
                continue
            if ob.hi[2] < z_lo or ob.lo[2] > z_hi:
                continue
            if x + pad >= ob.lo[0] and x - pad <= ob.hi[0] and \
               y + pad >= ob.lo[1] and y - pad <= ob.hi[1]:
                return True
        return False

    def raycast(self, origin, direction, max_t=float("inf"), ground=True):
        """返回最近命中距离 t（None 表示无命中）。direction 需归一化。"""
        best = None
        gz = self.venue["ground_z"]
        if ground and direction[2] < -1e-9:
            t = (gz - origin[2]) / direction[2]
            if 0 <= t <= max_t:
                best = t
        for ob in self.obstacles:
            if ob.kind == "tree":
                # 树干圆柱（球冠仅视觉，lidar 不反射枝叶）
                gz = self.venue["ground_z"]
                t = geo.ray_cylinder(origin, direction, (ob.cx, ob.cy),
                                     ob.trunk_r, gz, gz + ob.trunk_h)
                if t is not None and 0 <= t <= max_t:
                    if best is None or t < best:
                        best = t
                continue
            t = geo.ray_aabb(origin, direction, ob.lo, ob.hi)
            if t is not None and 0 <= t <= max_t:
                if best is None or t < best:
                    best = t
        return best
