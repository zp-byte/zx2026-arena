# -*- coding: utf-8 -*-
"""arena_world::scene_markers — 从 Scene 构建静态场景消息（PoseArray + MarkerArray）。

python 后端由 world_node 调用，Gazebo 后端由 scene_marker_node 调用：
同一份森林/投放点/区域描述，确保两种物理后端在 RViz 里看到完全一致的场景。
"""
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray


def build_scene_messages(scene):
    """返回 (PoseArray, MarkerArray)：投放点列表 + RViz 森林/区域/投放点标记。"""
    # 投放点 PoseArray
    pa = PoseArray()
    pa.header.frame_id = "world"
    pa.header.stamp.secs = 0
    for dp in scene.drop_points:
        p = Pose()
        p.position = Point(dp.xyz[0], dp.xyz[1], dp.xyz[2])
        p.orientation.w = 1.0
        pa.poses.append(p)

    # RViz MarkerArray
    ma = MarkerArray()
    colors = {"check_in": (1, 1, 0), "waiting": (1, 0.5, 0),
              "takeoff": (0, 1, 0), "transit": (0, 0.8, 0.3), "return": (0, 1, 0)}
    mid = 0
    vis = scene.scene_cfg.get("visual", {})
    ground_z = scene.venue["ground_z"]
    for z in scene.zones.values():
        if z.kind == "forest":
            continue  # 森林用柱子标记
        mk = Marker()
        mk.header.frame_id = "world"
        mk.ns = "zones"
        mk.id = mid
        mid += 1
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale.x = 0.1
        col = colors.get(z.id, (0.5, 0.5, 0.5))
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = col[0], col[1], col[2], 1.0
        pts = [Point(x, y, ground_z + 0.01) for (x, y) in z.polygon]
        pts.append(Point(z.polygon[0][0], z.polygon[0][1], ground_z + 0.01))
        mk.points = pts
        ma.markers.append(mk)

    # 森林（树：圆柱树干 + 球冠；其余障碍：立方体）
    fs = float(vis.get("forest_marker_scale", [0.3, 0.3, 2.5])[2])
    ob_count = 0
    for ob in scene.obstacles:
        if ob_count > 400:
            break
        if ob.kind == "tree":
            # 树干（棕色圆柱）
            mk = Marker()
            mk.header.frame_id = "world"
            mk.ns = "forest"
            mk.id = mid
            mid += 1
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position = Point(ob.cx, ob.cy, ground_z + ob.trunk_h / 2.0)
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 2.0 * ob.trunk_r
            mk.scale.z = ob.trunk_h
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = (0.45, 0.32, 0.18, 0.95)
            ma.markers.append(mk)
            # 球冠（绿色球）
            mk = Marker()
            mk.header.frame_id = "world"
            mk.ns = "forest"
            mk.id = mid
            mid += 1
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position = Point(ob.cx, ob.cy, ob.crown_z)
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 2.0 * ob.crown_r
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = (0.14, 0.48, 0.22, 0.9)
            ma.markers.append(mk)
            ob_count += 1
            continue
        mk = Marker()
        mk.header.frame_id = "world"
        mk.ns = "forest"
        mk.id = mid
        mid += 1
        mk.type = Marker.CUBE
        mk.action = Marker.ADD
        cx = (ob.lo[0] + ob.hi[0]) / 2
        cy = (ob.lo[1] + ob.hi[1]) / 2
        cz = (ob.lo[2] + ob.hi[2]) / 2
        mk.pose.position = Point(cx, cy, cz)
        mk.pose.orientation.w = 1.0
        mk.scale.x = ob.hi[0] - ob.lo[0]
        mk.scale.y = ob.hi[1] - ob.lo[1]
        mk.scale.z = ob.hi[2] - ob.lo[2]
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (0.4, 0.6, 0.3, 0.9)
        ma.markers.append(mk)
        ob_count += 1

    # 投送平台（直径 1.2m 薄盘 h=0.05，色=dp.color；外缘白色描边=口径标识）
    platform_colors = {"red": (0.90, 0.20, 0.20), "blue": (0.20, 0.45, 0.90),
                     "yellow": (0.90, 0.85, 0.10)}
    for dp in scene.drop_points:
        col = platform_colors.get(dp.color, (0.6, 0.6, 0.6))
        mk = Marker()
        mk.header.frame_id = "world"
        mk.ns = "drop_points"
        mk.id = mid
        mid += 1
        mk.type = Marker.CYLINDER
        mk.action = Marker.ADD
        mk.pose.position = Point(dp.xyz[0], dp.xyz[1], ground_z + 0.025)
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = 1.2   # 规则口径：平台直径 1.2m
        mk.scale.z = 0.05
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (col[0], col[1], col[2], 0.95)
        ma.markers.append(mk)
        # 口径描边环（白，薄）
        mk = Marker()
        mk.header.frame_id = "world"
        mk.ns = "drop_points"
        mk.id = mid
        mid += 1
        mk.type = Marker.CYLINDER
        mk.action = Marker.ADD
        mk.pose.position = Point(dp.xyz[0], dp.xyz[1], ground_z + 0.01)
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = 1.36
        mk.scale.z = 0.02
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (0.95, 0.95, 0.95, 0.9)
        ma.markers.append(mk)

    return pa, ma
