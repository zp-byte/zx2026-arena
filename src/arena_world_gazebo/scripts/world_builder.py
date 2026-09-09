#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world_gazebo::world_builder — 从 scene_topology 生成 Gazebo 11 世界。

复用 zx2026_common.scene 的确定性场景（同 seed=42）：树/灌木取 Scene 内 AABB 的
外接圆柱/盒，保证与 python 后端的碰撞模型和 A* 规划几何一致 —— 换物理引擎不换场景。
输出: arena_world_gazebo/worlds/forest_world.world
用法: python3 world_builder.py
"""
import os
import sys

_WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, os.path.join(_WS, "src/zx2026_common/scripts"))
sys.path.insert(0, os.path.join(_WS, "src/arena_world_gazebo/scripts"))

from zx2026_common.scene import Scene

OUT = os.path.join(os.path.dirname(__file__), "..", "worlds", "forest_world.world")

# 平台色板（唯一色表；权威源=scene.color_map/dp.color 的颜色名，此处只做名字→RGB）
PLATFORM_COLORS = {
    "red": (0.90, 0.20, 0.20),
    "blue": (0.20, 0.45, 0.90),
    "yellow": (0.90, 0.85, 0.10),
}
DRONE_COLORS = [
    (0.90, 0.30, 0.30), (0.30, 0.75, 0.95), (0.30, 0.85, 0.45),
    (0.95, 0.65, 0.15), (0.75, 0.45, 0.95), (0.95, 0.90, 0.30),
]


def mat(r, g, b, a=1.0):
    return ('<material><ambient>%.2f %.2f %.2f %.2f</ambient>'
            '<diffuse>%.2f %.2f %.2f %.2f</diffuse><specular>0.2 0.2 0.2 1</specular></material>'
            % (r, g, b, a, r, g, b, a))


def col_geo(geom):
    return '<collision name="col"><geometry>%s</geometry></collision>' % geom


def vis_geo(geom, c):
    return '<visual name="vis"><geometry>%s</geometry>%s</visual>' % (geom, c)


def vis_pose(name, geom, color, x, y, z, yaw=0.0):
    """带位姿的视觉（几何+材质），用于单 link 多视觉模型。"""
    return ('<visual name="%s"><pose>%.3f %.3f %.3f 0 0 %.3f</pose>'
            '<geometry>%s</geometry>%s</visual>'
            % (name, x, y, z, yaw, geom, mat(*color)))


def tree_model(oid, ob):
    """圆柱树干（碰撞+视觉）+ 蓬松球形树冠（仅视觉）；几何取 ob 的 trunk/crown 字段。"""
    cx = ob.cx
    cy = ob.cy
    r = ob.trunk_r
    h = ob.trunk_h
    cyl = '<cylinder><radius>%.2f</radius><length>%.2f</length></cylinder>' % (r, h)
    # 树冠：多个绿色球组成,更茂密可见（主冠半径=ob.crown_r，其余小球保留）
    crown_links = []
    crown_offsets = [
        (0.0, 0.0, h * 0.55, ob.crown_r),
        (0.55, 0.0, h * 0.75, 0.80),
        (-0.55, 0.15, h * 0.70, 0.80),
        (0.0, 0.55, h * 0.68, 0.78),
        (0.0, -0.50, h * 0.65, 0.75),
        (0.40, 0.35, h * 0.85, 0.65),
    ]
    green_shades = [
        (0.14, 0.48, 0.22),
        (0.18, 0.55, 0.26),
        (0.12, 0.44, 0.20),
    ]
    for i, (ox, oy, oz, cr) in enumerate(crown_offsets):
        color = green_shades[i % len(green_shades)]
        sph = '<sphere><radius>%.2f</radius></sphere>' % cr
        crown_links.append("""  <link name="crown_%d">
    <pose>%.2f %.2f %.2f 0 0 0</pose>
    %s
  </link>""" % (i, ox, oy, oz, vis_geo(sph, mat(*color))))
    return """<model name="tree_%d">
  <static>true</static>
  <pose>%.2f %.2f %.2f 0 0 0</pose>
  <link name="trunk">
    %s
    %s
  </link>
%s
</model>""" % (oid, cx, cy, h / 2.0,
              col_geo(cyl), vis_geo(cyl, mat(0.45, 0.32, 0.18)),
              "\n".join(crown_links))


def bush_model(oid, lo, hi):
    cx = (lo[0] + hi[0]) / 2.0
    cy = (lo[1] + hi[1]) / 2.0
    w = hi[0] - lo[0]
    d = hi[1] - lo[1]
    h = hi[2] - lo[2]
    box = '<box><size>%.2f %.2f %.2f</size></box>' % (w, d, h)
    return """<model name="bush_%d">
  <static>true</static>
  <pose>%.2f %.2f %.2f 0 0 0</pose>
  <link name="box">
    %s
    %s
  </link>
</model>""" % (oid, cx, cy, h / 2.0,
              col_geo(box), vis_geo(box, mat(0.20, 0.55, 0.28)))


def pad_model(i, x, y):
    cyl = '<cylinder><radius>0.3</radius><length>0.05</length></cylinder>'
    return """<model name="pad_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.025 0 0 0</pose>
  <link name="pad">
    %s
    %s
  </link>
</model>""" % (i, x, y, col_geo(cyl), vis_geo(cyl, mat(0.25, 0.55, 0.35)))


def drop_model(dp):
    """投送平台（规则口径：直径约 1.2m 的有色标识平台）：r=0.6 薄盘 h=0.05，
    色=dp.color（color_map 权威）；外缘白色描边环作 1.2m 口径标识。

    平台整体即碰撞体（薄盘 h=0.05），任何巡航/悬停高度层均不构成障碍。
    """
    r, g, b = PLATFORM_COLORS.get(dp.color, (0.90, 0.90, 0.90))
    disc = '<cylinder><radius>0.60</radius><length>0.05</length></cylinder>'
    ring = '<cylinder><radius>0.68</radius><length>0.02</length></cylinder>'
    return """<model name="drop_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.0 0 0 0</pose>
  <link name="platform">
    %s
    <visual name="disc"><pose>0 0 0.025 0 0 0</pose><geometry>%s</geometry>%s</visual>
    <visual name="rim"><pose>0 0 0.01 0 0 0</pose><geometry>%s</geometry>%s</visual>
  </link>
</model>""" % (dp.id, dp.xyz[0], dp.xyz[1],
              col_geo(disc), disc, mat(r, g, b), ring, mat(0.95, 0.95, 0.95))


def drone_model(i, x, y):
    r, g, b = DRONE_COLORS[i % len(DRONE_COLORS)]
    # 碰撞保持紧凑盒（0.5×0.5×0.2），与 python 后端 drone_radius/避障几何一致，物理行为不变；
    # 视觉升级为四旋翼：机身 + 顶部盖板 + 4 机臂 + 4 电机 + 4 螺旋桨（均仅视觉，无碰撞）。
    col_box = '<box><size>0.5 0.5 0.2</size></box>'

    vis = []
    # 中心机架（机身，用各机颜色）
    vis.append(vis_pose("chassis", '<box><size>0.16 0.16 0.05</size></box>',
                        (r, g, b), 0, 0, 0))
    # 顶部盖板（飞控/电池仓，深色）
    vis.append(vis_pose("top", '<box><size>0.09 0.09 0.015</size></box>',
                        (0.16, 0.16, 0.18), 0, 0, 0.033))
    # 机头航向标记（+X 方向小红点，便于观察朝向）
    vis.append(vis_pose("nose", '<box><size>0.03 0.03 0.035</size></box>',
                        (0.95, 0.25, 0.20), 0.085, 0, 0.01))
    # 十字型四旋翼：4 机臂 + 电机 + 螺旋桨
    motor = '<cylinder><radius>0.032</radius><length>0.03</length></cylinder>'
    prop = '<cylinder><radius>0.10</radius><length>0.006</length></cylinder>'
    for k, (dx, dy) in enumerate([(1, 0), (-1, 0), (0, 1), (0, -1)]):
        arm = ('<box><size>0.24 0.024 0.016</size></box>' if dx != 0
               else '<box><size>0.024 0.24 0.016</size></box>')
        ax, ay = dx * 0.12, dy * 0.12   # 机臂中心
        tx, ty = dx * 0.24, dy * 0.24   # 电机/桨中心
        vis.append(vis_pose("arm%d" % k, arm, (0.24, 0.24, 0.26), ax, ay, 0.01))
        vis.append(vis_pose("motor%d" % k, motor, (0.33, 0.33, 0.35), tx, ty, 0.032))
        vis.append(vis_pose("prop%d" % k, prop, (0.82, 0.82, 0.84, 0.55), tx, ty, 0.062))

    return """<model name="drone_%d">
  <pose>%.2f %.2f 1.0 0 0 0</pose>
  <link name="base_link">
    <inertial>
      <mass>2.0</mass>
      <inertia><ixx>0.04833</ixx><iyy>0.04833</iyy><izz>0.08333</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    %s
%s
  </link>
  <plugin name="drone_vel_%d" filename="libdrone_vel_plugin.so">
    <drone_id>%d</drone_id>
    <kp>3.0</kp>
    <g>9.81</g>
  </plugin>
</model>""" % (i, x, y, col_geo(col_box), "\n".join(vis), i, i)


# ---------------------------------------------------------------------------
# 新场景元素
# ---------------------------------------------------------------------------
def thin_box_model(name, x, y, z, sx, sy, sz, color, alpha=1.0, collision=True):
    """通用扁平盒模型，用于场地、功能区、穿越区、路缘、围栏等。"""
    box = '<box><size>%.2f %.2f %.2f</size></box>' % (sx, sy, sz)
    m = mat(color[0], color[1], color[2], alpha)
    col = col_geo(box) if collision else ""
    return """<model name="%s">
  <static>true</static>
  <pose>%.2f %.2f %.2f 0 0 0</pose>
  <link name="link">
    %s
    %s
  </link>
</model>""" % (name, x, y, z + sz / 2.0, col, vis_geo(box, m))


def ground_plane_model(size_x, size_y, color=(0.42, 0.56, 0.32)):
    # 用薄盒代替 plane,材质显示更稳定,且避免与 court 地面 z-fighting
    return thin_box_model("ground_plane", 0, 0, -0.02, size_x, size_y, 0.02,
                          color, collision=True)


def court_models():
    """主场地：沥青面 + 白色边线 + 路缘石。"""
    models = []
    # 34 × 26 沥青面（厚 0.02）
    models.append(thin_box_model("court_asphalt", 0, 0, 0.0, 34, 26, 0.02,
                                 (0.34, 0.35, 0.36)))
    # 四条白色边线（内缩 0.5m，高 0.02，宽 0.22）
    e_x = 34 / 2 - 0.5
    e_y = 26 / 2 - 0.5
    line_h = 0.03
    line_w = 0.22
    models.append(thin_box_model("line_n", 0,  e_y, line_h / 2, 34 - 1.0, line_w, line_h,
                                 (0.95, 0.95, 0.95)))
    models.append(thin_box_model("line_s", 0, -e_y, line_h / 2, 34 - 1.0, line_w, line_h,
                                 (0.95, 0.95, 0.95)))
    models.append(thin_box_model("line_w", -e_x, 0, line_h / 2, line_w, 26 - 1.0, line_h,
                                 (0.95, 0.95, 0.95)))
    models.append(thin_box_model("line_e",  e_x, 0, line_h / 2, line_w, 26 - 1.0, line_h,
                                 (0.95, 0.95, 0.95)))
    # 路缘石（高 0.32，厚 0.5，沿场地外圈）
    curb_h = 0.32
    curb_t = 0.5
    cx = 34 / 2 + curb_t / 2
    cy = 26 / 2 + curb_t / 2
    models.append(thin_box_model("curb_n", 0,  cy, 0.0, 34 + curb_t, curb_t, curb_h,
                                 (0.73, 0.71, 0.67)))
    models.append(thin_box_model("curb_s", 0, -cy, 0.0, 34 + curb_t, curb_t, curb_h,
                                 (0.73, 0.71, 0.67)))
    models.append(thin_box_model("curb_w", -cx, 0, 0.0, curb_t, 26 + curb_t, curb_h,
                                 (0.73, 0.71, 0.67)))
    models.append(thin_box_model("curb_e",  cx, 0, 0.0, curb_t, 26 + curb_t, curb_h,
                                 (0.73, 0.71, 0.67)))
    return models


def takeoff_pad_model():
    """左侧起降区：水泥垫 + 黄色标识带。"""
    models = []
    models.append(thin_box_model("takeoff_pad", -23, 9, 0.02, 8, 6.5, 0.02,
                                 (0.73, 0.71, 0.67)))
    models.append(thin_box_model("takeoff_band", -23, 9 + 6.5 / 2 - 0.5, 0.05, 8, 1.0, 0.02,
                                 (0.94, 0.75, 0.25)))
    return models


def crossing_zone_models():
    """穿越区：橙色透明面 + 棕色边框。"""
    models = []
    # 中心 (1,2), 12 × 8
    models.append(thin_box_model("crossing_zone", 1, 2, 0.04, 12, 8, 0.02,
                                 (0.85, 0.48, 0.31), alpha=0.5))
    # 边框
    border_h = 0.04
    border_w = 0.18
    hx, hy = 6, 4
    models.append(thin_box_model("cz_border_n", 1, 2 + hy - border_w / 2, border_h / 2,
                                 12, border_w, border_h, (0.54, 0.29, 0.16)))
    models.append(thin_box_model("cz_border_s", 1, 2 - hy + border_w / 2, border_h / 2,
                                 12, border_w, border_h, (0.54, 0.29, 0.16)))
    models.append(thin_box_model("cz_border_w", 1 - hx + border_w / 2, 2, border_h / 2,
                                 border_w, 8, border_h, (0.54, 0.29, 0.16)))
    models.append(thin_box_model("cz_border_e", 1 + hx - border_w / 2, 2, border_h / 2,
                                 border_w, 8, border_h, (0.54, 0.29, 0.16)))
    return models


def fence_models():
    """场地外围围栏（与 scene_topology.yaml static_obstacles fence AABB 对齐）。

    修复历史失和：fence_e 原 18.6 vs yaml 22.50~22.80（中心 22.65）、
    N/S 长度 37.2 vs yaml 41.4（x -18.60~22.80，中心 2.10）。
    """
    models = []
    # x 方向两侧（y 跨 -15.5~15.5，长 31）
    models.append(thin_box_model("fence_w", -18.60, 0, 0.0, 0.3, 31.0, 1.3,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_e",  22.65, 0, 0.0, 0.3, 31.0, 1.3,
                                 (0.31, 0.42, 0.27)))
    # y 方向两侧（x 跨 -18.6~22.8，长 41.4，中心 2.1）
    models.append(thin_box_model("fence_n", 2.1,  15.5, 0.0, 41.4, 0.3, 1.3,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_s", 2.1, -15.5, 0.0, 41.4, 0.3, 1.3,
                                 (0.31, 0.42, 0.27)))
    return models


# ---------------------------------------------------------------------------
# 主构建流程
# ---------------------------------------------------------------------------
def build():
    scene = Scene()
    v = scene.venue
    out = []
    out.append('<?xml version="1.0"?>')
    out.append('<sdf version="1.7">')
    out.append('<world name="zx2026_forest">')
    out.append("""  <physics type="ode">
    <max_step_size>0.002</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>500</real_time_update_rate>
  </physics>
  <gravity>0 0 -9.81</gravity>
  <include><uri>model://sun</uri></include>""")

    # 地面（按 venue size）
    out.append(ground_plane_model(v["size"][0], v["size"][1]))

    # 主场地元素
    out.extend(court_models())
    out.extend(takeoff_pad_model())
    out.extend(crossing_zone_models())
    out.extend(fence_models())

    # 树/灌木
    n_tree = n_bush = 0
    for ob in scene.obstacles:
        if ob.kind == "tree":
            out.append(tree_model(ob.id, ob))
            n_tree += 1
        elif ob.kind == "bush":
            out.append(bush_model(ob.id, ob.lo, ob.hi))
            n_bush += 1

    # 起降点、投放点、无人机
    for i, (px, py) in enumerate(scene.get_pads()):
        out.append(pad_model(i, px, py))
    for dp in scene.drop_points:
        out.append(drop_model(dp))
    for i, (px, py) in enumerate(scene.get_pads()):
        out.append(drone_model(i, px, py))

    out.append("</world>")
    out.append("</sdf>")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("\n".join(out))
    print("wrote %s  (trees=%d bushes=%d pads=%d drops=%d drones=%d)" %
          (OUT, n_tree, n_bush, len(scene.get_pads()),
           len(scene.drop_points), len(scene.get_pads())))


if __name__ == "__main__":
    build()
