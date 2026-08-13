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

TYPE_COLORS = {
    "TYPE_A": (0.90, 0.20, 0.20),
    "TYPE_B": (0.95, 0.60, 0.10),
    "TYPE_C": (0.90, 0.85, 0.10),
    "TYPE_D": (0.20, 0.80, 0.30),
    "TYPE_E": (0.20, 0.45, 0.90),
}
DRONE_COLORS = [
    (0.90, 0.30, 0.30), (0.30, 0.75, 0.95), (0.30, 0.85, 0.45),
    (0.95, 0.65, 0.15), (0.75, 0.45, 0.95), (0.95, 0.90, 0.30),
]

# 右侧标识柱颜色（与 subject3-scene.js 对应）
MARKER_COLORS = [
    (0.61, 0.35, 0.71),  # purple
    (0.49, 0.85, 0.34),  # green
    (0.12, 0.56, 0.24),  # dark green
    (0.95, 0.76, 0.19),  # yellow
    (0.23, 0.51, 0.96),  # blue
    (0.35, 0.78, 0.85),  # cyan
]


def mat(r, g, b, a=1.0):
    return ('<material><ambient>%.2f %.2f %.2f %.2f</ambient>'
            '<diffuse>%.2f %.2f %.2f %.2f</diffuse><specular>0.2 0.2 0.2 1</specular></material>'
            % (r, g, b, a, r, g, b, a))


def col_geo(geom):
    return '<collision name="col"><geometry>%s</geometry></collision>' % geom


def vis_geo(geom, c):
    return '<visual name="vis"><geometry>%s</geometry>%s</visual>' % (geom, c)


def tree_model(oid, lo, hi):
    """AABB -> 外接圆柱（trunk 碰撞+视觉），顶加蓬松球形树冠（仅视觉）。"""
    cx = (lo[0] + hi[0]) / 2.0
    cy = (lo[1] + hi[1]) / 2.0
    r = (hi[0] - lo[0]) / 2.0
    h = hi[2] - lo[2]
    cyl = '<cylinder><radius>%.2f</radius><length>%.2f</length></cylinder>' % (r, h)
    # 树冠：多个绿色球组成,更茂密可见
    crown_links = []
    crown_offsets = [
        (0.0, 0.0, h * 0.55, 1.20),
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
    r, g, b = TYPE_COLORS.get(dp.type_id, (0.9, 0.9, 0.9))
    disc = '<cylinder><radius>0.45</radius><length>0.1</length></cylinder>'
    tag = '<box><size>0.25 0.25 0.9</size></box>'
    return """<model name="drop_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.05 0 0 0</pose>
  <link name="disc">
    %s
    %s
  </link>
  <link name="tag">
    <pose>0 0 0.55 0 0 0</pose>
    %s
  </link>
</model>""" % (dp.id, dp.xyz[0], dp.xyz[1],
              col_geo(disc), vis_geo(disc, mat(r, g, b)),
              vis_geo(tag, mat(1.0, 1.0, 1.0)))


def drone_model(i, x, y):
    r, g, b = DRONE_COLORS[i % len(DRONE_COLORS)]
    box = '<box><size>0.5 0.5 0.2</size></box>'
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
</model>""" % (i, x, y, col_geo(box), vis_geo(box, mat(r, g, b)), i, i)


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


def marker_post_model(i, x, y, color):
    """彩色标识柱：底座 + 立柱 + 彩色环（环用薄圆柱近似）。"""
    r, g, b = color
    models = []
    # 底座圆柱
    base = '<cylinder><radius>0.9</radius><length>0.18</length></cylinder>'
    models.append("""<model name="marker_base_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.09 0 0 0</pose>
  <link name="link">
    %s
    %s
  </link>
</model>""" % (i, x, y, col_geo(base), vis_geo(base, mat(0.60, 0.60, 0.58))))
    # 立柱
    pole = '<cylinder><radius>0.09</radius><length>0.9</length></cylinder>'
    models.append("""<model name="marker_pole_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.6 0 0 0</pose>
  <link name="link">
    %s
    %s
  </link>
</model>""" % (i, x, y, col_geo(pole), vis_geo(pole, mat(0.47, 0.47, 0.47))))
    # 彩色环
    ring = '<cylinder><radius>1.05</radius><length>0.08</length></cylinder>'
    models.append("""<model name="marker_ring_%d">
  <static>true</static>
  <pose>%.2f %.2f 1.15 0 0 0</pose>
  <link name="link">
    %s
    %s
  </link>
</model>""" % (i, x, y, col_geo(ring), vis_geo(ring, mat(r, g, b))))
    return models


def fence_models():
    """场地外围围栏。"""
    models = []
    # x 方向两侧
    models.append(thin_box_model("fence_w", -18.6, 0, 0.0, 0.3, 30, 1.3,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_e",  18.6, 0, 0.0, 0.3, 30, 1.3,
                                 (0.31, 0.42, 0.27)))
    # z 方向两侧
    models.append(thin_box_model("fence_n", 0,  15.5, 0.0, 37.2, 0.3, 1.3,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_s", 0, -15.5, 0.0, 37.2, 0.3, 1.3,
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

    # 右侧标识柱
    marker_zs = [-10, -6, -2, 2, 6, 10]
    for i, z in enumerate(marker_zs):
        out.extend(marker_post_model(i, 21, z, MARKER_COLORS[i]))

    # 树/灌木
    n_tree = n_bush = 0
    for ob in scene.obstacles:
        if ob.kind == "tree":
            out.append(tree_model(ob.id, ob.lo, ob.hi))
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
