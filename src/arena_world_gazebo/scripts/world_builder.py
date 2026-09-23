#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world_gazebo::world_builder — 从 scene_topology 生成 Gazebo 11 世界。

复用 zx2026_common.scene 的确定性场景（同 seed=42）：树/灌木取 Scene 内 AABB 的
外接圆柱/盒，保证与 python 后端的碰撞模型和 A* 规划几何一致 —— 换物理引擎不换场景。
输出: arena_world_gazebo/worlds/forest_world.world
用法: python3 world_builder.py
"""
import math
import os
import sys

_WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, os.path.join(_WS, "src/zx2026_common/scripts"))
sys.path.insert(0, os.path.join(_WS, "src/arena_world_gazebo/scripts"))

from zx2026_common.scene import Scene

OUT = os.path.join(os.path.dirname(__file__), "..", "worlds", "forest_world.world")

# 杨树林 mesh（table 场景视觉；STL 由 tools/import_poplar_sdf.py 配套拷入）
MESH_URI = "file:///home/ubuntu/zx2026_arena_ws/src/arena_world_gazebo/meshes/poplar"

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


def branch_links(ob):
    """树枝碰撞体（gz 树枝物理缺环补齐）：消费 Scene ob.branches（世界系
    线段+半径），逐位生成 SDF cylinder 碰撞+视觉。

    设计律与 python 后端一致：枝参与硬碰撞（物理+事件），球冠仍仅视觉。
    branches.enabled=false 时 Scene 不生枝（ob.branches=None）→ 本函数零输出，
    A/B 单旗天然成立（无需额外开关）。

    几何注意：Gazebo Classic 11（sdformat 1.7）无 capsule 几何（sdformat
    1.8+ 才有），用无帽 cylinder 近似——恰与 monitor 的 point_segment_dist
    线段判定同几何，端帽差 ≤ 枝径 2.2cm 可忽略。SDF cylinder 沿自身 Z 轴；
    姿态取 roll=0 / pitch=acos(dz) / yaw=atan2(dy,dx)（同胶囊朝向约定）。
    """
    links = []
    brs = getattr(ob, "branches", None) or []
    # model pose=(cx,cy,h/2)：世界坐标→model 系
    ox, oy, oz = ob.cx, ob.cy, ob.trunk_h / 2.0
    for k, (sx, sy, sz, ex, ey, ez, br) in enumerate(brs):
        dx, dy, dz = ex - sx, ey - sy, ez - sz
        ln = math.sqrt(dx * dx + dy * dy + dz * dz)
        if ln < 1e-9:
            continue
        ux, uy, uz = dx / ln, dy / ln, dz / ln
        pitch = math.acos(max(-1.0, min(1.0, uz)))
        yaw = math.atan2(uy, ux)
        mx, my, mz = (sx + ex) / 2.0 - ox, (sy + ey) / 2.0 - oy, (sz + ez) / 2.0 - oz
        cyl = '<cylinder><radius>%.3f</radius><length>%.3f</length></cylinder>' % (br, ln)
        links.append("""  <link name="branch_%d">
    <pose>%.3f %.3f %.3f 0 %.4f %.4f</pose>
    %s
    %s
  </link>""" % (k, mx, my, mz, pitch, yaw,
               col_geo(cyl), vis_geo(cyl, mat(0.45, 0.32, 0.18))))
    return links


def tree_model_poplar(oid, ob):
    """杨树（table 场景）：wood/leaves STL 视觉（与外部 SDF 同构）+ 树干/
    冠包络两圆柱碰撞（=SDF c_trunk/c_crown 契约，gz 物理与 py Scene 一致）。

    冠包络圆柱参与碰撞是杨树场景新增语义：冠下缘 ≥2.81m 在巡航带 (≤2.6)
    之上，正常飞行永不接触；仅当异常爬升进冠区时由 gz 物理硬拦。
    """
    vi = ob.mesh_variant
    s = ob.mesh_scale or 1.0
    mesh = lambda part: ('<mesh><uri>%s/tree_%d_%s.stl</uri>'
                         '<scale>%.3f %.3f %.3f</scale></mesh>'
                         % (MESH_URI, vi, part, s, s, s))
    trunk_col = ('<collision name="c_trunk"><pose>0 0 %.3f 0 0 0</pose>'
                 '<geometry><cylinder><radius>%.3f</radius><length>%.3f'
                 '</length></cylinder></geometry></collision>'
                 % (ob.trunk_h / 2.0, ob.trunk_r, ob.trunk_h))
    crown_h = ob.crown_h if ob.crown_h else 0.64 * ob.trunk_h / 0.78
    crown_col = ('<collision name="c_crown"><pose>0 0 %.3f 0 0 0</pose>'
                 '<geometry><cylinder><radius>%.3f</radius><length>%.3f'
                 '</length></cylinder></geometry></collision>'
                 % (ob.crown_z, ob.crown_r, crown_h))
    wood_vis = ('<visual name="v_wood"><geometry>%s</geometry>%s</visual>'
                % (mesh("wood"), mat(0.45, 0.31, 0.19)))
    leaf_vis = ('<visual name="v_leaf"><geometry>%s</geometry>%s</visual>'
                % (mesh("leaves"), mat(0.22, 0.47, 0.16)))
    return """<model name="tree_%d">
  <static>true</static>
  <pose>%.3f %.3f 0 0 0 %.3f</pose>
  <link name="tree">
    %s
    %s
    %s
    %s
  </link>
</model>""" % (oid, ob.cx, ob.cy, ob.mesh_yaw or 0.0,
               wood_vis, leaf_vis, trunk_col, crown_col)


def tree_model(oid, ob):
    """圆柱树干（碰撞+视觉）+ 蓬松球形树冠（仅视觉）+ 树枝圆柱（碰撞+视觉，
    若 Scene 生枝）；几何取 ob 的 trunk/crown/branches 字段。"""
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
%s
</model>""" % (oid, cx, cy, h / 2.0,
              col_geo(cyl), vis_geo(cyl, mat(0.45, 0.32, 0.18)),
              "\n".join(crown_links), "\n".join(branch_links(ob)))


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
    """起降清理带：土面（x∈[-30,-24] 清理带，取代旧沥青球场）。"""
    models = []
    models.append(thin_box_model("clearing_dirt", -27, 0, 0.0, 6.2, 60.4, 0.02,
                                 (0.60, 0.52, 0.36)))
    return models


def takeoff_pad_model():
    """起降区：水泥垫 + 黄色标识带（pads x=-27.75~-24.75, y=±3.75）。"""
    models = []
    models.append(thin_box_model("takeoff_pad", -27, 0, 0.02, 5.6, 11.0, 0.02,
                                 (0.73, 0.71, 0.67)))
    models.append(thin_box_model("takeoff_band", -27, 0, 0.05, 5.6, 1.0, 0.02,
                                 (0.94, 0.75, 0.25)))
    return models


def crossing_zone_models():
    """穿越区：橙色透明面 + 棕色边框（林带 52×56，中心 (2,0)）。"""
    models = []
    models.append(thin_box_model("crossing_zone", 2, 0, 0.04, 52, 56, 0.02,
                                 (0.85, 0.48, 0.31), alpha=0.5))
    return models


def fence_models():
    """场地外围围栏（与 scene_topology.yaml static_obstacles fence AABB 对齐）：
    ±30 墙线、高 2.80（冠下体制真实侧墙；2.80 > 点云带上限 2.60）。"""
    models = []
    models.append(thin_box_model("fence_w", -30.15, 0, 0.0, 0.3, 60.9, 2.8,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_e",  30.15, 0, 0.0, 0.3, 60.9, 2.8,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_n", 0,  30.15, 0.0, 60.9, 0.3, 2.8,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_s", 0, -30.15, 0.0, 60.9, 0.3, 2.8,
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

    # 树/灌木（table 场景=杨树 mesh 视觉；旧场景=圆柱树）
    n_tree = n_bush = 0
    for ob in scene.obstacles:
        if ob.kind == "tree":
            if ob.mesh_variant is not None:
                out.append(tree_model_poplar(ob.id, ob))
            else:
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
