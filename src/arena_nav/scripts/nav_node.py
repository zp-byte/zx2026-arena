#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_nav::nav_node — 单机规划与轨迹跟踪（每机一个）。

输入: /drone_<id>/odom, /drone_<id>/cloud (PointCloud2), /drone_<id>/planning/goal
输出: /drone_<id>/vel_cmd (Twist)

两种模式（sim_settings.yaml `closed_loop.enabled`）:
  - false（god-mode，原版）: goal → 2.5D 全局 A*（scene 真值占用 + inflation）
                             → 路径跟随 P 位置环 + 动量感知静态避障。
  - true（闭环）: 订阅 lidar 点云（含顶盲区）→ 累积成全局持久占用栅格（SLAM）
                  → 全局 A* 重规划 → P 位置环。定位带随机游走漂移，导航只用
                  est_pose（置信位姿）与点云建图，不再读 scene.obstacles/collides_xy。
"""
import heapq
import math
import random
import struct
from collections import deque

import rospy
import numpy as np
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from sensor_msgs.msg import PointCloud2

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from zx2026_common.msg import TaskUpdate
from arena_nav.astar import AStar


def vo_deflect(px, py, pz, vx, vy, drone_id, nj, nv,
               engage_r, safe_r, t_pred, z_band):
    """单邻居 VO-lite 判据与切向偏转方向（纯函数，无 ROS 依赖）。

    W6 机间速度预测避撞的数学核心（tools/vo_selftest.py 离线复用）。
    标准约定：p=邻居−自机（指向邻居），rv=邻居速度−自机速度；
    闭合判据 p·rv<0，tcpa=−(p·rv)/|rv|²，CPA 相对位置 c=p+rv·tcpa，
    dcpa=|c|。冲突 = 0<tcpa<t_pred 且 dcpa<safe_r 且水平距<engage_r
    且 |dz|≤z_band（与 swarm 同门控：爬降段不干预）。

    偏转方向：沿 rv 垂直单位向量 t̂，取推离 CPA 线的一侧——自机沿
    s·t̂ 加速 → CPA 相对位 c−s·t̂·tcpa，|·| 增大取 s=−sign(c·t̂)。
    镜像几何（两机互看）c 与 t̂ 同时反号 → c·t̂ 两机同号 → 同侧取号
    → 世界系反向偏转（各向己右，相对分离率 2×push）；|c·t̂|≈0
    （完全对头/纯追尾，sign 不稳定）固定 s=+1，两机同取 +t̂ 仍互补。
    drone_id 仅进遥测，不进数学。
    """
    dz = nj[2] - pz
    if abs(dz) > z_band:
        return None
    dx, dy = nj[0] - px, nj[1] - py
    d = math.hypot(dx, dy)
    if d < 1e-6 or d > engage_r:
        return None
    if nv is None:
        return None
    rvx, rvy = nv[0] - vx, nv[1] - vy
    rv2 = rvx * rvx + rvy * rvy
    if rv2 < 1e-6:
        return None
    pv = dx * rvx + dy * rvy
    if pv >= 0.0:
        return None                     # 正在远离，无冲突
    tcpa = -pv / rv2
    if tcpa > t_pred:
        return None
    cx, cy = dx + rvx * tcpa, dy + rvy * tcpa    # CPA 相对位置
    dcpa = math.hypot(cx, cy)
    if dcpa >= safe_r:
        return None
    inv = 1.0 / math.sqrt(rv2)
    t1x, t1y = -rvy * inv, rvx * inv            # rv 左转 90° 单位向量
    s1 = cx * t1x + cy * t1y
    side = -1.0 if s1 > 0.0 else 1.0            # 推离 CPA 线（≈0 → +1）
    w = (safe_r - dcpa) / safe_r
    urg = 1.0 - min(tcpa / t_pred, 1.0)
    score = w * (0.4 + 0.6 * urg)
    return (side * t1x, side * t1y, score, tcpa, dcpa, d)


class NavNode:
    def __init__(self):
        rospy.init_node("nav_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        rules = cfg.load("competition_rules.yaml")
        self.max_vel = float(rules["motion"].get("default_max_vel", 2.0))
        self.max_acc = float(rules["motion"].get("default_max_acc", 4.0))
        self.cruise_z = float(rules["heights"].get("cruise_z", 2.5))
        # Scene 仅用于取物理常量（drone_radius/venue/drone_count），不作占用感知
        self.scene = Scene()
        settings = cfg.load("sim_settings.yaml")
        self.res = float(settings.get("nav", {}).get("resolution", 0.5))
        margin = float(settings.get("nav", {}).get("inflation", 0.4))
        self.inflation = self.scene.drone_radius + margin

        # ---- 闭环配置 ----
        cl = settings.get("closed_loop", {})
        self.closed_loop = bool(cl.get("enabled", False))
        self.drift_rate = float(cl.get("drift_rate", 0.02))
        self.drift_max = float(cl.get("drift_max", 0.3))
        # ---- 到位停拉圈按高度特化（P0-1 释放 offset 骑线，run13/14 法证） ----
        # 停拉圈 0.4 内均匀随机停点 + 漂移 = 释放 offset 0.26-0.58 分布的真身；
        # goal z 低于 low_z_max（drop 悬停 0.8 / touchdown 0.12 等精确任务段）
        # 收紧到 low_z 圈：停点贴近 goal，真值 offset 收敛到 ~|漂移| 下界。
        # 漂移把估计推出小圈时 nav 重启小修正（良性抖动趋真），不会躺平。
        self._arrive_tol = float(cl.get("arrive_tol", 0.4))
        self._arrive_tol_low_z = float(cl.get("arrive_tol_low_z", 0.15))
        self._arrive_tol_low_z_max = float(cl.get("arrive_tol_low_z_max", 1.5))
        # 伺服收敛信号圈（est 系 xy）：goal_reached 广播用。水平控制律驱动
        # est 系，平衡点 est=goal——此圈判"控制环已收敛"（cmd≈0 零爬行），
        # 与真值停拉圈（arrive_tol，odom 系）分工：后者只拦漂移小的幸运情形
        self._arrive_tol_est = float(cl.get("arrive_tol_est", 0.10))
        # ---- 漂移感知裕度（2026-08-31 碰撞法证：低速贴树挤入 = est 系控制误差） ----
        # 点云是真值系（lidar 真值位姿 raycast），clearance 测量无偏；但控制几何
        # 在 est 系（est=truth+drift），漂移背向最近障碍时 est 距离比真值大
        # |drift·u|——控制器高估裕度，真值裕度被侵蚀，平衡点压进接触。
        self._dam_enabled = bool(cl.get("drift_aware_margin", {}).get("enabled", False))
        # v2 深近区门控：地板衰减只在 eff<deep_r 内生效。0.7 = 基线地板 bind
        # 边界（max(0.35, clr/2) 中 clr/2<0.35 ⟺ clr<0.7）——dam 恰好只接管
        # "基线地板本来就要兜底"的贴脸带，带外逐位回基线。v1 全域衰减的
        # 任务级减速（P45 满窗 FAIL）由此结构性排除。
        self._dam_deep_r = float(cl.get("drift_aware_margin", {}).get("deep_r", 0.7))
        # ---- 近停区（2026-08-31 碰撞法证另一半机制） --------------------------------
        # vcap 0.35 地板意味着贴脸仍保底 0.53m/s，goal/云推力把平衡点压进接触。
        # 近停区：距 goal<zone_r 时地板线性衰减，deadband 处归零——允许真正刹停。
        # de 逃逸期间不衰减（逃逸方向已过点云走廊验证，地板是安全项不是风险项）。
        nsz = cl.get("near_stop_zone", {}) or {}
        self._nstop_enabled = bool(nsz.get("enabled", False))
        self._nstop_zone_r = float(nsz.get("zone_r", 1.0))
        self._nstop_deadband = float(nsz.get("deadband", 0.2))
        # scale_min：衰减钳位下限——归 0 会冻结末段收敛（run7 法证①）
        self._nstop_scale_min = float(nsz.get("scale_min", 0.25))
        # ---- W3 塌缩卫兵（返航冻结：前瞻航点=自身 → 指令≈0 伺服自锁） ----
        cg = cl.get("collapse_guard", {}) or {}
        self._cg_enabled = bool(cg.get("enabled", False))
        self._cg_min_cmd = float(cg.get("min_cmd", 0.3))
        self._cg_floor_speed = float(cg.get("floor_speed", 0.8))
        self._cg_min_target = float(cg.get("min_target", 0.75))
        self._cg_dgoal_min = float(cg.get("dgoal_min", 1.5))
        self._cg_clear_min = float(cg.get("clear_min", 1.0))
        # ---- W5 围栏顶楔死（2026-09-05 活体取证）：云避障符号反转吸引子 ----------
        # 原实现 u=(drone−point)=远离方向、v_app=cmd·u>0(正在远离)时触发、
        # cmd -= push*u 且 push>=v_app → 远离分量被抵消再反向 = 逃离惩罚；
        # 正对撞(v_app<0)反而不设防。W1 sep_obs_guard 同逻辑用 ex=x−px 指向
        # 障碍削向分量，是正确镜像。实测 d5 坐西围栏顶缘(z=1.40, 点环 z=1.30
        # d=0.14)时 8 个环点把 z 指令压到 −1.9 钉死机身。sign_fix=触发取反
        # (靠近才推)+推力沿远离方向：正对撞→减速推离，逃离/悬停零侵入。
        ca = cl.get("cloud_avoidance", {}) or {}
        self._ca_sign_fix = bool(ca.get("sign_fix", False))
        self.replan_hz = float(cl.get("replan_hz", 10.0))
        self.closed_max_vel = float(cl.get("max_vel", 1.5))
        self._rng = random.Random(int(settings.get("run_seed", 42)) + self.drone_id * 7919)

        # ---- P1 时间一致性（路径粘滞 + 方向翻转锁定，DeFoP anti-oscillation 移植） ----
        tc = cl.get("temporal_consistency", {})
        self.tc_enabled = bool(tc.get("enabled", False))
        self.tc_stick = float(tc.get("stick_s", 1.5))
        self.tc_window = float(tc.get("flip_window_s", 2.0))
        self.tc_flip_rate_max = float(tc.get("flip_rate_max", 0.6))
        self.tc_bias = float(tc.get("bias_gain", 0.35))
        self.tc_unlock = float(tc.get("unlock_clearance", 1.5))
        self.tc_max_lock = float(tc.get("max_lock_s", 6.0))
        self._flip_times = deque()   # 侧向符号翻转时刻（无论开关都采样，供 P0 指标）
        self._side_last = 0          # 最近一次有效采样的侧向符号（+1 左 / -1 右 / 0 无）
        self._lock_side = 0          # 锁定侧（0=未锁）
        self._lock_t0 = -1e9
        # tc-wind 联动告警：把"开风场压力测须同开 tc"的人肉约定变机制。
        # 历史 A/B：风臂下 tc 使碰撞 13.5→8.0（run_logs/matrix_* D 臂口径）。
        # 仅提醒不强制——压力测试想跑"裸风"基线时仍可显式关 tc。
        if (self.closed_loop and not self.tc_enabled
                and bool(settings.get("wind", {}).get("enabled", False))):
            rospy.logwarn("nav_node: drone %d wind.enabled=true 而 temporal_"
                          "consistency.enabled=false —— 风场扰动下建议同开 tc"
                          "（历史 A/B col 13.5→8.0）", self.drone_id)

        # ---- P3 死端基元逃逸（DeFoP motion primitives 移植，仅借死端脱困） ----
        # 规划连续失败（碰撞标记膨胀壳封死自格 → _plan_global None → 永久悬停）
        # 时进入：均匀方向栅格射线评分选逃逸方向，慢速驶出；规划恢复即退出。
        de = cl.get("deadend_escape", {})
        self._de_enabled = bool(de.get("enabled", False))
        self._de_confirm = float(de.get("confirm_s", 0.5))
        self._de_dwell = float(de.get("dwell_s", 1.2))
        self._de_speed = float(de.get("speed", 0.6))
        self._de_ray = float(de.get("ray_len", 3.0))
        self._de_ndirs = int(de.get("n_dirs", 16))
        self._de_pierce = int(de.get("pierce_cells", 8))
        self._de_free_need = int(de.get("free_need", 3))
        self._de_goal_bias = float(de.get("goal_bias", 1.5))
        self._de_active = False    # 逃逸进行中
        self._de_dir = None        # 当前逃逸方向（单位向量 xy）
        self._de_t0 = -1e9         # 当前方向开始时刻
        self._de_total_t0 = -1e9   # 本轮逃逸进入时刻
        self._de_fail_t0 = None    # 规划连续失败起点（None=未在失败中）
        # P3b 目标封锁直达：goal 格被膨胀栅格封死时 A* 退化为"最近可达格"，
        # 无人机停在可达格终点（lookahead 航点=自身 → 指令≈0）永久停车。
        # 实测投放点净空 ~4m、树干格膨胀 1m 后即封死（矩阵 2 FAIL + P3 验证
        # 2 FAIL 均此形态，零碰撞也发生）。停滞看门狗触发后直达逼近。
        self._gs_stall_s = float(de.get("goal_stall_s", 3.0))
        self._gs_speed = float(de.get("goal_speed", 0.8))
        self._gs_max_s = float(de.get("goal_max_s", 10.0))
        self._de_cloud_r = float(de.get("cloud_clear_r", 1.0))
        self._de_exit_hyst = float(de.get("exit_hyst_s", 2.5))
        self._gs_active = False    # 目标封锁直达进行中
        self._gs_stall_t0 = None
        self._gs_total_t0 = -1e9
        self._plan_fail_since = None   # 规划连续失败起点（末次有效路径保持期间计时）
        self._de_ok_t0 = None          # 退出迟滞：规划连续成功起点（None=未在成功中）
        self._stall_t0 = None          # 停滞看门狗起点（诊断日志用）
        self._stuck_t0 = None          # 平台邻域卡死诊断起点（run18 立案）

        # ---- 恢复链路仲裁（2026-09-02 drone5 级联法证后的修复对） ----
        # M1 救援互斥：de 逃逸进行中或规划连续失败期间 goal-seal 盲推禁发
        # （两救援方向相反 → 原地研磨 + flip 风暴，drone5 单机 220s+6 碰撞）。
        rm = cl.get("rescue_mutex", {})
        self._rm_enabled = bool(rm.get("enabled", False))
        # M2 bounce 走廊化：逃逸方向评分加连续走廊裕度项——同样可达时偏好
        # 宽走廊，抑制窄点两侧反向弹摆（drone3 ±0.99 同 y 双撞形态）。
        bc = cl.get("bounce_corridor", {})
        self._bc_enabled = bool(bc.get("enabled", False))
        self._bc_w = float(bc.get("corridor_w", 1.0))

        # ---- 全史碰撞聚类驱动的两项避障优化（2026-09-03，默认关） ----
        # 734 事件（python 706 + gazebo 28）聚类：Top4 树占 41%，主形态=
        # 分离硬推（障碍全盲）与云避障拔河，残余分量 + vcap 地板挤入接触。
        # M3 分离增量障碍投影：分离增量不许含指向贴身障碍的分量（保切向）。
        sg = cl.get("sep_obs_guard", {})
        self._soa_enabled = bool(sg.get("enabled", False))
        # guard 触发分桶计数（sep/swarm 各自计，零也打——零触发≠未需要，
        # 是 Gazebo"挤入零 guard 行"判读的证据链）。纯观测，不挂旗。
        self._soa_hits = {"sep": 0, "swarm": 0}
        # M4 热点树定向膨胀：对配置的热点树世界坐标格 +extra_cells 膨胀，
        # A* 提前绕开——只加热点格，不重蹈全局 inflation 0.9 的绕行超窗。
        hi = cl.get("hotspot_inflate", {})
        self._hi_enabled = bool(hi.get("enabled", False))
        self._hi_extra = int(hi.get("extra_cells", 1))
        self._hi_trees = [tuple(map(float, t)) for t in hi.get("trees", [])]

        # ---- P2 指令否决层（DeFoP M1 几何安全监督移植） ----
        vt = cl.get("veto_gate", {})
        self._vt_enabled = bool(vt.get("enabled", False))
        self._vt_t_react = float(vt.get("t_react", 0.2))
        self._vt_a_brake = float(vt.get("a_brake", 2.5))
        self._vt_margin = float(vt.get("margin", 0.3))
        self._vt_lat_margin = float(vt.get("lateral_margin", 0.1))
        self._vt_ray = float(vt.get("ray", 2.5))
        self._vt_scale_min = float(vt.get("scale_min", 0.35))
        self._vt_vetoes = 0       # 触发计数（验证日志证据用）

        # ---- 风前馈（顶风补偿，抵消共享风稳态漂移） ----
        # 风项 acc += (wind-vel)/tau 是速度耦合扰动，nav 指令减去共享风（mean+阵风）
        # 即"朝风指令"，悬停偏移从 w/1.2 降到 ~0.14w。仅 wind.enabled 时订阅
        # /zx2026/wind（world_node 只在 P4 飞行期发真实值，其余发 0 → 无幻影风）。
        # 每机湍流不共享、不补偿（残差抖动，真实系统亦如此）。
        wd2 = settings.get("wind", {})
        self.wind_ff = (float(wd2.get("feedforward", 1.0))
                        if bool(wd2.get("enabled", False)) else 0.0)
        self.wind_vec = np.zeros(3)
        if self.wind_ff > 0.0:
            rospy.Subscriber("/zx2026/wind", Vector3Stamped, self._on_wind)

        v = self.scene.venue
        if self.closed_loop:
            # 全局 SLAM 栅格：lidar 点云累积成持久占用图（覆盖全场，去上帝视角）
            v_size = v["size"]
            self.g_x0 = -v_size[0] / 2.0
            self.g_y0 = -v_size[1] / 2.0
            self.g_nx = int(v_size[0] / self.res)
            self.g_ny = int(v_size[1] / self.res)
            self._occ_cells = set()   # 已观测占用格 (ix,iy)，世界系真值测量，持久累积
            # ---- P4 滑窗点缓存（DeFoP M3 移植） ----
            # 占用格带时间戳：重观测刷新，超 ttl_s 未重观测的格衰减（重建时惰性
            # 剪枝）。治"只加不删"的两个病灶：噪声/漂移错位格永生（树周涂抹+膨胀
            # 加宽封锁带）；被重观测证伪的旧格不消失。无重观测区域保持记忆（远处
            # 真树仍被规划绕开）。碰撞标记（事件真值）独立持久，不随窗衰减。
            pc = cl.get("point_cache", {})
            self._pc_enabled = bool(pc.get("enabled", False))
            self._pc_ttl = float(pc.get("ttl_s", 45.0))
            self._occ_seen = {}       # 滑窗模式: (ix,iy) -> 最近观测 sim 时刻
            self._gocc = np.zeros((self.g_ny, self.g_nx), dtype=bool)
            self._gocc_infl = None
            self._gpath = None        # 全局 A* 格路径 [(ix,iy),...]
            self._gpath_t = -1e9
            self._grid_t = -1e9       # 栅格重建时刻（与路径选择时刻解耦，供 P1 粘滞）
            self._path_force = False  # 强制重选路径（新 goal / 碰撞 / 路径被切断）
            self.drift = np.zeros(3)
            self.est_pos = np.array([0.0, 0.0, 1.0])
            # 发布闭环累积占用栅格（rviz Map 可视化建图效果）
            self.pub_occ = rospy.Publisher(ns + "/occ_grid", OccupancyGrid, queue_size=2)
        else:
            self.astar = AStar(self.scene, resolution=self.res, inflation=self.inflation,
                               x_range=(-v["size"][0] / 2, v["size"][0] / 2),
                               y_range=(-v["size"][1] / 2, v["size"][1] / 2),
                               z_lo=0.5, z_hi=self.cruise_z + 1.0)

        self.odom = (0.0, 0.0, 1.0)
        self.cloud = []      # 最近一帧点云 [(x,y,z),...]（闭环避障 + 群集缩放用）
        self.path = []          # [(x,y,z),...]（仅 god-mode 使用）
        self.waypoint_idx = 0
        self.goal = None
        self.has_odom = False

        # 多机间距保持：订阅全部 odom，距离过近时叠加分离速度（防互撞）
        self.sep_radius = float(settings.get("nav", {}).get("separation_radius", 1.5))
        self.sep_gain = float(settings.get("nav", {}).get("separation_gain", 3.5))
        self.neighbors = {}

        # ---- Boids 群集（真正的集群算法：分离已有，对齐+聚合新增） ----
        # 与 formation.yaml（静态编队）互斥：二者都影响队形，同时开启会打架。
        sw = settings.get("swarm", {})
        self.swarm_enabled = bool(sw.get("enabled", False))
        self.alignment_gain = float(sw.get("alignment_gain", 0.4))
        self.cohesion_gain = float(sw.get("cohesion_gain", 0.2))
        self.perception_radius = float(sw.get("perception_radius", 8.0))
        self.swarm_z_band = float(sw.get("z_band", 0.5))
        self.swarm_min_z = float(sw.get("min_z", 2.0))
        self.obstacle_scale = bool(sw.get("obstacle_scale", True))
        self.neighbor_vel = {}     # {j: (vx,vy,vz)} 邻居速度（odom twist）
        self.v_odom = np.zeros(3)  # 本机速度（odom twist）

        # ---- W1 swarm 层治理（2026-09-03 贴树判官团定稿，matrix_w1b 后默认开） ----
        # obs_guard：聚合/对齐增量过 sep_obs_guard 同款投影（保切向）。代码
        # 事实：_apply_separation 的 pre-guard 只管分离增量，_apply_swarm 的
        # coh/ali 直加无任何障碍管辖——obstacle_scale 只缩模不改向，分离
        # guard 反把 swarm 增量当"受保护 pre 意图"对待。734 聚类主因的聚合
        # 半边由此补全（Gazebo 实弹"挤入零 guard 行"头号根因，控制/群集两
        # 视角独立实锤）。
        self._swg_enabled = bool(sw.get("obs_guard", {}).get("enabled", False))
        # rescue_quiet：救援态（de 逃逸/gs 盲推/规划失败期）静默 coh/ali 两个
        # 聚合项——聚合推 0.4-0.8 m/s 与逃逸基速 0.6 同量级，救援期间不改写
        # 逃逸矢量（链内二次蹭 ×2 / 0.65s 即刻再暴露 / 跟车级联的公共力源）；
        # 分离与 hard_r 硬推层在下游 _apply_separation 照常（机间安全底线
        # 不动）。与 rescue_mutex 同思想：mutex 仲裁 de/gs 两救援原语互抗，
        # 本项移除救援期间的第三方力源（群集）。退出迟滞 ramp_s 与
        # exit_hyst_s 同值同源（2.5）：群集重入不应早于救援退出确认。
        swq = sw.get("rescue_quiet", {}) or {}
        self._swq_enabled = bool(swq.get("enabled", False))
        self._swq_ramp = float(swq.get("ramp_s", 2.5))
        self._swq_until = -1e9     # 恢复爬坡重入截止时刻（de 退出时置 now+ramp）

        # ---- W6 VO-lite 机间速度预测避撞（2026-09-07 d2/d3 返程互撞立案） ----
        # 纯位置分离的反应距离不足之外层：TCPA/DCPA 预判汇聚冲突 + 相对速度
        # 垂直切向偏转（数学核心=模块级纯函数 vo_deflect，selftest 离线验证）。
        # 增量过 sep_obs_guard 投影（tag=vo）；救援态静默+ramp 与 coh/ali
        # 同款（不改写逃逸矢量，硬推底线不动）；swarm z_band/min_z 门控；
        # 邻居 age>stale_s 剔除。默认关；判词与翻线见 closed_loop.vo_avoid。
        vo = cl.get("vo_avoid", {}) or {}
        self._vo_enabled = bool(vo.get("enabled", False))
        self._vo_engage_r = float(vo.get("engage_r", 3.5))
        self._vo_safe_r = float(vo.get("safe_r", 1.1))
        self._vo_t_pred = float(vo.get("t_pred", 3.0))
        self._vo_gain = float(vo.get("gain", 1.2))
        self._vo_max_push = float(vo.get("max_push", 1.0))
        self._vo_stale_s = float(vo.get("stale_s", 0.5))
        self._vo_hits = 0         # 触发计数（纯观测，不挂旗）

        # ---- 通信模型（邻居信息经无线链路：延迟/丢包由 comm_model_node 模拟） ----
        # enabled=false 时保持直连订阅 /drone_j/odom → 与今天逐位一致（零侵入）。
        cm = settings.get("comms", {})
        self.comms_enabled = bool(cm.get("enabled", False))
        self.comms_expire_s = float(cm.get("expire_s", 2.0))
        self.neighbor_last = {}    # {j: 最近一次收到邻居 odom 的时间}（失联剪枝）

        # ---- P0 指标埋点（纯观测，不改行为；收集器 nav_metrics_node 落盘 run_logs/） ----
        # Float32MultiArray 布局（12 项；W6 追加只能加尾部——matrix_post 对
        # ts.csv 按位置切片 r[3:14]，重排/插中会错位）：
        #   0 碰撞次数  1 卡滞累计s  2 卡滞段数  3 最长单段卡滞s  4 侧向翻转总数
        #   5 累计飞行距离m  6 全程最小clearance m  7 最大速度m/s  8 sim时间s
        #   9 当前速度m/s  10 当前clearance m（闭环才有值，其余 999）
        #   11 飞行段最小机间距m（对邻居真值 odom 3D 距，z≥swarm_min_z 计，
        #      world_node 仲裁同口径；垫段 ~1.5m 间距不计防锚死）
        self.metrics_enabled = bool(settings.get("nav", {}).get(
            "metrics", {}).get("enabled", True))
        self.m_col = 0
        self.m_stuck_s = 0.0
        self.m_stuck_n = 0
        self.m_stuck_max = 0.0
        self.m_flip = 0
        self.m_dist = 0.0
        self.m_min_clear = 1e9
        self.m_min_dclear = 1e9
        self.m_max_spd = 0.0
        self._m_prev_pos = None
        self._m_prev_t = None
        self._m_stuck_t0 = None
        self._m_pub_t = -1e9
        self._clr_now = 1e9
        if self.metrics_enabled:
            self.pub_metrics = rospy.Publisher(ns + "/nav_metrics",
                                               Float32MultiArray, queue_size=5)

        self.pub_vel = rospy.Publisher(ns + "/vel_cmd", Twist, queue_size=10)
        # 真值到位信号（P0-1 第三刀）：goal_reached 是 nav 的停拉判据（真值
        # norm<cur_arrive_tol），广播给 mission 层做 SCAN 入场/释放门——释放点
        # 语义与判分（真值）对齐，漂移只影响门时机不影响释放点。纯观测话题，
        # 消费端（executor arrive_gate）默认开、可关。
        self.pub_arrived = rospy.Publisher(ns + "/nav/arrived", Bool,
                                           queue_size=1, latch=True)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.Subscriber(ns + "/planning/goal", PoseStamped, self._on_goal)
        # 碰撞事件两模式都订阅：闭环用于注入占用，P0 指标用于计数
        rospy.Subscriber(ns + "/collision", Bool, self._on_collision)
        # 真落地门控（与 world_node collides skip_ground 同语义）：armed=
        # 最后下降段，贴地 vz 下限豁免——否则与 touchdown_z 下降对抗，
        # 平衡在 z≈0.4 悬停永不落地（P4 验证实证）
        self.landing_armed = False
        rospy.Subscriber(ns + "/mission/landing_armed", Bool, self._on_landing_armed)
        # 着陆终态机退出控制/分离场（run8 法证）：pads 同排间距 1.5m <
        # 分离作用半径，已落地机仍在 Boids 分离场——后降机被托在
        # z≈0.35-0.41 永不达 touchdown 判据 0.27（d1 armed×3 卡 70s+），
        # 先降机被托离地面（d0 0.05→0.78）。LANDED/DONE=真落地：自己
        # 停控（一次性零速——断流后 world cmd_vel 保持末值须显式归零）
        # 并从他机分离场剔除；RETIRED/FAILED/ABORT=悬停锁定，保留在场。
        # 旗 closed_loop.halt_on_landed（默认开）。
        cl = settings.get("closed_loop", {}) or {}
        self._halt_on_landed = bool(cl.get("halt_on_landed", True))
        self._halted = False
        self._landed_fleet = set()
        if self._halt_on_landed:
            rospy.Subscriber("/zx2026/task_update", TaskUpdate,
                             self._on_task_update)
        if self.closed_loop:
            rospy.Subscriber(ns + "/cloud", PointCloud2, self._on_cloud)
            self._collision_obs = set()  # 碰撞标记：绕过 footprint clearing 的占用格
        for j in range(self.scene.drone_count):
            if j == self.drone_id:
                continue
            if self.comms_enabled:
                # 通信模型开启：邻居状态经无线链路中继（延迟/丢包/突发）
                rospy.Subscriber("/drone_%d/odom_from_%d" % (self.drone_id, j),
                                 Odometry, lambda m, j=j: self._on_neighbor(j, m))
            else:
                rospy.Subscriber("/drone_%d/odom" % j, Odometry,
                                 lambda m, j=j: self._on_neighbor(j, m))

        self.last_plan_t = -1.0
        rospy.loginfo("nav_node: drone %d closed_loop=%s max_vel=%.1f cruise_z=%.1f "
                      "tc=%s de=%s veto=%s metrics=%s soa=%s swg=%s swq=%s vo=%s",
                      self.drone_id, self.closed_loop, self.max_vel, self.cruise_z,
                      self.tc_enabled, self._de_enabled, self._vt_enabled,
                      self.metrics_enabled, self._soa_enabled, self._swg_enabled,
                      self._swq_enabled, self._vo_enabled)

    # ---- callbacks ------------------------------------------------------------
    def _on_wind(self, msg):
        self.wind_vec[0] = msg.vector.x
        self.wind_vec[1] = msg.vector.y

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, p.z)
        self.has_odom = True
        v = msg.twist.twist.linear
        self.v_odom = np.array([v.x, v.y, v.z])

    def _on_neighbor(self, j, msg):
        if j in self._landed_fleet:
            return   # 已落地机退出感知（防 odom 流把剔除项填回分离场）
        p = msg.pose.pose.position
        self.neighbors[j] = (p.x, p.y, p.z)
        v = msg.twist.twist.linear
        self.neighbor_vel[j] = (v.x, v.y, v.z)
        self.neighbor_last[j] = rospy.get_time()

    def _on_goal(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y, p.z)
        rospy.loginfo("nav_node: drone %d new goal (%.1f, %.1f, %.1f)",
                      self.drone_id, p.x, p.y, p.z)
        if self.closed_loop and self.tc_enabled:
            self._path_force = True   # 粘滞模式下立即重选路径奔新 goal
        self._replan()

    def _on_landing_armed(self, msg):
        self.landing_armed = bool(msg.data)

    def _on_task_update(self, msg):
        """着陆终态机退出控制/分离场（halt_on_landed，run8 法证）。

        LANDED/DONE=真落地（executor disarm）：自己停控+从全部机的
        分离场剔除；RETIRED/FAILED/ABORT=悬停锁定仍在空中，保留在场。
        """
        if msg.mission_state not in ("LANDED", "DONE"):
            return
        i = int(msg.drone_id)
        if i == self.drone_id:
            if not self._halted:
                self._halted = True
                self.pub_vel.publish(Twist())   # 一次性零速
                rospy.loginfo("nav_node: drone %d landed, halt control",
                              self.drone_id)
        else:
            self._landed_fleet.add(i)
            self.neighbors.pop(i, None)
            self.neighbor_vel.pop(i, None)
            self.neighbor_last.pop(i, None)

    def _on_collision(self, msg):
        """收到碰撞事件 → 把碰撞点强行注入占用栅格，绕过 footprint clearing。
        解决了 lidar 噪声导致树干漏出栅格 → A* 规划穿树 → 循环碰撞的问题。
        （P0：任何模式都计数。）"""
        if not msg.data:
            return
        self.m_col += 1
        if not self.closed_loop:
            return
        cx, cy = self._g_to_cell(self.est_pos[0], self.est_pos[1])
        self._collision_obs.add((cx, cy))
        self._rebuild_global_grid()
        if self.tc_enabled:
            # 粘滞模式下路径不随栅格周期重选：碰撞后必须立即强制绕开
            self._grid_t = rospy.get_time()
            self._path_force = True
        if self._de_enabled and self._de_active:
            self._de_t0 = -1e9   # 逃逸中再碰撞：下拍强制重选逃逸方向
        # W1 遥测：接触法向闭合速度 + 接触距离。慢速贴树挤入是全史主形态
        # （0.12-0.98 m/s 全低速带），闭合速度是它的定义性指标——比碰撞计数
        # （A/B 主判据只有 ~20-30 事件/臂）高一个量级的判别力。纯观测。
        v_close, clr_now = self._closing_speed()
        rospy.loginfo("nav_node: drone %d collision marker at (%.1f,%.1f) cell (%d,%d) "
                      "close=%.2f clr=%.2f",
                      self.drone_id, self.est_pos[0], self.est_pos[1], cx, cy,
                      v_close, clr_now)

    def _closing_speed(self):
        """接触法向闭合速度遥测：本机速度在"指向最近点云点"方向上的投影
        （v·u，>0=向障碍闭合）与该点距离。碰撞瞬间采样，无点云返回 (0, 极大)。
        只读状态不改行为；grep 前缀 "collision marker at" 保持不变。"""
        v = self.v_odom
        px, py, pz = self.odom[0], self.odom[1], self.odom[2]
        dr = self.scene.drone_radius
        best_d, best_u = 1e9, None
        for (x, y, z) in (self.cloud or []):
            if z < pz - dr or z > pz + dr:
                continue
            d = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
            if d < best_d:
                best_d = d
                if d > 1e-6:
                    best_u = ((x - px) / d, (y - py) / d, (z - pz) / d)
        if best_u is None:
            return 0.0, 1e9
        return (float(v[0] * best_u[0] + v[1] * best_u[1] + v[2] * best_u[2]),
                float(best_d))

    def _on_cloud(self, msg):
        # 解析 PointCloud2（x/y/z 三个 FLOAT32 字段）
        n = msg.width
        data = msg.data
        step = msg.point_step
        off = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = off.get("x", 0), off.get("y", 4), off.get("z", 8)
        pts = []
        for i in range(n):
            b = i * step
            x = struct.unpack_from('<f', data, b + ox)[0]
            y = struct.unpack_from('<f', data, b + oy)[0]
            z = struct.unpack_from('<f', data, b + oz)[0]
            pts.append((x, y, z))
        self.cloud = pts
        # 全局 SLAM：把巡航带内点投到持久占用格（世界系真值测量，累积建图）。
        # 树杆贯穿 [0,3.2]，巡航带 [1.5,3.5] 能稳定命中；地面(0)/围栏(≤1.3)/
        # 标识柱(≤1.15)/路缘(≤0.32) 都低于该带，不进入地图（与 god-mode 一致）。
        t_now = rospy.get_time() if self._pc_enabled else 0.0
        for (x, y, z) in pts:
            if 1.5 <= z <= 3.5:
                c = self._g_to_cell(x, y)
                if self._pc_enabled:
                    self._occ_seen[c] = t_now   # 重观测刷新时间戳
                else:
                    self._occ_cells.add(c)

    # ---- 定位漂移（闭环） -----------------------------------------------------
    def _update_drift(self):
        if not self.has_odom:
            return
        self.drift += np.array([self._rng.gauss(0.0, self.drift_rate),
                                self._rng.gauss(0.0, self.drift_rate),
                                self._rng.gauss(0.0, self.drift_rate * 0.5)])
        dn = float(np.linalg.norm(self.drift))
        if dn > self.drift_max:
            self.drift *= self.drift_max / dn
        # 置信位姿 = 真值 odom + 漂移（LIO 长期漂移；真值仅世界动力学知道）
        self.est_pos = np.array(self.odom) + self.drift

    # ---- 规划 ---------------------------------------------------------------
    def _replan(self):
        if self.goal is None:
            return
        if self.closed_loop:
            self._update_drift()
            self.last_plan_t = rospy.get_time()
            return
        path2 = self.astar.plan(self.odom[:2], self.goal[:2])
        self.path = [(x, y, self.cruise_z) for (x, y) in path2]
        self.waypoint_idx = 0
        self.last_plan_t = rospy.get_time()

    # ---- 闭环：全局占用栅格（SLAM） ----------------------------------------
    def _g_to_cell(self, x, y):
        ix = int((x - self.g_x0) / self.res)
        iy = int((y - self.g_y0) / self.res)
        return (max(0, min(self.g_nx - 1, ix)), max(0, min(self.g_ny - 1, iy)))

    def _g_to_world(self, ix, iy):
        return (self.g_x0 + (ix + 0.5) * self.res,
                self.g_y0 + (iy + 0.5) * self.res)

    def _rebuild_global_grid(self):
        """从占用格集合重建全局栅格：标记 → 膨胀 → 清自机足迹。

        滑窗模式（P4）只收 ttl 窗内格子并顺手剪枝（界住字典规模）；
        否则用持久集合（逐位原行为）。
        """
        occ = np.zeros((self.g_ny, self.g_nx), dtype=bool)
        if self._pc_enabled:
            cut = rospy.get_time() - self._pc_ttl
            for c in [c for c, t in self._occ_seen.items() if t < cut]:
                del self._occ_seen[c]
            cells = self._occ_seen.keys()
        else:
            cells = self._occ_cells
        for (ix, iy) in cells:
            if 0 <= ix < self.g_nx and 0 <= iy < self.g_ny:
                occ[iy, ix] = True
        infl = int(math.ceil(self.inflation / self.res))
        occ = self._dilate_rect(occ, infl)
        # 热点树定向膨胀：全史碰撞聚类 Top 树额外 +extra_cells，A* 提前绕开
        # （只加热点格；全局 inflation 0.9 的绕行超窗教训不重蹈，默认关）
        if self._hi_enabled and self._hi_trees:
            mask = np.zeros((self.g_ny, self.g_nx), dtype=bool)
            for (wx, wy) in self._hi_trees:
                hx, hy = self._g_to_cell(wx, wy)
                if 0 <= hx < self.g_nx and 0 <= hy < self.g_ny:
                    mask[hy, hx] = True
            if mask.any():
                occ = occ | self._dilate_rect(mask, max(1, self._hi_extra))
        # 清空自机足迹（膨胀后自机所在格必可通行，避免起点被占死锁）
        ci, cj = self._g_to_cell(self.est_pos[0], self.est_pos[1])
        for iy in range(max(0, cj - 1), min(self.g_ny, cj + 2)):
            for ix in range(max(0, ci - 1), min(self.g_nx, ci + 2)):
                occ[iy, ix] = False
        # 碰撞标记：撞树后强制占用，绕过足迹清除，避免 lidar 盲区循环碰撞
        if self._collision_obs:
            col_occ = np.zeros((self.g_ny, self.g_nx), dtype=bool)
            for (cix, ciy) in self._collision_obs:
                if 0 <= cix < self.g_nx and 0 <= ciy < self.g_ny:
                    col_occ[ciy, cix] = True
            col_occ = self._dilate_rect(col_occ, infl)
            occ = occ | col_occ
        self._gocc_infl = occ
        self._publish_occ_grid()

    def _publish_occ_grid(self):
        """把膨胀后全局占用栅格发布为 OccupancyGrid（world 系，rviz Map 显示）。"""
        if self._gocc_infl is None:
            return
        g = OccupancyGrid()
        g.header.frame_id = "world"
        g.header.stamp = rospy.Time.now()
        g.info.resolution = self.res
        g.info.width = self.g_nx
        g.info.height = self.g_ny
        g.info.origin.position.x = self.g_x0
        g.info.origin.position.y = self.g_y0
        g.info.origin.orientation.w = 1.0
        g.data = np.where(self._gocc_infl, 100, 0).astype(np.int8).ravel().tolist()
        self.pub_occ.publish(g)

    def _min_clearance(self):
        """真值 odom 到最近点云点的距离（飞行高度带内）；无点返回大数。"""
        px, py, pz = self.odom
        best = 1e9
        for (x, y, z) in self.cloud:
            if z < pz - 0.6 or z > pz + 0.6:
                continue
            d = math.hypot(x - px, y - py, z - pz)
            if d < best:
                best = d
        return best if best < 1e9 else 1e9

    def _dam_clearance(self):
        """漂移感知有效裕度：真值 clearance − 漂移在"背向最近障碍"方向的投影。

        u 指向最近点云点（真值系，与 _min_clearance 同帧），漂移背离障碍
        （drift·u<0）时 est 距离比真值大 |drift·u|，该份裕度是虚的。
        返回 (eff, erosion)；无点云返回 (大数, 0)。
        """
        px, py, pz = self.odom
        best, best_u = 1e9, None
        for (x, y, z) in self.cloud:
            if z < pz - 0.6 or z > pz + 0.6:
                continue
            d = math.hypot(x - px, y - py, z - pz)
            if d < best:
                best = d
                if d > 1e-6:
                    best_u = ((x - px) / d, (y - py) / d, (z - pz) / d)
        if best >= 1e9 or best_u is None:
            return best, 0.0
        ero = max(0.0, -(self.drift[0] * best_u[0] +
                         self.drift[1] * best_u[1] +
                         self.drift[2] * best_u[2]))
        return max(0.0, best - ero), ero

    def _nstop_scale(self):
        """近停区地板衰减系数：距 goal<zone_r 线性衰减，钳到 scale_min。

        治 2026-08-31 法证的挤压机制：vcap 地板 0.35 贴脸仍保底 0.53m/s，
        前推力把平衡点压进真值接触。仅作用于非 dam 分支的地板（dam 开时以
        dam 衰减为准，二者不同开）。flag 关 / de 逃逸中 / goal 缺失 → 1.0
        （逐位原行为）。
        run7 法证两处修订：
        ① deadband 处归 0 会把 vcap 压到 0（贴地 clearance/2≈0）→ 下降
          冻结在 touchdown 阈值上方（d1 卡 z=0.31 70s+）/ 平台上空收敛卡死腿
          超时（d5 COLOR_UNRESOLVED）——P2 scale→0 磨树同类病灶。
          修：钳 scale_min（0.25→保底速度 ~0.13m/s，末段 0.2m 约 1.5s 收口，
          平台上空沉深 0.06m 级）。
        ② armed（计划降落窗口）不衰减：touchdown 目标本就贴地，需满速下降。
        """
        if not self._nstop_enabled or self._de_active or self.goal is None:
            return 1.0
        if self.landing_armed:
            return 1.0
        dg = math.hypot(self.odom[0] - self.goal[0],
                        self.odom[1] - self.goal[1])
        if dg >= self._nstop_zone_r:
            return 1.0
        span = max(1e-6, self._nstop_zone_r - self._nstop_deadband)
        return max(self._nstop_scale_min,
                   min(1.0, (dg - self._nstop_deadband) / span))

    @staticmethod
    def _dilate_rect(occ, k):
        """对矩形布尔栅格做 k 次 8 邻域膨胀（numpy 移位，兼容非方阵）。"""
        out = occ.copy()
        for _ in range(k):
            prev = out
            out = prev.copy()
            out[1:, :] |= prev[:-1, :]
            out[:-1, :] |= prev[1:, :]
            out[:, 1:] |= prev[:, :-1]
            out[:, :-1] |= prev[:, 1:]
            out[1:, 1:] |= prev[:-1, :-1]
            out[1:, :-1] |= prev[:-1, 1:]
            out[:-1, 1:] |= prev[1:, :-1]
            out[:-1, :-1] |= prev[1:, 1:]
        return out

    @staticmethod
    def _grid_astar(occ, start, goal):
        """在占用栅格 occ[ny,nx] 上做 8 邻域 A*；返回 [(ix,iy),...]，失败返回 None。"""
        ny, nx = occ.shape
        if occ[start[1], start[0]] or occ[goal[1], goal[0]]:
            return None
        open_h = [(0.0, start)]
        came = {}
        g_cost = {start: 0.0}
        closed = set()
        while open_h:
            _, cur = heapq.heappop(open_h)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return path
            cx, cy = cur
            for dx, dy, cost in ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                                 (-1, -1, 1.414), (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414)):
                tx, ty = cx + dx, cy + dy
                if not (0 <= tx < nx and 0 <= ty < ny):
                    continue
                if occ[ty, tx]:
                    continue
                ng = g_cost[cur] + cost
                if (tx, ty) not in g_cost or ng < g_cost[(tx, ty)]:
                    g_cost[(tx, ty)] = ng
                    f = ng + math.hypot(tx - goal[0], ty - goal[1])
                    came[(tx, ty)] = cur
                    heapq.heappush(open_h, (f, (tx, ty)))
        return None

    @staticmethod
    def _nearest_free_grid(occ, i, j):
        """从 (i,j) 螺旋向外找最近非占用格；找不到返回 None。"""
        ny, nx = occ.shape
        if not occ[j, i]:
            return (i, j)
        for r in range(1, max(nx, ny)):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < nx and 0 <= nj < ny and not occ[nj, ni]:
                        return (ni, nj)
        return None

    def _plan_global(self):
        """全局 A*：目标被占时优先"目标容忍孔径"规划，孔径不通再退化为
        最近可达格（对齐 god-mode 语义）。

        投放点净空 ~4m 常被树干格 2 格膨胀（环内沿距点心 ~1.1m）封死：
        原退化语义把 goal 换成环外最近可达格 → 无人机停在环外（lookahead
        航点=自身 → 指令≈0）永不投放（P3 验证 2 FAIL、矩阵 2 FAIL 均涉及）。
        孔径法在规划副本里只豁免 goal 周边 5x5 幻影，真实占用照常阻挡，
        A* 自动从环上最宽缺口钻进清空中心——比盲推直达更安全直接。
        """
        occ = self._gocc_infl
        s = self._g_to_cell(self.est_pos[0], self.est_pos[1])
        g = self._g_to_cell(self.goal[0], self.goal[1])
        if occ[s[1], s[0]]:
            return None
        if not occ[g[1], g[0]]:
            return self._grid_astar(occ, s, g)
        occ2 = occ.copy()
        for iy in range(max(0, g[1] - 2), min(self.g_ny, g[1] + 3)):
            for ix in range(max(0, g[0] - 2), min(self.g_nx, g[0] + 3)):
                occ2[iy, ix] = False
        path = self._grid_astar(occ2, s, g)
        if path is not None:
            return path
        alt = self._nearest_free_grid(occ, g[0], g[1])
        if alt is None:
            return None
        return self._grid_astar(occ, s, alt)

    def _cur_arrive_tol(self):
        """到位停拉圈：goal z 低于 low_z_max（精确任务段）用收紧圈。

        注意方法名不能叫 _arrive_tol——L111 的同名 float 属性会遮蔽方法
        （run15 全瘫根因：'float' object is not callable）。
        """
        return (self._arrive_tol_low_z
                if self.goal[2] < self._arrive_tol_low_z_max
                else self._arrive_tol)

    def _closed_loop_cmd(self):
        # 已到位（真值判定）：停住，等 mission 换 goal
        if np.linalg.norm(np.array(self.goal) - np.array(self.odom)) \
                < self._cur_arrive_tol():
            return np.zeros(3)

        # 周期重建全局栅格（点云累积建图，视野覆盖全场，保持 0.5s 新鲜度）
        now = rospy.get_time()
        rebuilt = False
        if self._gocc_infl is None or now - self._grid_t > 0.5:
            self._rebuild_global_grid()
            self._grid_t = now
            rebuilt = True
        if not self.tc_enabled:
            # 原版行为：栅格与路径绑定，每 0.5s 一并重选。
            # P2 修复：规划失败不再清空 _gpath（末次有效路径保持）——清空会让
            # 指令间歇归零成"flap 停滞"；失败起点记入 _plan_fail_since，连续
            # 失败 confirm_s 由 de 逃逸接管，旧路径期间由反应层/否决层兜底。
            if rebuilt or now - self._gpath_t > 0.5:
                p = self._plan_global()
                if p is not None:
                    self._gpath = p
                    self._plan_fail_since = None
                elif self._plan_fail_since is None:
                    self._plan_fail_since = now
                self._gpath_t = now
        else:
            # P1 路径粘滞：路径只在 驻留到期 / 被新占用切断 / 强制标志 时重选。
            # 治"绕树左右两解等代价时每 0.5s 重选来回翻"的规划抖动。
            if (self._gpath is None or self._path_force
                    or self._plan_fail_since is not None
                    or now - self._gpath_t > self.tc_stick
                    or self._path_blocked()):
                p = self._plan_global()
                if p is not None:
                    self._gpath = p
                    self._plan_fail_since = None
                elif self._plan_fail_since is None:
                    self._plan_fail_since = now
                self._gpath_t = now
                self._path_force = False

        # P3 死端逃逸状态机（规划连续稳定 exit_hyst_s 才退出，见 _de_tick）
        # P3b 目标封锁直达：goal 格被占 + 停滞 goal_stall_s → 直达逼近；
        # 激活期间压制起点侧逃逸（直达时 start 常在膨胀环内，两者会打架）。
        if self._de_enabled:
            gocc = None
            if self._gocc_infl is not None:
                gi, gj = self._g_to_cell(self.goal[0], self.goal[1])
                gocc = bool(self._gocc_infl[gj, gi])   # np.bool_ 不是 True 单例，is 判断会失效
            if self._gs_active:
                if gocc is False:
                    self._gs_active = False
                    self._gs_stall_t0 = None
                    rospy.loginfo("nav_node: drone %d goal-seal cleared", self.drone_id)
                elif now - self._gs_total_t0 > self._gs_max_s:
                    # 单周期上限：盲推被真实障碍推力平衡时停推，回正常规划
                    # （停滞 3s 后可再进——周期性试探，有界不锁死）
                    self._gs_active = False
                    self._gs_stall_t0 = None
                    rospy.loginfo("nav_node: drone %d goal-seal cycle timeout", self.drone_id)
            elif (gocc is True and float(np.linalg.norm(self.v_odom)) < 0.15
                  and not self._gs_mutex_block()):
                if self._gs_stall_t0 is None:
                    self._gs_stall_t0 = now
                elif now - self._gs_stall_t0 >= self._gs_stall_s:
                    self._gs_active = True
                    self._gs_total_t0 = now
                    self._gs_stall_t0 = None
                    rospy.loginfo("nav_node: drone %d GOAL-SEAL direct approach", self.drone_id)
            else:
                self._gs_stall_t0 = None
            if not self._gs_active:
                self._de_tick(now)

        # 沿全局路径取前瞻航点（找路径上离自机最近格，再向前 lookahead 步）
        lookahead = int(1.5 / self.res)
        if self._gs_active:
            # P3b 目标封锁直达：直指 goal 慢速逼近（膨胀环是幻影占用，
            # 真实 clearance 2m+；限速/反应避障/分离照常兜底）
            gx = self.goal[0] - self.odom[0]
            gy = self.goal[1] - self.odom[1]
            gn = math.hypot(gx, gy) or 1.0
            cmd = np.array([gx / gn * self._gs_speed,
                            gy / gn * self._gs_speed, 0.0])
        elif self._de_active and self._de_dir is not None:
            # P3 逃逸中：沿评分方向慢速驶出（垂直/限速/反应层照常生效）
            cmd = np.array([self._de_dir[0] * self._de_speed,
                            self._de_dir[1] * self._de_speed, 0.0])
        elif self._gpath is not None and len(self._gpath) >= 1:
            # P0-2（run23 法证）：len>=2 → len>=1。start 格==goal 格时规划器
            # 返回 1 格路径，旧条件把它落进"无路径悬停"分支 cmd=0——goal 点
            # 在 cell 角上时停点离 goal 可达 0.5-0.7，arrived 永假，平台腿
            # 冻结 30-48s 超时（d3 @P3 48s/d1 @P2 10s/d0 35s 同签名：
            # gpath=len1 + cmd=0.000 + horiz 冻结 + settled=True）。
            # len1 时 best=0、k=0=len-1、est-goal<res 对角 0.707<0.75，
            # 下方末段去量化直达分支必然接管，伺服正常收敛。
            ci, cj = self._g_to_cell(self.est_pos[0], self.est_pos[1])
            best = 0
            best_d = 1e18
            for k, (px, py) in enumerate(self._gpath):
                d = (px - ci) * (px - ci) + (py - cj) * (py - cj)
                if d < best_d:
                    best_d, best = d, k
            k = min(best + lookahead, len(self._gpath) - 1)
            wx, wy = self._g_to_world(self._gpath[k][0], self._gpath[k][1])
            # P0-1 末段去量化（run19/20 法证）：前瞻航点已到路径末端且 est 距
            # goal 不足 1.5 格时直指 goal 本身——cell 中心量化残差可达
            # ~res/2*sqrt2≈0.35 > arrive_tol_est，伺服锁死在 cell 中心
            # （cmd=0.000 + est 误差 ~0.3 + arrived=False 悬置签名），
            # SCAN 入场门永不放行、平台腿 45s 超时。直达段 ≤0.75m 且
            # 反应层（云避障/分离/vcap）照常兜底，与 GOAL-SEAL 直达同构。
            if k == len(self._gpath) - 1 and \
                    math.hypot(self.goal[0] - self.est_pos[0],
                               self.goal[1] - self.est_pos[1]) < 1.5 * self.res:
                wx, wy = self.goal[0], self.goal[1]
            cmd = np.array([np.clip(wx - self.est_pos[0], -2.0, 2.0),
                            np.clip(wy - self.est_pos[1], -2.0, 2.0),
                            0.0])
        else:
            # 无路径：水平悬停（不再盲冲入障碍），仅保持/收敛高度
            cmd = np.zeros(3)

        # 垂直：基于真值高度（气压/光流较准），向 goal z 收敛
        cmd[2] = np.clip((self.goal[2] - self.odom[2]) * 1.5, -1.0, 1.0)
        # 贴地 vz 下限（防误贴地）；armed=最后下降段真落地，豁免
        if not self.landing_armed:
            cmd[2] = max(cmd[2], (0.5 + self.scene.venue["ground_z"] - self.odom[2]) * 1.0)

        # 动态限速：点云最近障碍越近越慢（真实规划器遇障减速）
        # （clearance 每拍只算一次，P0 指标与 P1 锁定复用）
        clearance = self._min_clearance()
        self._clr_now = clearance
        # W3 塌缩卫兵：接管退化的水平基指令（群集/分离/云避障/vcap 照常叠加）
        if self._cg_enabled:
            cmd = self._collapse_guard(cmd, now, clearance)
        # P1 时间一致性：侧向采样始终执行（供 P0 指标），锁定偏置仅 enabled 时
        self._sample_side(cmd, now)
        if self.tc_enabled:
            cmd = self._tc_bias(cmd, now, clearance)
        return self._apply_vcap(cmd, clearance)

    def _apply_vcap(self, cmd, clearance):
        """动态限速：点云最近障碍越近越慢（真实规划器遇障减速）。

        dam（漂移感知裕度）开启时用有效裕度 eff 限速，且 0.35 地板随侵蚀占比
        线性衰减到 0——治"贴脸仍有 0.53m/s 上限 + 前推力把平衡点压进真值接触"
        的慢挤入（2026-08-31 三次碰撞法证：接触时速度 0.12-0.98 m/s，全为低速
        挤入）。地板连续衰减而非硬冻结，避免复辟 P2 scale→0 磨树病灶。
        关闭时逐位原行为（floor 恒 0.35、用原始 clearance）。
        近停区（near_stop_zone）开启时，本分支地板乘 _nstop_scale() 衰减
        （距 goal<zone_r 线性降到 deadband 处 0）；关时 scale 恒 1.0 逐位不变。
        """
        vcap = self.closed_max_vel
        if self._dam_enabled:
            eff, ero = self._dam_clearance()
            if eff < self._dam_deep_r:
                # 深近区（v2）：地板随侵蚀占比连续衰减到 0。v1 在 eff<2.0 全域
                # 衰减，持续向量漂移背对走廊时整条腿地板归零 → 任务级减速
                # （P45 满窗 FAIL）；v2 收缩到贴脸带（0.7=基线地板 bind 边界），
                # 接触只发生在这里，带外逐位回基线公式。
                floor = 0.35 * max(0.0, 1.0 - ero / self.drift_max)
                vcap = self.closed_max_vel * max(floor, eff / 2.0)
            elif clearance < 2.0:
                vcap = self.closed_max_vel * max(0.35 * self._nstop_scale(),
                                                 clearance / 2.0)
        elif clearance < 2.0:
            vcap = self.closed_max_vel * max(0.35 * self._nstop_scale(),
                                             clearance / 2.0)
        vn = float(np.linalg.norm(cmd))
        if vn > vcap:
            cmd = cmd * (vcap / vn)
        return cmd

    # ---- W3 返航冻结（伺服自锁）修复：前瞻塌缩卫兵 -----------------------------
    def _collapse_guard(self, cmd, now, clearance):
        """规划自认健康而水平基指令≈0 → 前瞻航点塌缩回自身，接管基指令。

        签名同 P3b 注释"lookahead 航点=自身 → 指令≈0"：路径分支 cmd=航点−est，
        航点落在自机格/身边格时指令退化到厘米级，无人机被伺服锁死在定点
        （返航腿起飞数米后 mm/s 爬行，run 214546/194437/live 三局同形态，
        d5 三局全中）。P3b GOAL-SEAL 只治了 goal 封锁形态，这是通用形态：
        沿路径从最近格向前找 ≥min_target 的航点接管；全路径都在身边（路径
        退化为最近可达格终点）则直指 goal——同 GOAL-SEAL 语义，膨胀环是
        幻影占用，反应层/限速照常兜底。接管只发生在基指令层，群集/分离/
        云避障/vcap 全部照常叠加。
        不接管的地盘：规划失败/无路径（de 逃逸接管）、距 goal 近（到位
        收敛区）、真实净空不足（近障减速归反应层）。
        触发日志 COLLAPSE-GUARD 同时是法证采样：base/best/len/pick 落实
        塌缩机制本身。
        """
        if self._plan_fail_since is not None or self._gpath is None:
            return cmd
        if math.hypot(cmd[0], cmd[1]) >= self._cg_min_cmd:
            return cmd
        d_goal = math.hypot(self.goal[0] - self.odom[0],
                            self.goal[1] - self.odom[1])
        if d_goal < self._cg_dgoal_min:
            return cmd
        if clearance < self._cg_clear_min:
            return cmd
        ex, ey = self.est_pos[0], self.est_pos[1]
        ci, cj = self._g_to_cell(ex, ey)
        best = 0
        best_d = 1e18
        for k, (px, py) in enumerate(self._gpath):
            d = (px - ci) * (px - ci) + (py - cj) * (py - cj)
            if d < best_d:
                best_d, best = d, k
        tx = ty = None
        pick = -1
        for kk in range(best, len(self._gpath)):
            wx, wy = self._g_to_world(self._gpath[kk][0], self._gpath[kk][1])
            if math.hypot(wx - ex, wy - ey) >= self._cg_min_target:
                tx, ty, pick = wx, wy, kk
                break
        if tx is None:
            # 全路径都在身边：路径本身已退化（最近可达格终点）——直指 goal
            tx, ty = self.goal[0], self.goal[1]
        n = math.hypot(tx - ex, ty - ey) or 1.0
        rospy.loginfo_throttle(
            1.0, "nav_node: drone %d COLLAPSE-GUARD base=(%.3f,%.3f) "
            "d_goal=%.1f best=%d len=%d pick=%d target=(%.2f,%.2f) "
            "est=(%.2f,%.2f) clear=%.2f",
            self.drone_id, cmd[0], cmd[1], d_goal, best, len(self._gpath),
            pick, tx, ty, ex, ey, clearance)
        cmd[0] = (tx - ex) / n * self._cg_floor_speed
        cmd[1] = (ty - ey) / n * self._cg_floor_speed
        return cmd

    # ---- P1 时间一致性（DeFoP anti-oscillation 移植） --------------------------
    def _sample_side(self, cmd, now):
        """采样水平指令相对 goal 连线的侧向符号并记录翻转时刻。

        side=+1 指令在 goal 连线左侧（cross>0），-1 右侧；|cmd_xy| 过弱或已接近
        goal 时不采样（噪声/到位段不计翻转）。无论 tc_enabled 都采样——翻转率
        本身是 P0 观测指标，A/B 双方都要有数据。
        """
        gx = self.goal[0] - self.odom[0]
        gy = self.goal[1] - self.odom[1]
        if math.hypot(cmd[0], cmd[1]) < 0.15 or math.hypot(gx, gy) < 0.4:
            return
        cross = gx * cmd[1] - gy * cmd[0]
        side = 1 if cross > 0.0 else -1
        if self._side_last != 0 and side != self._side_last:
            self._flip_times.append(now)
            self.m_flip += 1
        self._side_last = side
        while self._flip_times and now - self._flip_times[0] > self.tc_window:
            self._flip_times.popleft()

    def _tc_bias(self, cmd, now, clearance):
        """翻转率超阈值且近障碍 → 锁定当前侧并加侧向偏置；clearance 达标或超时解锁。"""
        if self._lock_side != 0:
            if (now - self._lock_t0 > self.tc_max_lock
                    or clearance >= self.tc_unlock):
                self._lock_side = 0
        if (self._lock_side == 0 and self._side_last != 0 and self._flip_times
                and clearance < self.tc_unlock):
            span = now - self._flip_times[0]
            # 持续抖动才锁：窗口内 ≥3 次翻转且窗口已积累 ≥0.5s（单次翻转即锁
            # 会把偶发偏移也锁住，实测造成频繁短停）
            if len(self._flip_times) >= 3 and span >= 0.5 \
                    and len(self._flip_times) / span > self.tc_flip_rate_max:
                self._lock_side = self._side_last
                self._lock_t0 = now
                rospy.loginfo_throttle(
                    5.0, "nav_node: drone %d dither-lock side=%d rate=%.1f/s clr=%.2f",
                    self.drone_id, self._lock_side,
                    len(self._flip_times) / span, clearance)
        if self._lock_side == 0:
            return cmd
        gx = self.goal[0] - self.odom[0]
        gy = self.goal[1] - self.odom[1]
        gn = math.hypot(gx, gy)
        if gn < 0.4:
            return cmd
        # 侧向单位向量 = goal 方向左旋 90°（side=+1 即"继续从左侧过"）
        cmd[0] += self._lock_side * (-gy / gn) * self.tc_bias
        cmd[1] += self._lock_side * (gx / gn) * self.tc_bias
        return cmd

    def _path_blocked(self):
        """当前全局路径是否被（膨胀后）占用格切断；自机足迹 3x3 例外。"""
        if self._gpath is None or self._gocc_infl is None:
            return True
        occ = self._gocc_infl
        ci, cj = self._g_to_cell(self.est_pos[0], self.est_pos[1])
        for (px, py) in self._gpath:
            if occ[py, px] and not (abs(px - ci) <= 1 and abs(py - cj) <= 1):
                return True
        return False

    # ---- P3 死端基元逃逸（DeFoP motion primitives 移植，仅借死端脱困） --------
    def _gs_mutex_block(self):
        """M1 救援互斥：de 逃逸进行中或规划连续失败（含 de confirm 窗口）时
        goal-seal 禁发。drone5 法证：盲推在 de 活跃/plan_fail 持续期间照样
        发射 → 双救援方向相反 → 原地研磨 + flip 风暴。优先级"谁先激活谁持
        轮"：gs 激活期间 _de_tick 本就不跑；互斥只挡 gs 的触发竞态，逃逸
        优先——先脱困，脱困后仍停滞 3s 再试直达（周期性试探保留）。"""
        if not self._rm_enabled:
            return False
        return self._de_active or self._plan_fail_since is not None

    def _de_tick(self, now):
        """逃逸状态机：规划连续失败 confirm_s 进入；规划连续稳定 exit_hyst_s 才退出。

        失败判定用 _plan_fail_since（P2 修复后规划失败不清空 _gpath，
        末次有效路径继续跟随，故不能再用 _gpath is None 作失败信号）。
        退出迟滞（P5）：est 在膨胀标记/树格边缘时规划成败逐拍翻转（单次成功
        窗口 < 2s），单次成功即退出会让逃逸频繁进出、无人机停在云推力平衡点
        原地 flap（B43r stuck 772.9s 教训）——迟滞期间继续沿逃逸方向驶出，
        把自机推离边缘后规划自然持续稳定，满 exit_hyst_s 再交还路径跟随。
        驻留到期重选方向（此刻栅格/碰撞标记已更新）。不设总超时——脱困尝试
        一直持续（失败→重评分→再试），最坏退化为原悬停行为，不会更差。
        """
        if self._de_active:
            if self._plan_fail_since is None:
                if self._de_ok_t0 is None:
                    self._de_ok_t0 = now
                elif now - self._de_ok_t0 >= self._de_exit_hyst:
                    self._de_active = False
                    self._de_dir = None
                    self._de_fail_t0 = None
                    self._de_ok_t0 = None
                    if self._swq_enabled:
                        self._swq_until = now + self._swq_ramp   # 爬坡重入窗
                        rospy.loginfo("nav_node: drone %d swarm quiet window "
                                      "open (ramp %.1fs)", self.drone_id,
                                      self._swq_ramp)
                    rospy.loginfo("nav_node: drone %d dead-end escape done (t=%.1fs)",
                                  self.drone_id, now - self._de_total_t0)
                    return
            else:
                self._de_ok_t0 = None
            if self._de_dir is None or now - self._de_t0 >= self._de_dwell:
                self._de_dir = self._de_pick_dir()
                self._de_t0 = now
                rospy.loginfo_throttle(
                    5.0, "nav_node: drone %d escape pick dir=%s",
                    self.drone_id, self._de_dir_str())
            return
        self._de_ok_t0 = None
        if self._plan_fail_since is not None:
            if self._de_fail_t0 is None:
                self._de_fail_t0 = now
            elif now - self._de_fail_t0 >= self._de_confirm:
                self._de_active = True
                self._de_total_t0 = now
                self._de_t0 = now
                self._de_ok_t0 = None
                self._de_dir = self._de_pick_dir()
                rospy.loginfo("nav_node: drone %d DEAD-END escape engaged dir=%s",
                              self.drone_id, self._de_dir_str())
        else:
            self._de_fail_t0 = None

    def _de_dir_str(self):
        if self._de_dir is None:
            return "None(hover)"
        return "%.2frad" % math.atan2(self._de_dir[1], self._de_dir[0])

    def _de_pick_dir(self):
        """均匀 n_dirs 方向做占用栅格射线评分，选逃逸方向。

        逃逸场景自机被碰撞标记膨胀壳包住：射线允许穿过 ≤pierce 个占用采样
        （壳厚 ~2 格 ≈ 5-6 个 0.25m 采样），其间 free_run 清零（验证须连续），
        之后累计 free_need 个连续自由采样才算可行，可达距离 reach 一直延伸到
        再遇占用为止；score = reach + goal_bias·cos(方向, goal)。
        前置点云走廊检查：沿线上 cloud_clear_r 内有点云障碍的方向直接淘汰
        （栅格壳可穿≠真树可穿；真树近旁的逃逸方向会被否决层冻结成磨树）。
        出界按占用处理（围栏低于建图带不在栅格内，防逃向围栏）。
        全方向不可行（reach 全 0）返回 None → 悬停等下拍重试，绝不盲冲占用。
        """
        occ = self._gocc_infl
        if occ is None:
            return None
        gx = self.goal[0] - self.est_pos[0]
        gy = self.goal[1] - self.est_pos[1]
        gn = math.hypot(gx, gy) or 1.0
        hx = self.g_nx * self.res * 0.5
        hy = self.g_ny * self.res * 0.5
        step = self.res * 0.5
        # 点云走廊：逃逸方向沿线上 cloud_clear_r 内不得有点云障碍（半宽 =
        # drone_radius + 0.1，与 veto 包络同参数）。标记壳只进栅格不进点云的
        # 部分可穿；真树就在近旁的方向拒绝——否则逃逸指令被否决层冻结，
        # 退化为原地磨树级联（P2 矩阵 B 臂 highZ 磨树 40:0 教训）。
        dr = self.scene.drone_radius
        cloud_corridor = dr + 0.1
        pz = self.odom[2]
        cloud = getattr(self, 'cloud', None) or []
        best, best_s, best_reach = None, -1e18, 0.0
        for k in range(self._de_ndirs):
            th = 2.0 * math.pi * k / self._de_ndirs
            dx, dy = math.cos(th), math.sin(th)
            cloud_ok = True
            for (x, y, z) in cloud:
                if z < pz - dr or z > pz + dr:
                    continue
                ex = x - self.est_pos[0]
                ey = y - self.est_pos[1]
                t = ex * dx + ey * dy
                if t <= 0.0 or t > self._de_cloud_r:
                    continue
                if abs(ex * dy - ey * dx) <= cloud_corridor:
                    cloud_ok = False
                    break
            if not cloud_ok:
                continue
            pierce = free_run = 0
            reach = 0.0
            d = step
            while d <= self._de_ray:
                x = self.est_pos[0] + dx * d
                y = self.est_pos[1] + dy * d
                if abs(x) >= hx or abs(y) >= hy:
                    break
                ix, iy = self._g_to_cell(x, y)
                if occ[iy, ix]:
                    free_run = 0
                    if reach > 0.0:
                        break      # 已验证可行后遇占用 → 到此为止
                    pierce += 1
                    if pierce > self._de_pierce:
                        break      # 穿透配额用尽 → 该方向不可行
                else:
                    free_run += 1
                    if free_run >= self._de_free_need:
                        reach = d
                d += step
            score = reach + self._de_goal_bias * (dx * gx + dy * gy) / gn
            if self._bc_enabled and reach > 0.0:
                # M2 bounce 走廊化：入口二值检查只看前 cloud_r 窗口，本项看
                # 整条可达段的最小侧距——入口过得了、前方收窄的方向被压分。
                # 同样可达时偏好宽走廊，抑制窄点两侧反向弹摆。
                score += self._bc_w * self._bc_corridor_margin(dx, dy, reach)
            if score > best_s:
                best_s, best, best_reach = score, (dx, dy), reach
        return best if best_reach > 0.0 else None

    def _bc_corridor_margin(self, dx, dy, reach):
        """M2 bounce 走廊化：候选方向沿线 [0,reach] 段到点云（z 带过滤，与
        入口检查同参数）的最小侧向距离，截断到 cloud_clear_r 归一到 [0,1]。
        无点云（传感空窗）返回 1.0 中性值——不改变可达性判定，只参与计分。"""
        cloud = getattr(self, 'cloud', None) or []
        if not cloud:
            return 1.0
        pz = self.odom[2]
        dr = self.scene.drone_radius
        m = self._de_cloud_r
        for (x, y, z) in cloud:
            if z < pz - dr or z > pz + dr:
                continue
            ex = x - self.est_pos[0]
            ey = y - self.est_pos[1]
            t = ex * dx + ey * dy
            if t <= 0.0 or t > reach:
                continue
            perp = abs(ex * dy - ey * dx)
            if perp < m:
                m = perp
        return m / self._de_cloud_r

    # ---- god-mode：全局 A* 跟随（原版逻辑） ---------------------------------
    def _god_mode_cmd(self):
        pos = np.array(self.odom)
        cmd = np.zeros(3)
        if self.waypoint_idx < len(self.path):
            wp = np.array(self.path[self.waypoint_idx])
            err = wp - pos
            if math.hypot(err[0], err[1]) < 0.8:
                self.waypoint_idx += 1
            if self.waypoint_idx < len(self.path):
                wp = np.array(self.path[self.waypoint_idx])
                err = wp - pos
            elif len(self.path) == 1:
                err = np.array(self.goal) - pos
            else:
                err = np.array(self.goal) - pos
            vx = np.clip(err[0], -2.0, 2.0)
            vy = np.clip(err[1], -2.0, 2.0)
            vz = np.clip(err[2] * 1.5, -1.0, 1.0)
            cmd = np.array([vx, vy, vz])
            vn = np.linalg.norm(cmd)
            if vn > self.max_vel:
                cmd = cmd * (self.max_vel / vn)
            if self.waypoint_idx >= len(self.path) and np.linalg.norm(err) < 0.4:
                cmd = np.zeros(3)
        else:
            err = np.array(self.goal) - pos
            if np.linalg.norm(err) < 0.4:
                cmd = np.zeros(3)
            else:
                vx = np.clip(err[0], -2.0, 2.0)
                vy = np.clip(err[1], -2.0, 2.0)
                vz = np.clip(err[2] * 1.5, -1.0, 1.0)
                cmd = np.array([vx, vy, vz])
                vn = np.linalg.norm(cmd)
                if vn > self.max_vel:
                    cmd = cmd * (self.max_vel / vn)
        cmd[2] = max(cmd[2], (0.5 + self.scene.venue["ground_z"] - self.odom[2]) * 1.0)
        return cmd

    # ---- Boids 群集（真正的集群算法：聚合+对齐，分离由 _apply_separation 负责）--
    # 经典 Reynolds flocking，编队从局部交互涌现（无领航、无中心）。只作用于
    # 水平分量（垂直由任务高度控制）；仅在巡航高度（z≥min_z）生效，避免起降/
    # 投放阶段被拉向地面邻居。近障碍时按 clearance 缩放，防止聚合把 drone 拉
    # 进树。本函数在分离/避障之前执行，二者可安全覆盖群集力（安全优先）。
    def _apply_swarm(self, cmd):
        if not self.swarm_enabled or self.odom[2] < self.swarm_min_z:
            return cmd
        # W1 rescue_quiet：救援态静默 coh/ali（分离/硬推/云避障在下游照常，
        # 机间与障碍安全底线不动）。含 _plan_fail_since：规划失败期 pre 已
        # 悬停，聚合推成为唯一水平力，也一并静默。
        if self._swq_enabled and (self._de_active or self._gs_active
                                  or self._plan_fail_since is not None):
            return cmd
        # 恢复爬坡重入：de 退出后 ramp_s 内聚合增益线性回升（0→1）。Gazebo
        # 法证：逃逸"成功"后 0.65s 即刻再暴露——聚合推力即刻满额回场是公共
        # 时标。_swq_until 仅在 _swq_enabled 时被置位，此处开关粒度一致。
        quiet = 1.0
        if self._swq_enabled:
            now_q = rospy.get_time()
            if now_q < self._swq_until:
                quiet = max(0.0, 1.0 - (self._swq_until - now_q)
                            / max(self._swq_ramp, 1e-6))
        # 局部编组齐巡航才启用：任一感知半径内邻居未达巡航高度 → 关闭。
        # 起降区 pads 仅 1.5m 间隔 + 错峰起飞，此时无人机散布在各高度带，
        # 群集（尤其聚合）会把它们撮到一起撞机；等全组都上到巡航带再聚合。
        for j, p in self.neighbors.items():
            if math.hypot(p[0] - self.odom[0], p[1] - self.odom[1]) < self.perception_radius \
                    and p[2] < self.swarm_min_z:
                return cmd
        n_pos, n_vel = [], []
        for j, p in self.neighbors.items():
            if abs(p[2] - self.odom[2]) > self.swarm_z_band:
                continue  # 只与同高度带邻居交互（爬升/下降中的无人机不互相吸引）
            if math.hypot(p[0] - self.odom[0], p[1] - self.odom[1]) < self.perception_radius:
                n_pos.append(p)
                v = self.neighbor_vel.get(j)
                if v is not None:
                    n_vel.append(v)
        if not n_pos:
            return cmd
        # W1 obs_guard：快照聚合前的指令，coh/ali 叠加后过同款障碍投影。
        # 投影保切向——树挡着时聚合方向本就不可达，剪掉朝树分量不丢可达性。
        pre_sw = np.array(cmd) if self._swg_enabled else None
        coh = (np.mean(n_pos, axis=0) - np.array(self.odom)) * self.cohesion_gain
        if n_vel:
            ali = (np.mean(n_vel, axis=0) - self.v_odom) * self.alignment_gain
        else:
            ali = np.zeros(3)
        if self.obstacle_scale:
            cl = self._min_clearance()
            if cl < 2.0:
                s = max(0.0, cl / 2.0)
                coh *= s
                ali *= s
        if quiet < 1.0:
            coh = coh * quiet
            ali = ali * quiet
        cmd[0] += coh[0] + ali[0]
        cmd[1] += coh[1] + ali[1]
        if pre_sw is not None:
            cmd = self._sep_obs_guard(cmd, pre_sw, tag="swarm")
        return cmd

    # ---- 多机分离（两模式共用） ---------------------------------------------
    def _apply_separation(self, cmd):
        pre = np.array(cmd) if self._soa_enabled else None
        hard_r = 2.0 * self.scene.drone_radius + 1.1
        for j, nj in self.neighbors.items():
            dx = self.odom[0] - nj[0]
            dy = self.odom[1] - nj[1]
            dz = self.odom[2] - nj[2]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d <= 0:
                continue
            if d < self.sep_radius:
                w = (self.sep_radius - d) / self.sep_radius
                cmd[0] += (dx / d) * self.sep_gain * w
                cmd[1] += (dy / d) * self.sep_gain * w
                cmd[2] += (dz / d) * self.sep_gain * w
            if d < hard_r:
                ux, uy, uz = -dx / d, -dy / d, -dz / d
                v_app = cmd[0] * ux + cmd[1] * uy + cmd[2] * uz
                if v_app > 0:
                    push = v_app + (hard_r - d) * 4.0
                    cmd[0] -= push * ux
                    cmd[1] -= push * uy
                    cmd[2] -= push * uz
        if pre is not None:
            cmd = self._sep_obs_guard(cmd, pre)
        vn = np.linalg.norm(cmd)
        if vn > self.max_vel:
            cmd = cmd * (self.max_vel / vn)
        return cmd

    # ---- W6 VO-lite 机间速度预测避撞（默认关，A/B 过线后翻） -------------------
    def _apply_vo_avoid(self, cmd):
        """TCPA/DCPA 预判汇聚冲突 + 相对速度垂直切向偏转（纯增量预测层）。

        分离是纯位置反应（贴脸才推），反应窗 ~0.5s 不够返程对头汇聚
        （d2/d3 互撞立案）；本层在分离上游用相对速度外推：对每个同高度带
        邻居算 tcpa/dcpa，冲突（0<tcpa<t_pred 且 dcpa<safe_r 且水平距
        <engage_r）时沿推离 CPA 线一侧给切向推力（⊥相对速度，一阶不减速
        ——dam v2 教训：贴脸减速=延长暴露）。镜像几何两机推力天然反向
        （各向己右，相对分离率 2×push；vo_selftest B 组不变量），无需
        仲裁/奇偶定向。多邻居时取冲突分最高者生效（单推力防矢量打架）。
        自机速度用 v_odom 实际值（与邻居 odom twist 同帧；comms 关直连时
        镜像不变量严格成立，comms 开时 100-140ms 延迟使其在近零脱靶窗
        退化为共模侧移——该窗由分离硬推层兜底，压力臂以 dclr/idcol 实测
        判读）；邻居 age>stale_s 剔除（通信过期不预判）。
        增量过 _sep_obs_guard（tag=vo，往树上推的分量被投影）；
        救援态静默+恢复爬坡与 coh/ali 同款（不改写逃逸矢量）。
        """
        if not self._vo_enabled or self.odom[2] < self.swarm_min_z:
            return cmd
        if self._swq_enabled and (self._de_active or self._gs_active
                                  or self._plan_fail_since is not None):
            return cmd
        quiet = 1.0
        if self._swq_enabled:
            now_q = rospy.get_time()
            if now_q < self._swq_until:
                quiet = max(0.0, 1.0 - (self._swq_until - now_q)
                            / max(self._swq_ramp, 1e-6))
        now = rospy.get_time()
        best = None            # (score, ux, uy, tcpa, dcpa, d, j)
        for j, nj in self.neighbors.items():
            if now - self.neighbor_last.get(j, -1e9) > self._vo_stale_s:
                continue
            nv = self.neighbor_vel.get(j)
            if nv is None:
                continue
            r = vo_deflect(self.odom[0], self.odom[1], self.odom[2],
                           self.v_odom[0], self.v_odom[1], self.drone_id,
                           nj, (nv[0], nv[1]),
                           self._vo_engage_r, self._vo_safe_r,
                           self._vo_t_pred, self.swarm_z_band)
            if r is None:
                continue
            ux, uy, score, tcpa, dcpa, d = r
            if best is None or score > best[0]:
                best = (score, ux, uy, tcpa, dcpa, d, j)
        if best is None:
            return cmd
        score, ux, uy, tcpa, dcpa, d, j = best
        push = min(self._vo_gain * score, self._vo_max_push) * quiet
        if push <= 0.0:
            return cmd
        pre = np.array(cmd) if self._soa_enabled else None
        cmd[0] += ux * push
        cmd[1] += uy * push
        if pre is not None:
            cmd = self._sep_obs_guard(cmd, pre, tag="vo")
        self._vo_hits += 1
        rospy.loginfo_throttle(
            2.0, "vo_avoid: drone %d vs %d d=%.2f dcpa=%.2f tcpa=%.2f "
                 "push=%.2f hits=%d",
            self.drone_id, j, d, dcpa, tcpa, push, self._vo_hits)
        return cmd

    def _sep_obs_guard(self, cmd, pre, tag="sep"):
        """分离/群集增量障碍投影：把增量中指向贴身障碍的分量投影掉（保切向）。

        分离/硬推本身障碍全盲，把机往树上推后靠云避障拔河，残余 + vcap
        地板即挤入形态（全史聚类主因，tree#24 双向撞/跟车同点连撞均此）。
        W1 起 _apply_swarm 的 coh/ali 增量同走本函数（tag="swarm"）——聚合
        增量直加原本无障碍管辖，是 734 聚类的聚合半边根因。
        机间防撞语义不变：树在两机之间时投影后推力为零，几何上不可能因此
        发生机间接触（树挡着则本就不可达）。
        tag 仅用于分桶计数/遥测（_soa_hits），不影响投影数学。
        """
        dsep = np.array([cmd[0] - pre[0], cmd[1] - pre[1]])
        if float(np.linalg.norm(dsep)) < 1e-6:
            return cmd
        px, py, pz = self.odom[0], self.odom[1], self.odom[2]
        dr = self.scene.drone_radius
        guard_r = dr + 0.9
        changed = False
        hit_d = None            # 最近被投影障碍（遥测：距离/坐标/朝向分量）
        hit_xy = (0.0, 0.0)
        hit_into = 0.0
        for (x, y, z) in (self.cloud or []):
            if z < pz - dr or z > pz + dr:
                continue
            ex = x - px
            ey = y - py
            d = math.sqrt(ex * ex + ey * ey)
            if d < 1e-6 or d > guard_r + 0.6:
                continue
            ux, uy = ex / d, ey / d
            into = dsep[0] * ux + dsep[1] * uy
            if into > 0.0:
                dsep[0] -= into * ux
                dsep[1] -= into * uy
                changed = True
                if hit_d is None or d < hit_d:
                    hit_d, hit_xy, hit_into = d, (x, y), into
        if changed:
            self._soa_hits[tag] = self._soa_hits.get(tag, 0) + 1
            rospy.loginfo_throttle(
                5.0, "nav_node: drone %d %s_guard hit n=%d obs=(%.2f,%.2f) "
                "d=%.2f into=%.2f", self.drone_id, tag,
                self._soa_hits[tag], hit_xy[0], hit_xy[1], hit_d, hit_into)
            cmd[0] = pre[0] + float(dsep[0])
            cmd[1] = pre[1] + float(dsep[1])
        return cmd

    # ---- 闭环：点云动量感知避障（安全网） -----------------------------------
    # 用真值 odom 计算相对距离——等价机体系雷达测量（无漂移）。真实 LIO 漂移只
    # 影响定位/建图(局部地图)，不影响即时相对测距；此处据此让漂移作用在规划层，
    # 而安全网保持无漂移，避免"漂移让自机误判距离而撞树"。
    def _apply_cloud_avoidance(self, cmd):
        dr = self.scene.drone_radius
        now_v = rospy.get_time()
        if not hasattr(self, '_v_est'):
            self._v_est = np.zeros(3)
            self._prev_odom_v = np.array(self.odom)
            self._prev_odom_t = now_v
        else:
            dt_e = now_v - self._prev_odom_t
            if dt_e >= 0.04:
                self._v_est = 0.6 * self._v_est + 0.4 * \
                    (np.array(self.odom) - self._prev_odom_v) / max(dt_e, 1e-3)
                self._prev_odom_v = np.array(self.odom)
                self._prev_odom_t = now_v
        speed = float(np.linalg.norm(self._v_est))
        acc_cap = 4.0   # 与物理后端 max_acc 一致，制动距离不低估
        brake_dist = speed * speed / (2.0 * acc_cap)
        look = brake_dist + dr + 0.15
        px, py, pz = self.odom[0], self.odom[1], self.odom[2]
        if speed > 0.1:
            fx = px + (self._v_est[0] / speed) * look
            fy = py + (self._v_est[1] / speed) * look
            fz = pz + (self._v_est[2] / speed) * look
        else:
            fx = px + cmd[0] * 0.3
            fy = py + cmd[1] * 0.3
            fz = pz + cmd[2] * 0.3
        react_r = dr + 0.15
        pred_r = dr + 0.05
        for (x, y, z) in self.cloud:
            if z < pz - dr or z > pz + dr:
                continue
            ex = px - x
            ey = py - y
            ez = pz - z
            ed = math.sqrt(ex * ex + ey * ey + ez * ez)
            ddx = fx - x
            ddy = fy - y
            ddz = fz - z
            dd = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
            if ed >= react_r and dd >= pred_r:
                continue
            if ed <= 1e-6:
                ex, ed = 1.0, 1e-3
            ux, uy, uz = ex / ed, ey / ed, ez / ed
            if self._ca_sign_fix:
                # 修复形态：v_in = 指向障碍的分量 >0（正在接近）才推，方向沿远离。
                # 逃离/悬停 v_in<=0 零侵入——原形态的"逃离惩罚"吸引子由此消除。
                v_in = -(cmd[0] * ux + cmd[1] * uy + cmd[2] * uz)
                if v_in > 0.0:
                    danger = min(ed, dd)
                    push = v_in + max(0.0, react_r - danger) * 4.0 \
                           + max(0.0, pred_r - dd) * 6.0
                    cmd[0] += push * ux
                    cmd[1] += push * uy
                    cmd[2] += push * uz
            else:
                v_app = cmd[0] * ux + cmd[1] * uy + cmd[2] * uz
                if v_app > 0:
                    danger = min(ed, dd)
                    push = v_app + max(0.0, react_r - danger) * 4.0 \
                           + max(0.0, pred_r - dd) * 6.0
                    cmd[0] -= push * ux
                    cmd[1] -= push * uy
                    cmd[2] -= push * uz
        vn = np.linalg.norm(cmd)
        if vn > self.max_vel:
            cmd = cmd * (self.max_vel / vn)
        return cmd

    # ---- P2 指令否决层（DeFoP M1 几何安全监督移植） -------------------------
    # 对最终水平指令做"刹车包络"检查：沿指令方向扫点云走廊（半宽 =
    # drone_radius + lateral_margin，只计 t>0 的前方点——保证永不阻挡远离/切向
    # 分量），取最近沿轨距离 t_near；允许速度 v_a 满足
    #   v_now·t_react + v_a²/(2·a_brake) + margin ≤ t_near
    # 超出则等比缩水平指令到 v_a（只缩模不改向，不反馈进规划）。风前馈之后执行，
    # 门的是最终发布指令；与 vcap（全向减速）互补：只约束指向障碍的分量。
    def _apply_veto(self, cmd):
        if not self._vt_enabled:
            return cmd
        vh = math.hypot(cmd[0], cmd[1])
        if vh < 0.05:
            return cmd
        ux, uy = cmd[0] / vh, cmd[1] / vh
        px, py, pz = self.odom[0], self.odom[1], self.odom[2]
        dr = self.scene.drone_radius
        corridor = dr + self._vt_lat_margin
        v_est = getattr(self, '_v_est', None)
        v_now = math.hypot(v_est[0], v_est[1]) if v_est is not None else 0.0
        t_near = None
        for (x, y, z) in self.cloud:
            if z < pz - dr or z > pz + dr:
                continue
            dx = x - px
            dy = y - py
            t = dx * ux + dy * uy          # 沿轨投影（只计前方点）
            if t <= 0.0 or t > self._vt_ray:
                continue
            lat = abs(dx * uy - dy * ux)   # 垂轨偏移
            if lat > corridor:
                continue
            if t_near is None or t < t_near:
                t_near = t
        if t_near is None:
            return cmd
        room = max(0.0, t_near - self._vt_margin - v_now * self._vt_t_react)
        v_allow = math.sqrt(2.0 * self._vt_a_brake * room)
        if v_allow >= vh:
            return cmd
        # 缩放地板：对齐 vcap 的 0.35 系数——否决层减速但不完全冻结水平进度
        # （scale→0 会把"碰撞弹起后水平脱离"冻死成磨树级联，P2 矩阵 B 臂教训）
        s = max(self._vt_scale_min, v_allow / vh)
        self._vt_vetoes += 1
        if (self._vt_vetoes % 50) == 1:
            rospy.loginfo("nav_node: drone %d VETO scale=%.2f v_cmd=%.2f "
                          "v_allow=%.2f t_near=%.2f v_now=%.2f",
                          self.drone_id, s, vh, v_allow, t_near, v_now)
        cmd[0] *= s
        cmd[1] *= s
        return cmd

    # ---- god-mode：动量感知静态避障（scene 真值，原版逻辑） -----------------
    def _apply_scene_avoidance(self, cmd):
        dr = self.scene.drone_radius
        now_v = rospy.get_time()
        if not hasattr(self, '_v_est'):
            self._v_est = np.zeros(3)
            self._prev_odom_v = np.array(self.odom)
            self._prev_odom_t = now_v
        else:
            dt_e = now_v - self._prev_odom_t
            if dt_e >= 0.04:
                self._v_est = 0.6 * self._v_est + 0.4 * \
                    (np.array(self.odom) - self._prev_odom_v) / max(dt_e, 1e-3)
                self._prev_odom_v = np.array(self.odom)
                self._prev_odom_t = now_v
        speed = float(np.linalg.norm(self._v_est))
        acc_cap = 6.0
        brake_dist = speed * speed / (2.0 * acc_cap)
        look = brake_dist + dr + 0.15
        if speed > 0.1:
            fx = self.odom[0] + (self._v_est[0] / speed) * look
            fy = self.odom[1] + (self._v_est[1] / speed) * look
            fz = self.odom[2] + (self._v_est[2] / speed) * look
        else:
            fx = self.odom[0] + cmd[0] * 0.3
            fy = self.odom[1] + cmd[1] * 0.3
            fz = self.odom[2] + cmd[2] * 0.3
        react_r = dr + 0.15
        pred_r = dr + 0.05
        px, py, pz = self.odom[0], self.odom[1], self.odom[2]
        for ob in self.scene.obstacles:
            if ob.kind in ("fence", "marker"):
                continue
            if ob.kind == "tree":
                if ob.crown_z + ob.crown_r < pz - dr:
                    continue
            elif ob.hi[2] < pz - dr or ob.lo[2] > pz + dr:
                continue
            rx, ry, rz, ed = self._obstacle_nearest(ob, (px, py, pz))
            ex, ey, ez = rx - px, ry - py, rz - pz
            cx, cy, cz, dd = self._obstacle_nearest(ob, (fx, fy, fz))
            if ed >= react_r and dd >= pred_r:
                continue
            if ed <= 0:
                ex, ed = 1.0, 1e-3
            ux, uy, uz = ex / ed, ey / ed, ez / ed
            v_app = cmd[0] * ux + cmd[1] * uy + cmd[2] * uz
            if v_app > 0:
                danger = min(ed, dd)
                push = v_app + max(0.0, react_r - danger) * 4.0 \
                       + max(0.0, pred_r - dd) * 6.0
                cmd[0] -= push * ux
                cmd[1] -= push * uy
                cmd[2] -= push * uz
        vn = np.linalg.norm(cmd)
        if vn > self.max_vel:
            cmd = cmd * (self.max_vel / vn)
        return cmd

    def _obstacle_nearest(self, ob, q):
        """返回障碍 ob 上距 q 最近的点 (nx,ny,nz) 与距离 nd。

        tree 按「圆柱树干」计算最近点（球冠仅视觉，不参与避障）；
        其余 kind 退化为 AABB 最近点。
        """
        qx, qy, qz = q
        if ob.kind == "tree":
            gz = self.scene.venue["ground_z"]
            z0 = gz
            z1 = gz + ob.trunk_h
            # 树干（竖直圆柱侧面 + z 夹紧）；球冠仅视觉，不参与避障
            dxy = math.hypot(qx - ob.cx, qy - ob.cy)
            if dxy > 1e-9:
                nx = ob.cx + ob.trunk_r * (qx - ob.cx) / dxy
                ny = ob.cy + ob.trunk_r * (qy - ob.cy) / dxy
            else:
                nx, ny = ob.cx + ob.trunk_r, ob.cy
            nz = min(max(qz, z0), z1)
            nd = math.sqrt((nx - qx) ** 2 + (ny - qy) ** 2 + (nz - qz) ** 2)
            return nx, ny, nz, nd
        lo, hi = ob.lo, ob.hi
        nx = min(max(qx, lo[0]), hi[0])
        ny = min(max(qy, lo[1]), hi[1])
        nz = min(max(qz, lo[2]), hi[2])
        return nx, ny, nz, math.sqrt((nx - qx) ** 2 + (ny - qy) ** 2 + (nz - qz) ** 2)

    # ---- P0 指标（纯观测，两模式通用；clearance 仅闭环有值） -----------------
    def _update_metrics(self, cmd, now):
        spd = float(np.linalg.norm(self.v_odom))
        dt = 0.0
        if self._m_prev_t is not None:
            dt = min(max(now - self._m_prev_t, 0.0), 0.5)
            if self._m_prev_pos is not None:
                self.m_dist += float(np.linalg.norm(
                    np.array(self.odom) - self._m_prev_pos))
        self._m_prev_t = now
        self._m_prev_pos = np.array(self.odom)
        if spd > self.m_max_spd:
            self.m_max_spd = spd
        if self._clr_now < self.m_min_clear:
            self.m_min_clear = self._clr_now
        # 机间最小距离（P0 第 12 项，W6）：对邻居真值 odom 的 3D 距——
        # world_node 互撞仲裁同口径（d<2×机半径 触发）。机不在点云里，
        # min_clear 对机间失明由本项补上（d2/d3 互撞前无任何指标预警）。
        # 只计飞行段（z≥swarm_min_z，与 VO 作用带一致）：起飞垫 ~1.5m 间距
        # 会把全程最小值锚死在垫距、飞行段净空改善被削顶（对抗审查立项）。
        if self.odom[2] >= self.swarm_min_z:
            dmin = 1e9
            for _nj in self.neighbors.values():
                if _nj[2] < self.swarm_min_z:
                    continue   # 邻居在起降段（垫上悬停 z≈1.5）不计——垫距锚定
                _d = math.sqrt((_nj[0] - self.odom[0]) ** 2
                               + (_nj[1] - self.odom[1]) ** 2
                               + (_nj[2] - self.odom[2]) ** 2)
                if _d < dmin:
                    dmin = _d
            if dmin < self.m_min_dclear:
                self.m_min_dclear = dmin
        # 卡滞：未到 goal 且速度近乎为零（含碰撞冷却冻结——恢复开销也算卡滞）
        d_goal = float(np.linalg.norm(np.array(self.goal) - np.array(self.odom)))
        if d_goal > 0.4 and spd < 0.1:
            if self._m_stuck_t0 is None:
                self._m_stuck_t0 = now
            self.m_stuck_s += dt
            if now - self._m_stuck_t0 > self.m_stuck_max:
                self.m_stuck_max = now - self._m_stuck_t0
        elif self._m_stuck_t0 is not None:
            self._m_stuck_t0 = None
            self.m_stuck_n += 1
        # 5Hz 发布快照（布局见 __init__ 注释）
        if self.metrics_enabled and now - self._m_pub_t >= 0.2:
            self._m_pub_t = now
            self.pub_metrics.publish(Float32MultiArray(data=[
                float(self.m_col), round(self.m_stuck_s, 2), float(self.m_stuck_n),
                round(self.m_stuck_max, 2), float(self.m_flip),
                round(self.m_dist, 2), min(self.m_min_clear, 999.0),
                round(self.m_max_spd, 3), now, round(spd, 3),
                min(self._clr_now, 999.0),
                min(self.m_min_dclear, 999.0)]))

    # ---- 主循环 -------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        if self._halted:
            return   # 落地终态：零速已发，不再发布（世界侧 cmd_vel 保持零速）
        if not self.has_odom or self.goal is None:
            self.pub_arrived.publish(Bool(data=False))
            return
        now = rospy.get_time()
        self._clr_now = 1e9   # 本拍 clearance 由闭环分支覆写（god-mode/到位段无值）
        # 通信失联剪枝：超过 expire_s 未收到更新的邻居视为失联，从感知中移除
        # （"感知不到的避不开"——群集/分离自然跳过；world_node 互撞检测是真值仲裁者）
        if self.comms_enabled and self.neighbor_last:
            stale = [j for j, t in self.neighbor_last.items()
                     if now - t > self.comms_expire_s]
            for j in stale:
                self.neighbors.pop(j, None)
                self.neighbor_vel.pop(j, None)
                self.neighbor_last.pop(j, None)
        if self.closed_loop:
            if now - self.last_plan_t >= 1.0 / self.replan_hz:
                self._replan()
            cmd = self._closed_loop_cmd()
            cmd = self._apply_swarm(cmd)
            cmd = self._apply_separation(cmd)
            cmd = self._apply_vo_avoid(cmd)   # W6 VO-lite（默认关）
            cmd = self._apply_cloud_avoidance(cmd)
        else:
            if now - self.last_plan_t > 2.0:
                self._replan()
            cmd = self._god_mode_cmd()
            cmd = self._apply_swarm(cmd)
            cmd = self._apply_separation(cmd)
            cmd = self._apply_vo_avoid(cmd)   # W6 VO-lite（默认关）
            cmd = self._apply_scene_avoidance(cmd)

        # 风前馈：顶风补偿（共享风 world 系水平分量；风关/非飞行期时值为 0 → 零侵入）
        if self.wind_ff > 0.0:
            cmd[0] -= self.wind_ff * self.wind_vec[0]
            cmd[1] -= self.wind_ff * self.wind_vec[1]

        # P2 指令否决层：门最终发布指令（合成完毕、含风补偿），见 _apply_veto
        cmd = self._apply_veto(cmd)

        # 停滞看门狗（纯观测）：未到 goal 而持续近零速 → 1Hz 诊断日志。
        # 各形态死锁（flap 停滞/贴靠平衡/磨树）从此日志自证，不再靠trace反推。
        if self.closed_loop:
            d_goal = float(np.linalg.norm(np.array(self.goal) - np.array(self.odom)))
            # 卡死成分诊断（run18 立案）：真值距 goal<1.5m 近零速 2s——平台
            # 邻域悬停不收敛形态（远距 STALL diag 不覆盖 d_goal<1.0 段）。
            # 打印 est/drift/路径/前后指令成分，一次跑定位平衡点由谁构成。
            if d_goal < 1.5 and float(np.linalg.norm(self.v_odom)) < 0.15:
                if self._stuck_t0 is None:
                    self._stuck_t0 = now
                elif now - self._stuck_t0 > 2.0:
                    fail = (now - self._plan_fail_since
                            if self._plan_fail_since is not None else -1.0)
                    gp = "None" if self._gpath is None else "len%d" % len(self._gpath)
                    rospy.loginfo_throttle(
                        2.0, "nav_node: drone %d STUCK diag odom=(%.2f,%.2f,%.2f) "
                        "goal=(%.2f,%.2f,%.2f) est=(%.2f,%.2f) drift=(%.2f,%.2f,%.2f) "
                        "gpath=%s plan_fail=%.1f arrived=%s cmd=%.3f",
                        self.drone_id, self.odom[0], self.odom[1], self.odom[2],
                        self.goal[0], self.goal[1], self.goal[2],
                        self.est_pos[0], self.est_pos[1],
                        self.drift[0], self.drift[1], self.drift[2],
                        gp, fail, self.goal_reached(), float(np.linalg.norm(cmd)))
            else:
                self._stuck_t0 = None
            if d_goal > 1.0 and float(np.linalg.norm(self.v_odom)) < 0.05:
                if self._stall_t0 is None:
                    self._stall_t0 = now
                elif now - self._stall_t0 > 2.0:
                    fail = (now - self._plan_fail_since
                            if self._plan_fail_since is not None else 0.0)
                    gp = "None" if self._gpath is None else "len%d" % len(self._gpath)
                    rospy.loginfo_throttle(
                        1.0, "nav_node: drone %d STALL diag odom=(%.2f,%.2f,%.2f) "
                        "est=(%.2f,%.2f) gpath=%s plan_fail=%.1fs clear=%.2f",
                        self.drone_id, self.odom[0], self.odom[1], self.odom[2],
                        self.est_pos[0], self.est_pos[1], gp, fail,
                        min(self._clr_now, 999.0))
            else:
                self._stall_t0 = None

        vn = np.linalg.norm(cmd)
        if vn > self.max_vel:
            cmd = cmd * (self.max_vel / vn)

        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = cmd[0], cmd[1], cmd[2]
        self.pub_vel.publish(tw)
        self.pub_arrived.publish(Bool(data=self.goal_reached()))
        # P0 指标（纯观测）
        self._update_metrics(cmd, now)

    # ---- 到达查询（供 mission 使用） ----------------------------------------
    def goal_reached(self):
        """到位信号 = 控制环已停的两种形态之或（P0-1 第三刀，run17/18/19 法证）。

        形态一：真值停拉圈 fired（与 _closed_loop_cmd 首行零速判据同款同帧）
        ——圈停即到位，平台低空圈 0.15 → 释放真值 offset ≤0.15。
        形态二：est 系伺服锁（漂移大时真值圈到不了：水平律驱动 est 系，
        真值停在 goal−drift，|drift|>圈 时形态一永假——run17/18 五机 SCAN
        全超时根因）——est xy 误差 <arrive_tol_est 且 z 真值到位即认到达，
        释放 offset=|drift| ≤0.3=本架构理论下界。
        两形态覆盖全漂移域：offset ≤ min(圈, |drift|) 量级。
        """
        if self.goal is None:
            return False
        if np.linalg.norm(np.array(self.goal) - np.array(self.odom)) \
                < self._cur_arrive_tol():
            return True
        xy_ok = math.hypot(self.goal[0] - self.est_pos[0],
                           self.goal[1] - self.est_pos[1]) < self._arrive_tol_est
        z_ok = abs(self.goal[2] - self.odom[2]) < self._cur_arrive_tol()
        return xy_ok and z_ok


if __name__ == "__main__":
    try:
        node = NavNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
