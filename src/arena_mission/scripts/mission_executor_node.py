#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::mission_executor_node — 单机任务执行（每机一个）。

输入: /drone_<id>/odom, /zx2026/mission/<drone_id> (Mission),
      /zx2026/state (String), /drone_<id>/match/result (String),
      /drone_<id>/payload/done (Bool)
输出: /drone_<id>/planning/goal (PoseStamped),
      /drone_<id>/mission/phase (String),
      /drone_<id>/mission/at_drop (Bool),
      /drone_<id>/payload/command (UInt8),
      /drone_<id>/mission/crossed_zone (Bool)
状态机: IDLE → TAKEOFF → TAKEOFF_DONE → EXECUTE_PENDING → EXECUTE
        → [color_id 闭环: RECON] → AT_DROP → DROP → RETURN → DONE/FAILED/RETIRED
RECON（科目三颜色识别闭环）: 逐平台 巡航高度→平台上空 drop_hover_z 悬停
        scan_dwell_s，bucket_scan_step 连续 n_frames 命中本机箱色即锁平台
        （selected_dp）；全扫无果重扫 max_scan_rounds 轮后 COLOR_UNRESOLVED。
"""
import math

import rospy
import numpy as np
from std_msgs.msg import String, Bool, UInt8, Int32, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from zx2026_common import config as cfg
from zx2026_common import scene as sc
from zx2026_common import bucket_select
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission, TaskUpdate


# ---------------------------------------------------------------------------
# W1 via_slots（2026-09-03 贴树判官团）：越界点散点槽位
# 共享越界点 = 穿越区重心 (1.0,2.0)，几乎压在 tree#24 (1.1,2.0) 树干上：
# 六机漏斗串行过同一点 + goal 格恒被真树占用 → nav 侧"goal 周边 5×5 幻影
# 豁免"长期在线 + 停滞 3s 时 GOAL-SEAL 0.8 m/s 朝真树盲推——吸引子层根因。
# 散点槽位把六机汇聚点换成六个真值净空验证过的点，A* 与反应层完全自由
# （零改道）。纯几何无 ROS，tools/ 自检脚本可直接 import 复用。
# ---------------------------------------------------------------------------
def _slot_clearance(scene, x, y, z_lo, z_hi):
    """点 (x,y) 在 z 带 [z_lo,z_hi] 内对全部占用障碍的最小表面净空（m）。

    与 Scene.collides_xy 同一障碍集合（fence/marker 低于巡航高度不算）：
    树=树干圆盘表面距离，其余=AABB 盒表面距离。空场返回极大值。
    """
    best = 1e9
    for ob in scene.obstacles:
        if ob.kind in ("fence", "marker"):
            continue
        if ob.kind == "tree":
            d = math.hypot(x - ob.cx, y - ob.cy) - ob.trunk_r
        else:
            if ob.hi[2] < z_lo or ob.lo[2] > z_hi:
                continue
            dx = max(ob.lo[0] - x, 0.0, x - ob.hi[0])
            dy = max(ob.lo[1] - y, 0.0, y - ob.hi[1])
            d = math.hypot(dx, dy)
        if d < best:
            best = d
    return best


def select_via_slots(scene, drone_count, slot_clear=1.5, slot_sep=2.5,
                     cand_step=0.5, z_lo=2.0, z_hi=3.0, zone_margin=1.0):
    """在穿越区内选 drone_count 个净空达标的散点槽位（纯几何，无 ROS）。

    候选格按 cand_step 铺满穿越区内缩区，逐格过 in_crossing_zone + 表面
    净空 ≥ slot_clear 过滤（slot_clear 是安全地板永不降级）。zone_margin
    把候选区先按带缘内缩：executor 的 CROSS_ZONE 腿有"到点未过带即
    FAILED"绊线（原目标=带心，drop_tol 0.8m 球全含带内，理论不应发生）——
    贴缘槽（如北缘内 0.5m）的 drop_tol 球戳出带外，护栏壳骑行的机在带外
    进槽 0.8m 即被误杀（matrix_w1 实测 drone5 整机 0 分、D 臂 3/3 复现），
    故槽心距带缘 ≥ zone_margin(1.0 > drop_tol 0.8) 恢复"到槽⇒过带"不变量。
    y 向散开用结构化
    分带保证：zone 均分 drone_count 条横带、每带取净空最高且与已选互距
    ≥ slot_sep 的候选一槽（编队前散开意图由"每带一槽"结构性成立——
    逐对 |Δy|≥spread 的贪心装箱在 8m 走廊放 6 槽是刀锋 packing，selftest
    实测不可行，故弃）。某带无解时从剩余候选按净空降序补位；仍不足按
    回退梯子放宽 slot_sep（2.5→2.2→2.0→1.8，末档候选区内缩 0.3m）。
    返回 [(x, y, clearance), ...] 按 y 升序，调用方按 drop_y 名次对号
    入座（各机本地同参计算 → 同一槽位集，无需跨机通信）；
    无可行解返回 None（调用方回退共享越界点原行为）。
    """
    poly = scene.crossing_zone
    if not poly:
        return None
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    # 回退梯子：slot_sep 逐档放宽，末档候选区整体内缩（slot_clear 永不降）
    for sep, inset in ((slot_sep, 0.0), (2.2, 0.0), (2.0, 0.0), (1.8, 0.3)):
        x0, x1 = min(xs) + zone_margin + inset, max(xs) - zone_margin - inset
        b0, b1 = min(ys) + zone_margin + inset, max(ys) - zone_margin - inset
        if x1 < x0 or b1 < b0:
            continue
        band_h = (b1 - b0) / drone_count
        bands = [[] for _ in range(drone_count)]
        n_x = int(math.ceil((x1 - x0) / cand_step))
        n_y = int(math.ceil((b1 - b0) / cand_step))
        for iy in range(n_y + 1):
            py = min(b0 + iy * cand_step, b1)
            for ix in range(n_x + 1):
                px = min(x0 + ix * cand_step, x1)
                if not scene.in_crossing_zone((px, py)):
                    continue
                clr = _slot_clearance(scene, px, py, z_lo, z_hi)
                if clr >= slot_clear:
                    bi = min(int((py - b0) / band_h), drone_count - 1)
                    bands[bi].append((clr, px, py))
        for b in bands:
            b.sort(reverse=True)      # 带内净空优先（同净空按坐标稳定排序）
        slots = []
        # Pass 1：每带一槽（编队前散开的结构保证）
        for b in bands:
            for clr, px, py in b:
                if all(math.hypot(px - sx, py - sy) >= sep for sx, sy, _ in slots):
                    slots.append((px, py, clr))
                    break
        # Pass 2：带解不足 → 剩余候选（不分带）按净空降序补位
        if len(slots) < drone_count:
            chosen = {(sx, sy) for sx, sy, _ in slots}
            rest = [(clr, px, py) for b in bands for (clr, px, py) in b
                    if (px, py) not in chosen]
            rest.sort(reverse=True)
            for clr, px, py in rest:
                if len(slots) >= drone_count:
                    break
                if all(math.hypot(px - sx, py - sy) >= sep for sx, sy, _ in slots):
                    slots.append((px, py, clr))
        if len(slots) >= drone_count:
            slots.sort(key=lambda t: t[1])    # 按 y 升序交付
            return slots[:drone_count]
    return None


class MissionExecutor:
    def __init__(self):
        rospy.init_node("mission_executor_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        self.scene = Scene()
        # 非凸-α 编队协调层：本机相对共享航点的偏移（None = 未开启）
        self.formation_offset = self._compute_formation_offset()
        rules = cfg.load("competition_rules.yaml")
        h = rules["heights"]
        self.takeoff_hover_z = float(h.get("takeoff_hover_z", 1.5))
        self.cruise_z = float(h.get("cruise_z", 2.5))
        self.identify_z = float(h.get("identify_z", 1.2))
        self.drop_hover_z = float(h.get("drop_hover_z", self.identify_z))
        self.touchdown_z = float(h.get("touchdown_z", 0.12))
        self.arrival_tol = float(h.get("area_arrival_tol", 2.0))
        self.drop_tol = float(h.get("drop_arrival_tol", 0.8))
        # 精投放到位阈值：RECON GOTO_LO 用（释放偏移=S1 判平台余量，
        # 0.8 宽容差下释放点散布 0.05~0.63 会骑在判平台线 0.6 上）
        self.hover_tol = float(h.get("drop_hover_tol", 0.35))
        # 释放对准门：MATCH 只证明箱色，释放点=悬停点才是判平台依据。
        # dwell 期邻机爬升柱/排队避让的分离力会把悬停点推离平台心
        # （run4 实测 LOCK 后 120ms 即释放，offset 0.55/0.62 骑判平台线）
        self.release_tol = float(h.get("drop_release_tol", 0.30))
        self.release_align_timeout = float(h.get("drop_align_timeout_s", 6.0))
        # GOTO_LO 中继高度：缩短末段下降行程消 P 环过冲（两段直降
        # 1.2→0.55 过冲至 z≈0.35 扎进平台薄盘碰撞面 z<0.40，run4/5 簇1）。
        # <=0 退回旧两段（A/B 对照用）
        self.drop_mid_z = float(h.get("drop_mid_z", 0.90))
        self.max_vel = float(rules["motion"].get("default_max_vel", 2.0))
        # ---- 科目三颜色识别闭环（color_id） ----
        ci = rules.get("color_id", {}) or {}
        self._cid_enabled = bool(ci.get("enabled", False))
        self._cid_dwell = float(ci.get("scan_dwell_s", 2.0))
        self._cid_min_conf = float(ci.get("min_conf", 0.8))
        self._cid_nframes = int(ci.get("n_frames", 3))
        self._cid_rounds = int(ci.get("max_scan_rounds", 2))
        self._cid_leg_timeout = float(ci.get("scan_leg_timeout_s", 45.0))
        # SCAN 入场稳定门（P0-1 第二刀，run15' 法证）：GOTO_HI→GOTO_LO 在
        # d<drop_tol 即边飞边转下降，位置门（est 0.62/0.35）在摆动弧上被穿过
        # → MATCH/释放发生在未收敛摆动弧上（release x 全部偏东=来向过冲，
        # 0.624 越判线 0.6）。释放真值 offset 由"释放瞬间机在哪"决定而非门限
        # ——稳定门强迫开扫前悬停已稳定（nav 收敛进低空停拉圈，真值
        # ≤arrive_tol_low_z 0.15），漂移只影响门时机不影响释放点。
        self._cid_settle_on = bool(ci.get("settle_gate", True))
        self._cid_settle_win = float(ci.get("settle_window_s", 0.8))
        self._cid_settle_tol = float(ci.get("settle_tol_m", 0.10))
        # 到位信号门（P0-1 第三刀）：SCAN 入场/释放挂 nav 真值到位信号
        # /nav/arrived（nav 停拉判据 goal_reached 广播）。run16 教训：settle
        # 窗把 P 环尾段慢速爬行（<0.12m/s）误判为稳定，悬停点距 goal 0.3-0.5
        # 就开扫——真值到位信号根治（释放点=nav 停拉圈 ≤0.15）。
        self._cid_arrive_gate = bool(ci.get("arrive_gate", True))
        # 多机错峰起飞：drone i 延迟 i*stagger 秒，避免六机同挤入口
        self.stagger = float(rules.get("takeoff_stagger_s", 3.0))
        self.takeoff_at = None
        # P4 进入林区同样错峰：P4 触发后 drone i 再延迟 i*entry_stagger 秒出发，
        # 防止六机同时涌入穿越区入口（入口窄、A* 路径在此汇聚导致互撞）
        self.entry_stagger = float(rules.get("entry_stagger_s", 2.5))
        self.enter_at = None
        # 返航同样错峰：投放完成后 drone i 延迟 i*return_stagger 秒再返航，
        # 防止六机同时涌入返航走廊（y≈6~7 林缘狭缝）互撞被永久冻结
        self.return_stagger = float(rules.get("return_stagger_s", 3.0))
        self.return_at = None

        # ---- W1 via_slots：越界点散点槽位（matrix_w1b 后默认开，配置 mission.via_slots） ----
        vs = cfg.load("sim_settings.yaml").get("mission", {}).get("via_slots", {}) or {}
        self._vs_enabled = bool(vs.get("enabled", False))
        self._vs_clear = float(vs.get("slot_clear", 1.5))
        self._vs_sep = float(vs.get("slot_sep", 2.5))
        self._vs_margin = float(vs.get("zone_margin", 1.0))
        self._vs_slot = None    # 本机槽位（EXECUTE 入场时选定并缓存；None=回退共享点）

        self.state = "IDLE"
        self.mission = None
        self.goal = None
        self.odom = (0.0, 0.0, 1.0)
        self.pad = (0.0, 0.0)
        self.drop = (0.0, 0.0)     # 目标平台 xy（闭环=RECON 锁定后回填；fallback=任务参考指派）
        self.box_color = ""        # 本机箱色（Mission.box_color，空=回退 color_map）
        self.retry = 0
        self.max_retry = 2

        # ---- RECON 扫描状态（仅 color_id 闭环使用） ----
        self.selected_dp = None    # 锁定平台（DropPoint）；None=未锁定
        self.det_color = 255       # 最新颜色检测编码（255=无检测）
        self.det_conf_c = 0.0
        self._scan_order = []
        self._scan_idx = 0
        self._scan_round = 0
        self._scan_mode = "GOTO_HI"   # GOTO_HI(巡航) → GOTO_LO(降平台上空) → SCAN
        self._lo_stage = 0            # GOTO_LO 分级下降：0=缓冲层 1=中继 2=悬停
        self._collided = False        # /collision 边沿检测（重分级用）
        self._drop_hold = False       # 释放对准门激活（MATCH 已到但未对准平台心）
        self._drop_align_t0 = 0.0
        self._drop_step_t0 = 0.0      # 步进精调节流（P0-1 主动收敛）
        self._xy_hist = []            # est xy 短窗（settle 门用，(t,x,y)）
        self._nav_arrived = False     # nav 真值到位信号（arrive_gate 用）
        self._scan_until = 0.0
        self._scan_leg_t0 = 0.0
        self._sel_state = None
        # 同平台排队仲裁：规则口径每平台 2 箱 → 同色双机必锁同平台，按"先到先投"
        # 排队（他机已认领未完成 → 本机在平台侧偏移点+巡航高度避让等待）
        self._claims = {}          # drone_id → selected bucket_id（全部机）
        self._done_set = set()     # 已完成投放的机（payload/done）
        self._nb_pos = {}          # drone_id → (x,y,z) 邻居真值位置（让位仲裁）

        # 穿越区强制通过
        self.crossed_zone = False

        self.pub_goal = rospy.Publisher(ns + "/planning/goal", PoseStamped, queue_size=10)
        self.pub_phase = rospy.Publisher(ns + "/mission/phase", String, queue_size=1, latch=True)
        self.pub_at_drop = rospy.Publisher(ns + "/mission/at_drop", Bool, queue_size=1, latch=True)
        self.pub_payload_cmd = rospy.Publisher(ns + "/payload/command", UInt8, queue_size=10)
        self.pub_crossed = rospy.Publisher(ns + "/mission/crossed_zone", Bool, queue_size=1, latch=True)
        self.pub_selected = rospy.Publisher(ns + "/mission/selected_dp", Int32,
                                            queue_size=1, latch=True)
        self.pub_landing_armed = rospy.Publisher(ns + "/mission/landing_armed",
                                                 Bool, queue_size=1, latch=True)
        self.pub_task_update = rospy.Publisher("/zx2026/task_update", TaskUpdate,
                                               queue_size=10)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.Subscriber(ns + "/nav/arrived", Bool, self._on_nav_arrived)
        rospy.Subscriber("/zx2026/mission/%d" % self.drone_id, Mission, self._on_mission)
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber(ns + "/match/result", String, self._on_match)
        rospy.Subscriber(ns + "/payload/done", Bool, self._on_payload_done)
        rospy.Subscriber(ns + "/detected/color", UInt8, self._on_det_color)
        rospy.Subscriber(ns + "/detected/confidence", Float32, self._on_det_conf)
        # 同平台排队仲裁：订阅全部机的认领/完成事件
        for j in range(self.scene.drone_count):
            if j == self.drone_id:
                self._claims[j] = 0     # 占位（本机认领由 selected_dp 回填）
                continue
            rospy.Subscriber("/drone_%d/mission/selected_dp" % j, Int32,
                             lambda m, j=j: self._on_other_claim(j, m))
            rospy.Subscriber("/drone_%d/payload/done" % j, Bool,
                             lambda m, j=j: self._on_other_done(j, m))
            rospy.Subscriber("/drone_%d/odom" % j, Odometry,
                             lambda m, j=j: self._on_nb_odom(j, m))
        # GCS panic 通道（第 4 步 ②）：全局急停，全部 executor 各自进入 ABORT 终态
        rospy.Subscriber("/zx2026/abort", String, self._on_abort)
        # 比赛合规通道（P5 rule_monitor）：退赛裁决 → hover-lock 终态 RETIRED
        rospy.Subscriber("/drone_%d/rule/retire" % self.drone_id, String,
                         self._on_retire)
        # 碰撞感知重分级（world /collision latch）：平台上空下降被撞后回缓冲层重降
        rospy.Subscriber(ns + "/collision", Bool, self._on_collision)

        self._publish_phase("IDLE")
        self.pub_crossed.publish(Bool(data=False))
        rospy.loginfo("mission_executor_node: drone %d", self.drone_id)
        self._phase_hb = rospy.Timer(rospy.Duration(1.0), self._phase_heartbeat)

    # ---------------------------------------------------------------- callbacks
    def _on_collision(self, msg):
        """碰撞感知重分级：平台上空下降/扫描/投放悬停期被撞 → 回缓冲层重来。

        弹开恢复后 nav 以地板速度全速再降，深过冲会二次触地（run6 法证：
        二次碰撞 z=0.01/-0.72 → 贴地 1.5s → UNAUTHORIZED_LANDING 退休级联）。
        回 identify_z 缓冲层先水平收敛，末段 vcap 地板已被 near_stop_zone
        衰减，重降沉深回到 cm 级。RETURN/LANDING 段不接管（有 armed/豁免）。
        """
        now = bool(msg.data)
        if now and not self._collided:
            if getattr(self, "_pending", None) == "RECON" and \
                    self._scan_mode in ("GOTO_LO", "SCAN"):
                dp = self._scan_order[self._scan_idx]
                self._scan_mode = "GOTO_LO"
                self._lo_stage = 0
                self._scan_leg_t0 = rospy.get_time()
                self._goto((dp.xyz[0], dp.xyz[1], self.identify_z))
                rospy.logwarn("drone %d collision in RECON, re-stage descent "
                              "to buffer", self.drone_id)
            elif self.state == "AT_DROP":
                self._goto((self.drop[0], self.drop[1], self.drop_hover_z))
                rospy.logwarn("drone %d collision at drop, re-hover platform",
                              self.drone_id)
        self._collided = now

    def _on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.pose.position.z)
        # settle 门样本：漂移是慢随机游走，短窗内 est 位移≈0；飞行/摆动/推挤
        # 位移大——用窗内极差判"悬停已稳定"比位置门更抗漂移相位
        self._xy_hist.append((rospy.get_time(), self.odom[0], self.odom[1]))
        if len(self._xy_hist) > 60:
            del self._xy_hist[:-60]

    def _on_nav_arrived(self, msg):
        self._nav_arrived = bool(msg.data)

    def _xy_settled(self):
        """est xy 在 settle_window 窗内极差 < settle_tol → 悬停已稳定。"""
        if not self._cid_settle_on:
            return True
        now = rospy.get_time()
        recent = [(x, y) for (t, x, y) in self._xy_hist
                  if now - t <= self._cid_settle_win]
        if len(recent) < 5:
            return False   # 窗内样本不足（起步初期）视为未稳定
        xs = [p[0] for p in recent]
        ys = [p[1] for p in recent]
        return (max(xs) - min(xs) < self._cid_settle_tol
                and max(ys) - min(ys) < self._cid_settle_tol)

    def _on_det_color(self, msg):
        self.det_color = msg.data

    def _on_det_conf(self, msg):
        self.det_conf_c = msg.data

    def _on_other_claim(self, j, msg):
        self._claims[j] = msg.data

    def _on_other_done(self, j, msg):
        if msg.data:
            self._done_set.add(j)

    def _on_nb_odom(self, j, msg):
        p = msg.pose.pose.position
        self._nb_pos[j] = (p.x, p.y, p.z)

    def _low_occupant(self, dp):
        """平台上空低层占用者（让位仲裁，run10 法证）：他机在目标平台
        1.2m 水平邻域内且 z 低于本机 → 高层者/后到者让位 WAIT_HI，防同
        平台双机中继层分离场互顶（sep 1.5m 顶开 → 末段水平门 0.6 不过
        → 腿超时跳平台 → 轮询白耗 40s+，同色双机完成度退化根因）。
        z 差 <0.1 视同层，drone_id 小者保（确定性仲裁，防双向让位抖动）。
        landed 停控机 odom 仍发但位置在 pads（远离平台），水平门滤掉。"""
        me = self.odom
        if me is None:
            return None
        for j, p in self._nb_pos.items():
            if p is None:
                continue
            dz = me[2] - p[2]
            if abs(dz) < 0.1 and j < self.drone_id:
                dz = 1.0     # 同层：id 小者保，本机（id 大）视为被压
            if dz > 0.05 and math.hypot(p[0] - dp.xyz[0],
                                        p[1] - dp.xyz[1]) < 1.2:
                return j
        return None

    def _wait_pos(self, dp):
        """避让等待位：平台东侧 2.2m + 按 drone_id 沿 y 散点（同平台多机
        等待不重叠；同色对 id 差 3 → y 间隔 2.7m）。2.2m > sep 半径 1.5m
        ——等待机不得落在分离场内，否则悬停机被顶离平台心进不了 SCAN
        （run11 法证：1.2m 等待位把 d0 顶到 0.54 平衡点，腿超时白耗）。"""
        return (dp.xyz[0] + 2.2, dp.xyz[1] + (self.drone_id - 2.5) * 0.9,
                self.cruise_z)

    def _enter_wait(self, dp, reason, who):
        """转 WAIT_HI 避让：挂巡航高度等待位，等待位不构成对方眼中的
        低层占用（z 高于平台悬停层），被让者照常下降。"""
        self._scan_mode = "WAIT_HI"
        self._scan_leg_t0 = rospy.get_time()
        self._goto(self._wait_pos(dp))
        rospy.logwarn("drone %d RECON platform %d %s drone %d, waiting",
                      self.drone_id, dp.id, reason, who)

    def _bucket_holder(self, dp_id):
        """同平台占用者：已认领该平台且未完成投放的他机（先到先投仲裁）。
        返回最小 drone_id 或 None。"""
        holder = None
        for j, b in self._claims.items():
            if b == dp_id and j != self.drone_id and j not in self._done_set:
                if holder is None or j < holder:
                    holder = j
        return holder

    def _on_mission(self, msg):
        if self.state != "IDLE":
            return
        self.mission = msg
        self.box_color = msg.box_color or self.scene.color_for_type(msg.payload_type)
        pads = self.scene.get_pads()
        if self.drone_id < len(pads):
            self.pad = pads[self.drone_id]
        else:
            self.pad = (0.0, 0.0)
        if self._cid_enabled:
            # 闭环：目标平台由机载侦察识别择定——不从 drop_point_id 初始化导航目标
            #（drop_point_id 仅 GCS/审计参考）。drop 在 RECON 锁平台时回填。
            self.drop = None
        else:
            for dp in self.scene.drop_points:
                if dp.id == msg.drop_point_id:
                    self.drop = (dp.xyz[0], dp.xyz[1])
        self.crossed_zone = False
        self.pub_crossed.publish(Bool(data=False))
        rospy.loginfo("drone %d got mission: payload=%s box_color=%s mode=%s ref_dp=%d",
                      self.drone_id, msg.payload_type, self.box_color,
                      "closed-loop" if self._cid_enabled else "fallback",
                      msg.drop_point_id)

    def _on_state(self, msg):
        st = msg.data
        if st == "P3_TAKEOFF" and self.state == "IDLE":
            self.takeoff_at = rospy.get_time() + self.drone_id * self.stagger
            self.state = "TAKEOFF_PENDING"
            rospy.loginfo("drone %d takeoff scheduled in %.1fs", self.drone_id,
                          self.drone_id * self.stagger)
        elif st == "P4_EXECUTE" and self.state == "TAKEOFF_DONE":
            self.enter_at = rospy.get_time() + self.drone_id * self.entry_stagger
            self.state = "EXECUTE_PENDING"
            self._publish_phase("EXECUTE_PENDING")
            rospy.loginfo("drone %d execute entry scheduled in %.1fs",
                          self.drone_id, self.drone_id * self.entry_stagger)

    def _release_payload(self):
        """放行投放：位置对准平台心（释放对准门通过）后才发释放指令。"""
        self.state = "DROP"
        self._publish_phase("DROP")
        self.pub_payload_cmd.publish(UInt8(data=cfg.type_to_uint8(self.mission.payload_type)))

    def _on_match(self, msg):
        if self.state == "AT_DROP" and msg.data == "MATCH":
            horiz = math.hypot(self.odom[0] - self.drop[0],
                               self.odom[1] - self.drop[1])
            # 对准判据（P0-1 第三刀）：arrive_gate 开=nav 真值到位（释放点
            # =停拉圈 ≤0.15，与判分真值对齐）；旗关=旧 est 门（漂移 ±0.2 卷积
            # 出 run15' 0.624 越线分布）
            if self._cid_arrive_gate:
                unaligned = not self._nav_arrived
            else:
                unaligned = horiz > self.release_tol
            if unaligned:
                # MATCH 一次性锁存不会重发 → 未对准时挂起，tick 侧自驱动
                # 收敛/超时兜底放行（见 AT_DROP 分支）
                self._drop_hold = True
                self._drop_align_t0 = rospy.get_time()
                self._goto((self.drop[0], self.drop[1], self.drop_hover_z))
                rospy.logwarn("drone %d MATCH but offset %.2f > %.2f, hold re-align",
                              self.drone_id, horiz, self.release_tol)
                return
            self._release_payload()
        elif self.state == "AT_DROP" and msg.data == "TIMEOUT":
            self.retry += 1
            if self.retry > self.max_retry:
                self._fail("MATCH_TIMEOUT")
            else:
                rospy.logwarn("drone %d match timeout, retry %d", self.drone_id, self.retry)
                self.state = "EXECUTE"
                self._publish_phase("EXECUTE")
                self._goto((self.drop[0], self.drop[1], self.cruise_z))
                self._pending = "DESCEND"

    def _on_abort(self, msg):
        """GCS panic：原地悬停自锁（goal=当前位置，nav P 环收敛后指令≈0）。

        与 FAILED 语义分离（FAILED=任务失败，ABORT=操作员急停）——hub/ops
        的 terminal_phases 三态都算终局，但 FAILED 计数不含 ABORT。
        """
        if self.state in ("DONE", "FAILED", "ABORTED", "RETIRED"):
            return
        self.state = "ABORTED"
        self._publish_phase("ABORT")
        self._set_landing_armed(False)
        o = self.odom
        rospy.logwarn("drone %d ABORT (%s): hover-lock at (%.2f, %.2f, %.2f)",
                      self.drone_id, msg.data, o[0], o[1], o[2])
        self._goto((o[0], o[1], o[2]))

    def _on_retire(self, msg):
        """rule_monitor 退赛裁决：复用 ABORT hover-lock，终态 RETIRED。

        与 ABORT 语义分离（RETIRED=比赛合规退赛，hub/GCS 按 RETIRED 标红），
        计分侧 S1 保留、S2 剔除（scorekeeper mark_retired 同通道）。"""
        if self.state in ("DONE", "FAILED", "ABORTED", "RETIRED"):
            return
        self.state = "RETIRED"
        self._publish_phase("RETIRED")
        self._set_landing_armed(False)
        o = self.odom
        rospy.logwarn("drone %d RETIRED (%s): hover-lock at (%.2f, %.2f, %.2f)",
                      self.drone_id, msg.data, o[0], o[1], o[2])
        self._goto((o[0], o[1], o[2]))

    def _on_payload_done(self, msg):
        if self.state == "DROP" and msg.data:
            # 返航错峰：先爬到投放点上方巡航高度悬停，drone i 延迟 i*return_stagger 后再返航
            self.return_at = rospy.get_time() + self.drone_id * self.return_stagger
            self.state = "RETURN_PENDING"
            self._publish_phase("RETURN_PENDING")
            # 闭环模式爬升悬停点东偏 1.2m：同平台排队者（WAIT→下降柱）与本机
            # 爬升柱物理错开，防"done 即放行"窗口的垂直交错相撞
            if self._cid_enabled:
                self._goto((self.drop[0] + 1.2, self.drop[1], self.cruise_z))
            else:
                self._goto((self.drop[0], self.drop[1], self.cruise_z))
            rospy.loginfo("drone %d return scheduled in %.1fs",
                          self.drone_id, self.drone_id * self.return_stagger)

    # ---------------------------------------------------------------- helpers
    def _compute_formation_offset(self):
        """非凸-α 编队协调层：计算本机相对共享航点的偏移。

        返回 3 元组 (dx, dy, dz) 或 None（未开启）。
        reference=pad0 时按「相对 0 号机起降点」保持初始相对位置；
        reference=explicit 时读取 formation.yaml 的 offsets[drone_id]（对齐真机 formation/drone*）。
        """
        fc = cfg.load("formation.yaml").get("formation", {})
        if not fc.get("enabled", False):
            return None
        ref = fc.get("reference", "pad0")
        if ref == "explicit":
            offs = fc.get("offsets", []) or []
            if self.drone_id < len(offs):
                o = offs[self.drone_id]
                return (float(o[0]), float(o[1]), float(o[2]))
            return (0.0, 0.0, 0.0)
        pads = self.scene.get_pads()
        p0 = pads[0] if pads else (0.0, 0.0)
        pi = pads[self.drone_id] if self.drone_id < len(pads) else (0.0, 0.0)
        return (float(pi[0]) - float(p0[0]), float(pi[1]) - float(p0[1]), 0.0)

    def _goto(self, xyz, formation=False):
        if formation and self.formation_offset is not None:
            xyz = (xyz[0] + self.formation_offset[0],
                   xyz[1] + self.formation_offset[1],
                   xyz[2] + self.formation_offset[2])
        g = PoseStamped()
        g.header.frame_id = "world"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y, g.pose.position.z = xyz
        g.pose.orientation.w = 1.0
        self.goal = xyz
        self.pub_goal.publish(g)

    def _pick_via_slot(self):
        """W1 via_slots：本机越界槽位。槽位 y 升序 ↔ 名次一一对应（全局确定性：
        各机本地对同一 Scene + 同一配置计算 → 同一槽位集，无需跨机通信）。
        名次键：fallback=本机参考平台 drop_y 名次（旧行为）；闭环 RECON 模式
        drop 未定 → 直接按 drone_id 对号。首次调用选定并缓存；选槽失败
        （旗开但几何无解）返回 None，调用方回退共享越界点原行为。"""
        if self._vs_slot is not None:
            return self._vs_slot
        if self._cid_enabled or self.drop is None:
            rank = self.drone_id
        else:
            keys = sorted((dp.xyz[1], dp.xyz[0]) for dp in self.scene.drop_points)
            rank = keys.index((self.drop[1], self.drop[0]))
        slots = select_via_slots(self.scene, self.scene.drone_count,
                                 slot_clear=self._vs_clear, slot_sep=self._vs_sep,
                                 z_lo=self.cruise_z - 0.5, z_hi=self.cruise_z + 0.5,
                                 zone_margin=self._vs_margin)
        if slots is None or rank >= len(slots):
            rospy.logwarn("mission_executor_node: drone %d via_slots 无可行槽位集，"
                          "回退共享越界点", self.drone_id)
            return None
        slot = slots[rank]
        self._vs_slot = slot
        rospy.loginfo("mission_executor_node: drone %d VIA-SLOT (%.2f,%.2f) clr=%.2f",
                      self.drone_id, slot[0], slot[1], slot[2])
        return slot

    def _scan_begin(self):
        """RECON 起扫：逐平台 巡航高度→平台上空 drop_hover_z 的三段腿。

        同色双机错平台起扫：第二组（i>=n）起扫平台偏移 n-1，同色对 (i,i+n)
        的起扫平台恒不同（i%n vs (i+n-1)%n，n≥2 时必异），降低同平台悬停
        汇聚概率；起扫顺序是本地确定性计算（无需跨机通信）。
        """
        n = len(self.scene.drop_points)
        start = self.drone_id % n if self.drone_id < n else (self.drone_id + n - 1) % n
        self._scan_order = [self.scene.drop_points[(start + k) % n] for k in range(n)]
        self._scan_idx = 0
        self._scan_round = 0
        self._scan_mode = "GOTO_HI"
        self._scan_leg_t0 = rospy.get_time()
        self._sel_state = bucket_select.new_state()
        bx = self._scan_order[0].xyz[0]
        by = self._scan_order[0].xyz[1]
        self._goto((bx, by, self.cruise_z))

    def _scan_advance(self, skip_reason):
        """推进到下一扫描平台（全扫完→重扫或判失败）。"""
        self._scan_idx += 1
        if self._scan_idx >= len(self._scan_order):
            self._scan_round += 1
            if self._scan_round > self._cid_rounds:
                self._fail("COLOR_UNRESOLVED")
                return False
            self._scan_idx = 0
            rospy.logwarn("drone %d RECON round %d exhausted (%s), rescanning",
                          self.drone_id, self._scan_round, skip_reason)
        nxt = self._scan_order[self._scan_idx]
        self._scan_mode = "GOTO_HI"
        self._scan_leg_t0 = rospy.get_time()
        self._goto((nxt.xyz[0], nxt.xyz[1], self.cruise_z))
        return True

    def _recon_tick(self, d):
        """RECON 子阶段一拍：GOTO_HI(巡航到位) → GOTO_LO(降平台上空) →
        SCAN(悬停 dwell，bucket_scan_step 连续命中箱色即锁平台) →
        （同平台被占→WAIT_HI 避让等待，占用者完成后再降）。
        GOTO 腿带超时看门狗（plan_fail 类活锁兜底：跳平台继续扫）。"""
        dp = self._scan_order[self._scan_idx]
        if self._scan_mode in ("GOTO_HI", "GOTO_LO"):
            if rospy.get_time() - self._scan_leg_t0 > self._cid_leg_timeout:
                rospy.logwarn("drone %d RECON leg %s to platform %d timeout, skipping",
                              self.drone_id, self._scan_mode, dp.id)
                self._scan_advance("leg_timeout")
                return
            # 让位仲裁（run10 法证）：下降全程（未进末段）若他机已在平台
            # 低层占位 → 本机升等待位避让，串行投放。进末段（stage2）后
            # 不再让（本机已是最低层，SCAN 马上开扫）。
            if self._scan_mode == "GOTO_LO" and self._lo_stage < 2:
                occ = self._low_occupant(dp)
                if occ is not None:
                    self._enter_wait(dp, "low-occupied by", occ)
                    return
            if self._scan_mode == "GOTO_HI":
                if d < self.drop_tol:
                    self._scan_mode = "GOTO_LO"
                    self._scan_leg_t0 = rospy.get_time()
                    # 分级下降：先到平台上空 1.2m 缓冲层，经 drop_mid_z 中继
                    # 缓降 drop_hover_z。单段直降 2.5→0.55 的 P 环超调会
                    # 扎进贴地碰撞区（z≈0.35 触平台薄盘弹跳，P4 三跑法证）
                    self._lo_stage = 0
                    self._goto((dp.xyz[0], dp.xyz[1], self.identify_z))
                    rospy.loginfo("drone %d RECON over platform %d (%s), descending",
                                  self.drone_id, dp.id, dp.color)
                return
            # GOTO_LO 分级：stage0=identify_z 缓冲层 → stage1=drop_mid_z
            # 中继 → stage2=缓降平台上空。中继把末段行程压到 ~0.35m，P 环
            # 过冲 ~0.1m（min z≈0.45）不再触平台薄盘碰撞面（z<0.40 且
            # 水平<0.95，run4/5 簇1 法证）；drop_mid_z<=0 退回旧两段
            if self._lo_stage == 0:
                if abs(self.odom[2] - self.identify_z) < 0.35:
                    self._lo_stage = 1
                    self._goto((dp.xyz[0], dp.xyz[1],
                                self.drop_mid_z if self.drop_mid_z > 0
                                else self.drop_hover_z))
                return
            if self._lo_stage == 1 and self.drop_mid_z > 0:
                # 中继到位须自上方入带（过冲深处不接段，P 拉回再接）；
                # 末段放行加水平门（run9 法证）：段转移原只看 z，水平差
                # 0.85m 时连跳 stage2 → 斜线俯冲过冲触地（d0 弹开弹起
                # 1.61m）。水平 <0.6（平台投影内）才放末段，未对准先在
                # 1.0 中继层平移。腿超时 45s 兜底防分离推挤卡中继层。
                if abs(self.odom[2] - self.drop_mid_z) < 0.25 \
                        and self.odom[2] >= self.drop_mid_z - 0.10 \
                        and math.hypot(self.odom[0] - dp.xyz[0],
                                       self.odom[1] - dp.xyz[1]) < 0.6:
                    self._lo_stage = 2
                    self._goto((dp.xyz[0], dp.xyz[1], self.drop_hover_z))
                return
            # 精投放到位判据=水平（判平台上空径是水平投影），z 只需进悬停带
            # 且自上方入带（下限门防过冲深处开扫）：MATCH 一次性锁存，
            # 释放点=此刻悬停收敛点，宽容差会让释放偏移骑在判平台线 0.6 上
            # 入场门（P0-1 三刀合流）：arrive_gate 开=nav 真值到位 + est 窗稳
            # 定（到位后 nav 停拉，xy 窗必然稳定，双门互为防抖）；旗关=旧
            # z 下限门（自上方入带防过冲深处开扫，run9）+est 窗
            if self._cid_arrive_gate:
                entry_ok = self._nav_arrived and self._xy_settled()
            else:
                entry_ok = (self.odom[2] >= self.drop_hover_z - 0.10
                            and self._xy_settled())
            horiz = math.hypot(self.odom[0] - dp.xyz[0],
                               self.odom[1] - dp.xyz[1])
            if horiz < self.hover_tol \
                    and abs(self.odom[2] - self.drop_hover_z) < 0.3 \
                    and entry_ok:
                self._scan_mode = "SCAN"
                self._scan_until = rospy.get_time() + self._cid_dwell
                self._sel_state = bucket_select.new_state()
            elif rospy.get_time() - self._scan_leg_t0 > 10.0:
                # 入场门诊断（run21 法证仪，行为零变更）：腿开始 10s 后
                # 每 5s 打印各门状态——d1 独占平台 30s 仍锁不上，需要
                # 知道卡在哪扇门（arrived/settled/horiz/z/stage）。
                rospy.loginfo_throttle(
                    5.0,
                    "drone %d RECON entry-gate diag plat %d st=%s/%d "
                    "horiz=%.2f/%.2f z=%.2f arrived=%s settled=%s",
                    self.drone_id, dp.id, self._scan_mode, self._lo_stage,
                    horiz, self.hover_tol, self.odom[2], self._nav_arrived,
                    self._xy_settled())
            return
        # WAIT_HI：平台被先到者认领（未完成投放）→ 在平台侧偏移点+巡航高度等待
        #（垂直分层 + 水平偏移 1.2m 双分离，不与平台上空 0.55m 悬停机抢空间）
        if self._scan_mode == "WAIT_HI":
            if rospy.get_time() - self._scan_leg_t0 > self._cid_leg_timeout * 2:
                # 占用者长期不完成（卡死/失败）→ 放弃本平台走重扫流程
                rospy.logwarn("drone %d RECON wait for platform %d timeout, rescanning",
                              self.drone_id, dp.id)
                self._scan_advance("wait_timeout")
                return
            # 退出条件：认领者清空 且 低层占用者离开（让位型等待的退出，
            # 占用者 LOCK/完成/腿超时离开三者任一发生即放行）
            if self._bucket_holder(dp.id) is None \
                    and self._low_occupant(dp) is None:
                rospy.loginfo("drone %d RECON platform %d freed, descending",
                              self.drone_id, dp.id)
                self._scan_mode = "GOTO_LO"
                self._scan_leg_t0 = rospy.get_time()
                self._lo_stage = 0     # 分级下降从缓冲层重新开始
                self._goto((dp.xyz[0], dp.xyz[1], self.identify_z))
            return
        # SCAN：悬停采集颜色检测流
        box_u8 = cfg.color_to_uint8(self.box_color)
        self._sel_state, decision = bucket_select.bucket_scan_step(
            self.det_color, self.det_conf_c, box_u8,
            min_conf=self._cid_min_conf, n_frames=self._cid_nframes,
            state=self._sel_state)
        now = rospy.get_time()
        if now >= self._scan_until and decision == "LOCK":
            holder = self._bucket_holder(dp.id)
            if holder is not None:
                self._enter_wait(dp, "held by", holder)
                return
            self.selected_dp = dp
            self.drop = (dp.xyz[0], dp.xyz[1])
            self._drop_hold = False   # 重锁平台清残留对准门（MATCH_TIMEOUT
            self._drop_align_t0 = 0.0  # 重试重降后旧 hold/时刻会立即误放行）
            self.pub_selected.publish(Int32(data=dp.id))
            # SELECTED 上 task_update：GCS 指派栏锁定瞬间即显真桶（此前只有
            # LANDED 带桶号）。消费端 scorekeeper(L157)/nav(L487) 均按 state
            # 白名单过滤，SELECTED 对它们零副作用。
            tu = TaskUpdate()
            tu.drone_id = self.drone_id
            tu.mission_state = "SELECTED"
            if self.mission is not None:
                tu.payload_type = self.mission.payload_type
            tu.drop_point_id = dp.id
            self.pub_task_update.publish(tu)
            self._pending = None
            self.state = "AT_DROP"
            self._publish_phase("AT_DROP")
            self.pub_at_drop.publish(Bool(data=True))
            rospy.loginfo("drone %d RECON LOCK platform %d (%s) at (%.1f, %.1f)",
                          self.drone_id, dp.id, dp.color, dp.xyz[0], dp.xyz[1])
            return
        if now >= self._scan_until:
            # 本平台 dwell 结束仍未锁定 → 下一平台；全扫完 → 重扫或判失败
            self._scan_advance("dwell_exhausted")

    def _set_landing_armed(self, armed):
        """落地门控开关。armed 生存期严格限定最后下降段：
        touchdown/失败/急停立即复位，防"漏关放过贴地碰撞"。"""
        self.pub_landing_armed.publish(Bool(data=bool(armed)))

    def _publish_phase(self, ph):
        self.pub_phase.publish(String(data=ph))

    def _phase_heartbeat(self, _evt):
        """1Hz 相位心跳：晚到订阅者（GCS agent 等）不依赖一次性锁存的
        连接竞态——地面站按数据年龄判活，心跳比锁存一次性更稳。"""
        self._publish_phase(self.state)

    def _fail(self, reason):
        self.state = "FAILED"
        self._publish_phase("FAILED")
        self._set_landing_armed(False)
        rospy.logerr("drone %d FAILED: %s", self.drone_id, reason)

    # ---------------------------------------------------------------- tick
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        if self.state in ("ABORTED", "RETIRED"):
            return  # 急停/退赛后状态机冻结，不再发任何新航点/投放指令
        if self.state == "TAKEOFF_PENDING":
            if self.takeoff_at is not None and rospy.get_time() >= self.takeoff_at:
                self.state = "TAKEOFF"
                self._publish_phase("TAKEOFF")
                self._goto((self.pad[0], self.pad[1], self.takeoff_hover_z))
            return
        if self.state == "EXECUTE_PENDING":
            if self.enter_at is not None and rospy.get_time() >= self.enter_at:
                self.state = "EXECUTE"
                self._publish_phase("EXECUTE")
                self.retry = 0
                self.crossed_zone = False
                self.pub_crossed.publish(Bool(data=False))
                # W1 via_slots：逐机散点槽位替代共享穿越区重心（matrix_w1b 后默认开）。
                # 槽位本身已散开承担编队展开职能，不再叠加 formation 偏移；
                # 旗关或无可行槽位集时回退原共享点 + formation 偏移（逐位原行为）。
                slot = self._pick_via_slot() if self._vs_enabled else None
                if slot is not None:
                    self._goto((slot[0], slot[1], self.cruise_z))
                    self._pending = "CROSS_ZONE"
                    rospy.loginfo("drone %d heading to via slot (%.1f, %.1f)",
                                  self.drone_id, slot[0], slot[1])
                else:
                    # 先飞向穿越区中心（编队协调层：加每机偏移展开成队形），进入后再转向投放点
                    cz = self.scene.crossing_zone_center
                    self._goto((cz[0], cz[1], self.cruise_z), formation=True)
                    self._pending = "CROSS_ZONE"
                    rospy.loginfo("drone %d heading to crossing zone (%.1f, %.1f)",
                                  self.drone_id, cz[0], cz[1])
            return
        if self.state == "RETURN_PENDING":
            if self.return_at is not None and rospy.get_time() >= self.return_at:
                self.state = "RETURN"
                self._publish_phase("RETURN")
                # 两段式返航：先以巡航高度回到降落区上空，到位再降到悬停高度。
                # 避免穿越林区时提前降到悬停高度(1.5m)在林缘死点卡住。
                self._goto((self.pad[0], self.pad[1], self.cruise_z))
                self._pending = "DESCEND_PAD"
                rospy.loginfo("drone %d return start (cruise to pad)", self.drone_id)
            return
        if self.goal is None:
            return

        # 在执行阶段检查是否进入穿越区
        if self.state == "EXECUTE":
            if not self.crossed_zone and self.scene.in_crossing_zone((self.odom[0], self.odom[1])):
                self.crossed_zone = True
                self.pub_crossed.publish(Bool(data=True))
                rospy.loginfo("drone %d crossed zone", self.drone_id)

        d = np.linalg.norm(np.array(self.goal) - np.array(self.odom))
        if self.state == "TAKEOFF" and d < self.arrival_tol:
            self.state = "TAKEOFF_DONE"
            self._publish_phase("TAKEOFF_DONE")
            rospy.loginfo("drone %d takeoff done", self.drone_id)
            self.goal = None
        elif self.state == "EXECUTE":
            if getattr(self, "_pending", None) == "CROSS_ZONE":
                # 已到达/进入穿越区后：闭环→RECON 侦察择平台；fallback→转向参考投放点
                if self.crossed_zone:
                    if self._cid_enabled:
                        self._pending = "RECON"
                        self._scan_begin()
                        rospy.loginfo("drone %d crossed zone, RECON scan begin",
                                      self.drone_id)
                    else:
                        self._pending = "DESCEND"
                        self._goto((self.drop[0], self.drop[1], self.cruise_z))
                        rospy.loginfo("drone %d crossed zone, heading to drop",
                                      self.drone_id)
                elif d < self.drop_tol:
                    # 没进入穿越区却到了中心点附近（理论不应发生），强制失败
                    self._fail("CROSSING_ZONE_MISSED")
                    return
            elif getattr(self, "_pending", None) == "RECON":
                self._recon_tick(d)
            elif getattr(self, "_pending", None) == "DESCEND":
                if d < self.drop_tol:
                    # 强制穿越区检查
                    if not self.crossed_zone:
                        self._fail("CROSSING_ZONE_MISSED")
                        return
                    self._pending = None
                    self.state = "AT_DROP"
                    self._publish_phase("AT_DROP")
                    self.pub_at_drop.publish(Bool(data=True))
                    z = self.drop_hover_z if self._cid_enabled else self.identify_z
                    self._goto((self.drop[0], self.drop[1], z))
        elif self.state == "RETURN":
            if getattr(self, "_pending", None) == "DESCEND_PAD":
                # 已到降落区上空(巡航高度)，下降到悬停高度（降落区无树，安全）
                if d < self.arrival_tol:
                    self._pending = "TOUCHDOWN"
                    self._goto((self.pad[0], self.pad[1], self.takeoff_hover_z))
                    rospy.loginfo("drone %d at pad, descending to hover", self.drone_id)
            elif getattr(self, "_pending", None) == "TOUCHDOWN":
                # 悬停高度到位 → 最后下降段先 armed（世界侧豁免地面子句）再降
                if d < self.drop_tol:
                    self._pending = "LANDING"
                    self._set_landing_armed(True)
                    self._goto((self.pad[0], self.pad[1], self.touchdown_z))
                    rospy.loginfo("drone %d landing armed, descending to %.2f",
                                  self.drone_id, self.touchdown_z)
            elif getattr(self, "_pending", None) == "LANDING":
                # 真落地判定：按高度阈值（非 3D 距离）——goal 在 touchdown_z，
                # 3D 距离判据会停在目标上方提前触发"假落地"（实测 z=0.44）
                if self.odom[2] <= self.touchdown_z + 0.15 and \
                        math.hypot(self.odom[0] - self.pad[0],
                                   self.odom[1] - self.pad[1]) < 0.8:
                    self._set_landing_armed(False)
                    tu = TaskUpdate()
                    tu.drone_id = self.drone_id
                    tu.mission_state = "LANDED"
                    tu.drop_ok = 1
                    if self.mission is not None:
                        tu.payload_type = self.mission.payload_type
                        tu.drop_point_id = (self.selected_dp.id
                                            if self.selected_dp is not None
                                            else self.mission.drop_point_id)
                    self.pub_task_update.publish(tu)
                    self.state = "DONE"
                    self._publish_phase("DONE")
                    rospy.loginfo("drone %d TOUCHDOWN z=%.2f, mission DONE",
                                  self.drone_id, self.odom[2])
                    self.goal = None
                    self.pub_at_drop.publish(Bool(data=False))
        elif self.state == "AT_DROP":
            # 释放对准门 tick 侧自驱动：type_match MATCH 一次性锁存不会
            # 重发，对准收敛或超时兜底都在这里闭环放行
            if self._drop_hold:
                horiz = math.hypot(self.odom[0] - self.drop[0],
                                   self.odom[1] - self.drop[1])
                now = rospy.get_time()
                if self._cid_arrive_gate:
                    aligned = self._nav_arrived
                else:
                    aligned = horiz <= self.release_tol
                if aligned:
                    self._drop_hold = False
                    self._release_payload()
                elif now - self._drop_align_t0 >= self.release_align_timeout:
                    self._drop_hold = False
                    rospy.logwarn("drone %d align timeout, force release "
                                  "(offset %.2f)", self.drone_id, horiz)
                    self._release_payload()
                elif now - self._drop_step_t0 >= 1.5:
                    # 对准期兜底重拉（P0-1）：nav 到位停拉圈已按高度特化
                    # （goal z<1.5 收紧 0.15，run13/14 法证——0.4 圈内均匀
                    # 随机停点+漂移=offset 0.26-0.58 分布真身）。漂移把估计
                    # 位置推出小圈/放行门外时，重发平台心 goal 重启 nav 小
                    # 修正（真值 offset 收敛到 ~|漂移| 下界 ~0.1-0.3）。
                    # 1.5s 节流防打断拉动中的 nav。
                    self._goto((self.drop[0], self.drop[1], self.drop_hover_z))
                    self._drop_step_t0 = now
                    rospy.loginfo("drone %d align re-pull, offset %.2f",
                                  self.drone_id, horiz)


if __name__ == "__main__":
    try:
        node = MissionExecutor()
        node.run()
    except rospy.ROSInterruptException:
        pass
