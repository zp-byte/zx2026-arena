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
状态机: IDLE → TAKEOFF → TAKEOFF_DONE → EXECUTE_PENDING → EXECUTE → AT_DROP → DROP → RETURN → DONE/FAILED
"""
import math

import rospy
import numpy as np
from std_msgs.msg import String, Bool, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from zx2026_common import config as cfg
from zx2026_common import scene as sc
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission


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
        self.arrival_tol = float(h.get("area_arrival_tol", 2.0))
        self.drop_tol = float(h.get("drop_arrival_tol", 0.8))
        self.max_vel = float(rules["motion"].get("default_max_vel", 2.0))
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
        self.drop = (0.0, 0.0)
        self.retry = 0
        self.max_retry = 2

        # 穿越区强制通过
        self.crossed_zone = False

        self.pub_goal = rospy.Publisher(ns + "/planning/goal", PoseStamped, queue_size=10)
        self.pub_phase = rospy.Publisher(ns + "/mission/phase", String, queue_size=1, latch=True)
        self.pub_at_drop = rospy.Publisher(ns + "/mission/at_drop", Bool, queue_size=1, latch=True)
        self.pub_payload_cmd = rospy.Publisher(ns + "/payload/command", UInt8, queue_size=10)
        self.pub_crossed = rospy.Publisher(ns + "/mission/crossed_zone", Bool, queue_size=1, latch=True)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.Subscriber("/zx2026/mission/%d" % self.drone_id, Mission, self._on_mission)
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber(ns + "/match/result", String, self._on_match)
        rospy.Subscriber(ns + "/payload/done", Bool, self._on_payload_done)
        # GCS panic 通道（第 4 步 ②）：全局急停，全部 executor 各自进入 ABORT 终态
        rospy.Subscriber("/zx2026/abort", String, self._on_abort)

        self._publish_phase("IDLE")
        self.pub_crossed.publish(Bool(data=False))
        rospy.loginfo("mission_executor_node: drone %d", self.drone_id)

    # ---------------------------------------------------------------- callbacks
    def _on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.pose.position.z)

    def _on_mission(self, msg):
        if self.state != "IDLE":
            return
        self.mission = msg
        pads = self.scene.get_pads()
        if self.drone_id < len(pads):
            self.pad = pads[self.drone_id]
        else:
            self.pad = (0.0, 0.0)
        for dp in self.scene.drop_points:
            if dp.id == msg.drop_point_id:
                self.drop = (dp.xyz[0], dp.xyz[1])
        self.crossed_zone = False
        self.pub_crossed.publish(Bool(data=False))
        rospy.loginfo("drone %d got mission: payload=%s drop_point=%d",
                      self.drone_id, msg.payload_type, msg.drop_point_id)

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

    def _on_match(self, msg):
        if self.state == "AT_DROP" and msg.data == "MATCH":
            self.state = "DROP"
            self._publish_phase("DROP")
            self.pub_payload_cmd.publish(UInt8(data=cfg.type_to_uint8(self.mission.payload_type)))
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
        if self.state in ("DONE", "FAILED", "ABORTED"):
            return
        self.state = "ABORTED"
        self._publish_phase("ABORT")
        o = self.odom
        rospy.logwarn("drone %d ABORT (%s): hover-lock at (%.2f, %.2f, %.2f)",
                      self.drone_id, msg.data, o[0], o[1], o[2])
        self._goto((o[0], o[1], o[2]))

    def _on_payload_done(self, msg):
        if self.state == "DROP" and msg.data:
            # 返航错峰：先爬到投放点上方巡航高度悬停，drone i 延迟 i*return_stagger 后再返航
            self.return_at = rospy.get_time() + self.drone_id * self.return_stagger
            self.state = "RETURN_PENDING"
            self._publish_phase("RETURN_PENDING")
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
        """W1 via_slots：本机越界槽位。全部机的 drop_y 名次 ↔ 槽位 y 升序
        一一对应（全局确定性：各机本地对同一 Scene + 同一配置计算 → 同一
        槽位集，无需跨机通信）。首次调用选定并缓存；选槽失败（旗开但几何
        无解）返回 None，调用方回退共享越界点原行为。"""
        if self._vs_slot is not None:
            return self._vs_slot
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

    def _publish_phase(self, ph):
        self.pub_phase.publish(String(data=ph))

    def _fail(self, reason):
        self.state = "FAILED"
        self._publish_phase("FAILED")
        rospy.logerr("drone %d FAILED: %s", self.drone_id, reason)

    # ---------------------------------------------------------------- tick
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        if self.state == "ABORTED":
            return  # 急停后状态机冻结，不再发任何新航点/投放指令
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
                # 已到达/进入穿越区后，转向投放点
                if self.crossed_zone:
                    self._pending = "DESCEND"
                    self._goto((self.drop[0], self.drop[1], self.cruise_z))
                    rospy.loginfo("drone %d crossed zone, heading to drop", self.drone_id)
                elif d < self.drop_tol:
                    # 没进入穿越区却到了中心点附近（理论不应发生），强制失败
                    self._fail("CROSSING_ZONE_MISSED")
                    return
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
                    self._goto((self.drop[0], self.drop[1], self.identify_z))
        elif self.state == "RETURN":
            if getattr(self, "_pending", None) == "DESCEND_PAD":
                # 已到降落区上空(巡航高度)，下降到悬停高度（降落区无树，安全）
                if d < self.arrival_tol:
                    self._pending = None
                    self._goto((self.pad[0], self.pad[1], self.takeoff_hover_z))
                    rospy.loginfo("drone %d at pad, descending to hover", self.drone_id)
            elif d < self.drop_tol:
                # 已降到悬停高度并到位，任务完成
                self.state = "DONE"
                self._publish_phase("DONE")
                rospy.loginfo("drone %d mission DONE", self.drone_id)
                self.goal = None
                self.pub_at_drop.publish(Bool(data=False))


if __name__ == "__main__":
    try:
        node = MissionExecutor()
        node.run()
    except rospy.ROSInterruptException:
        pass
