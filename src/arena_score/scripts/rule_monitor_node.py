#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_score::rule_monitor_node — 比赛合规监视（全局 1 份，20Hz）。

科目三退赛条款（判定核心=rules.py 纯函数，P8 自检 B 组锚点）：
  * 出界：真值 odom 离开 geofence 多边形持续 >out_of_bounds_grace_s
    → RETIRE("OUT_OF_BOUNDS")。geofence 西侧覆盖起降区（pads 在围栏
    以西，东/北/南以围栏内缘为界）。
  * 计划外落地：pad 多边形外贴地（z<=ground_contact_z）且未 armed，
    防抖 unauthorized_landing_grace_s → RETIRE("UNAUTHORIZED_LANDING")；
    armed+pad 内 = 计划内 touchdown 事件 /zx2026/touchdown/<i>。
  * 超高：z>max_height_m 持续 height_grace_s → RETIRE("OVER_HEIGHT")。
  * 返程穿林：payload/done → DONE 的轨迹未命中 return_corridor
    （默认=树林外接框外扩 corridor_margin_m）→ 发 /zx2026/rule/corridor_miss
    剥 S2 计数（scorekeeper 消费）；retire_on_corridor_miss=true 才退赛。
退赛动作：/drone_<i>/rule/retire (String reason, latch) → executor
hover-lock 终态 RETIRED + scorekeeper mark_retired（S1 保留、S2 剔除）
+ TaskUpdate(RETIRED)。selftest_inject=true 时 /zx2026/rule/inject
"OOB:2"/"LAND:2"/"HIGH:2" 直接注入退赛（默认关）。
"""
import rospy
from std_msgs.msg import Bool, String, Int32
from nav_msgs.msg import Odometry

from zx2026_common import config as cfg
from zx2026_common import rules
from zx2026_common.scene import Scene
from zx2026_common.msg import TaskUpdate

DT = 0.05  # 20Hz 判定节拍


class RuleMonitorNode:
    def __init__(self):
        rospy.init_node("rule_monitor_node", anonymous=False)
        rules_cfg = cfg.load("competition_rules.yaml")
        rc = rules_cfg.get("rule", {}) or {}
        self.enabled = bool(rc.get("enabled", True))
        gf = rc.get("geofence") or [[-27.0, -15.35], [22.5, -15.35],
                                    [22.5, 15.35], [-27.0, 15.35]]
        self.geofence = [(float(p[0]), float(p[1])) for p in gf]
        self.oob_grace = float(rc.get("out_of_bounds_grace_s", 10.0))
        self.contact_z = float(rc.get("ground_contact_z", 0.30))
        self.land_grace = float(rc.get("unauthorized_landing_grace_s", 1.5))
        self.max_z = float(rc.get("max_height_m", 5.0))
        self.high_grace = float(rc.get("height_grace_s", 2.0))
        self.return_through_forest = bool(rc.get("return_through_forest", True))
        self.retire_on_miss = bool(rc.get("retire_on_corridor_miss", False))
        self.corridor_margin = float(rc.get("corridor_margin_m", 1.0))
        self.selftest_inject = bool(rc.get("selftest_inject", False))

        self.scene = Scene()
        self.n = self.scene.drone_count
        self.pad_poly = [(float(p[0]), float(p[1]))
                         for p in self.scene.zones["takeoff"].polygon]
        # 返程走廊：默认=树干外接框 ± corridor_margin（正=外扩容忍贴林缘，
        # 负=内缩严格）；yaml 显式给 return_corridor 时以配置为准
        rc_poly = rc.get("return_corridor")
        if rc_poly:
            self.corridor = [(float(p[0]), float(p[1])) for p in rc_poly]
        else:
            self.corridor = self._forest_bbox(self.corridor_margin)

        self._assert_fences()

        self.pos = {}
        self.armed = {i: False for i in range(self.n)}
        self.phase = {i: "" for i in range(self.n)}
        self.dropped = {i: False for i in range(self.n)}
        self.oob_acc = {i: 0.0 for i in range(self.n)}
        self.land_acc = {i: 0.0 for i in range(self.n)}
        self.high_acc = {i: 0.0 for i in range(self.n)}
        self.retired = set()
        self.td_published = set()
        self.miss_sent = set()
        self.done_checked = set()  # DONE 心跳会重发，走廊检查每机一次
        self.return_path = {i: [] for i in range(self.n)}

        self.pub_task = rospy.Publisher("/zx2026/task_update", TaskUpdate, queue_size=5)
        self.pub_miss = rospy.Publisher("/zx2026/rule/corridor_miss", Int32,
                                        queue_size=1, latch=True)
        self.pub_td = {}
        self.pub_retire = {}
        for i in range(self.n):
            ns = "/drone_%d" % i
            self.pub_td[i] = rospy.Publisher("/zx2026/touchdown/%d" % i, Bool,
                                             queue_size=1, latch=True)
            self.pub_retire[i] = rospy.Publisher(ns + "/rule/retire", String,
                                                 queue_size=1, latch=True)
            rospy.Subscriber(ns + "/odom", Odometry,
                             lambda m, i=i: self._on_odom(i, m))
            rospy.Subscriber(ns + "/mission/landing_armed", Bool,
                             lambda m, i=i: self._on_armed(i, m))
            rospy.Subscriber(ns + "/mission/phase", String,
                             lambda m, i=i: self._on_phase(i, m))
            rospy.Subscriber(ns + "/payload/done", Bool,
                             lambda m, i=i: self._on_done(i, m))
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber("/zx2026/rule/inject", String, self._on_inject)

        self.active = False
        if self.enabled:
            rospy.Timer(rospy.Duration(DT), self._tick)
        rospy.loginfo("rule_monitor: enabled=%s oob_grace=%.1fs corridor=%s",
                      self.enabled, self.oob_grace,
                      "auto(%.1f)" % self.corridor_margin if self.corridor is None
                      else str(self.corridor))

    # ---------------------------------------------------------------- helpers
    def _forest_bbox(self, margin):
        xs = [o.cx for o in self.scene.obstacles if o.kind == "tree"]
        ys = [o.cy for o in self.scene.obstacles if o.kind == "tree"]
        if not xs:
            return None
        x0, x1 = min(xs) - margin, max(xs) + margin
        y0, y1 = min(ys) - margin, max(ys) + margin
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def _assert_fences(self):
        """启动自检（world_builder fence 18.6 vs yaml 22.5 失和教训）：
        东/北/南三面围栏须覆盖 geofence 对应边界线，geofence 须含起降区。
        只 logerr 不崩——监视器失效好过整栈起不来。"""
        gx = [p[0] for p in self.geofence]
        gy = [p[1] for p in self.geofence]
        east, west = max(gx), min(gx)
        north, south = max(gy), min(gy)
        obs = self.scene.obstacles
        checks = [
            ("east", any(o.kind == "fence" and o.lo[0] <= east <= o.hi[0]
                         for o in obs)),
            ("north", any(o.kind == "fence" and o.lo[1] <= north <= o.hi[1]
                          for o in obs)),
            ("south", any(o.kind == "fence" and o.lo[1] <= south <= o.hi[1]
                          for o in obs)),
            ("takeoff", all(west <= p[0] <= east and south <= p[1] <= north
                            for p in self.pad_poly)),
        ]
        for name, ok in checks:
            if not ok:
                rospy.logerr("rule_monitor: geofence/fence mismatch on %s "
                             "side — check scene_topology vs rule.geofence", name)

    # ---------------------------------------------------------------- callbacks
    def _on_odom(self, i, msg):
        p = msg.pose.pose.position
        self.pos[i] = (p.x, p.y, p.z)
        if self.dropped[i] and not self.phase[i].startswith("RETURN_PEND"):
            # 返程轨迹采样（payload/done 起算；20Hz 上限 3 万点内）
            if len(self.return_path[i]) < 40000:
                self.return_path[i].append((p.x, p.y))

    def _on_armed(self, i, msg):
        self.armed[i] = bool(msg.data)

    def _on_phase(self, i, msg):
        ph = msg.data
        if ph == "DONE" and self.dropped[i]:
            self._check_corridor(i)
        self.phase[i] = ph

    def _on_done(self, i, msg):
        if msg.data:
            self.dropped[i] = True

    def _on_state(self, msg):
        if msg.data == "P4_EXECUTE":
            self.active = True

    def _on_inject(self, msg):
        if not self.selftest_inject:
            rospy.logwarn_throttle(30, "rule_monitor: inject ignored "
                                       "(rule.selftest_inject=false)")
            return
        try:
            kind, did = msg.data.split(":")
            i = int(did)
        except ValueError:
            rospy.logwarn("rule_monitor: bad inject %r", msg.data)
            return
        reasons = {"OOB": "OUT_OF_BOUNDS", "LAND": "UNAUTHORIZED_LANDING",
                   "HIGH": "OVER_HEIGHT"}
        if kind in reasons:
            rospy.logwarn("rule_monitor: INJECTED retire drone %d (%s)", i, kind)
            self._retire(i, reasons[kind])

    # ---------------------------------------------------------------- tick
    def _tick(self, _evt):
        if not self.active:
            return
        for i in range(self.n):
            if i in self.retired:
                continue
            p = self.pos.get(i)
            if p is None:
                continue
            x, y, z = p
            # 1) 出界（geofence 含起降区西侧开口）
            outside = not rules.point_in_poly(self.geofence, x, y)
            self.oob_acc[i], tripped = rules.oob_tick(
                outside, self.oob_acc[i], self.oob_grace, DT)
            if tripped:
                self._retire(i, "OUT_OF_BOUNDS")
                continue
            # 2) 落地合规（pad 外贴地未 armed 防抖；armed+pad 内=计划内）
            in_pad = rules.point_in_poly(self.pad_poly, x, y)
            self.land_acc[i], verdict = rules.landing_verdict(
                z, self.contact_z, self.armed[i], in_pad,
                self.land_acc[i], self.land_grace, DT)
            if verdict == "unauthorized":
                self._retire(i, "UNAUTHORIZED_LANDING")
                continue
            if verdict == "planned" and i not in self.td_published:
                self.td_published.add(i)
                self.pub_td[i].publish(Bool(data=True))
                rospy.loginfo("rule_monitor: drone %d planned touchdown "
                              "(z=%.2f)", i, z)
            # 3) 超高
            self.high_acc[i], tripped = rules.height_verdict(
                z, self.max_z, self.high_acc[i], self.high_grace, DT)
            if tripped:
                self._retire(i, "OVER_HEIGHT")

    # ---------------------------------------------------------------- corridor
    def _check_corridor(self, i):
        if not self.return_through_forest or self.corridor is None:
            return
        if i in self.miss_sent or i in self.done_checked:
            return
        self.done_checked.add(i)
        if rules.corridor_hit(self.return_path[i], self.corridor):
            rospy.loginfo("rule_monitor: drone %d return corridor HIT", i)
            return
        self.miss_sent.add(i)
        rospy.logwarn("rule_monitor: drone %d return corridor MISS — "
                      "S2 count stripped", i)
        self.pub_miss.publish(Int32(data=i))
        if self.retire_on_miss:
            self._retire(i, "CORRIDOR_MISS")

    # ---------------------------------------------------------------- retire
    def _retire(self, i, reason):
        if i in self.retired:
            return
        self.retired.add(i)
        rospy.logwarn("rule_monitor: drone %d RETIRED (%s)", i, reason)
        self.pub_retire[i].publish(String(data=reason))
        tu = TaskUpdate()
        tu.drone_id = i
        tu.mission_state = "RETIRED"
        self.pub_task.publish(tu)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RuleMonitorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
