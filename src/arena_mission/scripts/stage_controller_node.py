#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::stage_controller_node — 比赛阶段状态机（全局 1 份）。

P1_INIT → P2_WAIT → P3_TAKEOFF → P4_EXECUTE → P5_RETURN → DONE
  * P1_INIT：启动即发布（任务已由 task_generator 提前 latch 发布）
  * P2_WAIT：收到 /zx2026/fleet_ready 后进入
  * P3_TAKEOFF：收到 /zx2026/start (service) 后触发，等待所有机 TAKEOFF_DONE
  * P4_EXECUTE：所有机起飞完成后触发，等待所有机 DONE/FAILED
  * P5_RETURN：全部结束（或阶段超时）后触发
  * DONE：发布结束
"""
import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse


class StageController:
    def __init__(self):
        rospy.init_node("stage_controller_node", anonymous=True)
        from zx2026_common import config as cfg
        self.rules = cfg.load("competition_rules.yaml")
        self.n_drones = len(cfg.load("fleet.yaml").get("drones", []))

        self.state = "P1_INIT"
        self.takeoff_done = set()
        self.mission_done = set()
        self.mission_failed = set()
        self.phase_start = rospy.get_time()
        self.phase_timeout = float(self.rules.get("phase_timeout_s", 30.0))
        # P4 执行阶段用独立超时（默认整场时限），避免 30s 掐断真实任务
        self.execute_timeout = float(self.rules.get(
            "execute_timeout_s", self.rules.get("time_limit_s", 600.0)))

        self.pub_state = rospy.Publisher("/zx2026/state", String, queue_size=1, latch=True)
        self.pub_timeout = rospy.Publisher("/zx2026/phase_timeout", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/zx2026/fleet_ready", Bool, self._on_fleet_ready)
        for i in range(self.n_drones):
            rospy.Subscriber("/drone_%d/mission/phase" % i, String,
                             lambda m, i=i: self._on_phase(i, m.data))
        self.srv_start = rospy.Service("/zx2026/start", Trigger, self._on_start)

        self._set_state("P1_INIT")
        rospy.loginfo("stage_controller_node: P1_INIT, waiting for fleet + /zx2026/start")

    # ---------------------------------------------------------------- callbacks
    def _on_fleet_ready(self, msg):
        if msg.data and self.state == "P1_INIT":
            self._set_state("P2_WAIT")
            rospy.loginfo("fleet ready → P2_WAIT")

    def _on_start(self, req):
        if self.state == "P2_WAIT":
            self.phase_start = rospy.get_time()
            self._set_state("P3_TAKEOFF")
            rospy.loginfo("/zx2026/start → P3_TAKEOFF")
            return TriggerResponse(success=True, message="started")
        return TriggerResponse(success=False,
                               message="start only allowed in P2_WAIT (now %s)" % self.state)

    def _on_phase(self, i, phase):
        if phase == "TAKEOFF_DONE":
            self.takeoff_done.add(i)
            if len(self.takeoff_done) == self.n_drones and self.state == "P3_TAKEOFF":
                self.phase_start = rospy.get_time()
                self._set_state("P4_EXECUTE")
                rospy.loginfo("all takeoff done → P4_EXECUTE")
        elif phase == "DONE":
            self.mission_done.add(i)
            self._check_all_done()
        elif phase == "FAILED":
            self.mission_failed.add(i)
            self._check_all_done()

    def _check_all_done(self):
        finished = self.mission_done | self.mission_failed
        if len(finished) == self.n_drones and self.state == "P4_EXECUTE":
            self._set_state("P5_RETURN")
            rospy.loginfo("all %d missions finished (%d done, %d failed) → P5_RETURN",
                          self.n_drones, len(self.mission_done), len(self.mission_failed))
            self._set_state("DONE")
            rospy.loginfo("competition DONE")

    # ---------------------------------------------------------------- tick
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            limit = self.phase_timeout if self.state == "P3_TAKEOFF" else self.execute_timeout
            if self.state in ("P3_TAKEOFF", "P4_EXECUTE") and \
                    (now - self.phase_start) > limit:
                self.pub_timeout.publish(Bool(data=True))
                rospy.logwarn("phase %s timeout after %.0fs", self.state, limit)
                if self.state == "P3_TAKEOFF":
                    self._set_state("P4_EXECUTE")
                    self.phase_start = now
                else:
                    self._set_state("P5_RETURN")
                    self._set_state("DONE")
            rate.sleep()

    def _set_state(self, st):
        self.state = st
        self.pub_state.publish(String(data=st))


if __name__ == "__main__":
    try:
        node = StageController()
        node.run()
    except rospy.ROSInterruptException:
        pass
