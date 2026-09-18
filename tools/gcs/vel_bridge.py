#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/vel_bridge.py — 真机 vel_cmd→px4ctrl 桥（2026-09-18）。

契约（真机线，Fast-Drone-250 架构，源码核实见 profile_real_lio.yaml 注）：
  订阅 /drone_{id}/vel_cmd   geometry_msgs/Twist —— nav_node 世界系目标速度
                             + yaw 速率(angular.z)，20Hz（sim 插件同契约）；
  订阅 {odom}（默认 /Odometry） nav_msgs/Odometry —— faster_lio 里程计；
  发布 {out}（默认 /position_cmd） quadrotor_msgs/PositionCommand ——
                             px4ctrl CMD_CTRL 档的跟踪输入（traj_server 同
                             话题，两者勿同时活跃）。

注入语义：position = odom 位置 + v*lookahead（无状态前视点，不积分——
LIO 抖动零累积，lookahead 只起一小段引导）；velocity/acceleration 直传
（acc=0）；yaw=odom 当前 yaw，yaw_dot=angular.z；kx/ky/kz=0 → px4ctrl
用自己 cfg 的默认增益。

安全语义（对齐 sim drone_vel_plugin 断链保护 0.5s）：
  - vel_cmd 静默 > stale_timeout(0.5s，与插件 cmd_timeout 同款) → HOLD：
    定点点悬停（vel=0，锚 odom 当前位置），持续 hold_grace(2.0s)；
  - HOLD 超过 hold_grace → 停发（px4ctrl 自身 cmd 超时接管，参数名现场
    核实；停发后状态机回 IDLE）；
  - odom 静默 > odom_timeout(0.3s)（LIO 断）→ 立即停发；
  - 线速度限幅 max_vel(2.0 m/s)、yaw_dot 限幅 max_yaw_dot(1.0 rad/s)。
  模式切换不由本桥做：CMD_CTRL 靠 RC 档位（人手），起飞/降落走
  sh_files/takeoff.sh / land.sh（/px4ctrl/takeoff_land 服务），panic 同款
  ——land 服务不经过本桥，桥死机也能落地。

飞行顺序：no-prop bench（tools/gcs/bench_check.sh）→ takeoff.sh（AUTO_
HOVER）→ RC 扳 CMD_CTRL → 本桥输出生效 → 结束 land.sh。

自检（WSL 无 ROS/quadrotor_msgs 可跑）：
  python3 vel_bridge.py --selftest
用法（机载 Jetson，source ~/Diff-planner/devel/setup.sh 后）：
  python3 vel_bridge.py --id 0 [--max-vel 2.0] [--rate 50]
"""
import argparse
import math
import sys

# ---------------------------------------------------------------- 纯逻辑 --
STALE_TIMEOUT = 0.5    # s，与 sim drone_vel_plugin cmd_timeout 同款
HOLD_GRACE = 2.0       # s，HOLD 停发前宽限
ODOM_TIMEOUT = 0.3     # s，LIO 断流门
LOOKAHEAD = 0.15       # s，前视点


def clamp_v(v, vmax, wz, wmax):
    """速度限幅：超 max_vel 等比缩到模长=vmax，yaw_dot 独立限幅。"""
    sp = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if sp > vmax > 0:
        s = vmax / sp
        v = (v[0] * s, v[1] * s, v[2] * s)
    return v, max(-wmax, min(wmax, wz))


class BridgeLogic(object):
    """状态机 IDLE/ACTIVE/HOLD，纯函数化便于 --selftest（无 ROS 依赖）。"""

    def __init__(self, max_vel=2.0, max_yaw_dot=1.0, lookahead=LOOKAHEAD,
                 stale=STALE_TIMEOUT, hold=HOLD_GRACE, odom_to=ODOM_TIMEOUT):
        self.max_vel, self.max_yaw_dot = max_vel, max_yaw_dot
        self.lookahead, self.stale, self.hold, self.odom_to = (
            lookahead, stale, hold, odom_to)
        self.state = "IDLE"
        self.cmd = (0.0, 0.0, 0.0, 0.0)   # vx,vy,vz,wz（世界系）
        self.t_cmd = None
        self.t_odom = None
        self.odom = (0.0, 0.0, 0.0)       # 最近 odom 位置（HOLD 锚点）

    def on_cmd(self, v, now):
        self.cmd, self.t_cmd = v, now

    def on_odom(self, now):
        self.t_odom = now

    def tick(self, now):
        """返回 (act, pos, vel, yaw_dot)；act: PUB/STOP。pos=odom 位置。"""
        if self.t_odom is None or now - self.t_odom > self.odom_to:
            self.state = "IDLE"
            return ("STOP", None, None, None)
        if self.t_cmd is None:
            self.state = "IDLE"
            return ("STOP", None, None, None)
        age = now - self.t_cmd
        v, wz = clamp_v(self.cmd[:3], self.max_vel, self.cmd[3],
                        self.max_yaw_dot)
        if age <= self.stale:                     # ACTIVE：前视点+速度直传
            self.state = "ACTIVE"
            pos = tuple(p + vi * self.lookahead for p, vi in zip(self.odom,
                                                                 v))
            return ("PUB", pos, v, wz)
        if age <= self.stale + self.hold:         # HOLD：定点零速悬停
            self.state = "HOLD"
            return ("PUB", tuple(self.odom), (0.0, 0.0, 0.0), 0.0)
        self.state = "IDLE"                       # 宽限耗尽：停发交 px4ctrl
        return ("STOP", None, None, None)


def selftest():
    ok = [0]

    def chk(name, got, exp):
        assert got == exp, "%s: got %r exp %r" % (name, got, exp)
        ok[0] += 1
    b = BridgeLogic()
    # 无 odom / 无 cmd → STOP
    chk("no-odom", b.tick(10.0)[0], "STOP")
    b.on_odom(10.0)
    chk("no-cmd", b.tick(10.0)[0], "STOP")
    # ACTIVE：前视点 = pos + v*lookahead，限幅生效
    b.odom = (1.0, 2.0, 0.5)
    b.on_cmd((1.0, 0.0, 0.0, 0.2), 10.0)
    act, pos, v, wz = b.tick(10.05)
    chk("act", act, "PUB")
    chk("lead", (round(pos[0], 4), round(pos[1], 4)), (1.15, 2.0))
    chk("vel", v, (1.0, 0.0, 0.0))
    chk("wz", wz, 0.2)
    # 限幅：3-4-5 模长 5 → 缩到 2.0
    b.on_cmd((3.0, 4.0, 0.0, 9.9), 10.1)
    act, pos, v, wz = b.tick(10.15)
    chk("clamp", (round(v[0], 3), round(v[1], 3), wz), (1.2, 1.6, 1.0))
    # 断链：0.5-2.5s 内 HOLD（锚当前位置零速），之后 STOP
    # （每拍前补喂 odom——HOLD 锚点也要求里程计活，逻辑即语义）
    b.on_cmd((1.0, 0.0, 0.0, 0.0), 11.0)
    b.on_odom(11.05)
    chk("fresh", b.tick(11.1)[0], "PUB")
    b.on_odom(11.85)
    act, pos, v, wz = b.tick(11.9)
    chk("hold", (act, v, pos), ("PUB", (0.0, 0.0, 0.0), b.odom))
    b.on_odom(13.55)
    chk("stop", b.tick(13.6)[0], "STOP")
    # 新命令复活
    b.on_odom(13.65)
    b.on_cmd((0.5, 0.0, 0.0, 0.0), 13.7)
    chk("revive", b.tick(13.8)[0], "PUB")
    # LIO 断流 → 立即 STOP
    b.on_odom(13.8)
    chk("odom-dead", b.tick(14.2)[0], "STOP")
    print("selftest OK (%d checks)" % ok[0])


# ------------------------------------------------------------------ ROS --
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--odom", default="/Odometry")
    ap.add_argument("--out", default="/position_cmd")
    ap.add_argument("--max-vel", type=float, default=2.0)
    ap.add_argument("--max-yaw-dot", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    try:
        from quadrotor_msgs.msg import PositionCommand
    except ImportError:
        sys.exit("quadrotor_msgs 不可导入——先 source "
                 "~/Diff-planner/devel/setup.sh（或用 --selftest）")
    import rospy
    from geometry_msgs.msg import Twist
    rospy.init_node("vel_bridge_%d" % args.id)
    logic = BridgeLogic(args.max_vel, args.max_yaw_dot)
    state = {"yaw": 0.0}
    import nav_msgs.msg as nm

    def on_odom(m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        state["yaw"] = yaw
        logic.odom = (p.x, p.y, p.z)
        logic.on_odom(rospy.get_rostime().to_sec())

    def on_cmd(m):
        logic.on_cmd((m.linear.x, m.linear.y, m.linear.z, m.angular.z),
                     rospy.get_rostime().to_sec())

    rospy.Subscriber("/drone_%d/vel_cmd" % args.id, Twist, on_cmd, queue_size=5)
    rospy.Subscriber(args.odom, nm.Odometry, on_odom, queue_size=5)
    pub = rospy.Publisher(args.out, PositionCommand, queue_size=10)
    last = {"state": None}

    def tick(_):
        act, pos, vel, yaw_dot = logic.tick(rospy.get_rostime().to_sec())
        if logic.state != last["state"]:
            rospy.loginfo("vel_bridge: %s -> %s", last["state"],
                          logic.state)
            last["state"] = logic.state
        if act != "PUB":
            return
        c = PositionCommand()
        c.header.stamp = rospy.get_rostime()
        c.header.frame_id = "world"
        c.position.x, c.position.y, c.position.z = pos
        c.velocity.x, c.velocity.y, c.velocity.z = vel
        c.yaw = state["yaw"]
        c.yaw_dot = yaw_dot
        pub.publish(c)

    rospy.Timer(rospy.Duration(1.0 / args.rate), tick)
    rospy.loginfo("vel_bridge[%d]: in=/drone_%d/vel_cmd odom=%s out=%s "
                  "max_vel=%.1f", args.id, args.id, args.odom, args.out,
                  args.max_vel)
    rospy.spin()


if __name__ == "__main__":
    main()
