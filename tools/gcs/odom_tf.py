#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/odom_tf.py — LIO 局部系→场地 ENU 前向变换节点（真机适配 2026-09-26）。

pose_tf 数学与 gcs_agent._tf_apply 同款（p_e = R(yaw0)·(p_l − c) + [ex0, ny0]），
独立成节点让 nav/mission/vel_bridge 全链共用一份变换：真机 profile 的 agent
pose_tf 段已退役（防双重变换），agent 遥测直接上报本节点输出。

订阅 {odom}（默认 /ekf/ekf_odom，官方 EKF 融合输出——源码定谳 2026-09-26：
官方 fork faster-lio 发 /laserMapping/odometry (laser_mapping.cc:309)，经
faster-lio/ekf_pose/launch/ekf_lidar.launch 融合 PX4 IMU（cutoff 20Hz）后输出
此话题，文档教学话题即此）；
发布 {out}（默认 /drone_<id>/odom_enu）nav_msgs/Odometry —— nav/mission 经
launch remap 当作 ns+/odom 消费（代码零改动）。

变换语义（与 agent 同款）：
  - 位置：p_e = R(yaw0)·(p_l − c) + [ex0, ny0]；z 不转（yaw 旋转不涉 u 轴）
  - 速度：只转不移（平移不改速度）
  - yaw：加 yaw0；发布四元数为 yaw-only（roll/pitch 清零——nav 只消费
    pos/yaw/twist，LIO 的 roll/pitch 不进导航链）
  - header.stamp 保留原值（时间语义保真），frame_id="world"

auto_origin：起飞前静止 static_frames 帧均值扣 LIO 初始化偏置 c（窗内位移
超 static_gate_m = 判空中重启，永久放弃 c=(0,0) 不猜）；锁定/放弃后写
参数服务器 /drone_<id>/pose_tf/c 供联调对质。

用法（机载 Jetson，source 后）：
  python3 odom_tf.py --id 0 [--yaw0-deg 90 --ex0 1.0 --ny0 0]
  离线：python3 odom_tf.py --selftest
"""
import argparse
import math
import sys


# ---------------------------------------------------------------- 纯逻辑 --
class PoseTF(object):
    """前向变换 + auto_origin 静止窗采样（纯函数化，离线可测）。"""

    def __init__(self, yaw0_deg=0.0, ex0=0.0, ny0=0.0, auto=True,
                 static_n=50, gate=0.3):
        self.yaw0 = math.radians(yaw0_deg)
        self.c = math.cos(self.yaw0)
        self.s = math.sin(self.yaw0)
        self.ex0, self.ny0 = ex0, ny0
        self.auto = auto
        self.static_n = int(static_n)
        self.gate = gate
        self.buf = []
        self.cx, self.cy = 0.0, 0.0     # LIO 初始化偏置 c
        self.locked = False
        self.gave_up = False

    def sample(self, x, y):
        """auto_origin 静止窗采样，返回当前 (cx, cy)。"""
        if not self.auto or self.locked or self.gave_up:
            return (self.cx, self.cy)
        self.buf.append((x, y))
        if len(self.buf) > 1:
            x0, y0 = self.buf[0]
            if math.hypot(x - x0, y - y0) > self.gate:
                self.gave_up = True     # 窗内位移=空中重启，放弃不猜
                return (0.0, 0.0)
        if len(self.buf) >= self.static_n:
            n = float(len(self.buf))
            self.cx = sum(b[0] for b in self.buf) / n
            self.cy = sum(b[1] for b in self.buf) / n
            self.locked = True
        return (self.cx, self.cy)

    def apply(self, x, y, yaw, vx, vy):
        """LIO 局部系 (x,y,yaw,vx,vy) → ENU (x,y,yaw,vx,vy)。z 不转。"""
        cx, cy = self.sample(x, y)
        dx, dy = x - cx, y - cy
        return (dx * self.c - dy * self.s + self.ex0,
                dx * self.s + dy * self.c + self.ny0,
                yaw + self.yaw0,
                vx * self.c - vy * self.s,
                vx * self.s + vy * self.c)

    def inverse(self, ex, ny, vx_e, vy_e):
        """ENU → 局部系（逆变换，供对质/前视点核算）。z/yaw 逆同款。"""
        dx, dy = ex - self.ex0, ny - self.ny0
        return (dx * self.c + dy * self.s + self.cx,
                -dx * self.s + dy * self.c + self.cy,
                vx_e * self.c + vy_e * self.s,
                -vx_e * self.s + vy_e * self.c)

    def point(self, x, y):
        """单点位置变换（点云用，cloud_adapter 复用）：xy 转+z 不动。

        不做 auto_origin 采样（点云千点/帧会污染静止窗 buf）——用当前 c；
        未锁定时 c=(0,0) 恒等（起飞前点云不转，nav 未起飞不消费，无害）。
        """
        dx, dy = x - self.cx, y - self.cy
        return (dx * self.c - dy * self.s + self.ex0,
                dx * self.s + dy * self.c + self.ny0)


def selftest():
    ok = [0]

    def chk(name, got, exp, tol=1e-9):
        good = all(abs(g - w) <= tol for g, w in zip(got, exp))
        assert good, "%s: got %r exp %r" % (name, got, exp)
        ok[0] += 1

    # 1 恒等
    t = PoseTF(auto=False)
    chk("identity", t.apply(3.0, -4.0, 0.7, 1.0, 2.0), (3.0, -4.0, 0.7, 1.0, 2.0))
    # 2 纯旋转 90°
    t = PoseTF(yaw0_deg=90.0, auto=False)
    chk("rot90", t.apply(1.0, 0.0, 0.0, 2.0, 0.0),
        (0.0, 1.0, math.pi / 2, 0.0, 2.0), tol=1e-6)
    # 3 平移
    t = PoseTF(ex0=5.0, ny0=-2.0, auto=False)
    chk("trans", t.apply(1.0, 1.0, 0.0, 0.0, 0.0), (6.0, -1.0, 0.0, 0.0, 0.0))
    # 4 综合：auto 锁定 c=(1,1) 后 (2,1)@rot90+t(10,0) → (10,1)
    t = PoseTF(yaw0_deg=90.0, ex0=10.0, auto=True)
    t.cx, t.cy, t.locked = 1.0, 1.0, True
    chk("combo", t.apply(2.0, 1.0, 0.0, 0.0, 0.0),
        (10.0, 1.0, math.pi / 2, 0.0, 0.0), tol=1e-6)
    # 5 正逆往返：局部→ENU→局部 恒等
    t = PoseTF(yaw0_deg=37.0, ex0=3.0, ny0=-1.5, auto=True)
    t.cx, t.cy, t.locked = 0.4, -0.2, True
    e = t.apply(1.2, 3.4, 0.9, 0.7, -0.3)
    back = t.inverse(e[0], e[1], e[3], e[4])
    chk("roundtrip", back, (1.2, 3.4, 0.7, -0.3), tol=1e-9)
    # 6 auto 锁定：静止 5 帧
    t = PoseTF(auto=True, static_n=5)
    for _ in range(4):
        t.sample(0.1, 0.2)
    assert not t.locked
    t.sample(0.1, 0.2)
    assert t.locked and abs(t.cx - 0.1) < 1e-9
    chk("lock", t.apply(0.1, 0.2, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0))
    # 7 空中重启放弃
    t = PoseTF(auto=True, static_n=5, gate=0.3)
    t.sample(0.0, 0.0)
    t.sample(1.0, 0.0)
    assert t.gave_up
    chk("abort", t.apply(2.0, 0.0, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0, 0.0))
    print("selftest OK (%d checks)" % ok[0])


# ------------------------------------------------------------------ ROS --
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--odom", default="/ekf/ekf_odom")
    ap.add_argument("--out", default="")
    ap.add_argument("--yaw0-deg", type=float, default=0.0)
    ap.add_argument("--ex0", type=float, default=0.0)
    ap.add_argument("--ny0", type=float, default=0.0)
    ap.add_argument("--no-auto-origin", action="store_true")
    ap.add_argument("--static-frames", type=int, default=50)
    ap.add_argument("--static-gate-m", type=float, default=0.3)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    import rospy
    from nav_msgs.msg import Odometry

    out_t = args.out or ("/drone_%d/odom_enu" % args.id)
    rospy.init_node("odom_tf_%d" % args.id)
    tf = PoseTF(args.yaw0_deg, args.ex0, args.ny0,
                auto=not args.no_auto_origin,
                static_n=args.static_frames, gate=args.static_gate_m)
    pub = rospy.Publisher(out_t, Odometry, queue_size=10)
    last_state = {"gave": False}

    def _log_state():
        if tf.locked and not last_state.get("locked"):
            last_state["locked"] = True
            rospy.loginfo("odom_tf: auto_origin LOCKED c=(%.3f,%.3f)",
                          tf.cx, tf.cy)
            rospy.set_param("/drone_%d/pose_tf/c" % args.id, [tf.cx, tf.cy])
        elif tf.gave_up and not last_state["gave"]:
            last_state["gave"] = True
            rospy.logwarn("odom_tf: auto_origin ABORT (moving at startup) "
                          "— c=(0,0), 坐标原点未标定，须人工核实再起飞")

    def on_odom(m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = m.twist.twist.linear
        ex, ny, eyaw, evx, evy = tf.apply(p.x, p.y, yaw, v.x, v.y)
        o = Odometry()
        o.header = m.header
        o.header.frame_id = "world"
        o.child_frame_id = m.child_frame_id
        # yaw-only 四元数（roll/pitch 不进导航链）
        half = eyaw * 0.5
        o.pose.pose.orientation.z = math.sin(half)
        o.pose.pose.orientation.w = math.cos(half)
        o.pose.pose.position.x = ex
        o.pose.pose.position.y = ny
        o.pose.pose.position.z = p.z
        o.twist.twist.linear.x = evx
        o.twist.twist.linear.y = evy
        o.twist.twist.linear.z = v.z
        o.twist.twist.angular = m.twist.twist.angular
        pub.publish(o)
        _log_state()

    rospy.Subscriber(args.odom, Odometry, on_odom, queue_size=10)
    rospy.loginfo("odom_tf[%d]: in=%s out=%s yaw0=%.1fdeg ex0=%.2f ny0=%.2f "
                  "auto=%s", args.id, args.odom, out_t, args.yaw0_deg,
                  args.ex0, args.ny0, not args.no_auto_origin)
    rospy.spin()


if __name__ == "__main__":
    main()
