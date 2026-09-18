#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/cloud_adapter.py — 真机点云→nav 输入适配（2026-09-18）。

真机栈 faster_lio 配准点云（LIO world 系）→ nav_node 消费的
/drone_{id}/cloud（xyz FLOAT32 世界系，10Hz，sim lidar_node 同契约）。

nav 契约（nav_node._on_cloud 直取 xyz 世界系点、无 TF 无帧检查；sim 云=
自机为心 5m、10Hz、数百点、**无地面回波**）→ 适配器四道滤，前两道是
sim 没有而真云必有的形态：

  1. **地面滤（最要害）**：真云含地面回波，不滤=地面成幻影墙（反应层
     乱推/建图堵死——W5"围栏顶楔死"同族的真机特供版）。滤
     (z − odom_z) < z_min_rel（默认 −0.6m；anchor 取 /Odometry 位置 z，
     LIO world 原点在起飞点、重力对齐 → 地面恒在 odom_z 下方固定带内）
  2. 高度上限：(z − odom_z) > z_max_rel（默认 +2.5m）剪头顶杂波/树冠
  3. 半径：距自机 ≤ range（默认 5.0m=sim sensor_range——广度对齐，
     反应窗 react_r=1.10m 远在其内）
  4. 体素降样 + 硬上限：voxel 0.15m（点数对齐 sim 数百点量级；②孤立点
     阈值 ≤max(3,mem) 的密度语义靠降样保真——密簇降样后单格 1 点反而
     变"孤立"，故 voxel 须明显小于格级障碍特征）+ max_points 1200 硬顶
     （nav 20Hz Python 逐点环 × mem6 记忆，爆量=烧 CPU 拖垮控制环）

发布 ≤out_rate（默认 10Hz）跟随源频率，帧号透传；odom 断 >0.5s 停发
（无锚就没法做相对滤——宁缺勿幻影）。

topic 名现场可改：--src /cloud_registered（faster_lio 默认名；profile
grid_cloud TODO 现场定谳后同步 profile_real_lio.yaml）。
自检：python3 cloud_adapter.py --selftest（无 ROS 可跑，逻辑 8 检）
"""
import argparse
import math
import struct
import sys

# ------------------------------------------------------------- 纯逻辑 --
ODOM_TIMEOUT = 0.5     # s，锚断流停发


class CloudFilter(object):
    """四道滤纯逻辑（地面/高度/半径/体素+硬顶），--selftest 全覆盖。"""

    def __init__(self, range_=5.0, z_min_rel=-0.6, z_max_rel=2.5,
                 voxel=0.15, max_points=1200):
        self.range, self.z_min_rel, self.z_max_rel = range_, z_min_rel, z_max_rel
        self.voxel, self.max_points = voxel, max_points

    def filter(self, pts, odom, max_points=None):
        """pts=[(x,y,z),...]，odom=(x,y,z) 自机位置 → 滤后点列（质心）。"""
        if max_points is None:
            max_points = self.max_points
        vox = {}
        r2 = self.range * self.range
        for (x, y, z) in pts:
            dz = z - odom[2]
            if dz < self.z_min_rel or dz > self.z_max_rel:   # 地面/头顶
                continue
            dx, dy = x - odom[0], y - odom[1]
            if dx * dx + dy * dy + dz * dz > r2:             # 半径
                continue
            k = (int(math.floor(x / self.voxel)),
                 int(math.floor(y / self.voxel)),
                 int(math.floor(z / self.voxel)))
            acc = vox.get(k)
            if acc is None:
                vox[k] = [x, y, z, 1]
            else:
                acc[0] += x; acc[1] += y; acc[2] += z; acc[3] += 1
        out = [(a[0] / a[3], a[1] / a[3], a[2] / a[3]) for a in vox.values()]
        # 硬上限：均匀抽样保覆盖（按格键排序截断=空间上近似均匀）
        if len(out) > max_points:
            step = len(out) / float(max_points)
            out = [out[int(i * step)] for i in range(max_points)]
        return out


def selftest():
    ok = [0]

    def chk(name, got, exp):
        assert got == exp, "%s: got %r exp %r" % (name, got, exp)
        ok[0] += 1
    f = CloudFilter()
    o = (0.0, 0.0, 1.0)   # 自机离地 1m（LIO 原点=起飞点）
    # 地面滤：z=0（自机下方 1m <0.6）→ 丢；z=0.55 → 留
    r = f.filter([(0.0, 0.0, 0.0), (1.0, 0.0, 0.55)], o)
    chk("ground", r, [(1.0, 0.0, 0.55)])
    # 头顶滤：z=3.6（上方 2.6 >2.5）→ 丢
    r = f.filter([(0.0, 0.0, 3.6), (0.0, 0.0, 3.4)], o)
    chk("ceiling", r, [(0.0, 0.0, 3.4)])
    # 半径滤：水平 5.1m 丢、4.9m 留
    r = f.filter([(5.1, 0.0, 1.0), (4.9, 0.0, 1.0)], o)
    chk("range", r, [(4.9, 0.0, 1.0)])
    # 体素合并：同格两点（0.02m 间隔，避 0.15 格界）→ 1 质心
    r = f.filter([(1.0, 0.0, 1.0), (1.02, 0.0, 1.0)], o)
    chk("voxel-n", len(r), 1)
    chk("voxel-c", (round(r[0][0], 3),), (1.01,))
    # 体素保真：0.3m 间隔（跨格）→ 2 点
    r = f.filter([(1.0, 0.0, 1.0), (1.3, 0.0, 1.0)], o)
    chk("voxel-keep", len(r), 2)
    # 硬上限：8000 均匀点 in 4m³ → ≤1200
    import random
    random.seed(7)
    cloud = [(random.uniform(-2, 2), random.uniform(-2, 2),
              random.uniform(0, 2)) for _ in range(8000)]
    r = f.filter(cloud, o)
    chk("cap", len(r) <= 1200, True)
    # 空输入
    chk("empty", f.filter([], o), [])
    print("selftest OK (%d checks)" % ok[0])


# ------------------------------------------------------------------ ROS --
def parse_xyz(msg):
    """PointCloud2 → [(x,y,z),...]（仿 nav_node._on_cloud 的偏移查表解析）。"""
    off = {fl.name: fl.offset for fl in msg.fields}
    ox, oy, oz = off.get("x", 0), off.get("y", 4), off.get("z", 8)
    step, data, n = msg.point_step, msg.data, msg.width
    u = struct.Struct('<f').unpack_from
    return [(u(data, i * step + ox)[0], u(data, i * step + oy)[0],
             u(data, i * step + oz)[0]) for i in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--src", default="/cloud_registered")
    ap.add_argument("--odom", default="/Odometry")
    ap.add_argument("--out", default="")
    ap.add_argument("--range", type=float, default=5.0)
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--max-points", type=int, default=1200)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    import rospy
    import nav_msgs.msg as nm
    from sensor_msgs.msg import PointCloud2, PointField
    out_t = args.out or "/drone_%d/cloud" % args.id
    rospy.init_node("cloud_adapter_%d" % args.id)
    flt = CloudFilter(args.range, voxel=args.voxel, max_points=args.max_points)
    st = {"pc": None, "frame": "world", "odom": None, "t_odom": None,
          "t_pc": None, "warned": False}

    def on_pc(m):
        st["pc"], st["frame"], st["t_pc"] = m, m.header.frame_id, \
            rospy.get_rostime().to_sec()

    def on_odom(m):
        p = m.pose.pose.position
        st["odom"] = (p.x, p.y, p.z)
        st["t_odom"] = rospy.get_rostime().to_sec()

    rospy.Subscriber(args.src, PointCloud2, on_pc, queue_size=2)
    rospy.Subscriber(args.odom, nm.Odometry, on_odom, queue_size=2)
    pub = rospy.Publisher(out_t, PointCloud2, queue_size=2)

    def tick(_):
        now = rospy.get_rostime().to_sec()
        if st["odom"] is None or now - st["t_odom"] > ODOM_TIMEOUT:
            if not st["warned"]:
                rospy.logwarn("cloud_adapter: odom 无锚/断流 %ss → 停发"
                              "（宁缺勿幻影）", ODOM_TIMEOUT)
                st["warned"] = True
            return
        st["warned"] = False
        m = st["pc"]
        if m is None or now - st["t_pc"] > 1.0:
            return
        pts = flt.filter(parse_xyz(m), st["odom"])
        o = PointCloud2()
        o.header.stamp = m.header.stamp
        o.header.frame_id = st["frame"]
        o.height, o.width = 1, len(pts)
        o.fields = [PointField(n, 4 * i, PointField.FLOAT32, 1)
                    for i, n in enumerate(("x", "y", "z"))]
        o.point_step = 12
        o.is_bigendian = False
        o.row_step = 12 * len(pts)
        o.data = b"".join(struct.pack('<fff', *p) for p in pts)
        o.is_dense = True
        pub.publish(o)

    rospy.Timer(rospy.Duration(1.0 / args.rate), tick)
    rospy.loginfo("cloud_adapter[%d]: %s → %s range=%.1f voxel=%.2f "
                  "cap=%d z_rel=[%.1f,%.1f]", args.id, args.src, out_t,
                  args.range, args.voxel, args.max_points,
                  flt.z_min_rel, flt.z_max_rel)
    rospy.spin()


if __name__ == "__main__":
    main()
