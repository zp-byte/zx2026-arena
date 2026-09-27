#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/mission_loader.py — 真机任务指派装订器（真机适配 2026-09-26）。

sim 里任务由 task_generator_node（全局 1 份，读 scene_topology 自动分配）
latched 发 /zx2026/mission/<id>。真机各机独立 ROS master，且指派要人工
可控（赛前按抽签/平台实况改）——本加载器在每机本地把**人写的指派文件**
转成同款 Mission 消息 latched 发出，mission_executor/type_match/scorekeeper
零改动复用。

字段语义对齐 task_generator_node（科目三口径）：
  payload_type  本机携带物品类型 TYPE_A..E
  box_color     本机箱色（red/blue/yellow；空=接收端回退 color_map）
  drop_point_id 参考指派平台号（闭环 color_id 下仅审计用，执行器不得据此
                导航——目标由机载识别择定；无参考填 -1）
  marker_id     参考平台标识（tag）
  takeoff_pose  起降 pad 位姿（z=hover 高度，场地 ENU）
  cruise_z/max_vel/mission_seq 同名

指派文件格式（YAML；单文件可含全部机，本加载器按 --id 取自己那条）：
  missions:
    - drone: 0
      payload: TYPE_A
      box_color: red
      drop: 1            # 参考平台号；-1=无参考
      marker: P1         # 平台 tag；可空
      takeoff: [0.0, 0.0, 1.5]   # 场地 ENU，z=hover 高度
      cruise_z: 1.2      # 穿越区平飞高度（RECON latch 定谳：1.2 档）
      max_vel: 1.2       # 真机首飞保守档
      seq: 1

用法（机载，指派文件与工具同拷上机）：
  python3 mission_loader.py --id 0 --yaml missions.yaml
  周期重发保险（latch 已覆盖晚订户，一般不需要）：--rate 1.0
自检：python3 mission_loader.py --selftest（无 ROS 可跑，结构 6 检）
"""
import argparse
import sys

# ---------------------------------------------------------------- 纯逻辑 --
DEFAULT_TAKEOFF = [0.0, 0.0, 1.5]   # z=hover 高度（rules takeoff_hover_z 同款）
REQUIRED_KEYS = ("payload", "drop")


def load_assignment(path, drone_id):
    """读指派 yaml 取 drone_id 条目 → Mission 字段 dict（纯逻辑可离线测）。

    缺条目/缺必填键=人工配置错误，宁可启动失败也不带错飞（panic 纪律）。
    """
    import yaml
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    missions = (doc or {}).get("missions") or []
    for m in missions:
        if int(m.get("drone", -1)) != int(drone_id):
            continue
        for k in REQUIRED_KEYS:
            if k not in m:
                raise ValueError("mission[%d] 缺必填键 %r" % (drone_id, k))
        tk = m.get("takeoff") or DEFAULT_TAKEOFF
        if len(tk) != 3:
            raise ValueError("mission[%d] takeoff 须 [x,y,z]" % drone_id)
        return {"drone_id": int(drone_id),
                "payload_type": str(m["payload"]),
                "box_color": str(m.get("box_color", "")),
                "drop_point_id": int(m["drop"]),
                "marker_id": str(m.get("marker", "")),
                "takeoff": [float(c) for c in tk],
                "cruise_z": float(m.get("cruise_z", 1.2)),
                "max_vel": float(m.get("max_vel", 1.2)),
                "mission_seq": int(m.get("seq", 1))}
    raise ValueError("missions.yaml 无 drone=%s 条目" % drone_id)


def selftest():
    ok = [0]
    import tempfile
    import os
    good = """
missions:
  - drone: 0
    payload: TYPE_A
    box_color: red
    drop: 1
    marker: P1
    takeoff: [0.5, -1.0, 1.5]
    cruise_z: 1.2
    max_vel: 1.2
    seq: 1
  - drone: 1
    payload: TYPE_B
    drop: -1
"""
    fd, path = tempfile.mkstemp(suffix=".yaml")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(good)
        a = load_assignment(path, 0)
        assert a == {"drone_id": 0, "payload_type": "TYPE_A",
                     "box_color": "red", "drop_point_id": 1,
                     "marker_id": "P1", "takeoff": [0.5, -1.0, 1.5],
                     "cruise_z": 1.2, "max_vel": 1.2, "mission_seq": 1}, a
        ok[0] += 1
        # 默认值：takeoff/cruise_z/max_vel/seq/box_color/marker 缺省
        b = load_assignment(path, 1)
        assert b["takeoff"] == DEFAULT_TAKEOFF and b["cruise_z"] == 1.2 \
            and b["max_vel"] == 1.2 and b["mission_seq"] == 1 \
            and b["box_color"] == "" and b["marker_id"] == "", b
        ok[0] += 1
        # 缺条目报错
        try:
            load_assignment(path, 5)
            raise AssertionError("missing entry not detected")
        except ValueError:
            ok[0] += 1
        # 缺必填键报错
        fd2, path2 = tempfile.mkstemp(suffix=".yaml")
        with os.fdopen(fd2, "w", encoding="utf-8") as f:
            f.write("missions:\n  - drone: 2\n    payload: TYPE_C\n")
        try:
            load_assignment(path2, 2)
            raise AssertionError("missing key not detected")
        except ValueError:
            ok[0] += 1
        os.unlink(path2)
    finally:
        os.unlink(path)
    print("selftest OK (%d checks)" % ok[0])


# ------------------------------------------------------------------ ROS --
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=None)
    ap.add_argument("--yaml", default=None, help="指派文件路径")
    ap.add_argument("--rate", type=float, default=0.0,
                    help=">0 则按 Hz 周期重发（latch 已覆盖晚订户，一般 0）")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    if args.id is None or args.yaml is None:
        ap.error("--id and --yaml are required")
    try:
        from zx2026_common.msg import Mission
    except ImportError:
        sys.exit("zx2026_common.msg 不可导入——先 source 工作空间 setup.bash"
                 "（或用 --selftest）")
    import rospy
    from geometry_msgs.msg import Point, Quaternion
    a = load_assignment(args.yaml, args.id)
    rospy.init_node("mission_loader_%d" % args.id)
    pub = rospy.Publisher("/zx2026/mission/%d" % args.id, Mission,
                          queue_size=1, latch=True)
    m = Mission()
    m.drone_id = a["drone_id"]
    m.payload_type = a["payload_type"]
    m.box_color = a["box_color"]
    m.drop_point_id = a["drop_point_id"]
    m.marker_id = a["marker_id"]
    m.takeoff_pose.position = Point(*a["takeoff"])
    m.takeoff_pose.orientation = Quaternion(w=1.0)
    m.cruise_z = a["cruise_z"]
    m.max_vel = a["max_vel"]
    m.mission_seq = a["mission_seq"]
    rospy.loginfo("mission_loader[%d]: payload=%s box=%s ref_dp%d(%s) "
                  "cruise_z=%.1f max_vel=%.1f seq=%d", m.drone_id,
                  m.payload_type, m.box_color, m.drop_point_id, m.marker_id,
                  m.cruise_z, m.max_vel, m.mission_seq)
    if args.rate > 0:
        rate = rospy.Rate(args.rate)
        while not rospy.is_shutdown():
            pub.publish(m)
            rate.sleep()
    else:
        rospy.sleep(0.3)   # 连接建立窗口（latch 兜底，双保险）
        pub.publish(m)
        rospy.spin()


if __name__ == "__main__":
    main()
