#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_test_pose_tf.py — pose_tf 坐标对齐纯逻辑断言（B 方案，2026-09-25）。

object.__new__(Agent) 绕过 __init__（不进 ROS 初始化），手工塞 tf 属性。
"""
import math
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/home/ubuntu/zx2026_arena_ws/tools/gcs")
import gcs_agent as ga

OK = True


def mk(tf_on, yaw0_deg=0.0, ex0=0.0, ny0=0.0, auto=False, n=5, gate=0.3):
    a = object.__new__(ga.Agent)
    a.tf_on = tf_on
    a.tf_yaw0 = math.radians(yaw0_deg)
    a.tf_c = math.cos(a.tf_yaw0)
    a.tf_s = math.sin(a.tf_yaw0)
    a.tf_ex0, a.tf_ny0 = ex0, ny0
    a.tf_auto = auto
    a.tf_static_n = n
    a.tf_gate = gate
    a.tf_acc = {"0": {"buf": [], "c": (0.0, 0.0), "locked": False,
                      "gave_up": False}}
    return a


def apply(a, x, y, yaw=0.0, vx=0.0, vy=0.0):
    return a._tf_apply("0", x, y, yaw, vx, vy, 0.0)


def check(name, got, want, tol=1e-9):
    global OK
    good = all(abs(g - w) <= tol for g, w in zip(got, want))
    print("%s %s got=%s want=%s" % ("PASS" if good else "FAIL", name,
                                    ["%.4f" % g for g in got],
                                    ["%.4f" % w for w in want]))
    OK = OK and good


# 1 恒等（yaw0=0 无平移无 c）
a = mk(True)
check("identity", apply(a, 3.0, -4.0, 0.7, 1.0, 2.0),
      (3.0, -4.0, 0.7, 1.0, 2.0))

# 2 纯旋转 90°：LIO x 轴对 ENU 北 —— (1,0)→(0,1)，速度同转
a = mk(True, yaw0_deg=90.0)
check("rot90 pos/yaw", apply(a, 1.0, 0.0),
      (0.0, 1.0, math.pi / 2, 0.0, 0.0), tol=1e-6)
check("rot90 vel", apply(a, 5.0, 5.0, 0.0, 2.0, 0.0)[3:], (0.0, 2.0))

# 3 纯平移
a = mk(True, ex0=5.0, ny0=-2.0)
check("trans", apply(a, 1.0, 1.0), (6.0, -1.0, 0.0, 0.0, 0.0))

# 4 综合（auto=True+已锁定 c=(1,1)）：yaw0=90, t=(10,0), p_l=(2,1)
#   p−c=(1,0) → R90=(0,1) → +t=(10,1)
a = mk(True, yaw0_deg=90.0, ex0=10.0, ny0=0.0, auto=True)
a.tf_acc["0"]["c"] = (1.0, 1.0)
a.tf_acc["0"]["locked"] = True
check("combo", apply(a, 2.0, 1.0), (10.0, 1.0, math.pi / 2, 0.0, 0.0),
      tol=1e-6)

# 5 auto_origin：静止 5 帧均值锁定 c，后续同点报 (0,0)
a = mk(True, auto=True, n=5)
for _ in range(4):
    apply(a, 0.1, 0.2)
assert not a.tf_acc["0"]["locked"], "lock must wait for N frames"
apply(a, 0.1, 0.2)
assert a.tf_acc["0"]["locked"], "must lock at N frames"
assert abs(a.tf_acc["0"]["c"][0] - 0.1) < 1e-9
check("auto_c", apply(a, 0.1, 0.2), (0.0, 0.0, 0.0, 0.0, 0.0))

# 6 空中重启：窗内位移超门限 → 永久放弃，c=(0,0) 照报原值
a = mk(True, auto=True, n=5, gate=0.3)
apply(a, 0.0, 0.0)
apply(a, 1.0, 0.0)
assert a.tf_acc["0"]["gave_up"], "must abort on jump"
check("abort", apply(a, 2.0, 0.0), (2.0, 0.0, 0.0, 0.0, 0.0))

# 7 配置缺省：sim 档案无 pose_tf → 恒等；仅 auto_origin 也激活
a2 = ga.Agent({"ids": [0], "topics": {}})
assert a2.tf_on is False, "no pose_tf section must stay identity"
print("PASS config_default (sim 无 pose_tf → 恒等零开销)")
a3 = ga.Agent({"ids": [0], "topics": {}, "pose_tf": {"auto_origin": True}})
assert a3.tf_on is True, "auto_origin alone must activate tf"
print("PASS config_auto (仅 auto_origin 也激活)")

print("POSE_TF_TEST=" + ("PASS" if OK else "FAIL"))
sys.exit(0 if OK else 1)
