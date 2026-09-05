# -*- coding: utf-8 -*-
"""W5 sign_fix 离线数值自检——不复现 ROS 运行时，直接实例化 nav_node 的
_apply_cloud_avoidance 算术核心（importlib 加载模块、绕过 rospy 初始化）。

覆盖 5 形态：
  T1 逃离惩罚复现（旧形态）：远离障碍的指令被反向 → 修复后零侵入
  T2 正对撞不设防复现（旧形态）：靠近障碍的指令不被推 → 修复后减速推离
  T3 围栏顶环（实拍复现）：机身坐围栏顶、8 环点 z=1.30 → 旧形态 z 负压、
     修复形态不再下压（自然爬升越过围栏顶）
  T4 degenerate：ed≈0 不崩
  T5 修复形态推离方向正确：合成点在正东，指令正东 → 推离后 x 分量减小
"""
import importlib.util
import math
import sys
import types
import numpy as np

sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
sys.path.insert(0, '/home/ubuntu/zx2026_arena_ws/src/zx2026_common/scripts')
sys.path.insert(0, '/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts')

# ---- 以桩加载 nav_node（跳过 rospy/传感器初始化，只取算术） ----------------
rospy_stub = types.ModuleType('rospy')
rospy_stub.loginfo = lambda *a, **k: None
rospy_stub.loginfo_throttle = lambda *a, **k: None
rospy_stub.logwarn = lambda *a, **k: None
rospy_stub.logerr = lambda *a, **k: None
rospy_stub.get_time = lambda: 0.0
rospy_stub.get_param = lambda *a, **k: {}
rospy_stub.init_node = lambda *a, **k: None
rospy_stub.Subscriber = lambda *a, **k: None
rospy_stub.Publisher = lambda *a, **k: None
rospy_stub.Rate = lambda *a, **k: types.SimpleNamespace(sleep=lambda: None)
rospy_stub.is_shutdown = lambda: False
sys.modules['rospy'] = rospy_stub

spec = importlib.util.spec_from_file_location(
    'nav_node', '/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts/nav_node.py')
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)


class SceneStub(object):
    drone_radius = 0.25


def make_node(sign_fix):
    """构造只够跑 _apply_cloud_avoidance 的最小对象。"""
    n = nav.NavNode.__new__(nav.NavNode)
    n.scene = SceneStub()
    n.max_vel = 2.5
    n._ca_sign_fix = sign_fix
    n.odom = [0.0, 0.0, 1.40]
    return n


def run(sign_fix, cloud, cmd):
    n = make_node(sign_fix)
    n.cloud = list(cloud)
    out = n._apply_cloud_avoidance(np.array(cmd, dtype=float))
    return out


FAILS = []


def check(name, cond, detail=''):
    tag = 'PASS' if cond else 'FAIL'
    print('%s %s %s' % (tag, name, detail))
    if not cond:
        FAILS.append(name)


# 几何：机身 (0,0,1.40)，dr=0.25，react_r=0.40，pred_r=0.30
# T1 逃离：指令 +x 远离西侧点 (-0.3,0,1.40)
cmd = np.array([1.0, 0.0, 0.0])
pt = [(-0.30, 0.0, 1.40)]
old = run(False, pt, cmd)
new = run(True, pt, cmd)
check('T1-old attracts (x reduced)', old[0] < cmd[0], 'x %.3f->%.3f' % (cmd[0], old[0]))
check('T1-new zero-intrusion', abs(new[0] - 1.0) < 1e-9 and abs(new[2]) < 1e-9,
      'x %.3f z %.3f' % (new[0], new[2]))

# T2 正对撞：指令 +x 靠近东侧点 (0.3,0,1.40)
old = run(False, pt_swap := [(0.30, 0.0, 1.40)], np.array([1.0, 0.0, 0.0]))
new = run(True, [(0.30, 0.0, 1.40)], np.array([1.0, 0.0, 0.0]))
check('T2-old no defense', abs(old[0] - 1.0) < 1e-9, 'x %.3f' % old[0])
check('T2-new repels (x reduced)', new[0] < 0.9, 'x %.3f' % new[0])

# T3 围栏顶环：8 点 z=1.30 围绕 (0,0) 方位均布，半径 0.15（实拍 d=0.144-0.151）
ring = []
for k in range(8):
    a = math.pi * 2.0 * k / 8.0
    ring.append((0.15 * math.cos(a), 0.15 * math.sin(a), 1.30))
base = [1.0, 0.0, 1.0]  # 出航基指令：东 + 上（goal 2.5）
old = run(False, ring, base)
new = run(True, ring, base)
check('T3-old pins down (z<0)', old[2] < 0.0, 'z %.3f' % old[2])
check('T3-new keeps climb (z>0)', new[2] > 0.5, 'z %.3f' % new[2])

# T3b 悬停无指令：旧形态零触发（v_app=0），修复形态同零（最小触发不扩权）
old = run(False, ring, [0.0, 0.0, 0.0])
new = run(True, ring, [0.0, 0.0, 0.0])
check('T3b-old hover no force', np.abs(old).max() < 1e-9, '')
check('T3b-new hover no force', np.abs(new).max() < 1e-9, '')

# T4 degenerate：点与机身重合
out = run(True, [(0.0, 0.0, 1.40)], [1.0, 0.0, 0.0])
check('T4 no crash', np.all(np.isfinite(out)), str(out))

# T5 推离方向：点正东 0.3，指令正东 → 修复形态 x 减小且不越推越近
new = run(True, [(0.30, 0.0, 1.40)], [1.0, 0.0, 0.0])
check('T5 repel westward', new[0] < 1.0, 'x %.3f' % new[0])

print('SUMMARY %d/6 groups, fails=%s' % (6 - len(FAILS), FAILS))
sys.exit(1 if FAILS else 0)
