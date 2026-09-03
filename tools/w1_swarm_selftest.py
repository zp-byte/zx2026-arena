#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""W1 swarm 层治理 selftest：swarm_obs_guard + swarm_rescue_quiet + 闭合速度遥测。

三件套纪律（2026-09-03 贴树判官团 Wave 1）：
  1) W0 逐位对拍——工作区 nav_node.py 与 tools/w0_baseline_nav_node.py
     （pre-W1 git 快照）在旗关状态下对拍 _apply_swarm / _apply_separation /
     _sep_obs_guard（合成状态网格）：旗全关必须逐位原行为。
  2) 新机制合成用例（旗开）：投影保切向、分桶计数、救援静默、爬坡重入。
  3) 零点云中性：无点云时开/关输出逐位一致。

无 ROS 运行时：importlib 加载模块 + MethodType 绑定（沿用 avoid_selftest
范式）；rospy 以 SimpleNamespace 桩替换（get_time 可调钟）。模块本体
import rospy，故仍需 source devel 环境（但不需要 roscore）。
运行: source /opt/ros/noetic/setup.bash && source ~/zx2026_arena_ws/devel/setup.bash \
      && python3 tools/w1_swarm_selftest.py
"""
import importlib.util
import math
import os
import types

import numpy as np

WS = os.path.expanduser("~/zx2026_arena_ws")
NAV = os.path.join(WS, "src/arena_nav/scripts/nav_node.py")
BASE = os.path.join(WS, "tools/w0_baseline_nav_node.py")

CLOCK = {"t": 100.0}


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


NAV_NEW = load_module(NAV, "nav_node_w1")
NAV_BASE = load_module(BASE, "nav_node_w0")


def _stub_rospy(mod):
    mod.rospy = types.SimpleNamespace(
        get_time=lambda: CLOCK["t"],
        loginfo=lambda *a, **k: None,
        loginfo_throttle=lambda *a, **k: None,
        logwarn=lambda *a, **k: None,
        logerr=lambda *a, **k: None)


_stub_rospy(NAV_NEW)
_stub_rospy(NAV_BASE)


def make_state(mod, *, cloud=(), neighbors=None, neigh_vel=None,
               odom=(0.0, 0.0, 2.5), v_odom=(0.0, 0.0, 0.0),
               swarm_enabled=True, de=False, gs=False, pf=None,
               swq=False, swq_until=-1e9, swg=False, soa=False):
    """构造两代模块都能跑的最小 NavNode 状态（属性超集）。"""
    s = types.SimpleNamespace()
    s.swarm_enabled = swarm_enabled
    s.odom = tuple(odom)
    s.v_odom = np.array(v_odom, dtype=float)
    s.swarm_min_z = 2.0
    s.swarm_z_band = 0.5
    s.perception_radius = 8.0
    s.cohesion_gain = 0.2
    s.alignment_gain = 0.4
    s.obstacle_scale = True
    s.neighbors = dict(neighbors or {})
    s.neighbor_vel = dict(neigh_vel or {})
    s.cloud = list(cloud)
    s.scene = types.SimpleNamespace(drone_radius=0.2)
    s.drone_id = 0
    s.sep_radius = 1.5
    s.sep_gain = 3.5
    s.max_vel = 1.5
    s._swg_enabled = swg
    s._swq_enabled = swq
    s._swq_until = swq_until
    s._swq_ramp = 2.5
    s._de_active = de
    s._gs_active = gs
    s._plan_fail_since = pf
    s._soa_enabled = soa
    s._soa_hits = {"sep": 0, "swarm": 0}
    s._apply_swarm = types.MethodType(mod.NavNode._apply_swarm, s)
    s._apply_separation = types.MethodType(mod.NavNode._apply_separation, s)
    s._sep_obs_guard = types.MethodType(mod.NavNode._sep_obs_guard, s)
    s._min_clearance = types.MethodType(mod.NavNode._min_clearance, s)
    if hasattr(mod.NavNode, "_closing_speed"):
        s._closing_speed = types.MethodType(mod.NavNode._closing_speed, s)
    return s


N = 0


def ok(cond, msg):
    global N
    N += 1
    if not cond:
        raise AssertionError("case %d FAIL: %s" % (N, msg))


# ---------------------------------------------------------------------------
# 1) W0 逐位对拍（旗关）
# ---------------------------------------------------------------------------
SWARM_GRID = [
    dict(name="no-neighbors", neighbors={}),
    dict(name="three-neighbors", odom=(0.0, 0.0, 2.5), v_odom=(0.2, -0.1, 0.0),
         neighbors={1: (3, 1, 2.5), 2: (-2, 4, 2.4), 3: (5, -3, 2.6)},
         neigh_vel={1: (0.5, 0.2, 0.0), 2: (-0.3, 0.4, 0.0)}),
    dict(name="mixed-band", odom=(0.0, 0.0, 2.5),
         neighbors={1: (3, 1, 1.2), 2: (2, 2, 2.5), 3: (6, 6, 2.5)},
         neigh_vel={1: (1, 1, 0), 3: (0.2, 0.2, 0)}),
    dict(name="near-tree-scale", odom=(0.0, 0.0, 2.5),
         cloud=[(0.8, 0.1, 2.5), (3, 3, 2.5)],
         neighbors={1: (4, 0, 2.5)}, neigh_vel={1: (0, 0, 0)}),
    dict(name="below-minz", odom=(0.0, 0.0, 1.5), neighbors={1: (3, 1, 1.4)}),
]

for g in SWARM_GRID:
    cmd0 = np.array([1.0, 0.5, 0.0])
    kw = {k: v for k, v in g.items() if k != "name"}
    out_base = make_state(NAV_BASE, **kw)._apply_swarm(cmd0.copy())
    out_new = make_state(NAV_NEW, **kw)._apply_swarm(cmd0.copy())
    ok(np.array_equal(out_base, out_new),
       "swarm W0 mismatch (%s): base=%s new=%s" % (g["name"], out_base, out_new))

# swarm_enabled=False：命令原样返回（两代一致）
for mod, tag in ((NAV_BASE, "base"), (NAV_NEW, "new")):
    st = make_state(mod, swarm_enabled=False, neighbors={1: (3, 1, 2.5)})
    cmd0 = np.array([1.0, 0.5, 0.0])
    out = st._apply_swarm(cmd0.copy())
    ok(np.array_equal(out, cmd0), "swarm-off untouched (%s): %s" % (tag, out))

SEP_GRID = [
    dict(name="far-neighbor", neighbors={1: (4.0, 0.0, 2.5)}, cloud=[(1.0, 0.3, 2.5)]),
    dict(name="inside-sep", neighbors={1: (1.2, 0.0, 2.5)}, cloud=[(1.4, 0.1, 2.5)]),
    dict(name="inside-hard-behind-tree", neighbors={1: (1.3, 0.0, 2.5)},
         cloud=[(1.4, 0.0, 2.5), (0.2, -0.5, 2.5)]),
    dict(name="no-cloud", neighbors={1: (1.1, 0.2, 2.5)}),
    dict(name="two-neighbors", odom=(0.0, 0.0, 2.5),
         neighbors={1: (1.1, 0.0, 2.5), 2: (-1.0, 0.3, 2.5)},
         cloud=[(1.5, 0.0, 2.5)]),
]

for soa in (False, True):
    for g in SEP_GRID:
        cmd0 = np.array([0.8, 0.4, 0.0])
        kw = {k: v for k, v in g.items() if k != "name"}
        out_base = make_state(NAV_BASE, soa=soa, **kw)._apply_separation(cmd0.copy())
        out_new = make_state(NAV_NEW, soa=soa, **kw)._apply_separation(cmd0.copy())
        ok(np.array_equal(out_base, out_new),
           "sep W0 mismatch (soa=%s %s): base=%s new=%s"
           % (soa, g["name"], out_base, out_new))

# _sep_obs_guard 直接对拍（默认 tag）
g = dict(name="guard-direct", cloud=[(1.0, 0.3, 2.5), (0.3, -0.2, 2.5)])
cmd0 = np.array([0.9, 0.3, 0.0])
pre = np.array([0.5, 0.1, 0.0])
kw = {k: v for k, v in g.items() if k != "name"}
out_base = make_state(NAV_BASE, **kw)._sep_obs_guard(cmd0.copy(), pre.copy())
out_new = make_state(NAV_NEW, **kw)._sep_obs_guard(cmd0.copy(), pre.copy())
ok(np.array_equal(out_base, out_new),
   "guard W0 mismatch: base=%s new=%s" % (out_base, out_new))
ok(out_new[1] != cmd0[1], "guard-direct should have projected (into>0 expected)")

# ---------------------------------------------------------------------------
# 2) 新机制合成用例（NAV_NEW，旗开）
# ---------------------------------------------------------------------------
# G1 投影保切向：coh 指向正北树 → into 分量清零，cmd 回到 pre；带切向分量时保留
st = make_state(NAV_NEW, swg=True, cloud=[(0.0, 1.2, 2.5)],
                neighbors={1: (0, 3, 2.5)})
out = st._apply_swarm(np.array([1.0, 0.0, 0.0]))
ok(np.allclose(out, [1.0, 0.0, 0.0], atol=1e-12),
   "G1a north-tree coh fully projected: %s" % out)
ok(st._soa_hits["swarm"] == 1 and st._soa_hits["sep"] == 0,
   "G1b bucket counters: %s" % st._soa_hits)

st = make_state(NAV_NEW, swg=True, cloud=[(0.0, 1.2, 2.5)],
                neighbors={1: (2, 3, 2.5)})
out = st._apply_swarm(np.array([1.0, 0.0, 0.0]))
ok(np.allclose(out, [1.24, 0.0, 0.0], atol=1e-12),
   "G1c tangential kept (0.24): %s" % out)

# G3 远树中性：guard 带外 → 开/关逐位一致
off = make_state(NAV_NEW, swg=False, cloud=[(0.0, 5.0, 2.5)],
                 neighbors={1: (0, 3, 2.5)})
on = make_state(NAV_NEW, swg=True, cloud=[(0.0, 5.0, 2.5)],
                neighbors={1: (0, 3, 2.5)})
out_off = off._apply_swarm(np.array([1.0, 0.0, 0.0]))
out_on = on._apply_swarm(np.array([1.0, 0.0, 0.0]))
ok(np.array_equal(out_off, out_on), "G3 far-tree swg on==off")
ok(np.allclose(out_on, [1.0, 0.6, 0.0], atol=1e-12), "G3 expected full add: %s" % out_on)

# Z1 零点云中性：cloud 空 → 开/关逐位一致（guard 循环空转）
off = make_state(NAV_NEW, swg=False, neighbors={1: (3, 1, 2.5)},
                 neigh_vel={1: (0.4, 0.1, 0.0)})
on = make_state(NAV_NEW, swg=True, neighbors={1: (3, 1, 2.5)},
                neigh_vel={1: (0.4, 0.1, 0.0)})
ok(np.array_equal(off._apply_swarm(np.array([1.0, 0.5, 0.0])),
                  on._apply_swarm(np.array([1.0, 0.5, 0.0]))),
   "Z1 zero-cloud swg on==off")

# Q1 救援静默：de/gs/plan_fail 任一 → cmd 原样返回
for kw in (dict(de=True), dict(gs=True), dict(pf=7.0)):
    st = make_state(NAV_NEW, swq=True, neighbors={1: (0, 3, 2.5)}, **kw)
    cmd0 = np.array([0.7, -0.2, 0.0])
    out = st._apply_swarm(cmd0.copy())
    ok(np.array_equal(out, cmd0), "Q1 quiet (%s) untouched: %s" % (kw, out))

# Q1b 旗关时不静默（基线行为保持：救援期群集照旧）
st = make_state(NAV_NEW, swq=False, de=True, neighbors={1: (0, 3, 2.5)})
base = make_state(NAV_NEW, swq=False, de=False, neighbors={1: (0, 3, 2.5)})
ok(np.array_equal(st._apply_swarm(np.array([1.0, 0.0, 0.0])),
                  base._apply_swarm(np.array([1.0, 0.0, 0.0]))),
   "Q1b swq-off rescue behavior identical to normal")

# Q3 爬坡中段：until=t0+2.5, now=t0+1.25 → 增益 0.5（coh=(0,0.6)→(0,0.3)）
CLOCK["t"] = 101.25
st = make_state(NAV_NEW, swq=True, swq_until=102.5, neighbors={1: (0, 3, 2.5)})
out = st._apply_swarm(np.array([1.0, 0.0, 0.0]))
ok(np.allclose(out, [1.0, 0.3, 0.0], atol=1e-12), "Q3 ramp half: %s" % out)

# Q3b 爬坡起点（=de 退出瞬间 now=until-ramp）→ 增益 0
CLOCK["t"] = 100.0
st = make_state(NAV_NEW, swq=True, swq_until=102.5, neighbors={1: (0, 3, 2.5)})
out = st._apply_swarm(np.array([1.0, 0.0, 0.0]))
ok(np.allclose(out, [1.0, 0.0, 0.0], atol=1e-12), "Q3b ramp at-exit zero: %s" % out)

# Q4 爬坡结束：now >= until → 满额
CLOCK["t"] = 102.6
st = make_state(NAV_NEW, swq=True, swq_until=102.5, neighbors={1: (0, 3, 2.5)})
out = st._apply_swarm(np.array([1.0, 0.0, 0.0]))
ok(np.allclose(out, [1.0, 0.6, 0.0], atol=1e-12), "Q4 ramp done full: %s" % out)

# Q5 swq 开但无救援无窗 → 与关逐位一致
CLOCK["t"] = 100.0
a = make_state(NAV_NEW, swq=False, neighbors={1: (0, 3, 2.5)},
               neigh_vel={1: (0.3, 0.0, 0.0)})
b = make_state(NAV_NEW, swq=True, swq_until=-1e9, neighbors={1: (0, 3, 2.5)},
               neigh_vel={1: (0.3, 0.0, 0.0)})
ok(np.array_equal(a._apply_swarm(np.array([1.0, 0.0, 0.0])),
                  b._apply_swarm(np.array([1.0, 0.0, 0.0]))), "Q5 swq on(no-op)==off")

# C1 闭合速度遥测
st = make_state(NAV_NEW, v_odom=(0.3, 0.4, 0.0), cloud=[(1.0, 0.0, 2.5)])
vc, cl = st._closing_speed()
ok(abs(vc - 0.3) < 1e-12 and abs(cl - 1.0) < 1e-12, "C1 close/clr: %s %s" % (vc, cl))
st = make_state(NAV_NEW, v_odom=(-0.5, 0.0, 0.0), cloud=[(1.0, 0.0, 2.5)])
vc, cl = st._closing_speed()
ok(vc < -0.49, "C2 receding negative: %s" % vc)
st = make_state(NAV_NEW, cloud=[])
vc, cl = st._closing_speed()
ok(vc == 0.0 and cl == 1e9, "C3 empty cloud: %s %s" % (vc, cl))

# SEP-NEUTRAL：分离路径 tag 默认 "sep" 计数正确（sep_obs_guard 原语义未变）。
# 邻居在西、cmd 朝西（硬推触发）、树在东：分离+硬推合增量朝东指树 →
# guard 投影后精确回到 pre（树挡在机与目标方向之间 → 推力归零语义）。
st = make_state(NAV_NEW, soa=True, neighbors={1: (-1.2, 0.0, 2.5)},
                cloud=[(1.4, 0.0, 2.5)])
out = st._apply_separation(np.array([-0.8, 0.4, 0.0]))
ok(st._soa_hits["sep"] == 1, "S1 sep bucket counted: %s" % st._soa_hits)
ok(st._soa_hits["swarm"] == 0, "S2 swarm bucket untouched by sep: %s" % st._soa_hits)
ok(np.allclose(out, [-0.8, 0.4, 0.0], atol=1e-12),
   "S3 sep increment fully projected back to pre: %s" % out)

print("w1_swarm_selftest: all %d cases PASS" % N)
