#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_test_real_chain.py — 真机适配链离线对拍（2026-09-26，WSL 无 ROS 可跑）。

四类断言（对应部署最要害的四条缝）：
  1. 速度场互逆：vel_bridge.world_to_local（ENU→局部）与 odom_tf.PoseTF
     的速度旋转（局部→ENU）必须互逆——vel_cmd 逆变换与前视点同系的关键
  2. 点云同帧：PoseTF.point（cloud_adapter 用）与 PoseTF.apply 的 xy 分量
     一致——点云与 odom_enu 同帧的建图前提
  3. fleet 下行 E2E：Hub.fleet_broadcast 对 {"down":1} 注册连接实收快照
     （fleet/stage/tasks 三键+失联机剔除），socket 层全链
  4. 部署文件非默认值断言：settings_real.yaml 四差异+profile_real_lio.yaml
     挂点（pose_tf 退役/odom 挂 odom_tf 输出）——干测须含非默认值断言
     （mem6 教训：全默认值断言=模板轨假 OK）
  5. 真场地两文件：scene_topology_real.yaml（用户坐标原样/零树/栅格覆盖
     ≥110/四面 fence）+ competition_rules_real.yaml（真机高度链/1500s/
     真场 geofence/无林返航）
跑法: python3 _test_real_chain.py
"""
import json
import math
import os
import random
import socket
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import odom_tf as otf                      # noqa: E402
import vel_bridge as vb                    # noqa: E402
from gcs_hub import Hub                    # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(HERE))   # 工作空间根


def test_vel_inverse():
    """world_to_local(ENU v) 后经 PoseTF 速度旋转应还原原速度。"""
    rng = random.Random(11)
    for _ in range(200):
        yaw0_deg = rng.uniform(-180.0, 180.0)
        v_enu = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-1, 1))
        tf = otf.PoseTF(yaw0_deg, rng.uniform(-5, 5), rng.uniform(-5, 5),
                        auto=False)
        v_loc = vb.world_to_local(v_enu, tf.yaw0)
        # PoseTF.apply 只取速度旋转部分：喂零位移零 yaw，输出 vx/vy 即旋转
        _, _, _, vx_e, vy_e = tf.apply(0.0, 0.0, 0.0, v_loc[0], v_loc[1])
        assert abs(vx_e - v_enu[0]) < 1e-9 and abs(vy_e - v_enu[1]) < 1e-9, \
            "yaw0=%s v=%s -> %s" % (yaw0_deg, v_enu, (vx_e, vy_e))
        # z 轴直传、yaw_dot 不转（world_to_local 保 z）
        assert abs(v_loc[2] - v_enu[2]) < 1e-12


def test_cloud_same_frame():
    """point()（点云）与 apply()（位姿）xy 分量必须逐点一致。"""
    rng = random.Random(23)
    for _ in range(100):
        tf = otf.PoseTF(rng.uniform(-180, 180), rng.uniform(-5, 5),
                        rng.uniform(-5, 5), auto=True)
        tf.cx, tf.cy, tf.locked = rng.uniform(-1, 1), rng.uniform(-1, 1), True
        x, y = rng.uniform(-10, 10), rng.uniform(-10, 10)
        ex, ny, _, _, _ = tf.apply(x, y, 0.0, 0.0, 0.0)
        px, py = tf.point(x, y)
        assert abs(px - ex) < 1e-12 and abs(py - ny) < 1e-12


def test_fleet_downlink():
    """Hub fleet 广播全链：上行入库 → {"down":1} 注册连接实收三键快照。"""
    logf = tempfile.mkstemp(suffix=".jsonl")[1]
    hub = Hub(["0", "1", "2"], 3.0, 0.05, 5.0, 5.0, logf, "log",
              fleet_hz=200.0)
    # 上行遥测：d0 活、d2 不发（应被 link_lost 门剔除）
    hub.on_msg({"stage": "P2_WAIT",
                "drones": {"0": {"pos": [1.0, 2.0, 0.5],
                                 "vel": [0.1, 0.0, 0.0], "yaw": 90.0,
                                 "phase": "EXECUTE"},
                           "1": {"pos": [3.0, 4.0, 0.5], "vel": [0, 0, 0],
                                 "yaw": 0.0, "phase": "TAKEOFF"}},
                "tasks": {"1": {"state": "LANDED", "payload": "TYPE_A",
                                "drop": 1, "drop_ok": 1}}})
    a, b = socket.socketpair()
    hub.fleet_register(b)
    th = threading.Thread(target=hub.fleet_broadcast, daemon=True)
    th.start()
    a.settimeout(2.0)
    line = a.recv(65536).decode("ascii")
    snap = json.loads(line.split("\n")[0])
    assert set(snap["fleet"].keys()) == {"0", "1"}, snap["fleet"].keys()
    assert snap["fleet"]["0"]["pos"] == [1.0, 2.0, 0.5]
    assert abs(snap["fleet"]["0"]["yaw"] - 90.0) < 1e-9   # 度口径透传
    assert snap["stage"] == "P2_WAIT"
    assert snap["tasks"]["1"]["state"] == "LANDED"
    # 下行兴趣声明才入池：未声明的上行连接不收（Handler 层语义，此处验池）
    with hub.flock:
        assert len(hub.fleet_socks) == 1
    hub.fleet_unregister(b)
    a.close()
    b.close()
    os.unlink(logf)


def test_deploy_files():
    """部署两件套非默认值断言（模板轨假 OK 防线）。"""
    import yaml
    with open(os.path.join(WS, "src/zx2026_common/config/settings_real.yaml"),
              encoding="utf-8") as f:
        s = yaml.safe_load(f)
    assert s["use_sim_time"] is False                     # 差异①
    assert s["closed_loop"]["drift_rate"] == 0.0          # 差异②
    assert s["closed_loop"]["drift_max"] == 0.0
    assert s["comms"]["enabled"] is True                  # 差异③
    assert s["collision"]["recovery"]["enabled"] is False  # 差异④
    # 真值泄漏审计（真机必须全过）
    assert s["closed_loop"]["enabled"] is True
    assert s["mission"]["via_slots"]["source"] == "cloud"
    # 治理件必须保留（真机密林主战场）
    assert s["swarm"]["obs_guard"]["enabled"] is True
    assert s["closed_loop"]["cloud_avoidance"]["sign_fix"] is True
    assert s["closed_loop"]["rescue_mutex"]["enabled"] is True
    assert s["closed_loop"]["sep_obs_guard"]["enabled"] is True
    assert s["closed_loop"]["branch_handling"]["enabled"] is True
    with open(os.path.join(HERE, "profile_real_lio.yaml"),
              encoding="utf-8") as f:
        p = yaml.safe_load(f)
    assert p["topics"]["odom"] == "/drone_{id}/odom"       # 挂 odom_tf 输出
    assert "pose_tf" not in p                              # 退役（防双重变换）
    assert p["topics"]["stage"] is None                    # 非 1 号机：回声门
    assert p["topics"]["mission"] == "/zx2026/mission/{id}"
    assert p["topics"]["metrics"] == "/drone_{id}/nav_metrics"
    assert p["topics"]["liveness"] == ["/position_cmd"]
    # launch 三桥同源参数挂点在文件里（防 launch 重排丢参数的回归）
    with open(os.path.join(WS,
                           "src/arena_bridges/launch/real_fleet.launch"),
              encoding="utf-8") as f:
        lx = f.read()
    for tok in ("--yaw0-deg", "--ex0", "--ny0", "--fleet-bridge",
                "stage_controller_node.py", "--out /drone_$(arg id)/odom",
                "type_match_node.py",            # 释放门供给方（缺=AT_DROP 卡死）
                "fleet_ready"):                  # stage P1 卡死人工解法在案
        assert tok in lx, tok


def test_real_venue():
    """真场地两文件非默认值断言（用户口径 2026-09-26 定谳）。"""
    import yaml
    sc = yaml.safe_load(open(
        os.path.join(HERE, "scene_topology_real.yaml"), encoding="utf-8"))
    dps = [list(dp["xyz"]) for dp in sc["drop_points"]]
    assert dps == [[50.0, 0.0, 1.5], [50.0, 2.0, 1.5], [50.0, 4.0, 1.5]], dps
    # 零树（真机占用=激光点云；forest 段混入=杨树林文件误装订）
    assert not any(z.get("kind") == "forest" for z in sc["zones"])
    # 栅格覆盖：venue 中心原点口径（g_x0=-size/2），须罩住 0..50 且边距 ≥5m
    assert sc["venue"]["size"][0] >= 110 and sc["venue"]["size"][1] >= 110
    fences = [o for o in sc["static_obstacles"] if o["kind"] == "fence"]
    assert len(fences) == 4
    rl = yaml.safe_load(open(
        os.path.join(HERE, "competition_rules_real.yaml"), encoding="utf-8"))
    h = rl["heights"]
    assert h["drop_hover_z"] == 1.5 and h["identify_z"] == 1.9 \
        and h["drop_mid_z"] == 1.70, h          # 用户口径：1.5=释放悬停高
    assert h["cruise_z"] == 1.2 and h["flight_z_ceiling"] == 4.5
    assert rl["time_limit_s"] == 1500           # 真机 25min（sim 压缩 180 勿带上机）
    assert rl["rule"]["geofence"][2] == [50.35, 50.35]
    assert rl["rule"]["return_through_forest"] is False


def main():
    t0 = time.time()
    test_vel_inverse()
    print("[1/5] vel_bridge.world_to_local × PoseTF 速度旋转互逆  OK")
    test_cloud_same_frame()
    print("[2/5] PoseTF.point == apply.xy（点云/位姿同帧）       OK")
    test_fleet_downlink()
    print("[3/5] fleet 下行 E2E（注册/快照/剔除/任务）            OK")
    test_deploy_files()
    print("[4/5] settings_real + profile_real_lio + launch 断言   OK")
    test_real_venue()
    print("[5/5] 真场地 scene/rules（用户坐标/零树/高度链）      OK")
    print("ALL OK (%.2fs)" % (time.time() - t0))


if __name__ == "__main__":
    main()
