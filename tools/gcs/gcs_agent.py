#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gcs_agent.py — 非凸α 地面站机载代理（GCS v0 第 1 步：先"看得见"）。

设计铁律（zx2026 W4/W5 法证结论）：
  1. ROS 图不过 WiFi——agent 只把**遥测摘要**打包成 TCP JSON 行流发给地面站，
     断线 3s 重连；消息即心跳（5Hz），seq 单调，地面站按数据年龄判死活。
  2. 真机每机一个 agent（ids=[DRONE_ID]），仿真单 master 可一进程带 6 机
     （ids=[0..5]）。全部 topic 来自 profile yaml——仿真/真机只换配置不改码。
  3. 本节点只读不控：指令走第 3 步指令层，本步零风险上机。

用法：
  机载:  python3 gcs_agent.py --profile profile_real_lio.yaml
  仿真:  python3 gcs_agent.py --profile profile_sim.yaml
"""
import argparse
import base64
import json
import math
import socket
import sys
import threading
import time

import rospy
import yaml
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String


class Agent(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.ids = [str(i) for i in cfg["ids"]]
        self.lock = threading.Lock()
        self.state = {i: self._blank() for i in self.ids}
        self.grids = {}    # did -> RLE 压缩占用栅格（第 4 步 ③：建图上屏）
        self.seq = 0
        self.tp = cfg["topics"]
        self.stage = None  # 全局阶段机（/zx2026/state），非 per-drone

    @staticmethod
    def _blank():
        return {"pos": None, "yaw": None, "vel": [0.0, 0.0, 0.0], "speed": 0.0,
                "bat": None, "bat_v": None, "fc": None, "connected": None,
                "phase": None, "plan_age": -1.0, "live_t": {}, "prev": None}

    # ---- ROS 采集 ----------------------------------------------------------
    def spin_ros(self):
        sub_odo = self.tp.get("odom")
        sub_pha = self.tp.get("phase")
        sub_bat = self.tp.get("battery")
        sub_fc = self.tp.get("fc")
        live = self.tp.get("liveness") or []
        for i in self.ids:
            if sub_odo:
                rospy.Subscriber(sub_odo.format(id=i), Odometry,
                                 self._on_odom, i, queue_size=5)
            if sub_pha:
                rospy.Subscriber(sub_pha.format(id=i), String,
                                 self._on_phase, i, queue_size=2)
            if sub_bat:
                from sensor_msgs.msg import BatteryState
                rospy.Subscriber(sub_bat.format(id=i), BatteryState,
                                 self._on_bat, i, queue_size=1)
            if sub_fc:
                from mavros_msgs.msg import State
                rospy.Subscriber(sub_fc.format(id=i), State,
                                 self._on_fc, i, queue_size=1)
            for k, lt in enumerate(live):
                rospy.Subscriber(lt.format(id=i), rospy.AnyMsg,
                                 self._on_live, (i, k), queue_size=1)
            if self.tp.get("grid"):
                rospy.Subscriber(self.tp["grid"].format(id=i), OccupancyGrid,
                                 self._on_grid, i, queue_size=1)
        stp = self.tp.get("stage")
        if stp:
            rospy.Subscriber(stp, String, self._on_stage, queue_size=2)

    def _on_odom(self, msg, i):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = time.time()
        with self.lock:
            st = self.state[i]
            v = [msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                 msg.twist.twist.linear.z]
            if max(abs(c) for c in v) < 1e-6 and st["prev"]:
                # twist 缺失/恒零 → 位置差分（faster_lio 等只发位姿的源）
                pp, pt = st["prev"]
                dt = now - pt
                if dt > 0.05:
                    v = [(p.x - pp[0]) / dt, (p.y - pp[1]) / dt,
                         (p.z - pp[2]) / dt]
            st["pos"] = [p.x, p.y, p.z]
            st["yaw"] = math.degrees(yaw)
            st["vel"] = v
            st["speed"] = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
            st["prev"] = ([p.x, p.y, p.z], now)
            st["live_t"]["odom"] = now

    def _on_phase(self, msg, i):
        with self.lock:
            self.state[i]["phase"] = str(msg.data)
            self.state[i]["live_t"]["phase"] = time.time()

    def _on_bat(self, msg, i):
        with self.lock:
            st = self.state[i]
            st["bat"] = round(msg.percentage * (100.0 if msg.percentage <= 1.0
                                                else 1.0), 1)
            st["bat_v"] = round(msg.voltage, 2)

    def _on_fc(self, msg, i):
        with self.lock:
            st = self.state[i]
            st["fc"] = str(msg.mode)
            st["connected"] = bool(msg.connected)

    def _on_stage(self, msg):
        with self.lock:
            self.stage = str(msg.data)

    def _on_live(self, _msg, key):
        i, _k = key
        with self.lock:
            self.state[i]["live_t"]["plan"] = time.time()

    @staticmethod
    def _rle_grid(msg):
        """OccupancyGrid → RLE 压缩字典（值域 0/1/2=空闲/占用/未知，游程 ≤255）。

        全场 0.5m 栅格 ~1.7 万格，林地图 RLE 后典型几百字节；base64 编码
        保持 JSON 行流 ascii 安全。快照专用通道（不进遥测 JSONL）。
        未知(-1)单列一值：大图可区分"已探索空地"与"未探索"。
        """
        w, h = int(msg.info.width), int(msg.info.height)
        out = bytearray()
        prev, run = -1, 0
        for v in msg.data:
            b = 1 if v > 50 else (2 if v < 0 else 0)
            if b == prev and run < 255:
                run += 1
            else:
                if prev >= 0:
                    out.append(prev)
                    out.append(run)
                prev, run = b, 1
        if prev >= 0:
            out.append(prev)
            out.append(run)
        return {"w": w, "h": h, "res": float(msg.info.resolution),
                "x0": float(msg.info.origin.position.x),
                "y0": float(msg.info.origin.position.y),
                "rle": base64.b64encode(bytes(out)).decode("ascii")}

    def _on_grid(self, msg, i):
        try:
            g = self._rle_grid(msg)
        except Exception:
            return
        with self.lock:
            self.grids[i] = g

    # ---- TCP 发送（client 主动连地面站，断线重连，只发最新快照） ----------
    def spin_sender(self):
        host, port = self.cfg["server"]
        rate = float(self.cfg.get("send_hz", 5.0))
        while not rospy.is_shutdown():
            try:
                sock = socket.create_connection((host, port), timeout=3.0)
                rospy.loginfo("gcs_agent: connected %s:%d", host, port)
                while not rospy.is_shutdown():
                    batch = self.snapshot()
                    sock.sendall(
                        (json.dumps(batch, separators=(",", ":")) + "\n")
                        .encode("ascii"))
                    time.sleep(1.0 / rate)
            except Exception as e:
                rospy.logwarn_throttle(5.0, "gcs_agent: link down (%s), "
                                       "retry 3s", e)
                time.sleep(3.0)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def snapshot(self):
        now = time.time()
        self.seq += 1
        drones = {}
        grids = {}
        with self.lock:
            for i in self.ids:
                st = self.state[i]
                lt = st["live_t"]
                drones[i] = {
                    "pos": st["pos"], "yaw": st["yaw"], "speed": st["speed"],
                    "vel": st["vel"], "bat": st["bat"], "bat_v": st["bat_v"],
                    "fc": st["fc"], "connected": st["connected"],
                    "phase": st["phase"], "ts": lt.get("odom", 0.0),
                    "plan_age": (now - lt["plan"]) if "plan" in lt else -1.0,
                }
            grids = {i: g for i, g in self.grids.items() if g is not None}
        out = {"agent_ts": now, "seq": self.seq, "stage": self.stage,
               "drones": drones}
        if self.seq % 5 == 0 and grids:
            # 建图栅格 1Hz 随流捎带（5Hz 消息每 5 帧一次），独立顶层键——
            # 不进 drones 字典，遥测 JSONL 与看门狗零影响
            out["grids"] = grids
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", required=True)
    args = ap.parse_args()
    with open(args.profile, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    host, port = str(cfg["server"]).rsplit(":", 1)
    cfg["server"] = (host, int(port))

    # 等 master 就绪再 init（真机开机自启时 master/驱动晚于 agent；仿真联调
    # 时 agent 先挂、verify 后跑——消除编排竞态）
    for _ in range(240):
        try:
            rospy.init_node("gcs_agent", anonymous=False, disable_signals=True)
            break
        except Exception:
            time.sleep(5)
    else:
        sys.exit("gcs_agent: no ROS master in 20min")
    a = Agent(cfg)
    a.spin_ros()
    threading.Thread(target=a.spin_sender, daemon=True).start()
    rospy.loginfo("gcs_agent: up ids=%s -> %s:%d", a.ids, host,
                  cfg["server"][1])
    rospy.spin()


if __name__ == "__main__":
    main()
