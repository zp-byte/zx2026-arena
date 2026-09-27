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
  真机邻机感知: 加 --fleet-bridge（下行第二连接收 hub fleet 快照，转 ROS：
    邻居 → /drone_<id>/odom_from_<s>（nav comms 契约）；stage → /zx2026/state
    （仅本机无本地 stage 源时，防回声倒退竞态）；他机任务进度 →
    /zx2026/task_update）
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
from std_msgs.msg import String, Bool, Float32MultiArray


class Agent(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.ids = [str(i) for i in cfg["ids"]]
        self.lock = threading.Lock()
        self.state = {i: self._blank() for i in self.ids}
        self.grids = {}    # did -> RLE 压缩占用栅格（第 4 步 ③：建图上屏）
        self.grid_acc = {}  # did -> {(gx,gy):1} 世界格键累积（P1 帧间 OR 合并）
        self.scores = {}   # did -> 比分明细（scorekeeper /zx2026/score/<id>）
        self.missions = {}  # did -> 任务指派（/zx2026/mission/<id>）
        self.tasks = {}    # did -> 任务进度事件（/zx2026/task_update，LANDED/RETIRED）
        self.score_total = None  # 赛末总分 dict（/zx2026/score_summary 解析）
        self.seq = 0
        self.tp = cfg["topics"]
        self.stage = None  # 全局阶段机（/zx2026/state），非 per-drone
        # pose_tf（B 方案 2026-09-25）：真机 LIO 局部系 → 场地 ENU。
        # p_e = R(yaw0)·(p_l − c) + [ex0, ny0]；z 不转（yaw 旋转不涉 u 轴）。
        # sim 档案无 pose_tf 段 → tf_on=False 恒等零开销。
        tf = cfg.get("pose_tf") or {}
        self.tf_yaw0 = math.radians(float(tf.get("yaw0_deg", 0.0)))
        self.tf_c = math.cos(self.tf_yaw0)
        self.tf_s = math.sin(self.tf_yaw0)
        self.tf_ex0 = float(tf.get("ex0", 0.0))
        self.tf_ny0 = float(tf.get("ny0", 0.0))
        self.tf_auto = bool(tf.get("auto_origin", False))
        self.tf_static_n = int(tf.get("static_frames", 50))
        self.tf_gate = float(tf.get("static_gate_m", 0.3))
        self.tf_on = bool(tf) and (
            self.tf_yaw0 != 0.0 or self.tf_ex0 != 0.0 or self.tf_ny0 != 0.0
            or self.tf_auto)
        self.tf_acc = {i: {"buf": [], "c": (0.0, 0.0), "locked": False,
                           "gave_up": False} for i in self.ids}
        # fleet 下行桥（--fleet-bridge，真机邻机感知 2026-09-26）：独立第二
        # TCP 连接（上行只发不收——复用同一 socket 做收发会让 sender 死等），
        # 收 hub fleet 快照转 ROS。惰性建 pub（单 fleet 线程内，无竞态）。
        self.fleet_bridge = False
        self.fleet_pubs = {}
        self.stage_echo_pub = None
        self.stage_echo_last = None
        self.task_pub = None
        self.task_ok = None   # None=未探测；False=zx2026_common 缺失已降级

    @staticmethod
    def _blank():
        return {"pos": None, "yaw": None, "vel": [0.0, 0.0, 0.0], "speed": 0.0,
                "bat": None, "bat_v": None, "fc": None, "connected": None,
                "phase": None, "plan_age": -1.0, "live_t": {}, "prev": None,
                # FR-1.7 碰撞/clearance（nav_metrics 11 项布局，取 0/6 两项）
                "coll": 0, "min_clr": None,
                # FR-1.8 投放事件/首投时间戳
                "matched": None, "drop_done": False, "first_drop_t": None,
                # FR-1.9 匹配过程可视化
                "match_progress": None}

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
            # FR-1.8 投放事件 / FR-1.9 匹配过程 / FR-1.7 碰撞指标（sim 先通）
            if self.tp.get("match"):
                rospy.Subscriber(self.tp["match"].format(id=i), String,
                                 self._on_match, i, queue_size=1)
            if self.tp.get("drop_done"):
                rospy.Subscriber(self.tp["drop_done"].format(id=i), Bool,
                                 self._on_drop_done, i, queue_size=1)
            if self.tp.get("metrics"):
                rospy.Subscriber(self.tp["metrics"].format(id=i),
                                 Float32MultiArray, self._on_metrics, i,
                                 queue_size=1)
            if self.tp.get("match_progress"):
                rospy.Subscriber(self.tp["match_progress"].format(id=i),
                                 String, self._on_match_progress, i,
                                 queue_size=1)
            # FR-1.4 真机建图：PointCloud2 → RLE 栅格（sim 用 OccupancyGrid，
            # 此键 null；真机 LIO/VIO 累积点云现场定 topic 后填入）
            if self.tp.get("grid_cloud"):
                from sensor_msgs.msg import PointCloud2
                rospy.Subscriber(self.tp["grid_cloud"].format(id=i),
                                 PointCloud2, self._on_grid_cloud, i,
                                 queue_size=1)
        stp = self.tp.get("stage")
        if stp:
            rospy.Subscriber(stp, String, self._on_stage, queue_size=2)
        # 比分/任务指派/任务进度（sim scorekeeper latched；真机 profile 为 null
        # 自动跳过；车载无 zx2026_common 时仅告警降级，不影响其余采集）
        if self.tp.get("score") or self.tp.get("mission") or self.tp.get("task"):
            try:
                from zx2026_common.msg import Mission as MissionMsg
                from zx2026_common.msg import Score as ScoreMsg
                from zx2026_common.msg import TaskUpdate as TaskMsg
            except ImportError:
                rospy.logwarn("gcs_agent: zx2026_common.msg 不可用 — "
                              "score/mission/task 采集降级关闭")
            else:
                if self.tp.get("score"):
                    for i in self.ids:
                        rospy.Subscriber(self.tp["score"].format(id=i),
                                         ScoreMsg, self._on_score, i,
                                         queue_size=1)
                if self.tp.get("mission"):
                    for i in self.ids:
                        rospy.Subscriber(self.tp["mission"].format(id=i),
                                         MissionMsg, self._on_mission, i,
                                         queue_size=1)
                if self.tp.get("task"):
                    rospy.Subscriber(self.tp["task"], TaskMsg,
                                     self._on_task, queue_size=5)
        # 赛末总分条（std_msgs/String latched，scorekeeper 赛毕发布）
        susp = self.tp.get("summary")
        if susp:
            rospy.Subscriber(susp, String, self._on_summary, queue_size=2)

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
            px, py, pyaw, vx, vy = p.x, p.y, yaw, v[0], v[1]
            if self.tf_on:
                # LIO 局部系 → ENU 场地系；prev 恒存原始坐标，差分基准不变
                px, py, pyaw, vx, vy = self._tf_apply(i, p.x, p.y, yaw,
                                                      v[0], v[1], now)
            st["pos"] = [px, py, p.z]
            st["yaw"] = math.degrees(pyaw)
            st["vel"] = [vx, vy, v[2]]
            st["speed"] = math.sqrt(vx * vx + vy * vy + v[2] * v[2])
            st["prev"] = ([p.x, p.y, p.z], now)
            st["live_t"]["odom"] = now

    def _tf_sample(self, i, x, y, now):
        """auto_origin：起飞前静止窗采 LIO 初始化偏置 c（前 N 帧均值）。

        窗内位移超 static_gate_m（=agent 在空中重启，无静止窗）→ 永久
        放弃并保持 c=(0,0) 告警——不猜。锁定后不再采样。
        """
        if not self.tf_auto:
            return (0.0, 0.0)
        a = self.tf_acc[i]
        if a["locked"] or a["gave_up"]:
            return a["c"]
        a["buf"].append((x, y))
        if len(a["buf"]) > 1:
            x0, y0 = a["buf"][0]
            if math.hypot(x - x0, y - y0) > self.tf_gate:
                a["gave_up"] = True
                print("[agent] pose_tf auto_origin ABORT did=%s (moving at"
                      " startup) — c=(0,0)" % i, flush=True)
                return a["c"]
        if len(a["buf"]) >= self.tf_static_n:
            n = len(a["buf"])
            a["c"] = (sum(b[0] for b in a["buf"]) / n,
                      sum(b[1] for b in a["buf"]) / n)
            a["locked"] = True
            print("[agent] pose_tf auto_origin LOCKED did=%s c=(%.3f,%.3f)"
                  % (i, a["c"][0], a["c"][1]), flush=True)
        return a["c"]

    def _tf_apply(self, i, x, y, yaw, vx, vy, now):
        """p_e = R(yaw0)·(p_l − c) + [ex0, ny0]；速度只转不移；yaw 加 yaw0。"""
        cx, cy = self._tf_sample(i, x, y, now)
        dx, dy = x - cx, y - cy
        return (dx * self.tf_c - dy * self.tf_s + self.tf_ex0,
                dx * self.tf_s + dy * self.tf_c + self.tf_ny0,
                yaw + self.tf_yaw0,
                vx * self.tf_c - vy * self.tf_s,
                vx * self.tf_s + vy * self.tf_c)

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

    def _on_score(self, msg, i):
        with self.lock:
            self.scores[i] = {"score": int(msg.score),
                              "s1": int(getattr(msg, "s1", msg.score)),
                              "correct": int(msg.correct_drops),
                              "wrong": int(msg.wrong_drops),
                              "state": str(msg.mission_state),
                              "retire": int(getattr(msg, "retire_state", 0))}

    def _on_mission(self, msg, i):
        with self.lock:
            self.missions[i] = {"payload": str(msg.payload_type),
                                "drop": int(msg.drop_point_id),
                                "box_color": str(getattr(msg, "box_color", "")),
                                "marker": str(msg.marker_id),
                                "seq": int(msg.mission_seq)}

    def _on_task(self, msg):
        # 全局任务进度事件（LANDED/RETIRED 等）——S2 落地计数的实时来源
        with self.lock:
            self.tasks[str(msg.drone_id)] = {
                "state": str(msg.mission_state),
                "payload": str(msg.payload_type),
                "drop": int(msg.drop_point_id),
                "drop_ok": int(msg.drop_ok)}

    def _on_summary(self, msg):
        # scorekeeper 赛末总分条："total=.. s1=.. s2=.. landed=.. .. <path>"；
        # 同话题的 per-drone 行无 total= 键，解析后自然丢弃
        d = {}
        for tok in str(msg.data).split():
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            try:
                d[k] = int(v)
            except ValueError:
                pass
        if "total" in d:
            with self.lock:
                self.score_total = d

    # ---- FR-1.8/1.9/1.7 采集（随流捎带到地面站，只读） ----------------------
    def _on_match(self, msg, i):
        with self.lock:
            self.state[i]["matched"] = str(msg.data)

    def _on_drop_done(self, msg, i):
        if not msg.data:
            return
        now = time.time()
        with self.lock:
            st = self.state[i]
            st["drop_done"] = True
            if st["first_drop_t"] is None:
                st["first_drop_t"] = now

    def _on_metrics(self, msg, i):
        # 布局：0 碰撞 1 卡滞s 2 卡滞段 3 最长卡滞 4 翻转 5 距离 6 min_clear
        #       7 max_speed 8 sim_t 9 speed 10 clear
        d = msg.data
        if len(d) < 7:
            return
        with self.lock:
            st = self.state[i]
            st["coll"] = int(d[0])
            st["min_clr"] = round(float(d[6]), 3)

    def _on_match_progress(self, msg, i):
        with self.lock:
            self.state[i]["match_progress"] = str(msg.data)

    def _on_live(self, _msg, key):
        i, _k = key
        with self.lock:
            self.state[i]["live_t"]["plan"] = time.time()

    @staticmethod
    def _rle_encode(values):
        """值序列(0/1/2) → RLE bytes → base64 str（游程 ≤255）。

        值域 0=空闲/1=占用/2=未知。OccupancyGrid 和 PointCloud2 栅格化共用。
        """
        out = bytearray()
        prev, run = -1, 0
        for v in values:
            if v == prev and run < 255:
                run += 1
            else:
                if prev >= 0:
                    out.append(prev)
                    out.append(run)
                prev, run = v, 1
        if prev >= 0:
            out.append(prev)
            out.append(run)
        return base64.b64encode(bytes(out)).decode("ascii")

    @staticmethod
    def _rle_grid(msg):
        """OccupancyGrid → RLE 压缩字典（值域 0/1/2=空闲/占用/未知，游程 ≤255）。

        全场 0.5m 栅格 ~1.7 万格，林地图 RLE 后典型几百字节；base64 编码
        保持 JSON 行流 ascii 安全。快照专用通道（不进遥测 JSONL）。
        未知(-1)单列一值：大图可区分"已探索空地"与"未探索"。
        """
        w, h = int(msg.info.width), int(msg.info.height)
        values = [1 if v > 50 else (2 if v < 0 else 0) for v in msg.data]
        return {"w": w, "h": h, "res": float(msg.info.resolution),
                "x0": float(msg.info.origin.position.x),
                "y0": float(msg.info.origin.position.y),
                "rle": Agent._rle_encode(values)}

    def _on_grid(self, msg, i):
        try:
            g = self._rle_grid(msg)
        except Exception:
            return
        with self.lock:
            self.grids[i] = g

    def _on_grid_cloud(self, msg, i):
        """FR-1.4 真机建图（P1 累积版）：PointCloud2 → x-y 投影 → 帧间
        OR 合并 → 0.5m 栅格 → RLE。

        侦察建图语义=累积全场（单帧投影只见当前扫描窗，MAP 窗会闪没）；
        LIO 漂移在比赛 10min 尺度内不纠（展示用途，诚实口径：MAP 窗
        ≈LIO 累积图）。sim OccupancyGrid 路径（_on_grid）不动。
        """
        try:
            from sensor_msgs import point_cloud2
            pts = list(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True))
            cells = self._cloud_cells(pts)
            if not cells:
                return
            with self.lock:
                acc = self.grid_acc.setdefault(i, {})
                acc.update({k: 1 for k in cells})
                self.grids[i] = self._acc_grid(acc)
        except Exception:
            return

    # 累积栅格参数（类常量：纯逻辑 classmethod 可离线直测，不碰 ROS）
    GRID_RES = 0.5       # m，MAP 窗栅格分辨率（与 sim OccupancyGrid 同口径）
    GRID_Z = (0.3, 4.0)  # z 带：地面回波以下、树冠以上

    @classmethod
    def _cloud_cells(cls, pts):
        """点列 → 世界坐标占用格键集合（z 带内；纯逻辑可离线测）。"""
        zlo, zhi = cls.GRID_Z
        res = cls.GRID_RES
        return {(int(math.floor(x / res)), int(math.floor(y / res)))
                for (x, y, z) in pts if zlo <= z <= zhi}

    @classmethod
    def _acc_grid(cls, acc):
        """累积格键 dict → RLE 栅格 dict（行翻转原点左下；空=None）。"""
        if not acc:
            return None
        res = cls.GRID_RES
        gxs = [k[0] for k in acc]
        gys = [k[1] for k in acc]
        gx0, gy0 = min(gxs), min(gys)
        w = max(gxs) - gx0 + 1
        h = max(gys) - gy0 + 1
        if w * h > 4000000:
            return None
        cells = [2] * (w * h)          # 无点=未知(2)，有点=占用(1)
        for (gx, gy) in acc:
            cells[(h - 1 - (gy - gy0)) * w + (gx - gx0)] = 1
        return {"w": w, "h": h, "res": res,
                "x0": gx0 * res, "y0": gy0 * res,
                "rle": Agent._rle_encode(cells)}

    # ---- fleet 下行桥（--fleet-bridge，真机邻机感知 2026-09-26） ------------
    # nav comms 契约：comms.enabled 时邻居订阅 /drone_<id>/odom_from_<s>，
    # 新鲜度按到达时刻判（nav_node._on_neighbor 用 rospy.get_time()，不读
    # header.stamp）——转发时戳用当前时刻即可，expire 门由 nav 侧管。
    def spin_fleet(self):
        host, port = self.cfg["server"]
        while not rospy.is_shutdown():
            try:
                sock = socket.create_connection((host, port), timeout=3.0)
                sock.sendall(b'{"down":1}\n')   # 下行兴趣声明（hub 入池凭据）
                rospy.loginfo("gcs_agent: fleet downlink up %s:%d", host,
                              port)
                buf = b""
                while not rospy.is_shutdown():
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise IOError("closed by hub")
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            self.on_fleet_line(
                                json.loads(line.decode("ascii")))
                        except Exception:
                            continue
            except Exception as e:
                rospy.logwarn_throttle(
                    5.0, "gcs_agent: fleet downlink down (%s), retry 3s", e)
                time.sleep(3.0)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def on_fleet_line(self, obj):
        now = rospy.get_rostime()
        # 邻居状态 → odom_from_<s>（为本 master 的每架机各发一份；sim 单
        # master 全员直订 /drone_<j>/odom，不走本桥）
        fleet = obj.get("fleet") or {}
        for did, d in fleet.items():
            if did in self.ids:
                continue
            pos = d.get("pos")
            if not pos or len(pos) < 3:
                continue
            v = d.get("vel") or [0.0, 0.0, 0.0]
            yaw = math.radians(float(d.get("yaw") or 0.0))
            half = yaw * 0.5
            for me in self.ids:
                m = Odometry()
                m.header.stamp = now
                m.header.frame_id = "world"
                m.child_frame_id = "drone_%s" % did
                m.pose.pose.position.x = float(pos[0])
                m.pose.pose.position.y = float(pos[1])
                m.pose.pose.position.z = float(pos[2])
                m.pose.pose.orientation.z = math.sin(half)
                m.pose.pose.orientation.w = math.cos(half)
                m.twist.twist.linear.x = float(v[0])
                m.twist.twist.linear.y = float(v[1])
                m.twist.twist.linear.z = float(v[2])
                self._fleet_pub(me, did).publish(m)
        # 全局阶段机回声：仅本机无本地 stage 源时转发（profile topics.stage
        # 为 null）。1 号机跑 stage_controller 本地直发——回声会把滞后旧值
        # 倒灌回订户（阶段倒退竞态），必须跳过。
        stage = obj.get("stage")
        if stage and not self.tp.get("stage"):
            if self.stage_echo_pub is None:
                self.stage_echo_pub = rospy.Publisher(
                    "/zx2026/state", String, queue_size=1, latch=True)
            if stage != self.stage_echo_last:
                self.stage_echo_pub.publish(String(str(stage)))
                self.stage_echo_last = stage
        # 他机任务进度 → /zx2026/task_update（halt_on_landed 的落地机集合；
        # 本机进度由本地 mission_executor 直发，跳过防重复）
        tasks = obj.get("tasks") or {}
        if tasks and self.task_ok is not False:
            if self.task_pub is None:
                try:
                    from zx2026_common.msg import TaskUpdate as TaskMsg
                except ImportError:
                    rospy.logwarn("gcs_agent: zx2026_common.msg 不可用 — "
                                  "fleet task 转发降级关闭")
                    self.task_ok = False
                    return
                self.task_ok = True
                self.task_cls = TaskMsg
                self.task_pub = rospy.Publisher("/zx2026/task_update",
                                                TaskMsg, queue_size=5)
            for did, t in tasks.items():
                if did in self.ids:
                    continue
                m = self.task_cls()
                m.drone_id = int(did)
                m.payload_type = str(t.get("payload", ""))
                m.drop_point_id = int(t.get("drop", 0))
                m.mission_state = str(t.get("state", ""))
                m.drop_ok = int(t.get("drop_ok", 0))
                self.task_pub.publish(m)

    def _fleet_pub(self, me, did):
        key = (me, did)
        p = self.fleet_pubs.get(key)
        if p is None:
            p = rospy.Publisher("/drone_%s/odom_from_%s" % (me, did),
                                Odometry, queue_size=5)
            self.fleet_pubs[key] = p
        return p

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
        scores = {}
        missions = {}
        with self.lock:
            tasks = dict(self.tasks)
            total = dict(self.score_total) if self.score_total else None
            for i in self.ids:
                st = self.state[i]
                lt = st["live_t"]
                drones[i] = {
                    "pos": st["pos"], "yaw": st["yaw"], "speed": st["speed"],
                    "vel": st["vel"], "bat": st["bat"], "bat_v": st["bat_v"],
                    "fc": st["fc"], "connected": st["connected"],
                    "phase": st["phase"], "ts": lt.get("odom", 0.0),
                    "plan_age": (now - lt["plan"]) if "plan" in lt else -1.0,
                    # FR-1.7/1.8/1.9 新增字段（随流捎带，hub 透出到面板）
                    "coll": st["coll"], "min_clr": st["min_clr"],
                    "matched": st["matched"], "drop_done": st["drop_done"],
                    "first_drop_t": st["first_drop_t"],
                    "match_progress": st["match_progress"],
                }
            grids = {i: g for i, g in self.grids.items() if g is not None}
            scores = dict(self.scores)
            missions = dict(self.missions)
        out = {"agent_ts": now, "seq": self.seq, "stage": self.stage,
               "drones": drones}
        if self.seq % 5 == 0:
            # 栅格/比分/任务指派 1Hz 随流捎带（5Hz 消息每 5 帧一次），独立
            # 顶层键——不进 drones 字典，遥测 JSONL 与看门狗零影响
            if grids:
                out["grids"] = grids
            if scores:
                out["scores"] = scores
            if missions:
                out["missions"] = missions
            if tasks:
                out["tasks"] = tasks
            if total:
                out["score_total"] = total
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", required=True)
    ap.add_argument("--fleet-bridge", action="store_true",
                    help="开启 fleet 下行桥（真机邻机感知：hub 快照→"
                         "odom_from_<s>/zx2026/state/task_update）")
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
    a.fleet_bridge = bool(args.fleet_bridge)
    a.spin_ros()
    threading.Thread(target=a.spin_sender, daemon=True).start()
    if a.fleet_bridge:
        threading.Thread(target=a.spin_fleet, daemon=True).start()
    rospy.loginfo("gcs_agent: up ids=%s -> %s:%d fleet_bridge=%s", a.ids,
                  host, cfg["server"][1], a.fleet_bridge)
    rospy.spin()


if __name__ == "__main__":
    main()
