#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gcs_mavlink_bridge.py — 贫机 MAVLink→GCS 桥（领从架构 P0-③ 通信底座）。

贫机（PX4 + RTK，无机载算力）不做 agent——本进程在地面站侧充当
"贫机虚拟 agent"：
  上行：收 MAVLink（UDP/串口）→ 翻译成 agent 同构 JSON 批量 → hub 9870
        （hub 零改动：LINK_LOST/STALL/空管/仪表全部自动生效）
  下行：TCP 命令口（--cmd-port）收 ops 的 JSON 行命令 → MAVLink 发出
        land / rtl / upload_mission（带 ack 闭环 + 超时重发）

坐标系（FR-4.1）：GPS/RTK 是 WGS84，hub 围栏/投放点是局部米制——
ENU 切平面投影，原点=起降区（--enu-lat/--enu-lon），与 goals_enu.yaml
同一口径。mission 航点走 MAV_FRAME_GLOBAL_RELATIVE_ALT（z=相对起飞高，
与 ENU 的 u 同语义）。

用法：
  python3 gcs_mavlink_bridge.py --connect udpin:0.0.0.0:14550 \
      --map "1:0,2:1" --enu-lat 30.123456 --enu-lon 120.654321
"""
import argparse
import json
import math
import socket
import socketserver
import threading
import time

# PX4 custom_mode 主模式解码（custom_mode>>16 & 0xFF）
PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
            5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
GPS_FIX = {1: "NOGPS", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLOAT",
           6: "RTK_FIXED", 7: "STATIC"}


def wgs84_to_enu(lat, lon, lat0, lon0):
    """WGS84 → 局部 ENU 切平面（场地 60m 尺度，球面近似误差微米级）。"""
    mlat = 110540.0          # 米/度纬度
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return ((lon - lon0) * mlon, (lat - lat0) * mlat)


def enu_to_wgs84(e, n, lat0, lon0):
    mlat = 110540.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return (lat0 + n / mlat, lon0 + e / mlon)


class Bridge(object):
    def __init__(self, connect, ids_map, lat0, lon0, hub_addr, cmd_port,
                 send_hz=5.0):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.mav = mavutil.mavlink_connection(
            connect, source_system=255,
            source_component=190)  # MAV_COMP_ID_MISSIONPLANNER
        self.ids_map = ids_map       # px4 sysid(int) -> gcs did(str)
        self.lat0, self.lon0 = lat0, lon0
        self.hub_addr = hub_addr
        self.cmd_port = cmd_port
        self.send_hz = send_hz
        self.lock = threading.Lock()  # mav 对象单写
        # per-贫机 遥测缓存（did -> dict）
        self.buf = {did: {} for did in ids_map.values()}
        self.hub = None              # hub TCP socket（断线重连）
        self.hub_ts = 0.0
        self.seq = 0
        # mission 上传会话（一次一架；帧循环在收包线程内驱动）
        self.mission = None          # {"did","wps","expect_seq","t0","retry"}

    # ---- 上行：MAVLink → agent JSON ----------------------------------------
    def _feed(self, did, key, val):
        b = self.buf[did]
        if b.get(key) != val:
            b[key] = val

    def _translate(self, msg):
        import pymavlink.dialects.v20.all as mav
        sysid = msg.get_srcSystem()
        did = self.ids_map.get(sysid)
        if did is None:
            return
        t = msg.get_type()
        if t == "HEARTBEAT":
            main = (msg.custom_mode >> 16) & 0xFF
            self._feed(did, "fc", PX4_MAIN.get(main, "MODE%d" % main))
            self._feed(did, "connected", bool(
                msg.base_mode & mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
        elif t == "GLOBAL_POSITION_INT":
            e, n = wgs84_to_enu(msg.lat * 1e-7, msg.lon * 1e-7,
                                self.lat0, self.lon0)
            u = msg.relative_alt * 1e-3
            vx, vy, vz = msg.vx * 1e-2, msg.vy * 1e-2, msg.vz * 1e-2
            self._feed(did, "pos", [round(e, 3), round(n, 3), round(u, 3)])
            self._feed(did, "vel", [vx, vy, vz])
            self._feed(did, "speed", round(math.sqrt(vx * vx + vy * vy), 3))
        elif t == "SYS_STATUS":
            r = msg.battery_remaining
            self._feed(did, "bat", None if r < 0 else float(r))
        elif t == "GPS_RAW_INT":
            self._feed(did, "rtk", GPS_FIX.get(msg.fix_type, "?"))
            self._feed(did, "sats", msg.satellites_visible)
        elif t == "ATTITUDE":
            self._feed(did, "yaw",
                       round(math.degrees(msg.yaw) % 360.0, 1))
        elif t == "STATUSTEXT":
            self._feed(did, "statustext", "%d:%s" % (msg.severity,
                                                     msg.text[:60]))

    def _push_hub(self):
        """按 agent 批量格式喂 hub（fake_agent_test 同构），断线重连。"""
        now = time.time()
        drones = {}
        for did, b in self.buf.items():
            if not b:
                continue
            d = dict(b)
            d.setdefault("phase", None)
            d["ts"] = round(now, 3)
            drones[did] = d
        if not drones:
            return
        obj = {"agent_ts": round(now, 3), "seq": self.seq,
               "drones": drones}
        self.seq += 1
        try:
            if self.hub is None:
                self.hub = socket.create_connection(self.hub_addr, timeout=3)
            self.hub.sendall((json.dumps(obj, separators=(",", ":")) +
                              "\n").encode("ascii"))
        except Exception:
            try:
                self.hub.close()
            except Exception:
                pass
            self.hub = None

    def rx_loop(self):
        while True:
            m = self.mav.recv_match(blocking=True, timeout=1.0)
            if m is not None:
                self._translate(m)
            self._mission_drive()
            if time.time() - self.hub_ts >= 1.0 / self.send_hz:
                self.hub_ts = time.time()
                self._push_hub()

    # ---- mission 协议驱动（收包线程内推进状态机） ---------------------------
    def start_upload(self, did, wps):
        """wps: [(e,n,z), ...] ENU 米制 → RELATIVE_ALT frame mission。"""
        if self.mission is not None:
            return "busy"
        sysid = next((s for s, d in self.ids_map.items() if d == did), None)
        if sysid is None or not wps:
            return "bad_arg"
        items = []
        for seq, (e, n, z) in enumerate(wps):
            lat, lon = enu_to_wgs84(e, n, self.lat0, self.lon0)
            items.append((seq, int(lat * 1e7), int(lon * 1e7), float(z)))
        self.mission = {"sysid": sysid, "items": items, "expect": 0,
                        "t0": time.time(), "retry": 0, "did": did}
        return "ok(%d wps)" % len(items)

    def _mission_drive(self):
        m = self.mission
        if m is None:
            return
        mav = self.mavutil.mavlink
        now = time.time()
        if now - m["t0"] > 20.0:
            if m["retry"] < 1:          # 超时重发一轮（start 重试纪律）
                m["retry"] += 1
                m["expect"] = 0
                m["t0"] = now
                self._send_count(m)
            else:
                self.mission = None
                print("[bridge] mission upload TIMEOUT did=%s"
                      % m["did"], flush=True)
            return
        if m["expect"] == 0:
            self._send_count(m)
            m["expect"] = -1            # 等 MISSION_REQUEST_INT(0)，
            return                      # 后续推进在 _on_request（rx 旁路）

    def _send_count(self, m):
        with self.lock:
            self.mav.mav.mission_count_send(
                m["sysid"], 1, len(m["items"]))

    def _on_request(self, msg):
        """MISSION_REQUEST(_INT) → 发对应航点；MISSION_ACK → 收尾。"""
        m = self.mission
        if m is None:
            return False
        if msg.get_srcSystem() != m["sysid"]:
            return False   # UDP 广播下别机（或别会话）的请求不认
        if msg.get_type() == "MISSION_REQUEST_INT":
            seq = msg.seq
            if 0 <= seq < len(m["items"]):
                s, lat, lon, z = m["items"][seq]
                mav = self.mavutil.mavlink
                with self.lock:
                    self.mav.mav.mission_item_int_send(
                        m["sysid"], 1, seq, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mav.MAV_CMD_NAV_WAYPOINT, 0, 0, 0.0, 0.0, 0.0,
                        float("nan"), int(lat), int(lon), z)
                m["expect"] = seq + 1
                m["t0"] = time.time()
                if seq + 1 >= len(m["items"]):
                    pass  # 等 MISSION_ACK
            return True
        if msg.get_type() == "MISSION_ACK":
            if msg.type == 0:   # MAV_MISSION_ACCEPTED
                print("[bridge] mission UPLOAD_OK did=%s (%d wps)"
                      % (m["did"], len(m["items"])), flush=True)
            else:
                print("[bridge] mission NACK did=%s type=%d"
                      % (m["did"], msg.type), flush=True)
            self.mission = None
            return True
        return False

    # ---- 下行命令口 ---------------------------------------------------------
    def _cmd_handle(self, line):
        try:
            c = json.loads(line.decode("ascii"))
        except Exception:
            return "bad_json"
        cmd = c.get("cmd")
        did = str(c.get("did"))
        sysid = next((s for s, d in self.ids_map.items() if d == did), None)
        if sysid is None:
            return "unknown_did"
        mav = self.mavutil.mavlink
        with self.lock:
            if cmd == "land":
                self.mav.mav.command_long_send(
                    sysid, 1, mav.MAV_CMD_NAV_LAND, 0,
                    0, 0, 0, 0, 0, 0, 0)
            elif cmd == "rtl":
                self.mav.mav.command_long_send(
                    sysid, 1, mav.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                    0, 0, 0, 0, 0, 0, 0)
            elif cmd == "upload_mission":
                return self.start_upload(did, c.get("wps") or [])
            elif cmd == "ping":
                return "pong"
            else:
                return "unknown_cmd"
        return "ok"

    def cmd_server(self):
        class H(socketserver.StreamRequestHandler):
            br = self

            def handle(self):
                for line in self.rfile:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        r = self.br._cmd_handle(line)
                    except Exception as e:
                        r = "err:%s" % str(e)[:80]
                    self.wfile.write((json.dumps({"ret": r}) + "\n")
                                     .encode("ascii"))

        class S(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        S(("0.0.0.0", self.cmd_port), H).serve_forever()

    # ---- GCS 心跳（1Hz，PX4 需见 GCS 心跳才持续全速遥测） -------------------
    def hb_loop(self):
        mav = self.mavutil.mavlink
        n = 0
        while True:
            with self.lock:
                self.mav.mav.heartbeat_send(
                    mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            n += 1
            if n % 5 == 0:   # 诊断：udp_server 广播目标集合
                print("[bridge] clients=%s udp_server=%s"
                      % (getattr(self.mav, "clients", None),
                         getattr(self.mav, "udp_server", None)), flush=True)
            time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="udpin:0.0.0.0:14550",
                    help="pymavlink 连接串（udpin:ip:port / serial:dev:baud）")
    ap.add_argument("--map", default="1:0,2:1,3:2,4:3,5:4,6:5",
                    help="PX4 sysid:GCS did 映射，如 \"1:0,2:1\"")
    ap.add_argument("--enu-lat", type=float, required=True,
                    help="ENU 原点纬度（起降区，与 goals_enu.yaml 同源）")
    ap.add_argument("--enu-lon", type=float, required=True,
                    help="ENU 原点经度")
    ap.add_argument("--hub", default="127.0.0.1:9870",
                    help="hub 地址 ip:port")
    ap.add_argument("--cmd-port", type=int, default=9871,
                    help="下行命令 TCP 口")
    ap.add_argument("--send-hz", type=float, default=5.0)
    args = ap.parse_args()
    ids_map = {}
    for kv in args.map.split(","):
        s, d = kv.split(":")
        ids_map[int(s)] = d.strip()
    hub_addr = tuple(args.hub.split(":"))
    hub_addr = (hub_addr[0], int(hub_addr[1]))
    br = Bridge(args.connect, ids_map, args.enu_lat, args.enu_lon,
                hub_addr, args.cmd_port, send_hz=args.send_hz)
    threading.Thread(target=br.cmd_server, daemon=True).start()
    threading.Thread(target=br.hb_loop, daemon=True).start()
    # request/ack 消息在收包循环里旁路给 mission 状态机
    _orig_translate = br._translate

    def translate_with_mission(msg):
        br._on_request(msg)
        _orig_translate(msg)

    br._translate = translate_with_mission
    print("[bridge] up connect=%s map=%s enu=(%.7f,%.7f) hub=%s cmd=%d"
          % (args.connect, args.map, args.enu_lat, args.enu_lon,
             args.hub, args.cmd_port), flush=True)
    br.rx_loop()


if __name__ == "__main__":
    main()
