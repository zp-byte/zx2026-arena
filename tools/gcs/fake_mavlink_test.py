#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_mavlink_test.py — bridge 链路离线自检（两台假 PX4，无需实机）。

时间线：
  0-20s  sysid1(d0) 从 (0,0) → (2,0)；sysid2(d1) 从 (5,0) → (1,0)
         对飞接近 → hub 应打 PROX_WARN / PROX_CRIT / PROX_OK(分离后无)
  t=15s  主线程经 cmd 口发 land(d0) → 假机1 应收 MAV_CMD_NAV_LAND
  t=18s  经 cmd 口 upload_mission(d0, 2 航点) → mission 协议闭环，
         fake 校验收到的 lat/lon 反算 ENU 与下发一致

输出 PASS/FAIL 断言由本脚本自检打印。
"""
import json
import math
import socket
import threading
import time

from pymavlink import mavutil

LAT0, LON0 = 30.0, 120.0
CMD_PORT = 9871
LAND_SEEN = threading.Event()
MISSION_OK = threading.Event()
MISSION_E = []
RX_TYPES = {}   # 收包诊断：类型->计数（下行方向是否到达）


def enu_to_ll(e, n):
    mlat = 110540.0
    mlon = 111320.0 * math.cos(math.radians(LAT0))
    return LAT0 + n / mlat, LON0 + e / mlon


def fake_px4(sysid, p0, p1, dur):
    m = mavutil.mavlink_connection("udpout:127.0.0.1:14555",
                                   source_system=sysid, source_component=1)
    mav = m.mav      # 协议对象（send 方法）
    en = mavutil.mavlink  # dialects 模块（枚举常量住这里，不在实例上）
    t0 = time.time()
    for _ in range(200):
        t = time.time() - t0          # 相对时间（绝对戳会让 frac 恒 1.0）
        frac = min(1.0, t / dur)
        e = p0[0] + (p1[0] - p0[0]) * frac
        n = p0[1] + (p1[1] - p0[1]) * frac
        lat, lon = enu_to_ll(e, n)
        mav.heartbeat_send(en.MAV_TYPE_QUADROTOR,
                           en.MAV_AUTOPILOT_PX4, 0, (4 << 16), 3)
        mav.global_position_int_send(
            int(t * 1000) & 0x7FFFFFFF, int(lat * 1e7), int(lon * 1e7),
            2000, 0, 0, 0, 0, 6000)
        mav.sys_status_send(3, 0, 0, 0, 0, 12500, 85, 0, 0,
                            0, 0, 0, 0, 0)   # 14 参签名（voltage=12.5V bat=85%）
        # 收命令（非阻塞抽干；定向包按 target_system 过滤——广播下别机不抢答）
        while True:
            r = m.recv_match(blocking=False)
            if r is None:
                break
            if getattr(r, "target_system", 0) not in (0, sysid):
                continue
            RX_TYPES[r.get_type()] = RX_TYPES.get(r.get_type(), 0) + 1
            if r.get_type() == "COMMAND_LONG" and \
                    r.command == en.MAV_CMD_NAV_LAND:
                LAND_SEEN.set()
            if r.get_type() == "MISSION_COUNT":
                for i in range(r.count):
                    m.mav.mission_request_int_send(r.target_system,
                                                   r.target_component, i)
            if r.get_type() == "MISSION_ITEM_INT":
                MISSION_E.append((r.seq, r.x * 1e-7, r.y * 1e-7, r.z))
                if r.seq + 1 >= 2:
                    m.mav.mission_ack_send(
                        r.target_system, r.target_component, 0)
                    MISSION_OK.set()
        time.sleep(0.1)


def main():
    ths = [threading.Thread(target=fake_px4, args=(1, (0, 0), (2, 0), 20.0),
                            daemon=True),
           threading.Thread(target=fake_px4, args=(2, (5, 0), (1, 0), 20.0),
                            daemon=True)]
    for t in ths:
        t.start()
    time.sleep(15.0)
    # land 命令（did 0）
    c = socket.create_connection(("127.0.0.1", CMD_PORT), timeout=3)
    c.sendall(b'{"cmd":"land","did":"0"}\n')
    print("land ret:", c.rfile.readline() if hasattr(c, "rfile")
          else c.recv(200))
    time.sleep(3.0)
    # mission 上传（did 0，两航点）
    wps = [[1.5, 0.5, 2.0], [3.0, 1.0, 2.0]]
    c.sendall((json.dumps({"cmd": "upload_mission", "did": "0",
                           "wps": wps}) + "\n").encode("ascii"))
    print("mission ret:", c.recv(200))
    time.sleep(2.0)
    c.close()

    ok = True
    print("RX_TYPES:", RX_TYPES)
    if LAND_SEEN.wait(2.0):
        print("ASSERT land_rx: PASS")
    else:
        ok = False
        print("ASSERT land_rx: FAIL")
    if MISSION_OK.wait(2.0) and len(MISSION_E) == 2:
        lat, lon = enu_to_ll(*wps[0][:2])
        e_err = abs(MISSION_E[0][1] - lat) * 1e7
        n_err = abs(MISSION_E[0][2] - lon) * 1e7
        print("MISSION_E:", MISSION_E, "ll_err(e7)=%.2f/%.2f" %
              (e_err, n_err), "z=%.2f" % MISSION_E[0][3])
        good = e_err < 1.0 and n_err < 1.0 and \
            abs(MISSION_E[0][3] - wps[0][2]) < 0.01
        print("ASSERT mission_roundtrip:", "PASS" if good else "FAIL")
        ok = ok and good
    else:
        ok = False
        print("ASSERT mission_roundtrip: FAIL")
    print("FAKE_MAVLINK_DONE", "ALL_PASS" if ok else "HAS_FAIL")


if __name__ == "__main__":
    main()
