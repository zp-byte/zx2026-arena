#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_mavlink_idle.py — 挂机的假 PX4（panic 双轨 E2E 用）。

--sysids "2,3" 每台 10Hz 心跳+位置+电量（悬停不动）；收到 LAND/RTL 的
COMMAND_LONG 打印一行 [IDLE] 并记入 SEEN；25s 后打印 IDLE_DONE 收工。
"""
import argparse
import threading
import time

from pymavlink import mavutil

SEEN = []


def idle_px4(sysid, connect):
    m = mavutil.mavlink_connection(connect, source_system=sysid,
                                   source_component=1)
    mav = m.mav      # 协议对象（send 方法）
    en = mavutil.mavlink  # 枚举常量住 dialects 模块
    names = {20: "RTL", 21: "LAND"}
    t0 = time.time()
    while True:
        t = time.time() - t0
        mav.heartbeat_send(en.MAV_TYPE_QUADROTOR, en.MAV_AUTOPILOT_PX4,
                           0, (4 << 16), 3)
        mav.global_position_int_send(
            int(t * 1000) & 0x7FFFFFFF, int(30.0 * 1e7), int(120.0 * 1e7),
            1500, 500, 0, 0, 0, 6000)
        mav.sys_status_send(3, 0, 0, 0, 0, 12500, 85, 0, 0,
                            0, 0, 0, 0, 0)   # 14 参签名
        while True:
            r = m.recv_match(blocking=False)
            if r is None:
                break
            if getattr(r, "target_system", 0) not in (0, sysid):
                continue   # 定向包按 target 过滤——别机不抢答
            if r.get_type() == "COMMAND_LONG" and r.command in names:
                SEEN.append((sysid, names[r.command]))
                print("[IDLE] sysid=%d %s seen" % (sysid, names[r.command]),
                      flush=True)
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="udpout:127.0.0.1:14555")
    ap.add_argument("--sysids", default="2")
    a = ap.parse_args()
    for s in [int(x) for x in a.sysids.split(",")]:
        threading.Thread(target=idle_px4, args=(s, a.connect),
                         daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 25.0:
        time.sleep(0.5)
    print("IDLE_DONE seen=%s" % (SEEN,), flush=True)


if __name__ == "__main__":
    main()
