#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_e2e_gate_feeder.py — 闸门 E2E feeder（无 ROS 直喂 hub）。

d0=强机：空中+grid 覆盖率随时间爬升（0.1→0.5，t=5s 过 0.3 阈值）；
d1=贫机：悬停+fc=AUTO（AUTONOMY 门 liveness 换源验证数据源）。
"""
import base64
import json
import socket
import time


def rle(values):
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


def grid_known(frac_known):
    w = h = 20
    n_known = int(w * h * frac_known)
    vals = [0] * n_known + [1] * 5 + [2] * (w * h - n_known - 5)
    return {"w": w, "h": h, "res": 0.5, "x0": 0.0, "y0": 0.0,
            "rle": rle(vals)}


def main():
    s = socket.create_connection(("127.0.0.1", 9870), timeout=3)
    t0 = time.time()
    while time.time() - t0 < 14.0:
        t = time.time() - t0
        frac = min(0.5, 0.1 + t * 0.04)
        obj = {"agent_ts": t, "seq": int(t * 5),
               "drones": {
                   "0": {"pos": [2.0, 2.0, 2.0], "speed": 1.0,
                         "vel": [1.0, 0.0, 0.0], "bat": 80.0,
                         "fc": "AUTO", "connected": True,
                         "ts": round(time.time(), 3)},
                   "1": {"pos": [1.0, 1.0, 0.0], "speed": 0.0,
                         "vel": [0.0, 0.0, 0.0], "bat": 90.0,
                         "fc": "AUTO", "connected": True,
                         "rtk": "RTK_FIXED", "ts": round(time.time(), 3)}},
               "grids": {"0": grid_known(frac)}}
        s.sendall((json.dumps(obj) + "\n").encode("ascii"))
        time.sleep(0.2)
    print("FEEDER_DONE")


if __name__ == "__main__":
    main()
