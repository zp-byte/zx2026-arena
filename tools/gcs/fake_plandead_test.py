#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_plandead_test.py — PLANNER_DEAD 边沿自检: d2 plan_age=99 持续 10s."""
import json, socket, time
s = socket.create_connection(("127.0.0.1", 9870), timeout=3)
t0 = time.time()
while time.time() - t0 < 10.0:
    t = time.time() - t0
    drones = {"2": {"pos": [3.0, 1.0, 1.5], "yaw": 0.0, "speed": 1.0,
                    "vel": [1.0, 0.0, 0.0], "bat": None, "bat_v": None,
                    "fc": None, "connected": None, "phase": "EXECUTE",
                    "ts": t, "plan_age": 99.0 if t > 0 else 0.1}}
    s.sendall((json.dumps({"agent_ts": t, "seq": int(t * 5),
                           "drones": drones},
                          separators=(",", ":")) + "\n").encode("ascii"))
    time.sleep(0.2)
s.close()
print("plandead feeder done")
