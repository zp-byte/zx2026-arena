#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_agent_test.py — gcs_hub 离线自检（不需要 ROS）。

时间线（加速复现三类告警）：
  t0-6s   六机正常巡航 (v=1.0, phase=EXECUTE, plan_age=0.1)   → 零告警
  t6-12s  d5 冻结 (v=0.0)                                     → t~11s STALL
  t12-20s d3 停发遥测                                         → t~15s LINK_LOST
  t18-20s d5 恢复                                             → STALL_CLEAR
预期: hub 事件流出现 STALL(d5) / STALL_CLEAR(d5) / LINK_LOST(d3)，且
      t0-6s 无任何告警。
"""
import json
import socket
import time

POS = {i: [i * 2.0, 0.0, 1.5] for i in range(6)}


def batch(t, freeze5, dead3):
    drones = {}
    for i in range(6):
        if i == 3 and dead3:
            continue
        v = 0.0 if (i == 5 and freeze5) else 1.0
        POS[i][0] += v * 0.2
        drones[str(i)] = {
            "pos": list(POS[i]), "yaw": 90.0, "speed": v,
            "vel": [v, 0.0, 0.0], "bat": 85.0 if i % 2 else None,
            "bat_v": None, "fc": None, "connected": None,
            "phase": "EXECUTE", "ts": t, "plan_age": 0.1,
        }
    return {"agent_ts": t, "seq": int(t * 5), "drones": drones}


def main():
    s = socket.create_connection(("127.0.0.1", 9870), timeout=3)
    t0 = time.time()
    while time.time() - t0 < 20.0:
        t = time.time() - t0
        s.sendall((json.dumps(batch(t, freeze5=(t > 6 and t < 18),
                                    dead3=(t > 12)),
                             separators=(",", ":")) + "\n").encode("ascii"))
        time.sleep(0.2)
    s.close()
    print("fake agent done 20s")


if __name__ == "__main__":
    main()
