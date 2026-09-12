#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_e2e_feeder.py — 合并后离线 E2E feeder（不需要 ROS）。

驱动 hub(9870) → 面板 --selftest 自动 START → 全 FR 告警路径 → 全 DONE
退出。配合 _e2e_run.sh 使用。
时间线：
  A  t0-3s   六机平稳(P2_WAIT/静止) + stage=P2_WAIT   → READY 绿、倒计时活
  B  t3.0     d1 coll=3  (min_clr=0.31)                → COLLISION(d1)
  C  t3.2     d1 coll=4                                → COLLISION(d1) 再触发
  D  t4.0     d2 pos 突跳 (5→12, Δ≈7.6m)               → ESTIMATOR_JUMP(d2)
  E  t4.0-7.0 d4 EXECUTE + fc=MANUAL                   → OFFBOARD_LOST(d4)
  F  t7.0     d4 fc=OFFBOARD                           → OFFBOARD_OK(d4)+FC_MODE
  G  t8.0     d0 pos (23,0,1.5) 出围栏                  → FENCE_BREACH(d0)
  H  t9.5     d0 回 (5,3,1.5)                          → FENCE_OK(d0)
  I  t10.5    六机 matched/drop_done/match_progress     → 面板 meta 行
  J  t12.0+   六机 phase=DONE                           → SELFTEST 全终态退出
"""
import json
import socket
import time

POS = {i: [5.0 + i * 2.0, 3.0, 1.5] for i in range(6)}
PHASE = {i: "P2_WAIT" for i in range(6)}
FC = {i: None for i in range(6)}
COLL = {i: None for i in range(6)}
MINCLR = {i: None for i in range(6)}
META = {i: None for i in range(6)}


def send(s, t, stage="P2_WAIT"):
    drones = {}
    for i in range(6):
        d = {"pos": list(POS[i]), "yaw": 0.0, "speed": 0.0,
             "vel": [0.0, 0.0, 0.0], "bat": None, "bat_v": None,
             "fc": FC[i], "connected": None, "phase": PHASE[i],
             "ts": t, "plan_age": 0.1,
             "coll": COLL[i], "min_clr": MINCLR[i]}
        if META[i]:
            d.update(META[i])
        drones[str(i)] = d
    obj = {"agent_ts": t, "seq": int(t * 5), "drones": drones}
    if stage is not None:
        obj["stage"] = stage
    s.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode("ascii"))


def main():
    s = socket.create_connection(("127.0.0.1", 9870), timeout=3)
    t0 = time.time()

    def now():
        return time.time() - t0

    # A: 平稳 3s → READY 绿 → 面板 auto START
    while now() < 3.0:
        send(s, now())
        time.sleep(0.2)
    # B/C: d1 碰撞（count 递增二次触发）
    COLL[1], MINCLR[1] = 3, 0.31
    send(s, now()); time.sleep(0.2)
    COLL[1] = 4
    # D: d2 估计器跳变（静止帧跳 7.6m）
    POS[2] = [12.0, 3.0, 1.5]
    send(s, now()); time.sleep(0.4)
    # E/F: d4 OFFBOARD 丢失→恢复（短窗 EXECUTE 激活）
    PHASE[4], FC[4] = "EXECUTE", "MANUAL"
    send(s, now()); time.sleep(2.6)
    FC[4] = "OFFBOARD"
    send(s, now()); time.sleep(1.0)
    PHASE[4] = "P2_WAIT"
    # G/H: d0 围栏越界→回界
    POS[0] = [23.0, 0.0, 1.5]
    send(s, now()); time.sleep(1.5)
    POS[0] = [5.0, 3.0, 1.5]
    send(s, now()); time.sleep(1.0)
    # I: 任务数据透出（面板 meta 行 drop=✓ / m=2/3）
    for i in range(6):
        META[i] = {"matched": "TYPE_A", "drop_done": True,
                   "first_drop_t": 8.5, "match_progress": "2/3"}
    send(s, now()); time.sleep(1.0)
    # J: 全 DONE → 面板 selftest 全终态退出
    while now() < 16.0:
        for i in range(6):
            PHASE[i] = "DONE"
        send(s, now())
        time.sleep(0.2)
    s.close()
    print("e2e feeder done")


if __name__ == "__main__":
    main()
