# -*- coding: utf-8 -*-
"""无人机动力学后端：kinematic / cascade_pid。

所有后端实现同一接口：reset(pose) / set_vel_cmd(vel, yaw_rate) / step(dt, wind=None) /
get_state()。world_node 用 sim_settings.yaml 的 backend 字段选择。

step 的可选 wind 参数为外部风扰动（水平向量, m/s）：
  * cascade_pid：acc += (wind - vel)/tau_aero（气动阻力拖向风速），控制器反推；
  * kinematic：   速度指令直接叠加风（全卷入近似，低保真）。
wind=None 时行为与无风完全一致。
"""
import math

import numpy as np

from zx2026_common import config as cfg


class DroneState:
    __slots__ = ("pos", "vel", "att", "yaw")

    def __init__(self):
        self.pos = np.array([0.0, 0.0, 1.0])
        self.vel = np.zeros(3)
        self.att = np.zeros(3)   # roll,pitch,yaw (rad)
        self.yaw = 0.0

    def pos_tuple(self):
        return (float(self.pos[0]), float(self.pos[1]), float(self.pos[2]))

    def copy(self):
        s = DroneState()
        s.pos = self.pos.copy()
        s.vel = self.vel.copy()
        s.att = self.att.copy()
        s.yaw = self.yaw
        return s


class KinematicBackend:
    """速度指令直接积分，无姿态动态。低保真，用于快速调参/CI。"""

    def __init__(self, params):
        self._yaw_rate_max = float(params.get("yaw_rate_max", 1.5))
        self.state = DroneState()

    def reset(self, pose):
        self.state = DroneState()
        self.state.pos = np.array(pose[:3], dtype=float)
        self.state.yaw = float(pose[3]) if len(pose) > 3 else 0.0
        self.state.vel = np.zeros(3)

    def set_vel_cmd(self, vel, yaw_rate=0.0):
        self._cmd = np.array(vel[:3], dtype=float)
        self._yaw_rate = float(yaw_rate)

    def step(self, dt, wind=None):
        s = self.state
        s.vel = self._cmd
        if wind is not None:
            # 低保真近似：全卷入（kinematic 仅用于快速调参/CI，风仅按 cascade_pid 校调）
            s.vel = s.vel + np.array(wind[:3], dtype=float)
        s.pos = s.pos + s.vel * dt
        s.yaw = math.fmod(s.yaw + self._yaw_rate * dt, 2.0 * math.pi)
        s.att[2] = s.yaw

    def get_state(self):
        return self.state.copy()


class CascadePidBackend:
    """位置→速度→姿态-推力 PID 级联。中保真，思路同旧 marsim（重新实现）。

    输入: 期望速度 (vx,vy,vz)。
    内部: 期望速度直接作为内环目标，内环对实际速度做 PID 得到加速度，
          再把加速度换算为姿态与总推力（简化为 xyz 加速度指令 + 重力补偿）。
    """

    def __init__(self, params):
        p = params.get("cascade_pid", {})
        self._pos_kp = np.array(p.get("pos_kp", [1.2, 1.2, 1.5]))
        self._vel_kp = np.array(p.get("vel_kp", [2.0, 2.0, 2.5]))
        self._vel_kd = np.array(p.get("vel_kd", [0.6, 0.6, 0.4]))
        self._max_tilt = math.radians(float(p.get("max_tilt_deg", 30.0)))
        self._max_vel = float(p.get("max_vel", 2.5))
        self._acc_cap = float(p.get("acc_cap", 6.0))
        self._tau_aero = float(p.get("tau_aero", 0.6))
        self.state = DroneState()
        self._cmd = np.zeros(3)

    def reset(self, pose):
        self.state = DroneState()
        self.state.pos = np.array(pose[:3], dtype=float)
        self.state.yaw = float(pose[3]) if len(pose) > 3 else 0.0
        self.state.vel = np.zeros(3)
        self._cmd = np.zeros(3)
        self._last_vel_err = np.zeros(3)

    def set_vel_cmd(self, vel, yaw_rate=0.0):
        v = np.array(vel[:3], dtype=float)
        # 速度限幅
        vn = np.linalg.norm(v)
        if vn > self._max_vel:
            v = v * (self._max_vel / vn)
        self._cmd = v
        self._yaw_rate = float(yaw_rate)

    def step(self, dt, wind=None):
        s = self.state
        # 速度误差 → 净加速度指令（推力减重力，悬停时 = 0）
        err = self._cmd - s.vel
        acc = self._vel_kp * err - self._vel_kd * self._last_vel_err
        self._last_vel_err = err
        # 风扰动：气动阻力拖向风速（acc += (wind-vel)/tau_aero）。加在限幅之前，
        # 使总加速度受 cap 约束；下一拍 err 变非零 → 控制器可见地"抗风"。
        if wind is not None:
            acc = acc + (np.array(wind[:3], dtype=float) - s.vel) / self._tau_aero
        # 加速度限幅
        an = np.linalg.norm(acc)
        if an > self._acc_cap:
            acc = acc * (self._acc_cap / max(an, 1e-9))
        # 数值积分
        s.vel += acc * dt
        # 水平速度限幅
        vh = np.hypot(s.vel[0], s.vel[1])
        if vh > self._max_vel:
            s.vel[0] *= self._max_vel / vh
            s.vel[1] *= self._max_vel / vh
        s.pos += s.vel * dt
        # 简化姿态（由加速度推断）
        s.att[0] = math.atan2(acc[1], 9.81) if abs(acc[1]) > 1e-6 else 0.0
        s.att[1] = math.atan2(acc[0], 9.81) if abs(acc[0]) > 1e-6 else 0.0
        s.att[0] = geo_clamp(s.att[0], -self._max_tilt, self._max_tilt)
        s.att[1] = geo_clamp(s.att[1], -self._max_tilt, self._max_tilt)
        s.yaw = math.fmod(s.yaw + self._yaw_rate * dt, 2.0 * math.pi)
        s.att[2] = s.yaw

    def get_state(self):
        return self.state.copy()


def geo_clamp(v, lo, hi):
    return max(lo, min(hi, v))


def make_backend(name, params):
    if name == "kinematic":
        return KinematicBackend(params.get("kinematic", {}))
    if name == "cascade_pid":
        return CascadePidBackend(params)
    raise ValueError("unknown backend: %s" % name)


def default_backend():
    settings = cfg.load("sim_settings.yaml")
    return make_backend(settings.get("backend", "cascade_pid"), settings)
