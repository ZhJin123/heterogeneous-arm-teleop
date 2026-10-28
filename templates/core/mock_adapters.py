#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mock_adapters.py — 无硬件的假主手/假从手，用于先把 15Hz 闭环跑通。

主手：两连杆平面臂做正弦运动，输出 TCP 位置 + 绕 Z 的小幅旋转。
从手：保存当前位姿，按 speed·dt 追目标；没到就返回 "busy"（模拟 MoveL 语义）。
"""

import math
import time

import numpy as np

from adapters import MasterSample, SlavePose


class MockMaster:
    def __init__(self, l1=0.30, l2=0.25):
        self.l1, self.l2 = l1, l2
        self.t0 = time.monotonic()

    def open(self):
        self.t0 = time.monotonic()

    def read(self):
        t = time.monotonic() - self.t0
        q1 = 1.0 + 0.25 * math.sin(2 * math.pi * 0.15 * t)
        q2 = 0.7 + 0.20 * math.sin(2 * math.pi * 0.11 * t + 0.6)
        x = self.l1 * math.cos(q1) + self.l2 * math.cos(q1 + q2)
        z = self.l1 * math.sin(q1) + self.l2 * math.sin(q1 + q2)
        y = 0.05 * math.sin(2 * math.pi * 0.07 * t)
        yaw = 0.20 * math.sin(2 * math.pi * 0.09 * t)
        quat = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        return MasterSample(pos_m=np.array([x, y, z]), quat_xyzw=quat,
                            joints_rad=np.array([q1, q2]), stamp=time.monotonic())

    def close(self):
        pass


class MockSlave:
    def __init__(self, dt, start=(400.0, 0.0, 300.0, 180.0, 0.0, 180.0)):
        self.dt = float(dt)
        self.pose = np.array(start, dtype=float)
        self.speed_cap = 150.0

    def health(self):
        return {"ok": True, "tool_active": True, "has_error": False}

    def start(self, limits=None):
        return self.read_pose()

    def read_pose(self):
        return SlavePose(pos_mm=self.pose[0:3].copy(), rpy_deg=self.pose[3:6].copy(),
                         joints_deg=np.zeros(6))

    def move(self, target6, speed, acc, radius):
        target = np.array(target6, dtype=float)
        step = min(float(speed), self.speed_cap) * self.dt
        d = target[0:3] - self.pose[0:3]
        n = float(np.linalg.norm(d))
        if n > step:
            self.pose[0:3] += d * (step / n)
            self.pose[3:6] += (target[3:6] - self.pose[3:6]) * min(1.0, step / max(n, 1e-9))
            return "busy", self.read_pose()
        self.pose = target.copy()
        return "ok", self.read_pose()

    def stop(self, release=False):
        pass

    def close(self):
        pass
