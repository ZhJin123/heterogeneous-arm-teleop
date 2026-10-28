#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""virtual_tcp.py — 虚拟 TCP 状态机（异构主从映射的核心）。

为什么需要它：两台异构臂的 TCP 坐标系定义不同、安装姿态不同、基座朝向不同。
不要复制主手 TCP 的绝对姿态；为每一侧维护一个"与世界系对齐"的虚拟 TCP，只传增量。

初始化（每次锚定）：
    p_Vm,0 = 世界系下的主手真实 TCP 位置      R_Vm,0 = I
    p_Vs,0 = 从手真实 TCP 位置               R_Vs,0 = I
    R_offset = R_Vs,0^T · R_Ts,0 = R_Ts,0     （固定旋转偏置，只算一次）

每周期：
    p_Vm += Δp_W        R_Vm = ΔR_W · R_Vm
    p_Vs += k_pos·Δp_W  R_Vs = ΔR_W · R_Vs

下发从手（真实 TCP）：
    p_Ts = p_Vs
    R_Ts = R_Vs · R_offset

位置单位：米（调用方统一）。
"""

import numpy as np

from math3d import orthonormalize, rotation_angle_deg


class VirtualTcpManager:
    def __init__(self):
        self.p_vm = None      # 主虚拟 TCP 位置（世界系，米）
        self.R_vm = None      # 主虚拟 TCP 姿态（世界系）
        self.p_vs = None      # 从虚拟 TCP 位置（世界系，米）
        self.R_vs = None      # 从虚拟 TCP 姿态（世界系）
        self.R_offset = None  # 真 TCP ← 虚 TCP 的固定旋转
        self.p_ts0 = None
        self.R_ts0 = None

    @property
    def ready(self) -> bool:
        return self.R_offset is not None and self.p_vs is not None and self.p_vm is not None

    # ---- 初始化（锚定）----
    def initialize_master(self, p_vm0):
        self.p_vm = np.array(p_vm0, dtype=float)
        self.R_vm = np.eye(3)

    def initialize_slave(self, p_ts0, R_ts0):
        self.p_ts0 = np.array(p_ts0, dtype=float)
        self.R_ts0 = orthonormalize(R_ts0)
        self.p_vs = self.p_ts0.copy()
        self.R_vs = np.eye(3)
        self.R_offset = self.R_vs.T @ self.R_ts0
        return self.R_offset

    def clear(self):
        self.p_vm = self.R_vm = None
        self.p_vs = self.R_vs = None
        self.R_offset = self.p_ts0 = self.R_ts0 = None

    # ---- 周期更新 ----
    def update_master(self, dp_w, dR_w):
        self.p_vm = self.p_vm + np.asarray(dp_w, dtype=float)
        self.R_vm = orthonormalize(np.asarray(dR_w, dtype=float) @ self.R_vm)

    def update_slave(self, dp_w, dR_w, k_pos=1.0):
        self.p_vs = self.p_vs + float(k_pos) * np.asarray(dp_w, dtype=float)
        self.R_vs = orthonormalize(np.asarray(dR_w, dtype=float) @ self.R_vs)

    # ---- 从虚拟 → 从真实 ----
    def slave_real_target(self):
        return self.p_vs.copy(), orthonormalize(self.R_vs @ self.R_offset)

    def init_errors(self):
        """初始化一致性：应同时为 0。返回 (位置误差 mm, 姿态误差 deg)。"""
        p_cmd, R_cmd = self.slave_real_target()
        p_err_mm = float(np.linalg.norm(p_cmd - self.p_ts0)) * 1000.0
        r_err_deg = rotation_angle_deg(R_cmd, self.R_ts0)
        return p_err_mm, r_err_deg
