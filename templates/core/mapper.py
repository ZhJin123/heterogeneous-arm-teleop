#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mapper.py — 主手相邻帧差分 → 世界系增量。

    Δp_M = p_M,k − p_M,k−1
    ΔR_M = R_M,k · R_M,k−1^T
    Δp_W = R_WM · Δp_M
    ΔR_W = R_WM · ΔR_M · R_WM^T

死区采用"累加不丢弃"：小于阈值的增量先攒着，攒够一次放出。既抑制微抖，
又不会把慢速拖动吃掉（直接丢弃会累积漂移）。
k_rot 在旋转向量上缩放，不对旋转矩阵元素做线性缩放。
"""

import numpy as np

from math3d import (RAD2DEG, matrix_to_rotvec, orthonormalize, rotvec_to_matrix,
                    rotation_angle_deg)

IDENTITY3 = np.eye(3)


class MasterSlaveMapper:
    def __init__(self, R_wm, k_pos=1.0, k_rot=1.0,
                 pos_deadband_mm=0.5, rot_deadband_deg=1.0):
        self.R_wm = np.asarray(R_wm, dtype=float)
        self.k_pos = float(k_pos)
        self.k_rot = float(k_rot)
        self.pos_deadband_m = abs(float(pos_deadband_mm)) * 0.001
        self.rot_deadband_rad = abs(float(rot_deadband_deg)) / RAD2DEG
        self._last_p = None
        self._last_R = None
        self._pending_dp = np.zeros(3)
        self._pending_drv = np.zeros(3)

    def reset(self, p_master, R_master):
        """锚定时调用：以当前主手位姿为增量基准，清空死区缓存。"""
        self._last_p = np.array(p_master, dtype=float)
        self._last_R = orthonormalize(R_master)
        self._pending_dp = np.zeros(3)
        self._pending_drv = np.zeros(3)

    def step(self, p_master, R_master):
        """返回 (dp_W[m], dR_W)。没有可放出的增量时返回 (0, I)。"""
        p_master = np.array(p_master, dtype=float)
        R_master = orthonormalize(R_master)
        if self._last_p is None or self._last_R is None:
            self.reset(p_master, R_master)
            return np.zeros(3), IDENTITY3.copy()

        dp_m = p_master - self._last_p
        dR_m = R_master @ self._last_R.T
        self._last_p = p_master
        self._last_R = R_master

        dp_w = self.R_wm @ dp_m
        dR_w = orthonormalize(self.R_wm @ dR_m @ self.R_wm.T)

        # 位置死区（累加不丢）
        self._pending_dp = self._pending_dp + dp_w
        if float(np.linalg.norm(self._pending_dp)) >= self.pos_deadband_m:
            out_dp = self._pending_dp
            self._pending_dp = np.zeros(3)
        else:
            out_dp = np.zeros(3)

        # 旋转死区（旋转向量累加，k_rot 在放出时缩放）
        self._pending_drv = self._pending_drv + matrix_to_rotvec(dR_w)
        if float(np.linalg.norm(self._pending_drv)) >= self.rot_deadband_rad:
            out_dR = rotvec_to_matrix(self.k_rot * self._pending_drv)
            self._pending_drv = np.zeros(3)
        else:
            out_dR = IDENTITY3.copy()

        return out_dp, out_dR

    @staticmethod
    def delta_angle_deg(dR) -> float:
        return rotation_angle_deg(dR, IDENTITY3)
