#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""motion_controller.py — 由从虚拟 TCP 生成下发给从臂的绝对目标。

只做三件事：
  1. 每周期限速：位置沿直线每周期最多走 vmax·dt，姿态绕同一转轴每周期最多转 wmax·dt；
  2. 碎步抑制：与"上次已受理目标"差小于 min_step 不发；
  3. 提供 lag（目标−反馈）供日志/看门狗使用。

不做限位盒、工作空间/奇异点预检、速度爬坡。安全边界应是硬件急停 + 看门狗。

目标位姿单位：mm / deg（SDK 边界单位）；内部状态是"上次服务端已受理的目标"。
"""

import numpy as np

from math3d import (matrix_to_rotvec, matrix_to_rpy_deg, rotvec_to_matrix,
                    rpy_deg_to_matrix, rotation_angle_deg)


def limit_orientation_step(R_target, R_ref, max_step_deg):
    """把 R_target 相对 R_ref 的旋转限制在一步 ≤ max_step_deg（沿同一转轴截断）。"""
    R_target = np.asarray(R_target, dtype=float)
    R_ref = np.asarray(R_ref, dtype=float)
    rv = matrix_to_rotvec(R_target @ R_ref.T)
    n_deg = float(np.linalg.norm(rv)) * 180.0 / np.pi
    if n_deg <= float(max_step_deg) or n_deg == 0.0:
        return R_target
    return rotvec_to_matrix(rv * (float(max_step_deg) / n_deg)) @ R_ref


class MotionController:
    def __init__(self, vmax=150.0, wmax=90.0, min_step_mm=0.5, min_step_rot_deg=0.5):
        self.vmax = abs(float(vmax))
        self.wmax = abs(float(wmax))
        self.min_step_mm = abs(float(min_step_mm))
        self.min_step_rot_deg = abs(float(min_step_rot_deg))
        self.last_target = None   # [x,y,z,rx,ry,rz] mm/deg
        self.last_plan = None

    def reset(self, pose_mm_deg):
        self.last_target = np.array(pose_mm_deg, dtype=float)
        self.last_plan = self.last_target.copy()

    def plan(self, p_cmd_mm, R_cmd, dt):
        """返回 (target6 或 None, 位置步长 mm, 姿态步长 deg)。None = 本周期不下发。"""
        if self.last_target is None:
            raise RuntimeError("MotionController 未 reset")

        last_p = self.last_target[0:3]
        d = np.asarray(p_cmd_mm, dtype=float) - last_p
        n = float(np.linalg.norm(d))
        max_step = self.vmax * float(dt)
        if max_step > 0.0 and n > max_step:
            p_t = last_p + d * (max_step / n)
        else:
            p_t = np.asarray(p_cmd_mm, dtype=float)

        R_last = rpy_deg_to_matrix(self.last_target[3:6])
        R_t = limit_orientation_step(R_cmd, R_last, self.wmax * float(dt))

        dp = float(np.linalg.norm(p_t - last_p))
        dr = rotation_angle_deg(R_t, R_last)
        self.last_plan = np.concatenate([p_t, matrix_to_rpy_deg(R_t)])

        if dp < self.min_step_mm and dr < self.min_step_rot_deg:
            return None, dp, dr
        return self.last_plan.copy(), dp, dr

    def accept(self, target):
        """从臂已受理该目标，推进基准；busy 时不要调用（冻结目标）。"""
        self.last_target = np.array(target, dtype=float)

    def lag(self, pose_fb):
        if pose_fb is None or self.last_target is None:
            return 0.0, 0.0
        dp = float(np.linalg.norm(self.last_target[0:3] - np.asarray(pose_fb[0:3], dtype=float)))
        dr = rotation_angle_deg(rpy_deg_to_matrix(self.last_target[3:6]),
                                rpy_deg_to_matrix(pose_fb[3:6]))
        return dp, dr
