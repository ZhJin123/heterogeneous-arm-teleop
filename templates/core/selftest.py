#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""selftest.py — 不连硬件的数学/状态机自检。

覆盖最容易搞错的四件事：
  1. R_WM 方向（主手基座 → 世界系）
  2. 虚拟 TCP 初始化一致性（目标 == 当前真实位姿）
  3. 纯平移 / 纯旋转的增量映射
  4. 每周期限速与碎步抑制

运行：python selftest.py   （全部 PASS 退出码 0）
"""

import sys

import numpy as np

from mapper import MasterSlaveMapper
from math3d import rpy_deg_to_matrix, rotation_angle_deg, rotvec_to_matrix
from motion_controller import MotionController
from virtual_tcp import VirtualTcpManager

FAILS = []


def check(name, cond, detail=""):
    print("%s %s%s" % ("PASS" if cond else "FAIL", name, ("  | " + detail) if detail else ""))
    if not cond:
        FAILS.append(name)


def main():
    # 1. R_WM：主手 +X 应映射到世界 −Y（以常见的 Rz(−90°) 布局为例）
    R_wm = rpy_deg_to_matrix([0, 0, -90])
    check("R_WM(+X) -> 世界 −Y", np.allclose(R_wm @ np.array([1.0, 0, 0]), [0, -1, 0], atol=1e-9),
          "R_WM·ex=%s" % np.round(R_wm @ np.array([1.0, 0, 0]), 6).tolist())

    # 2. 初始化一致性
    vtm = VirtualTcpManager()
    R_ts0 = rpy_deg_to_matrix([180, 0, 180])
    vtm.initialize_slave(np.array([0.400, 0.0, 0.300]), R_ts0)
    vtm.initialize_master(np.array([0.0, 0.2, 0.15]))
    p_err, r_err = vtm.init_errors()
    check("初始化一致性 = 0", p_err < 1e-6 and r_err < 1e-6,
          "p=%.2e mm r=%.2e deg" % (p_err, r_err))

    # 3a. 纯平移：主手基座系 +X 10mm → 世界 −Y 10mm
    mapper = MasterSlaveMapper(R_wm, k_pos=1.0, k_rot=1.0,
                               pos_deadband_mm=0.0, rot_deadband_deg=0.0)
    mapper.reset(np.zeros(3), np.eye(3))
    p0 = vtm.p_vs.copy()
    for i in range(10):
        dp, dR = mapper.step(np.array([0.001 * (i + 1), 0, 0]), np.eye(3))
        vtm.update_master(dp, dR)
        vtm.update_slave(dp, dR, 1.0)
    d = vtm.p_vs - p0
    check("纯平移 1:1", np.allclose(d, [0, -0.010, 0], atol=1e-9),
          "Δp_Vs=%s" % np.round(d * 1000, 3).tolist())

    # 3b. 纯旋转：主手绕自身 Z 转 10° → 从臂目标相对 R_ts0 转 10°
    vtm2 = VirtualTcpManager()
    vtm2.initialize_slave(np.array([0.4, 0, 0.3]), R_ts0)
    vtm2.initialize_master(np.zeros(3))
    mapper2 = MasterSlaveMapper(R_wm, 1.0, 1.0, 0.0, 0.0)
    mapper2.reset(np.zeros(3), np.eye(3))
    for i in range(10):
        Rm = rotvec_to_matrix(np.array([0, 0, np.deg2rad(1.0) * (i + 1)]))
        dp, dR = mapper2.step(np.zeros(3), Rm)
        vtm2.update_master(dp, dR)
        vtm2.update_slave(dp, dR, 1.0)
    _, R_target = vtm2.slave_real_target()
    ang = rotation_angle_deg(R_target, R_ts0)
    check("纯旋转 1:1", abs(ang - 10.0) < 1e-6, "角度差=%.6f°" % ang)

    # 4a. 限速：远目标每周期最多走 vmax·dt
    mc = MotionController(vmax=150.0, wmax=90.0, min_step_mm=0.5, min_step_rot_deg=0.5)
    mc.reset([400, 0, 300, 180, 0, 180])
    dt = 1.0 / 15.0
    target, step_mm, _ = mc.plan(np.array([1000.0, 0, 300]), rpy_deg_to_matrix([180, 0, 180]), dt)
    check("限速 vmax·dt", target is not None and step_mm <= 150.0 * dt + 1e-9,
          "step=%.3fmm 上限=%.3fmm" % (step_mm, 150.0 * dt))

    # 4b. 碎步抑制：accept 后同样目标不再下发
    mc.accept(target)
    again, step2, rot2 = mc.plan(target[0:3], rpy_deg_to_matrix(target[3:6]), dt)
    check("碎步抑制", again is None and step2 < 0.5 and rot2 < 0.5,
          "step=%.4f rot=%.4f" % (step2, rot2))

    print("\n%s" % ("ALL PASS" if not FAILS else "FAILED: " + ", ".join(FAILS)))
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
