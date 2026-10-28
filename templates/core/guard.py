#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""guard.py — 最小停车逻辑（只兜数据/目标非法，不做安全顾问）。

只做三件事：
  1. 主手数据非法（NaN/Inf、四元数模长异常、位置量级异常）→ 停车；
  2. 主手断流（由 bridge 用时间戳判定）→ HOLD；
  3. 目标位姿非法（NaN/Inf、非正交旋转）→ 停车。

明确不做：限位盒、min_z、工作空间/奇异点、位姿跳变、速度爬坡。
安全边界 = 硬件急停 + 本层的"主手掉线/服务端报错就别继续跑"。
"""

import numpy as np

from math3d import is_rotation, rpy_deg_to_matrix


class StopGuard:
    @staticmethod
    def check_master(sample):
        """sample 需有 pos_m(3, 米)、quat_xyzw(4)。返回 None 或停车原因。"""
        if sample is None:
            return "主手数据为空"
        p = np.asarray(sample.pos_m, dtype=float)
        q = np.asarray(sample.quat_xyzw, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            return "主手位置非法: %r" % (p,)
        if q.shape != (4,) or not np.isfinite(q).all():
            return "主手四元数非法: %r" % (q,)
        qn = float(np.linalg.norm(q))
        if not (0.9 <= qn <= 1.1):
            return "主手四元数模长异常: %.6f" % qn
        if float(np.linalg.norm(p)) > 10.0:
            return "主手位置量级异常: %r" % (p,)
        return None

    @staticmethod
    def check_target(target6):
        t = np.asarray(target6, dtype=float)
        if t.shape != (6,) or not np.isfinite(t).all():
            return "目标位姿非法: %r" % (t,)
        if not is_rotation(rpy_deg_to_matrix(t[3:6])):
            return "目标姿态不是合法旋转矩阵: %r" % (t[3:6],)
        return None
