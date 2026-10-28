#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""math3d.py — 异构主从遥操作的坐标数学内核（与硬件无关）。

约定（整个脚手架统一，不要混用）：
  · 位姿 = (R, p)；A_T_B 把 B 系表达的量转到 A 系；增量左乘。
  · 位置增量：Δp_W = R_WM · Δp_M
  · 旋转增量：ΔR_W = R_WM · ΔR_M · R_WM^T      （主动旋转、左乘）
  · 四元数顺序 (x, y, z, w)
  · 位置内部一律「米」，只在 SDK/HTTP 边界转毫米。

R_WM = 主手基座系 → 世界系（世界系应取从臂基座系）的旋转，由 teleop_config.json
的 master_base.rpy_deg 给出；不要在这里硬编码任何具体机型。
"""

import math

import numpy as np

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


# ---------------------------------------------------------------------
# 四元数 (x, y, z, w)
# ---------------------------------------------------------------------
def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_conjugate(q) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([-x, -y, -z, w])


def quat_multiply(a, b) -> np.ndarray:
    ax, ay, az, aw = quat_normalize(a)
    bx, by, bz, bw = quat_normalize(b)
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_angle_deg(qa, qb) -> float:
    dot = abs(float(np.dot(quat_normalize(qa), quat_normalize(qb))))
    return 2.0 * math.acos(min(1.0, dot)) * RAD2DEG


# ---------------------------------------------------------------------
# 旋转矩阵 / 旋转向量 / RPY
# ---------------------------------------------------------------------
def rotvec_to_matrix(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.eye(3)
    k = v / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def matrix_to_rotvec(R) -> np.ndarray:
    """旋转矩阵 → 旋转向量（rad），接近 π 时用对角占优法保持数值稳定。"""
    R = np.asarray(R, dtype=float)
    cos_t = min(1.0, max(-1.0, (float(np.trace(R)) - 1.0) / 2.0))
    theta = math.acos(cos_t)
    sin_t = math.sin(theta)
    skew = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if sin_t > 1e-8:
        return (theta / (2.0 * sin_t)) * skew
    if theta < 1e-8:
        return 0.5 * skew
    d = np.diag(R)
    i = int(np.argmax(d))
    w_i = math.sqrt(max(0.0, (d[i] + 1.0) / 2.0))
    if w_i > 1e-8:
        axis = np.zeros(3)
        axis[i] = w_i
        for j in range(3):
            if j != i:
                axis[j] = (R[i, j] + R[j, i]) / (4.0 * w_i)
        n = float(np.linalg.norm(axis))
        if n > 1e-12:
            axis = axis / n
            if float(np.dot(skew, axis)) < 0.0 and sin_t >= 0.0:
                axis = -axis
            return theta * axis
    return theta * np.array([0.0, 0.0, 1.0])


def rpy_deg_to_matrix(rpy_deg) -> np.ndarray:
    """RPY(度) → R，约定 R = Rz(yaw)·Ry(pitch)·Rx(roll)（固定轴 XYZ）。"""
    rx, ry, rz = [float(v) * DEG2RAD for v in rpy_deg]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def matrix_to_rpy_deg(R) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    sy = min(1.0, max(-1.0, -float(R[2, 0])))
    pitch = math.asin(sy)
    if abs(sy) < 0.999999:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(R[0, 1]), float(R[1, 1]))
    return np.array([roll, pitch, yaw]) * RAD2DEG


def orthonormalize(R) -> np.ndarray:
    """SVD 正交化：抵消逐周期左乘累积的浮点漂移（det 强制 +1）。"""
    R = np.asarray(R, dtype=float)
    u, _s, vt = np.linalg.svd(R)
    Rn = u @ vt
    if np.linalg.det(Rn) < 0:
        u[:, -1] *= -1.0
        Rn = u @ vt
    return Rn


def is_rotation(R, tol=1e-5) -> bool:
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3) or not np.isfinite(R).all():
        return False
    if abs(float(np.linalg.det(R)) - 1.0) > tol:
        return False
    return float(np.linalg.norm(R @ R.T - np.eye(3))) <= tol


def rotation_angle_deg(R_a, R_b) -> float:
    return float(np.linalg.norm(matrix_to_rotvec(np.asarray(R_a) @ np.asarray(R_b).T))) * RAD2DEG


# ---------------------------------------------------------------------
# 齐次变换
# ---------------------------------------------------------------------
def make_transform(R, p) -> np.ndarray:
    T = np.eye(4)
    T[0:3, 0:3] = np.asarray(R, dtype=float)
    T[0:3, 3] = np.asarray(p, dtype=float)
    return T


def transform_inverse(T) -> np.ndarray:
    T = np.asarray(T, dtype=float)
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    Ti = np.eye(4)
    Ti[0:3, 0:3] = R.T
    Ti[0:3, 3] = -R.T @ p
    return Ti


# ---------------------------------------------------------------------
# 主手增量 → 世界系
# ---------------------------------------------------------------------
def master_delta_to_world(dp_m, dR_m, R_wm):
    """Δp_W = R_WM·Δp_M，ΔR_W = R_WM·ΔR_M·R_WM^T。单位由调用方统一（米）。"""
    R_wm = np.asarray(R_wm, dtype=float)
    dp_w = R_wm @ np.asarray(dp_m, dtype=float)
    dR_w = R_wm @ np.asarray(dR_m, dtype=float) @ R_wm.T
    return dp_w, orthonormalize(dR_w)


def parse_rpy(text) -> np.ndarray:
    """'0,0,-90' → np.array([0,0,-90])。"""
    parts = [p.strip() for p in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError("需要 3 个逗号分隔的角度，收到 %r" % (text,))
    return np.array([float(p) for p in parts])
