#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""adapters.py — 两台机械臂的适配层（唯一需要对接厂商 SDK 的地方）。

bridge.py 只认识下面两个 Protocol；把主手 SDK 包成 MasterSource，把从手 SDK 包成
SlaveSink，映射层完全不用改。

生成的项目里这两个类是 TODO 桩：按注释实现后，`python bridge.py` 即可跑真机。
（`python bridge.py --mock` 用 mock_adapters.py，不需要任何硬件。）
"""

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

import numpy as np


@dataclass
class MasterSample:
    """一帧主手状态。

    pos_m:       主手 TCP 位置，**主手基座系**，米
    quat_xyzw:   主手 TCP 姿态四元数 (x,y,z,w)
    joints_rad:  关节角（rad，仅日志/模型用，可为空数组）
    stamp:       time.monotonic() 采样时刻（用于断流判定）
    """
    pos_m: np.ndarray
    quat_xyzw: np.ndarray
    joints_rad: np.ndarray
    stamp: float


@dataclass
class SlavePose:
    """从手真实 TCP 位姿（SDK 边界单位：mm / deg）。"""
    pos_mm: np.ndarray
    rpy_deg: np.ndarray
    joints_deg: Optional[np.ndarray] = None

    @property
    def pose6(self) -> np.ndarray:
        return np.concatenate([np.asarray(self.pos_mm, float),
                               np.asarray(self.rpy_deg, float)])


class MasterSource(Protocol):
    def open(self) -> None:
        """建立连接/打开设备；失败抛异常。"""

    def read(self) -> Optional[MasterSample]:
        """非阻塞读**最新一帧**；没有新数据返回 None。"""

    def close(self) -> None:
        ...


class SlaveSink(Protocol):
    def health(self) -> dict:
        """返回至少 {"ok": bool}；建议带 tool_active、has_error、is_powered 等。"""

    def start(self, limits: Optional[dict]) -> SlavePose:
        """开遥操作会话并返回当前真实 TCP 位姿（锚定用）。limits 可为 None。"""

    def read_pose(self) -> SlavePose:
        """只读当前位姿（不产生运动）。"""

    def move(self, target6, speed: float, acc: float, radius: float) -> Tuple[str, Optional[SlavePose]]:
        """下发绝对目标；返回 (status, pose)。

        status: "ok"（已受理）| "busy"（上一条未完成，本周期目标应冻结）
        """

    def stop(self, release: bool) -> None:
        ...

    def close(self) -> None:
        ...


# ---------------------------------------------------------------------
# 真机适配器：生成项目后在这里填厂商 SDK
# ---------------------------------------------------------------------
class RealMaster:
    """TODO: 包装主手 SDK。

    返回的 pos/quat 必须是**主手基座系**下的真实 TCP；如果 SDK 给的是关节角，
    在这里做正运动学（并逐个关节核对方向，见 reference.md §6）。
    """

    def open(self) -> None:
        raise NotImplementedError("实现 RealMaster.open()：连接主手、开启零力/拖动模式")

    def read(self) -> Optional[MasterSample]:
        raise NotImplementedError(
            "实现 RealMaster.read()：读取关节角/TCP，返回 MasterSample("
            "pos_m=..., quat_xyzw=..., joints_rad=..., stamp=time.monotonic())")

    def close(self) -> None:
        pass


class RealSlave:
    """TODO: 包装从手 SDK（MoveL 绝对目标 / 位置伺服均可）。"""

    def health(self) -> dict:
        raise NotImplementedError("实现 RealSlave.health()：连接检查 + 工具坐标系是否激活 + 报警")

    def start(self, limits: Optional[dict]) -> SlavePose:
        raise NotImplementedError("实现 RealSlave.start()：开会话/使能，返回当前 PCS 位姿")

    def read_pose(self) -> SlavePose:
        raise NotImplementedError("实现 RealSlave.read_pose()：只读当前 PCS（+关节角）")

    def move(self, target6, speed: float, acc: float, radius: float):
        raise NotImplementedError(
            "实现 RealSlave.move()：MoveL/MoveJ 绝对目标；上一条未完成返回 ('busy', pose)")

    def stop(self, release: bool) -> None:
        raise NotImplementedError("实现 RealSlave.stop()：停车；release=True 关会话")

    def close(self) -> None:
        pass
