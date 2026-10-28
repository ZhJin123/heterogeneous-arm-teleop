#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge.py — 异构主从遥操作主循环（脚手架）。

数据流：
    MasterSource.read()  →  mapper（差分 + 世界系转换）→ virtual_tcp（虚拟TCP）
    → motion_controller（限速/碎步/busy冻结）→ SlaveSink.move(绝对目标)

用法：
    python bridge.py --mock                 # 假硬件，默认跑 300 周期后退出
    python bridge.py                        # 真机（先实现 adapters.py 的 RealMaster/RealSlave）
    python bridge.py --mock --cycles 60 --rate 30

配置：同目录 teleop_config.json（由 scripts/scaffold.py 生成）。
安全：本脚手架不设限位盒/工作空间检查；硬件急停是唯一权威安全手段。
"""

import argparse
import json
import os
import time

import numpy as np

from guard import StopGuard
from mapper import MasterSlaveMapper
from math3d import quat_to_matrix, rpy_deg_to_matrix
from motion_controller import MotionController
from virtual_tcp import VirtualTcpManager

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "teleop_config.json")


class Bridge:
    def __init__(self, cfg, master, slave, dt):
        self.cfg = cfg
        self.master = master
        self.slave = slave
        self.dt = float(dt)

        self.R_wm = rpy_deg_to_matrix(cfg["master_base"]["rpy_deg"])
        m = cfg["mapping"]
        self.vtm = VirtualTcpManager()
        self.mapper = MasterSlaveMapper(
            self.R_wm, m.get("k_pos", 1.0), m.get("k_rot", 1.0),
            m.get("master_pos_deadband_mm", 0.5), m.get("master_rot_deadband_deg", 1.0))
        self.motion = MotionController(
            m.get("vmax", 150.0), m.get("wmax", 90.0),
            m.get("min_step_mm", 0.5), m.get("min_step_rot_deg", 0.5))
        self.guard = StopGuard()

        s = cfg.get("safety", {})
        self.limits = s.get("limit_box")
        self.timeout = float(s.get("master_timeout_s", 0.3))
        self.speed = float(m.get("vmax", 150.0))
        self.acc = float(m.get("linear_acc", 300.0))
        self.radius = float(m.get("radius", 10.0))

        self.moves = 0
        self.busy = 0
        self._last_summary = 0.0

    # ---- 日志 ----
    @staticmethod
    def log(msg):
        print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)

    # ---- 阶段 ----
    def wait_master(self):
        while True:
            st = self.master.read()
            if st is not None:
                return st
            time.sleep(self.dt)

    def align(self, sample):
        pose = self.slave.start(self.limits)
        self.vtm.initialize_master(self.R_wm @ sample.pos_m)
        self.vtm.initialize_slave(np.asarray(pose.pos_mm, float) * 0.001,
                                  rpy_deg_to_matrix(pose.rpy_deg))
        self.mapper.reset(sample.pos_m, quat_to_matrix(sample.quat_xyzw))
        self.motion.reset(pose.pose6)

        p_err, r_err = self.vtm.init_errors()
        self.log("ALIGN p_f0(mm)=%s rpy_f0(deg)=%s" %
                 (np.round(pose.pos_mm, 2).tolist(), np.round(pose.rpy_deg, 2).tolist()))
        self.log("自检: 位置误差=%.6f mm 姿态误差=%.6f° → %s" %
                 (p_err, r_err, "PASS" if (p_err < 0.01 and r_err < 0.01) else "FAIL"))
        if p_err >= 0.01 or r_err >= 0.01:
            raise RuntimeError("初始化一致性失败（检查 R_offset / 单位 / 位姿来源）")

    def step(self, sample, now):
        reason = self.guard.check_master(sample)
        if reason:
            raise RuntimeError(reason)

        dp_w, dR_w = self.mapper.step(sample.pos_m, quat_to_matrix(sample.quat_xyzw))
        self.vtm.update_master(dp_w, dR_w)
        self.vtm.update_slave(dp_w, dR_w, self.cfg["mapping"].get("k_pos", 1.0))

        p_ts, R_ts = self.vtm.slave_real_target()
        target, step_mm, step_deg = self.motion.plan(p_ts * 1000.0, R_ts, self.dt)
        if target is None:
            return None

        reason = self.guard.check_target(target)
        if reason:
            raise RuntimeError(reason)

        status, pose = self.slave.move(target, self.speed, self.acc, self.radius)
        if status == "busy":
            self.busy += 1
            return None
        if status != "ok":
            raise RuntimeError("从手返回未知状态: %r" % (status,))

        self.motion.accept(target)
        self.moves += 1
        if now - self._last_summary >= 1.0:
            self._last_summary = now
            lag_mm, lag_deg = self.motion.lag(pose.pose6 if pose else None)
            self.log("RUN move=%d busy=%d tgt=(%.1f, %.1f, %.1f) lag=%.1fmm/%.1f°"
                     % (self.moves, self.busy, target[0], target[1], target[2],
                        lag_mm, lag_deg))
        return target

    def run(self, cycles=None):
        health = self.slave.health()
        if not health.get("ok", False):
            raise RuntimeError("从手健康检查失败: %r" % (health,))
        if health.get("tool_active") is False:
            raise RuntimeError("从手工具坐标系未激活（检查 tool 名称/配置）")
        self.log("从手就绪: %r" % (health,))

        self.master.open()
        try:
            sample = self.wait_master()
            self.log("收到首帧主手数据: p=%s" % np.round(sample.pos_m * 1000, 1).tolist())
            self.align(sample)
            self.log("进入 RUN（rate=%.1fHz vmax=%.0f wmax=%.0f）" %
                     (1.0 / self.dt, self.speed, self.cfg["mapping"].get("wmax", 90)))

            next_tick = time.monotonic() + self.dt
            n = 0
            while cycles is None or n < cycles:
                n += 1
                sample = self.master.read()
                now = time.monotonic()
                if sample is None or (now - sample.stamp) > self.timeout:
                    self.slave.stop(False)
                    self.log("主手断流 → 停车（HOLD）")
                    return 3
                self.step(sample, now)

                next_tick += self.dt
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.monotonic() + self.dt
            return 0
        except KeyboardInterrupt:
            self.log("Ctrl+C → 停车")
            return 130
        finally:
            try:
                self.slave.stop(True)
            except Exception as e:      # noqa: BLE001
                self.log("[WARN] stop(release=True) 失败: %s" % e)
            self.master.close()
            self.slave.close()


def load_config(path=CONFIG_PATH):
    if not os.path.exists(path):
        raise SystemExit("缺少 %s；用 scripts/scaffold.py 生成，或从 teleop_config.example.json 复制" % path)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description="异构主从遥操作桥（15Hz 虚拟TCP增量映射）")
    ap.add_argument("--mock", action="store_true", help="使用假主手/假从手（无需硬件）")
    ap.add_argument("--cycles", type=int, default=0, help="跑多少周期后退出（0=一直跑）")
    ap.add_argument("--rate", type=float, default=0.0, help="覆盖配置里的 rate_hz")
    args = ap.parse_args()

    cfg = load_config()
    rate = args.rate if args.rate > 0 else float(cfg.get("rate_hz", 15.0))
    dt = 1.0 / rate
    cycles = args.cycles if args.cycles > 0 else (300 if args.mock else None)

    if args.mock:
        from mock_adapters import MockMaster, MockSlave
        master, slave = MockMaster(), MockSlave(dt)
    else:
        from adapters import RealMaster, RealSlave
        master, slave = RealMaster(), RealSlave()

    bridge = Bridge(cfg, master, slave, dt)
    code = bridge.run(cycles)
    print("[%s] 退出码=%d（move=%d busy=%d）" %
          (time.strftime("%H:%M:%S"), code, bridge.moves, bridge.busy), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
