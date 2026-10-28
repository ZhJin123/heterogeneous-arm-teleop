#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_contract.py — 校验 teleop_config.json / 需求合同。

用法：python scripts/check_contract.py examples/teleop_contract.example.json
退出码 0 = 无错误（警告不阻塞），1 = 有必须修的字段。
"""

import json
import sys


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate(cfg):
    errors, warnings = [], []

    def need(path, cond, msg):
        if not cond:
            errors.append("%s: %s" % (path, msg))

    need("project", isinstance(cfg.get("project"), str) and cfg["project"].strip(),
         "缺少项目名")
    rate = cfg.get("rate_hz")
    need("rate_hz", _num(rate) and 0 < rate <= 200, "应为 0–200 的数值")
    if _num(rate) and not (5 <= rate <= 60):
        warnings.append("rate_hz=%.1f：常规遥操作在 5–60Hz；过高会挤爆从手 SDK" % rate)

    need("world.name", isinstance(cfg.get("world", {}).get("name"), str), "缺少世界系定义")

    rpy = cfg.get("master_base", {}).get("rpy_deg")
    need("master_base.rpy_deg", isinstance(rpy, list) and len(rpy) == 3 and all(_num(v) for v in rpy),
         "需要 3 个数值（度）")
    if isinstance(rpy, list) and len(rpy) == 3 and all(_num(v) and abs(v) < 1e-9 for v in rpy):
        warnings.append("master_base.rpy_deg 全为 0：确认两臂基座真的同向（异构布局很少同向）")

    off = cfg.get("master_tcp", {}).get("offset_m")
    need("master_tcp.offset_m", isinstance(off, list) and len(off) == 3 and all(_num(v) for v in off),
         "需要 3 个数值（米）")
    if isinstance(off, list) and len(off) == 3 and all(_num(v) and abs(v) < 1e-12 for v in off):
        warnings.append("master_tcp.offset_m 全为 0：确认主手 TCP 真的是法兰原点")

    slave = cfg.get("slave", {})
    units = slave.get("pose_units", {})
    need("slave.pose_units.pos", units.get("pos") in ("mm", "m"), "应为 mm 或 m")
    need("slave.pose_units.rot", units.get("rot") in ("deg", "rad"), "应为 deg 或 rad")
    if units.get("pos") != "mm" or units.get("rot") != "deg":
        warnings.append("适配层边界按 mm/deg 设计：非 mm/deg 时必须在 adapter 里转换")
    need("slave.busy_status", isinstance(slave.get("busy_status"), str) and slave["busy_status"],
         "必须写明上一条未完成时的返回值")
    need("slave.speed_limit", _num(slave.get("speed_limit")) and slave["speed_limit"] > 0,
         "缺少从手速度上限")

    m = cfg.get("mapping", {})
    for k in ("k_pos", "k_rot", "vmax", "wmax"):
        need("mapping." + k, _num(m.get(k)) and m[k] > 0, "应为正数")
    if _num(m.get("vmax")) and _num(slave.get("speed_limit")) and m["vmax"] > slave["speed_limit"]:
        warnings.append("mapping.vmax > slave.speed_limit：请求会被从手拒绝")
    if _num(m.get("wmax")) and _num(slave.get("rot_speed_limit")) \
            and m["wmax"] > slave["rot_speed_limit"]:
        warnings.append("mapping.wmax > slave.rot_speed_limit：请求会被从手拒绝")

    s = cfg.get("safety", {})
    need("safety.master_timeout_s", _num(s.get("master_timeout_s")) and s["master_timeout_s"] > 0,
         "缺少主手断流阈值")
    if s.get("hardware_estop") is not True:
        warnings.append("safety.hardware_estop 不是 true：本方案不设软限位，必须有硬件急停")
    if s.get("limit_box") is None:
        warnings.append("safety.limit_box=null：不做软限位，确认现场急停可用")

    return errors, warnings


def main(argv):
    path = argv[1] if len(argv) > 1 else "teleop_config.json"
    try:
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except Exception as e:  # noqa: BLE001
        print("ERROR 读取 %s 失败: %s" % (path, e))
        return 2

    errors, warnings = validate(cfg)
    for w in warnings:
        print("WARN  " + w)
    for e in errors:
        print("ERROR " + e)
    if errors:
        print("\n%d 个错误，先补齐再生成代码。" % len(errors))
        return 1
    print("\nOK  %s（%d 条提醒）" % (path, len(warnings)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
