#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scaffold.py — 从 teleop_config.json 生成一个可跑的遥操作代码库。

用法：
    python scripts/scaffold.py --target ~/my_teleop --name my_teleop \
        --contract examples/teleop_contract.example.json

生成内容：
    teleop_config.json   合同里的全部参数
    bridge.py            15Hz 主循环（已接好映射链）
    math3d / virtual_tcp / mapper / motion_controller / guard / adapters / mock_adapters
    selftest.py          无硬件自检
    CONTRACT.md          需求合同副本
    README.md / TEST_PLAN.md
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "templates" / "core"
DEFAULT_CONTRACT = ROOT / "examples" / "teleop_contract.example.json"
CONTRACT_TEMPLATE = ROOT / "templates" / "CONTRACT.md"

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_contract  # noqa: E402


README_TMPL = """# {name}

由 [heterogeneous-arm-teleop](https://github.com/ZhJin123/heterogeneous-arm-teleop) 脚手架生成：
异构主从遥操作（虚拟 TCP 增量映射，{rate:g}Hz，MoveL 绝对目标）。

## 文件

| 文件 | 作用 |
|---|---|
| `bridge.py` | 主循环：主手采样 → 映射 → 限速 → 从手 MoveL |
| `math3d.py` | 四元数/旋转/RPY/增量转换 |
| `virtual_tcp.py` | 主/从虚拟 TCP 与固定旋转偏置 |
| `mapper.py` | 相邻帧差分、死区、世界系转换 |
| `motion_controller.py` | 每周期限速、碎步抑制、busy 冻结 |
| `guard.py` | 数据合法性/断流/目标非法 → 停车 |
| `adapters.py` | **唯一需要对接厂商 SDK 的地方**（RealMaster/RealSlave） |
| `mock_adapters.py` | 假主手/假从手，无需硬件 |
| `teleop_config.json` | 全部参数（来自需求合同） |
| `CONTRACT.md` | 需求合同（接口/几何/策略/验收） |
| `TEST_PLAN.md` | 实机测试步骤 |

## 快速开始

```bash
python selftest.py                 # 数学/状态机自检（无硬件）
python bridge.py --mock --cycles 100   # 假硬件跑 100 周期
```

真机：

1. 按 `CONTRACT.md` 和 `teleop_config.json` 核对参数；
2. 在 `adapters.py` 实现 `RealMaster` / `RealSlave`（厂商 SDK 只出现在这里）；
3. `python bridge.py`；
4. 按 `TEST_PLAN.md` 做单轴方向与跟随验收。

## 不要改的设计

- 只传**增量**，不复制主手绝对姿态；
- `R_offset` 只在锚定算一次；
- 从手反馈不得写回虚拟目标；
- busy 时冻结目标（不推进 `last_target`）；
- 内部位置用米，SDK 边界用毫米；RPY = `Rz·Ry·Rx`。

## 配置摘要

- 世界系：`{world}`（X 前 / Y 左 / Z 上）
- `R_WM`（主手基座 → 世界）RPY：`{rpy}` deg
- 映射：k_pos={k_pos}, k_rot={k_rot}, vmax={vmax} mm/s, wmax={wmax} °/s, acc={acc}, radius={radius}
- 从手工具：`{tool}`；速度上限 `{speed_limit}` mm/s
- 软限位：`{limit}`；硬件急停：`{estop}`
"""

TEST_PLAN_TMPL = """# {name} 实机测试计划（生成）

> 只做必要的测试，不做多余离线验证；每一步通过再进下一步。

## 0. 生成后（无硬件）

- [ ] `python selftest.py` → `ALL PASS`
- [ ] `python bridge.py --mock --cycles 100` → 退出码 0，日志里能看到 `move=` 增长、`busy=` 有值

## 1. 参数核对

- [ ] `teleop_config.json` 的 `master_base.rpy_deg` 与实测基座朝向一致
- [ ] `master_tcp.offset_m` / 从手 `tool_name` 与实际工具一致
- [ ] `vmax/wmax` ≤ 从手速度上限；`master_timeout_s` 合适

## 2. 适配器（真机只读）

- [ ] 实现 `RealMaster.open/read`，先只打印 TCP，不接从手
- [ ] 实现 `RealSlave.health/read_pose`，确认工具坐标系已激活（不是静默回退到法兰）
- [ ] 两边 TCP 坐标都在**各自基座系**下，单位已转成 m / mm / deg

## 3. 单轴方向（每次只动一个方向，人手在急停旁）

| # | 主手动作（主手基座系） | 从手期望（世界系） | 通过 |
|---|---|---|---|
| 1 | 沿 +X 推 100mm | 按 `R_WM` 换算后的方向、100mm | ☐ |
| 2 | 沿 +Y 推 100mm | 同上 | ☐ |
| 3 | 沿 +Z 抬 100mm | +Z 100mm | ☐ |
| 4 | 绕 +X 滚 10° | 按 `R_WM` 换算后的轴、同向 10° | ☐ |
| 5 | 绕 +Y 俯仰 10° | 同上 | ☐ |
| 6 | 绕 +Z 偏航 10° | 同上 | ☐ |

方向/轴向不对：改 `master_base.rpy_deg`，不要用摆姿势补偿。
**注意**：手写 FK 的关节符号必须逐轴核对；只做重力补偿的运动学链可能把某个轴的
符号错误藏住（该轴重力矩≈0 时永远看不出来）。

## 4. 跟随验收

- [ ] 连续拖动 30s：无振荡、无报警、无过冲
- [ ] 快速大幅拖动：限速生效、松手即停
- [ ] 慢速拖动：位置/姿态丢失 < 死区 + min-step（不被吃掉）
- [ ] 目标-反馈 lag 在预期内；busy 时目标冻结

## 5. 异常演练

- [ ] 拔掉主手/停掉主手进程 → {timeout}s 内停车
- [ ] 从手报警/越界 → 停止且不自动重试
- [ ] 硬件急停按下 → 恢复流程走一遍
- [ ] 重新锚定（重启 bridge）后从手不跳

## 6. 记录

- 最终 k_pos/k_rot、vmax/wmax、acc/radius
- 遗留问题与下一步
"""


def main(argv=None):
    ap = argparse.ArgumentParser(description="生成异构主从遥操作代码库")
    ap.add_argument("--target", required=True, help="目标目录")
    ap.add_argument("--name", default="", help="项目名（默认取合同里的 project）")
    ap.add_argument("--contract", default=str(DEFAULT_CONTRACT), help="合同 JSON 路径")
    ap.add_argument("--force", action="store_true", help="目标目录非空时仍写入")
    args = ap.parse_args(argv)

    target = Path(args.target).expanduser().resolve()
    if target.exists() and any(target.iterdir()) and not args.force:
        print("ERROR 目标目录非空：%s（加 --force 覆盖）" % target)
        return 2

    with open(args.contract, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    errors, warnings = check_contract.validate(cfg)
    for w in warnings:
        print("WARN  " + w)
    if errors:
        for e in errors:
            print("ERROR " + e)
        print("\n合同有 %d 个错误，先补齐再生成。" % len(errors))
        return 1

    name = args.name.strip() or cfg.get("project") or target.name
    cfg["project"] = name

    target.mkdir(parents=True, exist_ok=True)
    for src in sorted(CORE.iterdir()):
        if src.suffix == ".py" or src.name == "requirements.txt":
            shutil.copy2(src, target / src.name)
    shutil.copy2(CONTRACT_TEMPLATE, target / "CONTRACT.md")
    with open(target / "teleop_config.json", "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
        f.write("\n")

    rate = float(cfg.get("rate_hz", 15.0))
    m = cfg.get("mapping", {})
    s = cfg.get("safety", {})
    readme = README_TMPL.format(
        name=name, rate=rate,
        world=cfg.get("world", {}).get("name", "slave_base"),
        rpy=cfg.get("master_base", {}).get("rpy_deg"),
        k_pos=m.get("k_pos", 1.0), k_rot=m.get("k_rot", 1.0),
        vmax=m.get("vmax", 150), wmax=m.get("wmax", 90),
        acc=m.get("linear_acc", 300), radius=m.get("radius", 10),
        tool=cfg.get("slave", {}).get("tool_name", "-"),
        speed_limit=cfg.get("slave", {}).get("speed_limit", "-"),
        limit=s.get("limit_box") if s.get("limit_box") is not None else "无（硬件急停兜底）",
        estop=s.get("hardware_estop", False),
    )
    test_plan = TEST_PLAN_TMPL.format(name=name, timeout=s.get("master_timeout_s", 0.3))
    (target / "README.md").write_text(readme, encoding="utf-8")
    (target / "TEST_PLAN.md").write_text(test_plan, encoding="utf-8")

    print("\n生成完成：%s" % target)
    print("  下一步：")
    print("    cd %s" % target)
    print("    python selftest.py")
    print("    python bridge.py --mock --cycles 100")
    print("  然后按 CONTRACT.md 实现 adapters.py 的 RealMaster/RealSlave，再跑 TEST_PLAN.md。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
