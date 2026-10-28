---
name: heterogeneous-arm-teleop
description: Scaffolds a heterogeneous master-slave teleoperation stack where one robot arm drives a different arm: interviews the hardware, derives base-frame mapping and virtual-TCP delta mapping, then generates a runnable Python codebase (adapters, 15Hz bridge, self-test, test plan). Use when building leader-follower teleoperation for two different arms, porting teleop to new hardware, or when the user mentions 异构机械臂主从遥操作, master-slave teleop, virtual TCP, or teleop scaffolding.
---

# Heterogeneous Arm Teleoperation

> Build a leader-follower teleop stack for two **different** robot arms: collect the hardware
> contract, generate a runnable codebase, then commission it on the real machine.

This skill is a **method + scaffold**, not a project manual. It produces a new repo; it does not
require any particular vendor SDK.

## 中文速览（TL;DR）

1. **先访谈**：主手/从手接口、世界系、`R_WM`、TCP、busy 语义、单位（填 `templates/CONTRACT.md`）。
2. **选虚拟 TCP 增量映射**：只传增量；锚定时冻结 `R_offset`；反馈不得覆盖目标。
3. **生成代码**：`python scripts/scaffold.py --target ... --contract ...`，只改 `adapters.py`。
4. **先自检再上真机**：`selftest.py` → `bridge.py --mock` → 单轴方向 → 六轴验收。
5. **硬件急停是唯一安全边界**；软限位/工作空间预检不是必需品。

## When to use

- One arm (master/leader, often back-drivable) should drive another arm (slave/follower).
- The two arms differ in brand, size, joint layout, TCP definition, or mounting orientation.
- You need position-level following (MoveL / position servo), with a safe stop path.

**Do not** use TCP-delta mapping when the arms are identical and you specifically want joint-space
mirroring — joint mapping is simpler there. This skill targets the heterogeneous case.

## Workflow (follow in order)

### Phase 0 — Interview: fill the contract

Copy [`templates/CONTRACT.md`](templates/CONTRACT.md) and get real answers. Minimum viable set:

1. **Master**: interface, rate, units, joint vs TCP, FK availability, TCP offset, attitude format,
   timeout signal.
2. **Slave**: pose read (which frame), motion API (absolute vs incremental), busy/completion
   semantics, tool frame name and how it is activated, speed limits, stop API.
3. **Geometry**: world frame (prefer slave base), **rotation from master base to world** (measure,
   don't guess), TCP definitions, extra mounting offsets.
4. **Policy**: `k_pos`, `k_rot`, `vmax`, `wmax`, acceleration, transition radius, deadbands,
   min-step, gripper, soft limits vs hardware e-stop.
5. **Acceptance**: expected direction of each of the 6 axes, allowed lag, failure handling.

Write the answers into `teleop_config.json` (see
[`examples/teleop_contract.example.json`](examples/teleop_contract.example.json)).
If a value is unknown, measure it — do not guess.

### Phase 1 — Decide the mapping architecture

Use this default unless the hardware forces otherwise:

```
master SDK → MasterSource.read()  →  mapper (Δp_W, ΔR_W)  →  virtual TCP (maintained per side)
            →  motion controller (rate limit / min-step / busy freeze)  →  SlaveSink.move()
```

Rules that make it work across heterogeneous arms:

- Transfer **increments**, never the master's absolute TCP pose.
- Each side keeps a virtual TCP initialized to world attitude; only increments move it.
- Freeze the fixed rotation offset between slave virtual and real TCP at anchor time.
- Anchor = record current master pose + current slave pose; no required home posture.
- Submit absolute targets at a fixed rate (10–30 Hz), with per-cycle rate limits.

### Phase 2 — Generate the codebase

```bash
python scripts/check_contract.py my_contract.json
python scripts/scaffold.py --target ~/my_teleop --name my_teleop --contract my_contract.json
```

The scaffold writes: `bridge.py`, `math3d.py`, `virtual_tcp.py`, `mapper.py`,
`motion_controller.py`, `guard.py`, `adapters.py`, `mock_adapters.py`, `selftest.py`,
`teleop_config.json`, `CONTRACT.md`, `README.md`, `TEST_PLAN.md`.

### Phase 3 — Implement only the adapters

Edit `adapters.py`:

- `RealMaster.open/read/close` → return `MasterSample(pos_m, quat_xyzw, joints_rad, stamp)`.
  Convert to **meters, master base frame, quaternion (x,y,z,w)**.
- `RealSlave.health/start/read_pose/move/stop/close` → return `SlavePose(pos_mm, rpy_deg)`.
  `move()` returns `("ok", pose)` when accepted and `("busy", pose)` when the previous target is
  still executing.

Nothing in the mapping math should need changes. If you feel like editing `virtual_tcp.py`,
the hardware contract is probably being violated.

### Phase 4 — Verify before touching the real slave

```bash
python selftest.py                      # math/state invariants, no hardware
python bridge.py --mock --cycles 100    # 15Hz loop with fake master/slave
```

Both must pass. Then, on hardware, verify **one axis at a time** with a hand on the e-stop.
Fill the direction table in `TEST_PLAN.md`; a global 90° error means `master_base.rpy_deg` is wrong.

### Phase 5 — Commission

- Run at reduced speed only long enough to confirm directions; then move to production values.
- Exercise: master stream loss, slave alarm, e-stop, re-anchor.
- Record final `k_pos/k_rot`, limits, and any sign fixes in the repo.

## Non-negotiable design rules

1. **Never copy absolute master attitude** to the slave.
2. `R_offset` (slave real ← virtual) is computed **once per anchor**, never per cycle.
3. Slave feedback must **never** overwrite the virtual TCP target.
4. On `busy`, freeze the last accepted target — do not advance it.
5. Internal positions in **meters**; convert to mm only at the SDK boundary.
6. RPY convention `R = Rz·Ry·Rx`; state it explicitly in the config.
7. Deadbands **accumulate**, never drop increments (dropping causes slow drift).
8. Rate-limit position and orientation per cycle (`vmax·dt`, `wmax·dt`).
9. A missing/invalid master sample must stop the slave; hardware e-stop is the primary safety.
10. Master and slave timestamps: consume the **latest** sample, do not queue.

## Defaults that work in practice

| Parameter | Default | Note |
|---|---|---|
| rate | 15 Hz | 10–30 Hz; match the slave's motion API |
| k_pos / k_rot | 1.0 / 1.0 | 1:1 mapping; tune after directions pass |
| vmax / wmax | 150 mm/s / 90 °/s | must be ≤ slave limits |
| linear_acc | 300 mm/s² | MoveL acceleration |
| radius | 10 mm | transition radius; avoids stop-at-every-waypoint |
| master deadband | 0.5 mm / 1.0° | zero-force jitter |
| min-step | 0.5 mm / 0.5° | skip tiny targets |
| master timeout | 0.3 s | stream loss → stop |

## Pitfalls (each one has bitten a real deployment)

- **Hand-written FK vs URDF**: a joint sign error can survive for years if that joint's gravity
  torque is ~0 (e.g., the last wrist joint with its COM on the axis). Verify every joint by
  single-joint motion, not by “the arm looks right”.
- **Base rotation sign**: ±90° errors are common. Measure: push master +X, watch world motion.
- **Tool axis/length**: confirm the tool point (not the flange) moves as expected; a 230 mm tool
  amplifies every tiny sign error.
- **Silent tool-frame fallback**: some SDKs keep running with the flange frame when the named tool
  is missing. Fail fast instead.
- **Busy spam**: repeatedly submitting blocking MoveL targets looks like following but is not.
- **Feedback overwrite**: replacing the virtual target with the measured pose every cycle makes
  the arm stutter and lose the master's lead.
- **Unit soup**: mm↔m, deg↔rad, quaternion order (w,x,y,z vs x,y,z,w) — write them down once.
- **URDF world link**: a `world→base` fixed rotation may exist only for simulation; verify which
  link corresponds to the controller's base before using a model for visualization.
- **.so / vendor SDK**: don't publish proprietary binaries; keep them out of public repos.

## Files

- [`templates/CONTRACT.md`](templates/CONTRACT.md) — the interview checklist.
- [`examples/teleop_contract.example.json`](examples/teleop_contract.example.json) — filled example.
- [`examples/micro-master-elfin3.md`](examples/micro-master-elfin3.md) — a real deployment case study.
- [`reference.md`](reference.md) — math, config schema, adapter contract, troubleshooting.
- [`scripts/scaffold.py`](scripts/scaffold.py), [`scripts/check_contract.py`](scripts/check_contract.py).

## Using this as a Qoder skill

Copy this directory to `~/.qoder-cn/skills/heterogeneous-arm-teleop/` (personal) or
`<repo>/.qoder/skills/heterogeneous-arm-teleop/` (project). The agent will then apply the workflow
when the user asks to build a heterogeneous-arm teleop stack.
