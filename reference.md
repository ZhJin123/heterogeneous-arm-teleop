# Reference — 数学、配置、适配器与排障

## 1. 为什么用"虚拟 TCP + 增量"

异构两臂的差异不止尺寸：

- TCP 定义不同（法兰 / 工具 / 相机）；
- 工具安装姿态不同；
- 基座朝向不同（常见相差 90°）；
- 主手可回零、从手有自己的一套姿态约定。

如果直接把主手 TCP 的绝对姿态下发给从手，从手会先"跳"到主手的姿态，且两臂工具轴不一致时
后续旋转也会错。虚拟 TCP 的做法是：

- 每一侧在**世界系**里维护一个"姿态与世界对齐"的虚拟 TCP；
- 位置与真实 TCP 共原点；
- 初始化时 `R_virtual,0 = I`，之后只被增量推动；
- 从手真实目标 = 从虚拟 TCP × **固定旋转偏置**（偏置在锚定时算一次）。

这样两臂的"初始朝向差"被冻结为常量，操作者拖主手时只传递运动，不传递绝对姿态。

## 2. 坐标与符号约定（写进配置，不要口头约定）

| 符号 | 含义 |
|---|---|
| W | 世界系（推荐 = 从手基座；X 前 / Y 左 / Z 上） |
| M | 主手基座系 |
| Tm / Ts | 主/从真实 TCP |
| Vm / Vs | 主/从虚拟 TCP |

- 齐次变换 `A_T_B`：把 B 系表达的量转到 A 系。
- 四元数顺序 `(x, y, z, w)`。
- RPY：`R = Rz(yaw)·Ry(pitch)·Rx(roll)`（固定轴 XYZ），与多数协作臂 SDK 一致。
- 单位：内部米；SDK/HTTP 边界毫米；关节角在 UDP/SDK 常见为 rad，控制器接口常见 deg。

## 3. 标定 `R_WM`（主手基座 → 世界）

不要猜，用一次实测：

1. 世界系选从手基座，确认轴向（X 前 / Y 左 / Z 上）。
2. 主手沿其基座 **+X** 推 100mm，记录世界系位移 `dX`。
3. 沿 **+Y** 推 100mm，记录 `dY`；沿 **+Z** 推 100mm，记录 `dZ`。
4. `R_WM` 的三列就是 `dX/|dX|, dY/|dY|, dZ/|dZ|`（注意单位一致）。
5. 验证 `det(R_WM) ≈ +1`、三列正交；转成 RPY 写入 `master_base.rpy_deg`。
6. 若 `det ≈ −1`，说明测到了镜像（某一轴方向反了），重新测，不要硬用。

常见结果：两臂相差 90° → `rpy_deg = [0, 0, ±90]`。选错符号会出现"全方向偏 90°"。

## 4. 映射数学

初始化（每次锚定）：

```
p_Vm,0 = R_WM · p_Tm,0      R_Vm,0 = I
p_Vs,0 = p_Ts,0             R_Vs,0 = I
R_offset = R_Vs,0ᵀ · R_Ts,0 = R_Ts,0
```

周期（主手相邻帧差分）：

```
Δp_M = p_M,k − p_M,k−1
ΔR_M = R_M,k · R_M,k−1ᵀ
Δp_W = R_WM · Δp_M
ΔR_W = R_WM · ΔR_M · R_WMᵀ
p_Vm += Δp_W           R_Vm = ΔR_W · R_Vm
p_Vs += k_pos · Δp_W   R_Vs = ΔR_W · R_Vs
```

下发从手：

```
p_Ts,target = p_Vs
R_Ts,target = R_Vs · R_offset
```

例子：`R_WM = Rz(−90°)`，`Δp_M = (10, 20, 30) mm` → `Δp_W = (20, −10, 30) mm`。

姿态限速与 `k_rot` 都在**旋转向量**上做：`rotvec → 缩放/截断 → rotvec_to_matrix`，
不要对旋转矩阵元素做线性缩放。

## 5. `teleop_config.json` 字段

| 字段 | 单位 | 含义 |
|---|---|---|
| `project` | — | 项目名 |
| `rate_hz` | Hz | 控制周期（默认 15） |
| `world.name/axes` | — | 世界系说明 |
| `master_base.rpy_deg` | deg | `R_WM`（必须实测） |
| `master_base.offset_m` | m | 主手基座原点（仅可视化） |
| `master_tcp.offset_m/axis` | m | 主手 TCP 相对法兰的偏移（裸法兰/工具） |
| `slave.tool_name` | — | 从手工具坐标系名 |
| `slave.pose_units` | — | 适配层边界单位（本脚手架按 mm/deg） |
| `slave.busy_status` | — | 上一条未完成时 SDK 返回什么 |
| `slave.speed_limit` | mm/s | 从手硬上限 |
| `mapping.k_pos/k_rot` | — | 位置/旋转比例 |
| `mapping.vmax/wmax` | mm/s, deg/s | 每周期限速 |
| `mapping.linear_acc` | mm/s² | MoveL 加速度 |
| `mapping.radius` | mm | MoveL 过渡半径 |
| `mapping.master_*_deadband_*` | mm, deg | 主手微抖死区（累加不丢） |
| `mapping.min_step_*` | mm, deg | 碎步阈值 |
| `safety.master_timeout_s` | s | 断流停车 |
| `safety.limit_box` | mm/deg | 软限位（可为 null） |
| `safety.hardware_estop` | bool | 现场是否有硬件急停 |

## 6. 适配器契约

### MasterSource

```python
MasterSample(pos_m: np.ndarray(3), quat_xyzw: np.ndarray(4),
             joints_rad: np.ndarray(n), stamp: float)
```

- `pos_m` 必须是**主手基座系**、米；`quat` 是 TCP 姿态（不是关节）。
- `read()` 必须非阻塞，返回**最新一帧**；没有新数据返回 `None`。
- SDK 是阻塞式时：后台线程持续读，`read()` 返回缓存的最新样本。
- `stamp = time.monotonic()`；bridge 用它判断断流。

### SlaveSink

```python
health() -> {"ok": bool, "tool_active": bool | None, ...}
start(limits) -> SlavePose
read_pose() -> SlavePose
move(target6, speed, acc, radius) -> ("ok" | "busy", SlavePose | None)
stop(release: bool) -> None
```

- `target6 = [x,y,z,rx,ry,rz]`，mm/deg，**绝对目标**。
- 上一条未完成 → 返回 `"busy"`（bridge 会冻结目标，不推进）。
- 硬错误（越界/报警/求逆解失败）→ 抛异常；bridge 停止并退出。
- 工具坐标系未激活 → `health().tool_active = False`，bridge 拒绝启动。

## 7. 生成的文件

| 文件 | 职责 |
|---|---|
| `bridge.py` | 主循环：INIT → WAIT → ALIGN → RUN；断流/异常 → 停 |
| `math3d.py` | 四元数/矩阵/旋转向量/RPY/正交化/增量转换 |
| `virtual_tcp.py` | 主/从虚拟 TCP、`R_offset`、初始化一致性 |
| `mapper.py` | 差分、死区（累加不丢）、世界系转换 |
| `motion_controller.py` | 每周期限速、min-step、busy 冻结基准 |
| `guard.py` | 数据合法性 / 目标合法性 |
| `adapters.py` | **唯一需要厂商 SDK 的地方** |
| `mock_adapters.py` | 假主手/假从手（正弦 + 限速追随） |
| `selftest.py` | 4 项无硬件自检 |

## 8. 验证

`selftest.py` 覆盖：

1. `R_WM` 方向（+X → 世界 −Y，示例）；
2. 初始化一致性（应 0）；
3. 纯平移 1:1 / 纯旋转 1:1；
4. 每周期限速与碎步抑制。

`bridge.py --mock --cycles 100` 覆盖：锚定、busy 冻结、1Hz 汇总、退出清理。

真机六轴表（世界系 = 从手基座；按你的 `R_WM` 填期望方向）：

| # | 主手动作（主手基座系） | 从手期望（世界系） | 通过 |
|---|---|---|---|
| 1 | +X 推 100mm | `R_WM·(+X)` 方向 100mm | ☐ |
| 2 | +Y 推 100mm | `R_WM·(+Y)` 方向 100mm | ☐ |
| 3 | +Z 抬 100mm | `R_WM·(+Z)` 方向 100mm | ☐ |
| 4 | 绕 +X 滚 10° | 绕 `R_WM·(+X)` 同向 10° | ☐ |
| 5 | 绕 +Y 俯仰 10° | 绕 `R_WM·(+Y)` 同向 10° | ☐ |
| 6 | 绕 +Z 偏航 10° | 绕 `R_WM·(+Z)` 同向 10° | ☐ |

## 9. 调参顺序

1. 方向先全部通过，再谈手感。
2. `k_pos/k_rot` 保持 1.0（1:1）；只有行程量级差很多才调。
3. 跟随滞后：先加 `radius`（10→20），再加 `linear_acc`（300→400），最后才动 `vmax`。
4. 抖动：加主手死区（0.5→1.0mm，1→2°）。
5. 慢速丢步/漂移：确认死区是"累加不丢"，单位没有 mm/m 混用。
6. 频繁 busy：降低主手速度或提高 `vmax`；不要靠"每周期重发同一个目标"硬顶。

## 10. 排障

| 现象 | 先查 |
|---|---|
| 进 RUN 时从手跳一下 | 初始化一致性是否 0；`R_offset` 是否每次锚定重算；目标是否用了绝对姿态 |
| 全方向都偏 90° | `master_base.rpy_deg` 符号/轴序 |
| 单轴反向 | 该关节符号（手写 FK）或 `R_WM` 的对应列 |
| 工具点位置差一个固定向量 | 主手/从手 TCP 偏移定义（法兰 vs 工具） |
| 跟随越来越偏 | 死区丢增量；单位混用；反馈覆盖了虚拟目标 |
| 从手一动就停 | busy 语义理解错（把"执行中"当错误）；或 `vmax` 超服务端上限 |
| 求逆解失败 | 起始位姿贴边界/奇异；换锚定姿态；确认工具坐标系正确 |
| 工具名生效失败但仍在动 | SDK 静默回退到法兰；改为 fail-fast |

## 11. 案例

[`examples/micro-master-elfin3.md`](examples/micro-master-elfin3.md)：自研 6 轴主手 → 大族 Elfin3，
含布局、`tool230`、J6 符号事故、模型基座坐标系等真实经验。

## 12. 发布注意

- 厂商 SDK（`.so`、Python 扩展）通常不可再分发：公开仓库不要带二进制，README 写获取方式。
- 本 skill 与脚手架代码可用 MIT；案例中的具体参数（工具长度、基座偏移）只是示例。
