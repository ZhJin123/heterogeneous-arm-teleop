# heterogeneous-arm-teleop

**一个可迁移的 Qoder Skill + 代码脚手架：让一台机械臂遥操作另一台异构机械臂。**

Heterogeneous master–slave teleoperation, scaffolded: interview the hardware → generate a runnable
codebase (virtual-TCP delta mapping, 15 Hz MoveL loop, self-test, commissioning plan).

## 它解决什么

两台机械臂通常在这些地方不一样：TCP 定义、工具长度、安装姿态、基座朝向、SDK 接口、单位。
直接把主手 TCP 的绝对位姿发给从手，会跳变、会镜像、手腕会转反。

本 skill 给出一套经过实机验证的做法：

- **虚拟 TCP 增量映射**：两侧各维护一个与世界对齐的虚拟 TCP，只传增量；
- **锚定一次**：初始化时冻结"从虚拟 TCP → 从真实 TCP"的固定旋转偏置；
- **位置级执行**：绝对目标 + 每周期限速 + 碎步抑制 + busy 冻结；
- **最小安全层**：断流/异常/非法数据停车，硬件急停是唯一权威安全边界；
- **可迁移**：映射数学与硬件无关，厂商 SDK 只出现在 `adapters.py`。

## 快速开始

```bash
# 1) 校验需求合同
python scripts/check_contract.py examples/teleop_contract.example.json

# 2) 生成一个可跑的代码库
python scripts/scaffold.py --target ~/my_teleop --name my_teleop \
    --contract examples/teleop_contract.example.json

# 3) 无硬件自检 + 假硬件闭环
cd ~/my_teleop
python selftest.py
python bridge.py --mock --cycles 100
```

生成的目录：

```
my_teleop/
├── bridge.py               # 15Hz 主循环（INIT → WAIT → ALIGN → RUN）
├── math3d.py               # 四元数/旋转/RPY/增量
├── virtual_tcp.py          # 主/从虚拟 TCP + 固定旋转偏置
├── mapper.py               # 差分 + 死区 + 世界系转换
├── motion_controller.py    # 限速 / min-step / busy 冻结
├── guard.py                # 数据与目标合法性
├── adapters.py             # ★ 唯一需要对接厂商 SDK 的地方
├── mock_adapters.py        # 假主手/假从手
├── selftest.py             # 4 项无硬件自检
├── teleop_config.json      # 合同参数
├── CONTRACT.md             # 需求合同
├── README.md / TEST_PLAN.md
└── requirements.txt        # numpy
```

## 仓库结构

| 路径 | 内容 |
|---|---|
| [`SKILL.md`](SKILL.md) | Skill 正文：何时用、5 个阶段、不可破坏的设计规则、默认参数、坑 |
| [`reference.md`](reference.md) | 数学推导、`R_WM` 标定、配置字段、适配器契约、验证、排障 |
| [`templates/CONTRACT.md`](templates/CONTRACT.md) | 需求访谈清单（生成代码前逐项填） |
| [`templates/core/`](templates/core/) | 生成用的核心代码（与硬件无关） |
| [`scripts/scaffold.py`](scripts/scaffold.py) | 由合同生成代码库 |
| [`scripts/check_contract.py`](scripts/check_contract.py) | 合同校验（单位/限位/busy 语义等） |
| [`examples/teleop_contract.example.json`](examples/teleop_contract.example.json) | 填好的示例合同 |
| [`examples/micro-master-elfin3.md`](examples/micro-master-elfin3.md) | 真实案例：自研 6 轴 → 大族 Elfin3（含 J6 符号事故） |

## 设计要点（生成代码里不要改）

1. 只传增量，不复制主手绝对姿态。
2. 固定旋转偏置只在锚定算一次。
3. 从手反馈不得写回虚拟目标。
4. busy 时冻结目标，不推进 `last_target`。
5. 内部米、SDK 边界毫米；RPY = `Rz·Ry·Rx`。
6. 死区累加不丢弃，避免慢速漂移。
7. 主手断流/数据非法 → 停车；硬件急停是最后一道防线。

## 作为 Qoder Skill 安装

把整个目录放到：

- 个人：`~/.qoder-cn/skills/heterogeneous-arm-teleop/`
- 项目：`<your-repo>/.qoder/skills/heterogeneous-arm-teleop/`

之后当你提出"用一台臂遥操作另一台异构臂 / 搭一个主从遥操作桥"时，Agent 会按
`SKILL.md` 的流程先访谈、再生成、再验收。

## 许可

MIT（见 [LICENSE](LICENSE)）。案例中的参数是实测示例，换成你自己的值。

不含任何厂商 SDK 二进制；`CPS.so`、`libsynarm.so` 等请从设备厂商获取，并遵守其许可。
