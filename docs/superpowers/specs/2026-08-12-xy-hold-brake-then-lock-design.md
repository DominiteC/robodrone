# XY 定点回中"先刹停再锁点" + 锁存前馈 — 设计文档

> 状态：已实施（代码完成，待硬件试飞验证）
> 参考：Z 轴定高"速度环刹车→位置环锁高"逻辑（commander.c `setpointNormalFlight()`）、MiniFly 回中缓冲思路
> 涉及文件：`user/services/commander.c`（仅此一个源文件，control.c / control_pid.c / position.c 不动）

## 1. 目标

修复两个实测飞行问题：

1. **回中后往反方向猛冲一下**（主问题）：前飞后松杆回中，飞机不是平滑停下，而是先停、再往**反方向**（后方）冲一下。
2. **拨杆前飞响应偏慢**（次要）：推杆后速度起来得慢，体感"P 太小"。

本次方案以修复问题 1 为目标；问题 2 属于速度环/测速链路参数问题，单独立项（见 §6 后续优化），不在本设计内动参数。

## 2. 现状与根因分析

### 2.1 当前 XY 定点状态机（`setpointXY()`）

```
拨杆 (|roll|/|pitch|>1.5°)       回中且未锁点               已锁点
  │  modeVelocity                 │  modeAbs                 │ modeAbs
  │  vel = 杆量×4                 │  holdPos=position(锁存)  │ pos=holdPos 固定
  │  xyHoldActive=false           │  xyHoldActive=true       │
  └──────────────►────────────────┴────────────►─────────────┘
```

### 2.2 回中反冲的根因

回中瞬间 `holdPosX = state->position.x` 锁的是**滞后的位置估计**：

- `position.c` 的测速链路有两级低通（前置 α=0.15、融合后 α=0.20）+ INAV 速度残差每帧只修正 `w·dt≈2%`，前飞过程中 `position` 持续落后真实位置一段距离（实测量级可达 10~20cm）。
- 这段"欠记的位移"在回中后以**估计位置继续缓慢增长**的形式补记出来 → 估计位置越过锁存点 → 位置环误差变负 → 输出反向速度指令 → 飞机真实地**往后冲**。
- 叠加：回中时机身快速回正，pitch/roll 角速度大（100°/s+），陀螺旋转补偿有 3°/s 死区、增益 1.0，补偿不干净时残差直接进位置积分，放大"猛地"的体感。

位置环本身 Kp=1.0×0.5、Ki=Kd=0、限幅 30cm/s，增益温和，**不是超调，是锁存那一刻的测量本身就偏了**。

## 3. 方案设计

### 3.1 目标状态机（三态）

```
状态A 拨杆                       状态B 刹车                       状态C 保持
stickActive=true               !stickActive && !xyHoldActive     xyHoldActive=true
  modeVelocity                   modeVelocity                      modeAbs
  vel = 杆量×4                   vel = 0                          pos = holdPos 固定
  xyHoldActive=false             （位置环不介入）                  ──────────────┐
  isBrakingPosXY=false           │                                                 │
  └──────────────►───────────────┤                                                 │
                                 │  |v|<XY_BRAKE_VEL_DZ 或超时                     │
                                 │  ⇒ holdPos = position + velocity×LOOKAHEAD      │
                                 │  ⇒ xyHoldActive=true, flightClearPosPID()       │
                                 │  ⇒ 切 modeAbs                                   │
                                 └────────────────────────────────►───────────────┘
  刹车期间再次拨杆 ⇒ 回状态A（isBrakingPosXY=false，速度立即恢复）
```

### 3.2 状态转移细节（均在 `setpointXY()` 内，1ms 主循环）

1. **状态A 拨杆**（`stickActive==true`）：行为不变。`xyHoldActive=false; isBrakingPosXY=false; brakePosXYTime=0; modeVelocity; vel.x=-pitch×4; vel.y=-roll×4;`
2. **状态B 刹车**（`!stickActive && !xyHoldActive`）：
   - 首次进入：`isBrakingPosXY=true; brakePosXYTime=0;`
   - 每帧：`modeVelocity; vel.x=0; vel.y=0;`（速度环以 0 为目标刹停，位置环完全旁路）
   - 完成条件（满足其一即锁点）：
     - `fabsf(state->velocity.x) < XY_BRAKE_VEL_DZ && fabsf(state->velocity.y) < XY_BRAKE_VEL_DZ`（5cm/s）
     - `brakePosXYTime++ > XY_BRAKE_TIMEOUT_MS`（1200ms 超时强制锁点，防卡死）
   - 锁点动作（一帧内完成，随后进入状态C）：
     ```
     holdPosX = state->position.x + state->velocity.x * XY_HOLD_LOOKAHEAD;
     holdPosY = state->position.y + state->velocity.y * XY_HOLD_LOOKAHEAD;
     xyHoldActive = true;
     flightClearPosPID();          // 清位置环积分（现有行为保留）
     setpoint->mode_x = modeAbs; setpoint->mode_y = modeAbs;
     setpoint->pos.x = holdPosX;   setpoint->pos.y = holdPosY;
     ```
3. **状态C 保持**：行为不变。`modeAbs; pos=holdPos 固定; vel.x=0; vel.y=0;`
4. **光流失效**（`position_IsXYFlowValid()==false`）：保持现有保护——复位流状态、`xyHoldActive=false; isBrakingPosXY=false; brakePosXYTime=0; modeDisable;`
5. **起飞首帧**（`takeoffXYInitDone` 分支）：在现有锁 `holdPosX/Y` 的同时置 `xyHoldActive=true`（起飞即锁定起飞点，爬升期间直接走状态C，不进刹车态）。行为与现状几乎一致（现状首帧同样锁 holdPos，仅标志晚一拍置位）。
6. **降落/非定点模式**：保持现有重置逻辑（`xyHoldActive=false; isBrakingPosXY=false; brakePosXYTime=0;`）。

### 3.3 参数

| 参数 | 值 | 说明 |
|---|---|---|
| `XY_STICK_VEL_SCALE` | 4.0（不变） | 拨杆速度映射 |
| `XY_BRAKE_VEL_DZ` | 5.0 cm/s（已有定义，本次启用） | 刹车完成速度阈值，与 Z 轴 `Z_BRAKE_VEL_DZ` 一致 |
| `XY_BRAKE_TIMEOUT_MS` | 1200U（已有定义，本次启用） | 刹车超时强制锁点 |
| `XY_HOLD_LOOKAHEAD` | 0.2f（新增） | 锁存前馈时间常数 τ（s），补偿测速滞后；试飞后按表现调 |

### 3.4 死代码清理（行为无关，随本次一并做）

以下变量/宏自 minifly 骨架移植后只写不读，本次启用 `isBrakingPosXY / brakePosXYTime / XY_BRAKE_VEL_DZ / XY_BRAKE_TIMEOUT_MS`，其余删除：

- 删除：`isAdjustingPosXY`、`errorPosX`、`errorPosY`（从未参与计算）
- 删除：`XY_BRAKE_ATT_DZ`、`XY_BRAKE_SETTLE_CYCLES`、`XY_HOLD_ENABLE_HEIGHT`、`XY_HOLD_DISABLE_HEIGHT`（未使用宏）
- 保留并启用：`isBrakingPosXY`、`brakePosXYTime`
- Z 轴 `isBrakingPosZ / brakePosZTime / errorPosZ` 等不动

## 4. 数据流与接口变化

- **接口**：`commanderGetSetpoint()` 签名、`setpoint_t`、`PosMode`、PID 接口均不变。`modeAbs/modeVelocity/modeDisable` 语义不变，control.c 位置环/速度环无需改动。
- **数据流**：仅 `setpointXY()` 内部状态机重构；`holdPosX/Y` 从"回中瞬间锁存"改为"刹车完成后带前馈锁存"。
- **改动文件**：仅 `user/services/commander.c`。

## 5. 失败保护

- 刹车超时 1200ms 强制锁点（与 Z 轴刹车逻辑一致），避免速度长时间降不下来导致"永不锁点"。
- 刹车期间光流失效 → modeDisable + 复位，降级为速度保持（现有保护）。
- 刹车期间再次拨杆 → 立即回状态A，速度响应不中断。
- 锁存前馈 τ 取值保守（0.2s），即使偏大也只会导致锁点略超前、回中后轻微后退，不会发散。

## 6. 风险点与后续优化

- **手感变化**：回中后多一个"减速段"，飞手需试飞确认（预期：从"反冲"变为"平滑停住再锁定"）。
- **锁点精度**：刹车期位置环不介入，若光流漂移，锁点可能偏离"拨杆前目标点"——但方向一致、幅度可控，优于现状"锁滞后点被反向拉回"。
- **拨杆响应慢（问题 2）**：属速度环参数问题，另立设计（速度环加 Ki 消除稳态误差、Kp 0.11→0.15~0.2、光流前置 LPF α 0.15→0.10），本次不动参数。

## 7. 硬件验证方法

1. 编译通过（EIDE build 或 Keil MDK）。
2. 大杆量前飞 → 快速回中：飞机应减速停下，**不再反向冲**；地面站观察 `debug_pos_pid_out_x` 不应出现回中后的反向尖峰。
3. 小杆量微调 → 回中：同上，动作更轻。
4. 刹车过程中再次拨杆：应立即恢复速度响应，无迟滞。
5. 定点悬停 + 手动推偏：拉回行为应与改造前一致（位置环未改动）。
6. 一键起飞：起飞即锁定起飞点，爬升过程 XY 不漂。
7. 地面站遥测指标：回中后 `state->velocity.x/y` 平滑归零；`debug_flow_residual_x` 不再引发反向位置修正。

## 8. 实施清单

- [x] `commander.c`：重写 `setpointXY()` 为三态状态机（§3.1）
- [x] `commander.c`：新增 `XY_HOLD_LOOKAHEAD` 宏（§3.3）
- [x] `commander.c`：起飞首帧分支置 `xyHoldActive=true`（§3.2-5）
- [x] `commander.c`：清理死变量/死宏（§3.4）
- [ ] 编译 + 硬件试飞验证（§7）

## 9. 实施记录（2026-08-12）

**改动文件**：`user/services/commander.c`（唯一改动文件，与设计一致）

**实际改动**：
1. 宏定义区：删除 `XY_HOLD_ENABLE_HEIGHT / XY_HOLD_DISABLE_HEIGHT / XY_BRAKE_ATT_DZ / XY_BRAKE_SETTLE_CYCLES`，新增 `XY_HOLD_LOOKAHEAD 0.2f`。
2. 状态变量区：删除 `isAdjustingPosXY / adjustPosXYTime / errorPosX / errorPosY`；`holdPosX/Y` 注释更新为"刹车完成后锁存"。
3. `setpointXY()` 重写为三态状态机：拨杆(modeVelocity) → 回中刹车(modeVelocity 目标 0，速度<5cm/s 或 1200ms 超时) → 带速度前馈锁点(modeAbs)。锁点帧调用 `flightClearPosPID()`。
4. 起飞首帧分支：删 `isAdjustingPosXY=false / errorPosX/Y=0`，加 `xyHoldActive=true`（起飞即锁定起飞点）。
5. 着陆分支与非定点分支：删除 `isAdjustingPosXY` 赋值。

**与设计的一致性**：全部按设计实施，无偏离。

**验证结果**：
- 语法/类型编译通过：`arm-none-eabi-gcc 14.2 -std=c99 -mcpu=cortex-m4 -fsyntax-only ../user/services/commander.c`（exit=0，含全部 include 路径与 `-DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4`）。
- 文件编码保持 UTF-8 + CRLF 无损坏（573 CRLF、0 孤立 LF、无 BOM）。
- 死变量引用清零：`grep isAdjustingPosXY|errorPosX|errorPosY|adjustPosXYTime|XY_BRAKE_ATT|XY_BRAKE_SETTLE|XY_HOLD_ENABLE|XY_HOLD_DISABLE` 无匹配。
- **未做**：AC6 真机固件构建（本机无 Keil 工具链）与硬件试飞（§7 验证清单），需在用户的 Keil/EIDE 环境 + 真机完成。
