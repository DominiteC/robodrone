# PID 解耦 & Commander 副作用分离 实现计划

> **For agentic workers:** 使用 `superpowers:executing-plans` 逐步执行，每步完成后编译 + 上机实测。步骤用 checkbox (`- [ ]`) 跟踪。

**Goal:** 分三步解耦 robodrone 最严重的 3 个耦合点：① PIDInstance 全 public → 加访问接口；② commander setter 带副作用 → 边沿触发分离；③ commander 四种角色 → 拆子模块。

**Architecture:** 纯增量式重构。不改控制算法、不改 PID 参数、不改任务周期/优先级。每步增加一层抽象接口，旧代码通过接口适配后逐步收口。

**Tech Stack:** C (Keil MDK ARMCC), STM32F407, FreeRTOS, HAL

**约束:**
- 不提交 `MDK-ARM/project.uvoptx`
- 编辑前检查文件编码/BOM/换行，写回时保持原风格
- 新增 `.c/.h` 需同步加入 Keil 工程分组
- 每 Phase 完成后必须 **编译 0 error 0 warning → 烧录 → 起飞悬停 5s + yaw 转一圈 + 一键降落** 才算通过

---

## Phase 1: PID 访问接口封装（P0 #2）

> **改动量:** ~150 行新增 + ~200 行改引用
> **风险:** 极低（纯增量，不改已有函数签名）
> **验证:** 编译 + 上机实测

### Task 1.1: 新增 `PIDDump` 快照结构体 & `pid_dump()` 函数

**Files:**
- Modify: `user/control/PIDcontroller.h:82`（在 `PIDInstance` 定义后追加）
- Modify: `user/control/PIDcontroller.c:246`（在 `PID_Reset` 后追加）

**要点:**
1. 在 `PIDcontroller.h` 中新增 `PIDDump` 结构体，包含所有调试/显示需要的运行状态字段：
   - `float Output, Pout, Iout, Dout, ITerm, Err, Measure, Ref`
   - 不含配置参数（Kp/Ki/Kd 等），配置参数走 `PID_Init_Config_s`
2. 声明 `void pid_dump(const PIDInstance *pid, PIDDump *out);`
3. 在 `PIDcontroller.c` 实现 `pid_dump()` —— 纯值拷贝，不加锁（当前单任务场景不需要）

**编译验证点:** 新增代码无语法错误，编译通过。此时还没有 caller，只验证声明+定义合法。

---

### Task 1.2: 新增 `pid_get_config()` / `pid_set_config()` 函数

**Files:**
- Modify: `user/control/PIDcontroller.h`
- Modify: `user/control/PIDcontroller.c`

**要点:**
1. 声明 `void pid_get_config(const PIDInstance *pid, PID_Init_Config_s *out);`
2. 声明 `void pid_set_config(PIDInstance *pid, const PID_Init_Config_s *cfg);`
3. `pid_get_config()` 拷贝 Kp/Ki/Kd 等 15 个配置字段
4. `pid_set_config()` 写入配置字段（Kp/Ki/Kd/MaxOut/DeadBand/IntegralLimit 等），不改运行状态

**编译验证点:** 编译通过。

---

### Task 1.3: `ANO_DT.c` 的 PID 读操作改用 `pid_dump()` + `pid_get_config()`

**Files:**
- Modify: `user/abstract/ANO_DT.c:138-266, 932-986`

**要点:**
1. 搜索 `ANO_DT.c` 中所有 `pid_xxx.Kp / pid_xxx.Ki / pid_xxx.Kd` 的读引用 → 改用 `pid_get_config(&pid_xxx, &cfg); cfg.Kp`
2. 搜索所有 `pid_xxx.Output / pid_xxx.Err / pid_xxx.Pout / pid_xxx.Iout / pid_xxx.Dout / pid_xxx.Measure` 读引用 → 改用 `pid_dump(&pid_xxx, &dump); dump.Output`
3. `ANO_DT.c:242` 的 `taskENTER_CRITICAL()` 包裹块 → 删掉临界区（dump 是值拷贝，不需要锁）
4. `ANO_DT.c:250` `pid_roll_angle.Output / pid_roll_rate.Measure` 等 → 改用 dump
5. `ANO_DT.c:409-430` 的 PID 写操作（地面站调参 `pid_xxx.Kp = ...`）→ 暂时保留不动（写操作在 Task 1.4 处理）

**编译验证点:** 编译通过，所有 `pid_xxx.` 读引用改为通过接口。确认 `pid_xxx.Kp =` 的写操作仍在但编译无误。

**上机实测点:** 烧录后连接地面站，确认 ANO_DT 上位机波形显示的 PID 值（Roll/Pitch/Yaw 的 P/I/D/Output）和之前一致。

---

### Task 1.4: `ANO_DT.c` 的 PID 写操作改用 `pid_set_config()`

**Files:**
- Modify: `user/abstract/ANO_DT.c:409-435`

**要点:**
1. `ANO_DT_Data_Receive_Anl()` 中 `pid_x_velocity.Kp = 0.001 * ...` 这一段 → 改为：
   - `PID_Init_Config_s cfg;`
   - `pid_get_config(&pid_x_velocity, &cfg);`
   - 修改 `cfg.Kp = ...;`
   - `pid_set_config(&pid_x_velocity, &cfg);`
2. 同理处理 `pid_y_velocity` / `pid_z_velocity` 等其他 PID 组

**编译验证点:** 编译通过，确认没有残留 `pid_xxx.Kp =` 的直接赋值。

**上机实测点:** 烧录后在地面站上调一个 PID 值（比如 Roll Rate Kp 加 0.01），确认写入生效（看波形变化）、掉电重连后确认值丢失（这是预期行为，持久化是 Phase 3 的事）。

---

### Task 1.5: 清理 `ANO_DT.c` 中对 `stcAcc/stcGyro/stcMag` 的直接引用

**Files:**
- Modify: `user/abstract/ANO_DT.c:124-155`

**要点:**
1. `ANO_DT.c:124` `int16_t acc_x = stcAcc.a[0];` → 改用 `gyro_getAcc()` 读 float
2. `ANO_DT.c:127` `int16_t gyro_x = stcGyro.w[0];` → 改用 `gyro_getAngularVelocity()` 读 float
3. `ANO_DT.c:130` `int16_t mag_x = stcMag.h[0];` → 暂时保留（gyro.h 没暴露 mag 接口）
4. `ANO_DT_Send_Senser` 的参数类型可能需要 float → int16 转换，加显式 cast

**编译验证点:** 编译通过，删掉 `ANO_DT.c` 中对 `stcAcc.a[] / stcGyro.w[]` 的直接引用（`stcMag` 暂时保留）。

**上机实测点:** 烧录后确认地面站传感器波形（加速度/陀螺仪）显示正常。

---

### Task 1.6: Phase 1 收尾 —— 更新 `control_pid.h` 注释 & Keil 工程

**Files:**
- Modify: `user/control/control_pid.h:1-28`（文件头注释）
- Modify: `user/control/PIDcontroller.h:38-82`（在 `PIDInstance` 定义上方加注释）
- Modify: `MDK-ARM/project.uvprojx`（如有新增 `.c/.h` 需要加入分组；Phase 1 不新增文件则跳过）

**要点:**
1. `PIDcontroller.h` 的 `PIDInstance` 定义上方加注释：`/* ⚠️ 外部代码禁止直接读写 PIDInstance 字段，请使用 pid_dump/pid_get_config/pid_set_config */`
2. `control_pid.h` 文件头注释新增一句说明 PID 访问规范

**编译验证点:** 全量编译 0 error 0 warning。

**上机实测点:** 完整飞行动作：上电 → 起飞悬停 5s → yaw ±90° → 一键降落。确认无异常。

---

## Phase 2: commander setter 副作用分离（P0 #1）

> **改动量:** ~80 行新增 + ~30 行改动
> **风险:** 中（改 setter 行为，逻辑路径变化）
> **前提:** Phase 1 全部通过

### Task 2.1: commander 边沿标志 + setter 净化

**Files:**
- Modify: `user/services/commander.h:18-27`（`commanderBits_t` 扩展）
- Modify: `user/services/commander.c:46-54`（静态状态变量扩展）
- Modify: `user/services/commander.c:581-610`（`setCommanderKeyFlight` 拆分）
- Modify: `user/services/commander.c:621-628`（`setCommanderKeyland` 拆分）

**要点:**
1. `commanderBits_t` 新增 2 个 bit（或不改位域、改用独立 bool）：`keyFlightRising` / `keyFlightFalling` / `keyLandRising` / `keyLandFalling`
2. `setCommanderKeyFlight(bool set)`:
   - 去掉所有副作用（`ResetFlightControlPIDs` / `position_ResetXY` / `minAccZ` 清零等）
   - 只记录：`keyFlight` 值 + `keyFlightRising`（0→1）/ `keyFlightFalling`（1→0）
3. `setCommanderKeyland(bool set)` 同理
4. 新增 getter：`bool consumeKeyFlightRising(void);` → 读取 rising 标志并清零
5. 新增 getter：`bool consumeKeyFlightFalling(void);`

**编译验证点:** 编译通过。此时 `commander.c` 不再 include `control.h` 和 `PIDcontroller.h`（删除这两条 include）。

---

### Task 2.2: 副作用移到 `Control_Task` 入口

**Files:**
- Modify: `user/control/control.c:139-171`（`Control_Task` 主循环）
- Modify: `user/control/control.c:96-116`（`ResetFlightControlPIDs` 不变）
- Modify: `user/control/control.c:195-254`（`Flight_Update` 入口改动）

**要点:**
1. 在 `Control_Task` 主循环顶部（`refreshState` 之前），添加：
   ```c
   if (consumeKeyFlightRising()) {
       ResetFlightControlPIDs();
       position_ResetXY();
       /* minAccZ/maxAccZ/maxAccZOverCnt/maxAccZWinCnt 清零延后到 commanderGetSetpoint 内部 */
   }
   if (consumeKeyLandRising()) {
       ResetFlightControlPIDs();
   }
   ```
2. `Flight_Update` 中原来 `getCommanderKeyFlight() || getCommanderKeyland()` 的判断不变（飞行/着陆行为逻辑不变）
3. `commander.c` 中原来 `setCommanderKeyFlight` 的副作用行（第 592-603 行 和 606-608 行）→ 删除

**编译验证点:** 编译通过。确认 `commander.c` 不再有 `#include "control.h"`。

**上机实测点:** 烧录后按以下序列验证：
1. 上电 → 一键起飞 → 悬停 3s → 一键降落 → 确认平稳降落
2. 上电 → 一键起飞 → 悬停 3s → 一键降落 → 再一键起飞 → 确认 PID 复位生效（无积分残留导致起飞不稳）

---

### Task 2.3: Phase 2 收尾 —— 确认循环依赖解除

**Files:**
- Modify: `user/services/commander.c:8-12`（删除多余 include）
- Modify: `user/control/control.c:8-9`（`control.c` 的 include 不变，仍需要 commander.h 读 getter）

**要点:**
1. `commander.c` 删掉 `#include "control.h"` 和 `#include "PIDcontroller.h"`（如果 Phase 1 已删则跳过）
2. 确认 `commander.c` 不再 `#include "alarm.h"`（如果还有 → 独立一个事件通知 task）
3. 确认 `control.c` → `commander.h` 的单向依赖成立（不再反向）

**编译验证点:** 全量编译 0 error 0 warning。

**上机实测点:** 完整飞行 + 失控保护测试：
1. 上电 → 起飞 → 悬停 5s → 关机遥控器 → 确认自动降落（commander arbitration 超时逻辑不受影响）
2. 上电 → 起飞 → 故意倾斜 >35° → 确认安全锁定（control_safety 调用 setCommanderSafetyLatched 路径不受影响）

---

## Phase 3: commander 拆子模块（P1）

> **改动量:** ~300 行新增 + commander.c 从 660 行缩到 ~150 行
> **风险:** 高（拆文件，Keil 工程需同步）
> **前提:** Phase 1 + Phase 2 全部通过

### Task 3.1: 抽出 `commander_persist.c`（EEPROM 读写）

**Files:**
- Create: `user/services/commander_persist.c`
- Create: `user/services/commander_persist.h`
- Modify: `user/services/commander.c:57-107`（删除，移入新文件）

**要点:** 四个函数 `CommanderPersist_*` 完整移走，接口不变。Keil 工程 services 分组新增 `commander_persist.c`。

---

### Task 3.2: 抽出 `commander_arbitration.c`（时序仲裁 + 自动降落）

**Files:**
- Create: `user/services/commander_arbitration.c`
- Create: `user/services/commander_arbitration.h`
- Modify: `user/services/commander.c:122-203`（`ctrlDataUpdate` + `flyerAutoLand` 移走）

**要点:** `ctrlDataUpdate()` / `flyerAutoLand()` 完整移走，接口不变。

---

### Task 3.3: 抽出 `commander_takeoff_land.c`（起飞斜坡 + 降落逻辑）

**Files:**
- Create: `user/services/commander_takeoff_land.c`
- Create: `user/services/commander_takeoff_land.h`
- Modify: `user/services/commander.c:209-223, 280-448`（起飞/降落状态机移走）

**要点:** 起飞斜坡 + 降落逻辑 + 高度保持相关静态变量移走。

---

### Task 3.4: 抽出 `commander_out_of_control.c`（失控保护）

**Files:**
- Create: `user/services/commander_out_of_control.c`
- Create: `user/services/commander_out_of_control.h`
- Modify: `user/services/commander.c:349-428`（maxAccZ 失控检测移走）

**要点:** 失控保护检测逻辑独立，阈值集中管理。

---

### Task 3.5: Phase 3 收尾 —— commander.c 缩到 ~150 行

**Files:**
- Modify: `user/services/commander.c`（只剩 setter/getter + `commanderGetSetpoint` 入口）
- Modify: `user/services/commander.h`（只暴露公开接口）
- Modify: `MDK-ARM/project.uvprojx`（确认 services 分组包含所有新增文件）

**要点:**
1. `commander.c` 缩至：setter/getter 函数 + `commanderGetSetpoint()` + 子模块调用
2. Keil 工程分组 services 新增 4 个 `.c`

**编译验证点:** 全量编译 0 error 0 warning。

**上机实测点:** 同 Phase 2.3 的实测序列，额外加 EEPROM 持久化验证（断电重连后姿态模式恢复）。

---

## 执行顺序总览

```
Phase 1 (Task 1.1 → 1.6)  ──编译+实测──→  Phase 2 (Task 2.1 → 2.3)  ──编译+实测──→  Phase 3 (Task 3.1 → 3.5)
                                                                                      │
                                                                                      └── 编译+实测
```

每步之间需要你确认飞行正常才能进下一步。Phase 3 是可选的——如果 Phase 1+2 完成后的 commander.c 可读性已经够好，可以不做 Phase 3。
