# 架构已知问题清单

## 文档定位

本文档归档当前工程中**已经摸清、但尚未解决**的架构耦合与代码异味问题，目的是为后续重构阶段提供一份稳定的优先级参考。

- 本文不是执行清单，只描述问题、影响和解决方向。
- 实际执行步骤仍记录在 `docs/architecture/refactor-notes.md`。
- 长期目标仍参考 `docs/architecture/project-refactor-roadmap.md`。
- 当前已落地架构参考 `docs/architecture/current-architecture.md`。

每条问题至少包含：

- **影响**：出错的可能性或重构阻力。
- **证据**：定位到具体文件与行号，便于复核。
- **建议方向**：给出大致解决方案，不细化到函数签名。
- **优先级**：P0=飞行安全 / P1=重大可维护性 / P2=影响范围有限 / P3=收尾型。

## 飞控端已知问题

### F-1. `commanderGetSetpoint` 是当前最大的长函数

- **影响**：单函数混合多种职责，新增模式或修改安全互锁时容易引入回归；单元测试与代码 review 成本高。
- **证据**：`user/services/commander.c:223-490`，函数体约 268 行。
- **建议方向**：按职责拆分为 `commander_pull_remote_data` / `commander_apply_mode_filter` / `commander_apply_safety` / `commander_apply_limits`，主函数只做调度。
- **优先级**：P1（重构阻力最大、可读性最差）。

### F-2. `ButtonCommand` 是命令分发器

- **影响**：长 if/else if 分发链，新增命令需要修改分发代码。
- **证据**：`user/services/remotedata.c:191-317`，约 127 行。
- **建议方向**：改为 `command_handler_t` 函数指针表 + `CmdId` 枚举。
- **优先级**：P2。

### F-3. `control/change.c` 存在舵机动作复制粘贴

- **影响**：动作修改时需要同步改多份近似函数，容易遗漏。
- **证据**：`user/control/change.c` 五个近似函数 `setServoSlow_1_Front`(55-93)、`setServoSlow_1_Back`(94-132)、`setServoSlow_2_Front`(133-175, static)、`setServoSlow_2_Back`(176-218)、`setServoSlow_2`(219-280)。
- **建议方向**：用动作表 + 单一执行器替代五个近似函数。
- **优先级**：P2。

### F-4. `services/alarm.c` 状态变量裸全局

- **影响**：电池、告警标志全局可变，无访问控制；`alarm.h` 直接 include `gpio.h` 把硬件头带入 services。
- **证据**：`user/services/alarm.c:15-17` 定义 `battery_voltage / battery_current / alarm_flag`；`user/services/alarm.h:10` include `gpio.h`。
- **建议方向**：状态字段改 static + 提供访问函数；移除 `alarm.h` 的 `gpio.h` include。
- **优先级**：P2。

### F-5. `setpoint_t target` 实例位置错放

- **影响**：控制目标实例定义在 `control.c`，又被 `commander.h` extern，归属混乱；控制目标本应归 services/commander 拥有。
- **证据**：定义 `user/control/control.c:43`；extern `user/services/commander.h:28`。
- **建议方向**：实例迁移到 `services/commander.c`，control 通过 `commander_get_setpoint(&target)` 读取。
- **优先级**：P2。

### F-6. module 层 HAL 头直接出现在 module 头文件中

- **影响**：任何 `#include "module/xxx.h"` 的上层代码会被强制拉入对应 HAL 头，module 没有真正抽象掉硬件。
- **证据**：
  - `module/gyro/jy901p.c:3` 直接 include `stm32f4xx_hal.h`
  - `module/motor/esc.h:4` include `tim.h`
  - `module/motor/servo.h:3` include `tim.h`
  - `module/motor/motor.h:4-5` include `tim.h` `gpio.h`
  - `module/bn220/bn220.h:4` include `usart.h`
  - `module/bmp280/bmp280.h:4` include `spi.h`
  - `module/mtf01/mtf_01.h:4` include `usart.h`
  - `module/at24c02/AT24Cxx.h:7,19` include `main.h` `i2c.h` `software_i2c.h`
- **建议方向**：HAL 类型移入 `.c`；HAL 句柄改 `void*` 不透明指针；module 头文件只暴露函数接口。
- **优先级**：P1（封死越层访问的根源）。

### F-7. `jy901p.h` 五个传感器状态裸全局

- **影响**：SAcc / SGyro / SAngle / SMag / SQ 五个结构体全局 extern，ANO_DT 等上层模块直接读写底层传感器字段，绕过 `gyro.h` 抽象。
- **证据**：`user/module/gyro/jy901p.h:51-55`。
- **建议方向**：仅 `jy901p.c` 持有这些结构体，`gyro.h` 只暴露读取接口。
- **优先级**：P1（与 F-6 共同封堵 ANO_DT 越层访问）。

### F-8. `commander.c` 跨层 include 多个领域

- **影响**：services 层既调 control 算法、又调 PID 实现、又访问 position 域、又直接读写 EEPROM 硬件，职责严重混合。
- **证据**：`user/services/commander.c:8-11` include `control.h` `PIDcontroller.h` `position.h` `AT24Cxx.h`。
- **建议方向**：services 只通过 domain 接口交互；持久化逻辑独立封装。
- **优先级**：P1。

### F-9. 飞控端 `abstract/ANO_DT.c` 越层依赖

- **影响**：地面站协议解析器跨层读写 12 个 PID 全部字段 + `state.*` + `stcAcc/Gyro/Mag/Angle` 传感器字段 + `RC_Control` 遥控字段 + 5 个 `debug_*` 调试字段。串口误数据可在飞行中直接修改 PID，无任何校验、互斥或回退保护。
- **证据**：`user/abstract/ANO_DT.c` 总计 1043 行；PID 裸赋值见 407-468 行（命令码 0x10-0x15）。
- **建议方向**：协议解析器剥离 control/commander/gyro 依赖；PID 写入走带范围 clamp 和互斥锁的 `PID_Tune()` 接口。
- **优先级**：P0（飞行安全相关，PID 全局变量当前虽保留 extern，但写入路径必须受保护）。

## 遥控器端已知问题

### R-1. `abstract/rtos_init.c` 是上帝初始化函数

- **影响**：单个 RTOS 初始化任务同时完成日志、定时器、串口、按键、配置、摇杆、遥控、LCD 与 5 个 `xTaskCreate`；飞控端已拆分为 `App_InitSystemModules` 的三个子函数，遥控器端没有对应拆分，结构不一致。
- **证据**：`projectRemote/user/abstract/rtos_init.c:24-60` 的 `RTOS_Init()`。
- **建议方向**：参照飞控端 `app_system.c`，拆分为全局服务、设备、业务模块三个初始化函数 + 独立的任务创建函数。
- **优先级**：P1。

### R-2. `abstract/wireless.h` 反向 include module

- **影响**：抽象层头文件直接 include `nRF24L01P.h`，把 module 层泄漏给所有包含 wireless.h 的调用方。
- **证据**：`projectRemote/user/abstract/wireless.h:4` include `nRF24L01P.h`。
- **建议方向**：新建 module 适配层；`wireless.h` 仅暴露 `Wireless_Send / Wireless_Receive` 接口。
- **优先级**：P2。

### R-3. `communicate/joystick.h` 反向 include abstract

- **影响**：通信层头文件通过 `config_param.h` 拿到 calibration 数据，间接依赖抽象层。
- **证据**：`projectRemote/user/communicate/joystick.h` 透传 `config_param.h` 依赖。
- **建议方向**：把校准相关类型下沉到 domain 或独立类型头，joystick 层只依赖类型。
- **优先级**：P2。

### R-4. `menu/main_ui.c` 权限过大

- **影响**：菜单层直接 include `remotestate.h` `joystick.h` `config_param.h`，能直接发命令、读状态、读配置。
- **证据**：`projectRemote/user/menu/main_ui.c:7,9,12`。
- **建议方向**：菜单层通过接口访问，配置读写走专门的 `config_service`。
- **优先级**：P2。

### R-5. `module/LCD/lcd.h` 把 HAL SPI 句柄全局暴露

- **影响**：module 头文件暴露 `extern SPI_HandleTypeDef LCD_HANDLE`，HAL 类型泄漏到所有 LCD 头用户。
- **证据**：`projectRemote/user/module/LCD/lcd.h:237`。
- **建议方向**：HAL 句柄移到 `.c`；module 头仅暴露 LCD 操作函数。
- **优先级**：P2。

### R-6. `communicate/remotestate.c` 反向 include 菜单

- **影响**：通信层 include `main_ui.h` `display.h`，与菜单层形成双向耦合。
- **证据**：`projectRemote/user/communicate/remotestate.c:17-18`。
- **建议方向**：移除反向 include；UI 状态通过订阅模式或事件队列下发。
- **优先级**：P2。

## 结构性观察

### S-1. 飞控与遥控器 abstract 目录语义不一致

- **影响**：两边都叫 `abstract/`，但飞控端承载通用抽象（ANO_DT、gyro、position），遥控器端承载具体驱动（button、config_param、wireless、rtos_init）。新人按目录名推测行为会出错。
- **证据**：对比 `project/user/abstract/` 与 `projectRemote/user/abstract/` 内容。
- **建议方向**：在两个项目的 `AGENTS.md` 或 `docs/architecture/architecture.md` 中显式声明两边的 abstract 语义差异；遥控器端可考虑将驱动类目录改名为 `drivers/`。
- **优先级**：P3。

### S-2. `project/user/communicate/` 是空目录

- **影响**：refactor-notes 显示 commander/remotedata 已迁到 services，但空壳目录未清理。
- **证据**：`ls project/user/communicate/` 输出为空。
- **建议方向**：删除空目录或在 Keil 工程中确认已移除分组引用。
- **优先级**：P3。

### S-3. 备份文件未清理

- **影响**：`ANO_DT.c.backup` / `ANO_DT.h.backup` 与正式文件同目录，干扰阅读。
- **证据**：`project/user/abstract/ANO_DT.{c,h}.backup`。
- **建议方向**：删除备份文件；如需历史版本依赖 Git。
- **优先级**：P3。

### S-4. 遥控器端缺失架构文档体系

- **影响**：飞控端已有 `docs/architecture/` 三件套 + `docs/learning/` 六篇指南，遥控器端没有 `docs/` 目录也没有 `AGENTS.md`，新人接手遥控器代码缺少设计地图。
- **证据**：`ls projectRemote/` 与 `ls projectRemote/docs/`（后者报错不存在）。
- **建议方向**：在 `projectRemote/docs/architecture/` 与 `projectRemote/docs/learning/` 下补齐与飞控端对应的文档骨架（先目录与职责表，细节后补）。
- **优先级**：P3。


## 已通过重构解决（保留为历史）

### F-10. yaw 闭环依赖 JY901P 9 轴融合的 `state.angle.yaw`

- **历史问题**：起飞时机体顺时针自旋但 `state.angle.yaw` 变正；停机手动顺时针旋转时 `state.angle.yaw` 与 `gyro.z` 变负，符号与机体一致。说明闭环测量源被污染。
- **影响**：起飞瞬态 yaw 控制器与被磁干扰/模块偏置污染的 `state.angle.yaw` 形成正反馈；磁力计配置（`ALGRITHM9=0`，即 9 轴）使磁干扰作为航向变化进入融合。
- **证据**：`user/control/control.c:392`（原）`PIDCalculate(&pid_yaw_angle, yaw_meas_cont, Desired_yaw)` 中 `yaw_meas_cont = state->angle.yaw + yawTurnNum * 360.0f`；`user/abstract/gyro.c` 上电初始化未写 `AXIS6` 寄存器，模块按 9 轴默认运行。
- **解决方式（2026-07-14 第四十一阶段）**：
  - `gyro.c` 新增 `gyro_calibrateGyroZOffset()`：上电后 1.0 秒采 200 帧，三轴方差 < 4.0 (°/s)² 才接受零偏（MiniFly 思路）。
  - `gyro_getAngularVelocity()` 扣 `gyro_z_offset`；`gyro_isGyroZCalibrated()` 标志位。
  - `control.c` `Yaw_Control` 入口加 `gyro_isGyroZCalibrated() == 0` 早返；角度环测量改用飞控端自积分的 `yaw_meas_cont = state->gyro.z * ANGEL_PID_DT`。
  - 移除 `isAdjustingYaw` 状态机、移除 ±180° 展开状态，改 MiniFly 隐式锁角。
- **状态**：已解决（参考 `refactor-notes.md` 第四十一阶段、`current-architecture.md` 2026-07-14 条目）。
- **残留风险**：陀螺零偏校准不通过时（`LOG_WARN "gyro z cali FAIL"`）yaw 会缓慢漂；本阶段未拆 `esc.c` 的 95% 输出钳位；未加加速度/陀螺二阶 LPF。

## 优先级汇总

| 优先级 | 问题 | 数量 |
|---|---|---|
| P0 | F-9 | 1 |
| P1 | F-1 / F-6 / F-7 / F-8 / R-1 | 5 |
| P2 | F-2 / F-3 / F-4 / F-5 / R-2 / R-3 / R-4 / R-5 / R-6 | 9 |
| P3 | S-1 / S-2 / S-3 / S-4 | 4 |

## 与现有文档的关系

- `current-architecture.md`：记录**已落地**架构与变更历史。
- `project-refactor-roadmap.md`：记录**长期目标**与推荐阶段顺序。
- `refactor-notes.md`：记录**已完成重构阶段**的具体改动与提交。
- **`known-issues.md`（本文档）**：记录**已摸清但未解决**的架构问题与建议方向。

新执行的重构阶段完成后，应同时更新：

1. `refactor-notes.md`：追加新阶段的提交与改动。
2. `current-architecture.md`：在"架构变更记录"追加新条目。
3. **本文档**：从问题清单中移除已解决问题，并保留问题变更历史。