# Yaw 控制对齐 MiniFly 实施方案

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 robodrone yaw 控制链路改造为与 MiniFly 一致的行为：MPU/JY901P 融合 yaw 作为测量 → 双环 PID → 松杆锁航向 → RC 失联 yaw 中性 → 定点减半。

**Architecture:** 
1. yaw 测量源从 `∫gyro.z·dt` 自积分切换到 JY901P 融合的 `state->angle.yaw`（已在 ±180° 范围内），并在 PIDCalculate 调用前对 yaw 误差做 ±180° 包裹。
2. `Desired_yaw` 改为 ±180° 包裹的连续积分角（MiniFly 方式）。
3. 松杆后 `Desired_yaw` 不变，角度环自然维持航向。
4. RC 锁定时（≥500ms）强制 yaw/roll/pitch 回中；commanderDropToGround 改用 setter 产生边沿触发 PID reset。

**Tech Stack:** C99, FreeRTOS, STM32F407, JY901P IMU, Keil AC6

---

### Task 1: 修改 yaw 测量源 — `Yaw_Control` 改用 `state->angle.yaw`

**Files:**
- Modify: `project/user/control/control.c:365-426`

- [ ] **Step 1: 替换 yaw 自积分为 JY901P 融合 yaw，并处理 ±180° 误差包裹**

将 `Yaw_Control` 角度环部分从自积分改为使用 `state->angle.yaw`，同时修正所有相关注释。

当前代码（`control.c:365-415`）：
```c
void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick)
{
    /* 上电 1.0s 陀螺零偏校准未通过: yaw 控制器直接不工作, 不产生任何差分.
       与 MiniFly sensorsAreCalibrated() 等价的策略: 未校准不进入控制. */
    if (gyro_isGyroZCalibrated() == 0)
    {
        return;
    }

    if (RATE_DO_EXECUTE(ANGEL_PID_RATE,tick))
    {
        float raw_stick = -target->angle.yaw;

        /* 飞控端用 state->gyro.z 自积分出连续 yaw 角度 (度).
           不再用 JY901P 9 轴融合的 state.angle.yaw, 完全摆脱磁干扰与模块偏置. */
        float yaw_step = state->gyro.z * ANGEL_PID_DT;
        if (fabsf(yaw_step) < YAW_INTEG_MAX_PER_STEP)
        {
            /* 杆回中时 |gyro.z|<0.5°/s 是电机振动导致的零偏偏移, 不积分.
               杆偏时正常积分 (MiniFly 用 Mahony accel 修正同理, 我们没这层). */
            if (fabsf(raw_stick) > YAW_DEADBAND || fabsf(state->gyro.z) > 0.5f)
            {
                yaw_meas_cont += yaw_step;
            }
        }
        debug_yaw_meas_cont = yaw_meas_cont;

        /* LPF 平滑杆量 (与 MiniFly commander.c:117 ctrlValLpf.yaw 思路一致).
           alpha=0.1 @250Hz → τ≈38ms, MiniFly α=0.2 @100Hz → τ≈45ms, 接近. */
        lpf_stick_yaw += (raw_stick - lpf_stick_yaw) * 0.1f;

        if (fabsf(lpf_stick_yaw) > YAW_DEADBAND)
        {
            Desired_yaw += (lpf_stick_yaw / 100.0f) * YAW_MAX_RATE * ANGEL_PID_DT;
        }
        else
        {
            /* 松手锁角 (MiniFly Kp=20.0 不需要, 我们 Kp=2.5 拉不回惯性过冲).
               LPF 衰减进死区后 Desired_yaw = yaw_meas_cont, 角度误差归 0. */
            Desired_yaw = yaw_meas_cont;
        }

        // 角度环
        PIDCalculate(&pid_yaw_angle, yaw_meas_cont, Desired_yaw);
        debug_target_angle_yaw = wrapYawDisplay(Desired_yaw);
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        // 角速度环
        PIDCalculate(&pid_yaw_rate, state->gyro.z, pid_yaw_angle.Output);
    }
}
```

替换为：
```c
void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick)
{
    /* 上电 1.0s 陀螺零偏校准未通过: yaw 控制器不工作.
       与 MiniFly sensorsAreCalibrated() 等价: 未校准不进入控制. */
    if (gyro_isGyroZCalibrated() == 0)
    {
        return;
    }

    if (RATE_DO_EXECUTE(ANGEL_PID_RATE,tick))
    {
        float raw_stick = -target->angle.yaw;

        /* LPF 平滑杆量 (与 MiniFly commander.c:117 ctrlValLpf.yaw 一致).
           α=0.2 @250Hz → τ≈20ms. MiniFly α=0.2 @100Hz → τ≈50ms. */
        lpf_stick_yaw += (raw_stick - lpf_stick_yaw) * 0.2f;

        if (fabsf(lpf_stick_yaw) > YAW_DEADBAND)
        {
            /* 杆偏出死区: 积分 Desired_yaw, 并 ±180° 包裹 (MiniFly 方式).
               lpf_stick_yaw ∈ [-200, 200], /100→[-2,2], ×YAW_MAX_RATE=42→[-84,84] deg/s. */
            Desired_yaw += (lpf_stick_yaw / 100.0f) * YAW_MAX_RATE * ANGEL_PID_DT;
            if (Desired_yaw > 180.0f)
                Desired_yaw -= 360.0f;
            else if (Desired_yaw < -180.0f)
                Desired_yaw += 360.0f;
        }
        /* else: 杆在死区内 → Desired_yaw 不变, 角度环自然维持当前航向 (MiniFly 隐式锁角) */

        /* ── 角度环: 测量 = JY901P 融合 yaw, 参考 = Desired_yaw ──
           处理 ±180° 误差包裹, 使 PID 误差始终取最短路径.
           做法: 把 ref 调整到 "measure + wrappedError", PIDCalculate 内部 Err = ref - measure = wrappedError. */
        float yaw_measure = state->angle.yaw;
        float yaw_error   = Desired_yaw - yaw_measure;
        if (yaw_error > 180.0f)
            yaw_error -= 360.0f;
        else if (yaw_error < -180.0f)
            yaw_error += 360.0f;
        float yaw_adjusted_ref = yaw_measure + yaw_error;

        PIDCalculate(&pid_yaw_angle, yaw_measure, yaw_adjusted_ref);

        /* 遥测快照 */
        debug_yaw_meas_cont = yaw_measure;
        debug_target_angle_yaw = Desired_yaw;
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        /* 角速度环: 测量 = gyro.z (已扣零偏), 参考 = 角度环输出 */
        PIDCalculate(&pid_yaw_rate, state->gyro.z, pid_yaw_angle.Output);
    }
}
```

同步更新文件顶部变量声明与注释（`control.c:24-46`）：
```c
#define YAW_MAX_RATE	42.0f		/* 满杆 yaw 角速率 deg/s (配合 lpf_stick_yaw/100) */
#define YAW_DEADBAND	5.0f		/* yaw 杆死区, 原始杆量单位 (满杆 ≈200) */

static float Desired_yaw = 0.0f;          /* 期望 yaw 角度, ±180° 包裹 (MiniFly 方式) */
/* 飞控 yaw 测量: 改用 JY901P 融合的 state->angle.yaw (±180°), 不再自积分 gyro.z.
   yaw_meas_cont 仅作为遥测快照, 不参与闭环. */
float yaw_meas_cont = 0.0f;
/* yaw 杆 LPF (与 MiniFly ctrlValLpf.yaw 相同思路) */
static float lpf_stick_yaw = 0.0f;
```

删除不再需要的宏 `YAW_INTEG_MAX_PER_STEP`（`control.c:35`）：
```c
/* 删除此行 */
#define YAW_INTEG_MAX_PER_STEP   5.0f
```

- [ ] **Step 2: 编译验证**

在 Keil/EIDE 中编译 `project`，确认无错误。

- [ ] **Step 3: 提交**

```bash
git -C project add user/control/control.c
git -C project commit -m "重构 yaw 测量源: 改用 JY901P 融合 yaw, ±180° 包裹, 松杆隐式锁角"
```

---

### Task 2: 调整 `ResetYawState` — 起飞锁存当前 JY901P yaw + 清零 LPF

**Files:**
- Modify: `project/user/control/control.c:65-76`

- [ ] **Step 1: 将初始 Desired_yaw 改为锁存当前融合 yaw，并清零 lpf_stick_yaw**

当前：
```c
static void ResetYawState(void)
{
    /* 起飞/降落瞬间把 yaw_meas_cont 重新定义成 0.
       理由: 上电后累积的角度反映"机上电后绕了多少",但用户可能在两次飞行之间手动
       移动飞机, 实际航向已经变了. 起飞瞬间归零 → 新航向以起飞时为准.
       代价: 两次起飞之间 yaw_meas_cont 不连续, 但每次起飞都从 0 开始安全. */
    Desired_yaw = 0.0f;
    yaw_meas_cont = 0.0f;
    debug_target_angle_yaw = wrapYawDisplay(Desired_yaw);
    PID_ClearIntegral(&pid_yaw_angle);
    PID_ClearIntegral(&pid_yaw_rate);
}
```

替换为：
```c
static void ResetYawState(void)
{
    /* 起飞瞬间锁存当前 JY901P 融合 yaw 作为目标航向 (MiniFly 方式).
       松杆后角度环自然维持该航向; 不再归零, 避免磁干扰初值污染. */
    Desired_yaw = state.angle.yaw;
    yaw_meas_cont = state.angle.yaw;
    lpf_stick_yaw = 0.0f;
    debug_target_angle_yaw = Desired_yaw;
    PID_ClearIntegral(&pid_yaw_angle);
    PID_ClearIntegral(&pid_yaw_rate);
}
```

- [ ] **Step 2: 编译验证**

- [ ] **Step 3: 提交**

```bash
git -C project add user/control/control.c
git -C project commit -m "fix: 起飞锁存 JY901P 融合 yaw 为初始航向, 并清零 yaw LPF"
```

---

### Task 3: RC 锁定后 yaw/roll/pitch 回中

**Files:**
- Modify: `project/user/services/commander.c:234-275, 113-120`

- [ ] **Step 1: `commanderGetSetpoint` 在 RC 锁定时强制杆量回中**

在 `commanderGetSetpoint` 中，`state->isRCLocked` 赋值之后、setpoint 赋值之前，加入锁定回中逻辑。

当前 `commander.c:260-261`：
```c
	state->isRCLocked = (isRCLocked || safetyLatched);

	/* 消费边沿：commander 内部状态复位（PID/position 复位在 Control_Task） */
```

替换为：
```c
	state->isRCLocked = (isRCLocked || safetyLatched);

	/* RC 锁定后强制杆量回中, 避免继续沿用最后杆量旋转.
	   data 是局部拷贝, 修改它即可, 不影响 RC_Control 全局缓存. */
	if (state->isRCLocked)
	{
		data.angle.roll  = 0.f;
		data.angle.pitch = 0.f;
		data.angle.yaw   = 0.f;
		/* throttle 保留 last_data, 供定高/定点模式继续悬停 */
	}

	/* 消费边沿：commander 内部状态复位（PID/position 复位在 Control_Task） */
```

- [ ] **Step 2: `commanderDropToGround` 改用 setter 以产生边沿**

`commanderDropToGround`（`commander.c:113-120`）当前直接写 `commander.keyLand/keyFlight`，不产生边沿，导致 `Control_Task` 收不到 `keyLandRising` 来做 PID reset。

当前：
```c
static void commanderDropToGround(void)
{
	if(commander.keyFlight)	/* 飞行过程中遥控器信号断开，一键降落 */
	{
		commander.keyLand = true;
		commander.keyFlight = false;
	}	
}
```

替换为：
```c
static void commanderDropToGround(void)
{
	if(commander.keyFlight)	/* 飞行过程中遥控器信号断开，一键降落 */
	{
		setCommanderKeyland(true);
		setCommanderKeyFlight(false);
	}	
}
```

同样修正 `commander.c:257-258`（safetyLatched 分支）和 `commander.c:400-401`（失控保护触发），这些已经直接操作 `commander.keyFlight/keyLand` 但 safetyLatched 分支通过 `setCommanderSafetyLatched` 已经内部处理了。

检查 `commander.c:257-258`：
```c
	if (safetyLatched)
	{
		commander.keyFlight = false;
		commander.keyLand = false;
	}
```

应改为使用 setter（因为 setter 中有 safetyLatched 守卫，这里需要绕过):
```c
	if (safetyLatched)
	{
		/* safetyLatched 已在 setCommanderSafetyLatched 中处理过 keyFlight/keyLand,
		   此处仅是 commanderGetSetpoint 内的局部防护, 直接写即可.
		   (若用 setter, safetyLatched==true 时 setCommanderKeyFlight(true) 会被拒绝,
		   但这里只是 keyFlight=false, 允许) */
		commander.keyFlight = false;
		commander.keyLand = false;
	}
```

不需要改 `commander.c:400-401`（失控保护），因为它紧跟着 `safetyLatched` 已在外部处理。

- [ ] **Step 3: 编译验证**

- [ ] **Step 4: 提交**

```bash
git -C project add user/services/commander.c
git -C project commit -m "fix: RC 锁定后杆量强制回中, commanderDropToGround 改用 setter 产生边沿"
```

---

### Task 4: 定点模式 yaw 减半

**Files:**
- Modify: `project/user/services/commander.c:495-497`

- [ ] **Step 1: 将 ×1.0 改为 ×0.5**

当前：
```c
		if(commander.ctrlMode == MODE_THREEHOLD && commander.attitudeMode == MODE_AIRPLANE)	/* 光流数据可用，定点模式 */
		{
			setpoint->angle.yaw *= 1.0f;	/* 定点模式使用完整yaw杆量 */
```

改为：
```c
		if(commander.ctrlMode == MODE_THREEHOLD && commander.attitudeMode == MODE_AIRPLANE)	/* 光流数据可用，定点模式 */
		{
			setpoint->angle.yaw *= 0.5f;	/* 定点模式 yaw 减半 (MiniFly 方式) */
```

- [ ] **Step 2: 编译验证**

- [ ] **Step 3: 提交**

```bash
git -C project add user/services/commander.c
git -C project commit -m "feat: 定点模式 yaw 减半, 对齐 MiniFly"
```

---

### Task 5: 更新 PID 角度环注释 — Kp 实际值修正

**Files:**
- Modify: `project/user/control/control_pid.c:130-143`

- [ ] **Step 1: 确认并修正注释中的 Kp 值**

当前 `control_pid.c:132`：
```c
.Kp = 3.0f,
```

这已经正确。上一轮报告指出的 "注释写 Kp=2.5 但实际 3.0" 是在 `control.c:402` 的注释中，该注释已在 Task 1 的代码重写中删除。

无需额外改动。

---

### Task 6: 清理 `control.c` 中被废止的旧代码和注释

**Files:**
- Modify: `project/user/control/control.c`

- [ ] **Step 1: 删除被注释掉的旧版 `Yaw_Control`**

`control.c:428-467` 是被注释掉的旧版 `Yaw_Control`，已完全废止。删除整个注释块。

- [ ] **Step 2: 删除 `Yaw_Unwrap_Debug` 中过时的 `debug_desired_yaw` 赋值说明**

`control.c:78-99` 中的 `Yaw_Unwrap_Debug` 保留（纯地面 debug 显示），但更新注释说明它跟踪的是 JY901P 原始 yaw 的展开值，不进入控制。

当前 `control.c:78-79`：
```c
/* Ground yaw unfold debug - independent from flight control, tracks yaw continuously */
/* debug 用: 仍跟踪 JY901P 内部 state.angle.yaw 的展开值, 纯显示用, 不进控制 */
```

保持不变，注释已经准确。

- [ ] **Step 3: 修正 `control.c:30` 的注释**

当前：
```c
static float Desired_yaw = 0.0f;          /* 期望 yaw 角度, 连续 deg, 不做 ±180° 包裹 (MiniFly 方式) */
```

Task 1 中已改为：
```c
static float Desired_yaw = 0.0f;          /* 期望 yaw 角度, ±180° 包裹 (MiniFly 方式) */
```

确认 Task 1 的改动已覆盖此项。

- [ ] **Step 4: 编译验证**

- [ ] **Step 5: 提交**

```bash
git -C project add user/control/control.c
git -C project commit -m "chore: 删除废止的旧版 Yaw_Control 注释代码"
```

---

### Task 7: 更新架构文档

**Files:**
- Modify: `project/docs/architecture/current-architecture.md`
- Modify: `project/docs/architecture/refactor-notes.md`
- Modify: `project/docs/architecture/known-issues.md`

- [ ] **Step 1: 更新 `current-architecture.md` 的 yaw 条目**

在 `current-architecture.md` 末尾追加新的变更记录：

```markdown
### 2026-07-20：yaw 控制对齐 MiniFly

- yaw 测量源从飞控自积分 `∫gyro.z·dt` 切换为 JY901P 融合的 `state->angle.yaw`（±180° 范围）。
- `Desired_yaw` 改为 ±180° 包裹的积分角；杆回中后 `Desired_yaw` 不变，角度环自然维持航向（MiniFly 隐式锁角）。
- yaw 杆 LPF α 从 0.1 改为 0.2。
- RC 失联 ≥500ms 后强制 yaw/roll/pitch 杆量回中；`commanderDropToGround` 改用 setter 产生 `keyLandRising` 边沿。
- 定点模式（MODE_THREEHOLD）yaw 减半（×0.5）。
- 起飞时锁存当前 JY901P 融合 yaw 为初始航向，同时清零 `lpf_stick_yaw`。
```

- [ ] **Step 2: 更新 `refactor-notes.md`**

追加第四十二阶段：

```markdown
## 第四十二阶段：yaw 控制对齐 MiniFly

本阶段将 yaw 闭环行为对齐 MiniFly：测量源切换、松杆隐式锁角、RC 失联安全、定点减半。

改动文件：
  - `user/control/control.c`
  - `user/services/commander.c`

做了什么：
  - yaw 测量源从 `∫gyro.z·dt` 自积分改为 `state->angle.yaw`（JY901P 融合，±180°）。
  - 角度环误差做 ±180° 包裹（调整 ref 使 Err 取最短路径）。
  - `Desired_yaw` 改为 ±180° 包裹，杆回中后不变（MiniFly 隐式锁角）。
  - yaw 杆 LPF α 0.1→0.2。
  - `ResetYawState` 锁存当前 `state->angle.yaw` 为初始航向，清零 `lpf_stick_yaw`。
  - RC 锁定（≥500ms）后 `commanderGetSetpoint` 强制杆量回中。
  - `commanderDropToGround` 改用 `setCommanderKeyland/setCommanderKeyFlight` setter。
  - 定点模式 `setpoint->angle.yaw *= 0.5f`。
  - 删除废止的旧版 `Yaw_Control` 注释代码和 `YAW_INTEG_MAX_PER_STEP` 宏。

是否改变运行逻辑：
  - 是。yaw 闭环测量源、松杆行为、失联行为、定点 yaw 速度均改变。
```

- [ ] **Step 3: 更新 `known-issues.md`**

将当前 yaw 相关问题标记为已解决，追加历史条目。

- [ ] **Step 4: 提交**

```bash
git -C project add docs/architecture/current-architecture.md docs/architecture/refactor-notes.md docs/architecture/known-issues.md
git -C project commit -m "docs: 记录 yaw 对齐 MiniFly 的架构变更"
```

---

## 验证要点

1. **编译**：每个 Task 完成后在 Keil/EIDE 中编译，确认无错误/警告。
2. **地面测试（拆桨）**：
   - 上电后观察 `state->angle.yaw` 是否在 [-180, 180] 范围内正常变化。
   - 解锁后打 yaw 杆，`Desired_yaw` 应在 ±180° 内积分变化。
   - 回中后 `Desired_yaw` 应保持不变。
   - 手动扭转飞机，yaw 角度环应产生回正力矩。
3. **RC 失联测试**：关闭遥控器 500ms 后，setpoint yaw 应为 0。
4. **定点模式测试**：yaw 杆满杆旋转速度应约为手动模式的一半。
5. **PID 参数调整**：Kp=3.0 可能不足以在惯性下维持航向（MiniFly Kp=20），实飞后可能需要逐步增大 `pid_yaw_angle.Kp`。

## 风险与注意事项

- **JY901P yaw 可能含磁力计**：若磁场干扰（电机电流）导致 yaw 跳动，可考虑通过 UART 命令将 JY901P 切为 6 轴模式（`WitWriteReg(AXIS6, ALGRITHM6)`），或后续实现 Mahony 互补滤波。
- **Kp 参数**：当前的 Kp=3.0 比 MiniFly 的 20 小得多，松杆后角度环回正力偏弱。若实飞出现"松杆后漂移"现象，需优先增大 `pid_yaw_angle.Kp`。
- **Desired_yaw 包裹频率**：±84°/s 满杆时每 ~4.3s 包裹一次（360/84），远低于控制频率，包裹跳变不影响 PID。
- **`control_utf8.c`**：该文件是 `control.c` 的 UTF-8 乱码副本，未进入编译列表。本次不改动，但后续建议删除或同步更新。
