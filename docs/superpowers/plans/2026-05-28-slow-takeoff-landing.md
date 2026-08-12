# Slow Takeoff & Slow Landing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace instant takeoff and free-fall landing with smooth constant-velocity profiles via the z-velocity PID cascade.

**Architecture:** Two-file change in `user/communicate/commander.c` and `user/control/control.c`. Takeoff uses `setpoint->vel.z = +15cm/s` until reaching 60cm. Landing uses `setpoint->vel.z = -10cm/s` with the PID still active (unlike current which cuts thrust). No new state machine bits — minimal diff.

**Tech Stack:** C, STM32 FreeRTOS, existing PID controller library

---

### Task 1: Add constants and fix Height_Control for landing thrust

**Files:**
- Modify: `user/communicate/commander.c:11-18`
- Modify: `user/control/control.c:627-635`

**Why this first:** The landing PID fix is a prerequisite — without it, slow landing can't work because Height_Control returns 0 thrust during landing.

- [ ] **Step 1: Add TAKEOFF_SPEED and LAND_SPEED defines in commander.c**

Replace lines 11-18 in `user/communicate/commander.c`:

```c
#define TAKEOFF_HEIGHT      60.f
#define COMMANDER_DT        0.001f

/* 速度优先定高参数 */
#define PILOT_SPEED_UP      60.f    // 最大爬升速度 cm/s
#define PILOT_SPEED_DN      40.f    // 最大下降速度 cm/s
#define PILOT_ACC_Z         60.f    // 垂直加速度限幅 cm/s²
#define THR_DZ              0.08f   // 摇杆中位死区 (0~1)
```

Replace with:

```c
#define TAKEOFF_HEIGHT      60.f
#define TAKEOFF_SPEED       15.f    // 起飞爬升速度 cm/s
#define LAND_SPEED          10.f    // 降落下降速度 cm/s
#define COMMANDER_DT        0.001f

/* 速度优先定高参数 */
#define PILOT_SPEED_UP      60.f    // 最大爬升速度 cm/s
#define PILOT_SPEED_DN      40.f    // 最大下降速度 cm/s
#define PILOT_ACC_Z         60.f    // 垂直加速度限幅 cm/s²
#define THR_DZ              0.08f   // 摇杆中位死区 (0~1)
```

- [ ] **Step 2: Fix Height_Control to keep PID active during landing**

In `user/control/control.c`, lines 627-635, change the early-return guard to also allow keyLand:

```c
    // 起飞前清空积分，防止之前的积分导致油门数值不正常
    if (!getCommanderKeyFlight())
```

Replace with:

```c
    // 起飞前/降落完成后清空积分，防止之前的积分导致油门数值不正常
    if (!getCommanderKeyFlight() && !getCommanderKeyland())
```

- [ ] **Step 3: Also update the thrust-learning guard to avoid resetting during landing**

In `user/control/control.c`, line 680:

```c
    if(getCommanderKeyFlight())
```

Replace with:

```c
    if(getCommanderKeyFlight() || getCommanderKeyland())
```

- [ ] **Step 4: Commit**

```bash
git add user/communicate/commander.c user/control/control.c
git commit -m "feat: add speed constants and fix landing thrust path

Height_Control now allows PID to run during keyLand, enabling
controlled descent instead of cutting thrust to zero."
```

---

### Task 2: Slow takeoff — velocity-mode climb in commanderGetSetpoint

**Files:**
- Modify: `user/communicate/commander.c:119-225`

- [ ] **Step 1: Add takeoffActive static variable**

After line 111 (`static bool isAdjustingPosZ = false;`), add:

```c
static bool takeoffActive = false;      /* 起飞爬升阶段 */
```

- [ ] **Step 2: Rewrite takeoff logic in commanderGetSetpoint**

Replace lines 162-225 (the `else if(commander.keyFlight)` block) with:

```c
            else if(commander.keyFlight)/*一键起飞*/
            {
                setpoint->thrust = 0;

                if (initHigh == false)
                {
                    initHigh = true;
                    takeoffActive = true;
                    isAdjustingPosXY = true;
                    errorPosX = 0.f;
                    errorPosY = 0.f;
                    isAdjustingPosZ = false;

                    holdHeight = state->height;
                    setpoint->height = holdHeight;
                    setpoint->vel.z = TAKEOFF_SPEED;
                }

                if (takeoffActive)
                {
                    /* 速度模式爬升至目标高度 */
                    if (state->height >= TAKEOFF_HEIGHT)
                    {
                        takeoffActive = false;
                        isAdjustingPosZ = false;
                        holdHeight = state->height;
                        setpoint->height = holdHeight;
                        setpoint->vel.z = 0;
                    }
                    else
                    {
                        /* 爬升中: 速度模式, 跟踪当前高度 */
                        setpoint->vel.z = TAKEOFF_SPEED;
                        holdHeight = state->height;
                        setpoint->height = holdHeight;
                    }
                }
                else
                {
                    /* 起飞完成后的正常飞行: 摇杆控制高度 */
                    float climbRaw = (data.throttle - 50.f) / 50.f;
                    if (fabsf(climbRaw) < THR_DZ) climbRaw = 0;

                    float climb;
                    if (climbRaw > 0.f)
                        climb = climbRaw * PILOT_SPEED_UP;
                    else
                        climb = climbRaw * PILOT_SPEED_DN;

                    if (fabsf(climb) > 5.f)
                    {
                        isAdjustingPosZ = true;
                        setpoint->vel.z = climb;
                        holdHeight = state->height;
                        setpoint->height = holdHeight;

                        if (climbRaw < -0.2f)
                        {
                            if (maxAccZ < state->acc.z)
                                maxAccZ = state->acc.z;
                            if (maxAccZ > 250.f)
                            {
                                commander.keyFlight = false;
                            }
                        }
                        else
                        {
                            maxAccZ = 0.f;
                        }
                    }
                    else if (isAdjustingPosZ == true)
                    {
                        isAdjustingPosZ = false;
                        setpoint->vel.z = 0;
                        setpoint->height = state->height;
                        holdHeight = setpoint->height;
                    }
                    else
                    {
                        setpoint->vel.z = 0;
                    }
                }
            }
```

- [ ] **Step 3: Clear takeoffActive on disarm**

In the `else/*着陆状态*/` block (around line 227), add `takeoffActive = false`:

Replace lines 227-233:

```c
            else/*着陆状态*/
            {
                setpoint->thrust = 0;
                setpoint->vel.z = 0;
                initHigh = false;
                holdHeight = TAKEOFF_HEIGHT;
            }
```

With:

```c
            else/*着陆状态*/
            {
                setpoint->thrust = 0;
                setpoint->vel.z = 0;
                initHigh = false;
                takeoffActive = false;
                holdHeight = TAKEOFF_HEIGHT;
            }
```

- [ ] **Step 4: Commit**

```bash
git add user/communicate/commander.c
git commit -m "feat: slow takeoff via velocity-mode climb at 15cm/s"
```

---

### Task 3: Slow landing — velocity-mode descent in flyerAutoLand

**Files:**
- Modify: `user/communicate/commander.c:70-102`

- [ ] **Step 1: Replace flyerAutoLand body**

Replace lines 70-102 (the entire `flyerAutoLand` function) with:

```c
void flyerAutoLand(setpoint_t *setpoint,const state_t *state)
{   
    static uint8_t lowThrustCnt = 0;

    setpoint->vel.z = -LAND_SPEED;
    setpoint->height = state->height;

    if(getAltholdThrust() < 30.f)
    {
        lowThrustCnt++;
        if(lowThrustCnt > 10)
        {
            lowThrustCnt = 0;
            commander.keyLand = false;
            commander.keyFlight = false;
        }
    }else
    {
        lowThrustCnt = 0;
    }

    if (minAccZ < -80.f && maxAccZ > 320.f)
    {
        commander.keyLand = false;
        commander.keyFlight = false;
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add user/communicate/commander.c
git commit -m "feat: slow landing via velocity-mode descent at 10cm/s"
```

---

### Task 4: Verify completeness

**Files:**
- Verify: `user/communicate/commander.c`
- Verify: `user/control/control.c`

- [ ] **Step 1: Verify no dangling references to old landing code**

Run: `grep -n "state->height - 4" user/communicate/commander.c`
Expected: No matches (the hardcoded 4cm step is gone)

- [ ] **Step 2: Verify defines exist**

Run: `grep -n "TAKEOFF_SPEED\|LAND_SPEED" user/communicate/commander.c`
Expected: Two matches each (define + usage)

- [ ] **Step 3: Verify Height_Control guards**

Run: `grep -n "getCommanderKeyFlight\|getCommanderKeyland" user/control/control.c`
Expected: The `!getCommanderKeyFlight()` at line ~628 now reads `!getCommanderKeyFlight() && !getCommanderKeyland()`, and the thrust-learning guard includes `|| getCommanderKeyland()`

- [ ] **Step 4: Commit final check**

```bash
git add user/communicate/commander.c user/control/control.c
git diff --cached --stat
```
