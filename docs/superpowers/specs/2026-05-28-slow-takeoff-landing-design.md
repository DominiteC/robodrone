# Slow Takeoff & Slow Landing Design

## Overview

Replace the aggressive instant-takeoff and fixed-speed landing with smooth, constant-velocity profiles via the existing z-velocity PID cascade.

## Constants

```c
#define TAKEOFF_SPEED   15.0f   // takeoff climb rate (cm/s)
#define LAND_SPEED      10.0f   // landing descent rate (cm/s)
```

Hardcoded in `commander.c`. Existing `TAKEOFF_HEIGHT` (60.0f) unchanged.

## Takeoff

**Current behavior:** target height jumps to 60cm instantly; PID chases hard.

**New behavior:** velocity-mode climb at 15 cm/s until reaching 60cm, then auto-switch to position-hold.

**Implementation (`commanderGetSetpoint()` in `commander.c`):**

1. Add `static bool takeoffActive = false`.
2. When `keyFlight` transitions 0→1: set `takeoffActive = true`.
3. While `takeoffActive && state.height < TAKEOFF_HEIGHT`:
   - `setpoint->vel.z = TAKEOFF_SPEED`
   - `holdHeight` tracks current height (prevents jump on mode switch)
4. When `state.height >= TAKEOFF_HEIGHT`: `takeoffActive = false`, switch to normal position-hold.
5. When `keyFlight` clears: `takeoffActive = false`.

**`Height_Control()` in `control.c`:** No changes needed. Base thrust stays at `ALTHOLD_THRUST_BASE` (44%), velocity PID adds the required climb thrust naturally.

## Landing

**Current behavior:** `flyerAutoLand()` decrements target height by 4cm per cycle (`setpoint->height = state->height - 4.0f`).

**New behavior:** velocity-mode descent at -10 cm/s via z-velocity PID.

**Implementation (`flyerAutoLand()` in `commander.c`):**

1. Replace height stepping with `setpoint->vel.z = -LAND_SPEED`.
2. Track `state->height` into `holdHeight` to prevent position-mode jump if landing is aborted.
3. Landing detection unchanged: thrust < 30% for 10 cycles, or impact acceleration trigger.
4. In `commanderGetSetpoint()`, when `keyLand` is active, skip position-hold height PID (stay in velocity mode only).

## State Flow

```
DISARMED (keyFlight=0, keyLand=0)
    │
    ├── CMD_FLIGHT (keyFlight=1) ──► TAKEOFF (vel.z = +15 cm/s)
    │                                    │
    │                                    │ height >= 60cm
    │                                    ▼
    │                                POSITION-HOLD (normal flight)
    │                                    │
    │                                    │ CMD_FLIGHT (keyLand=1)
    │                                    ▼
    │                                LANDING (vel.z = -10 cm/s)
    │                                    │
    │                                    │ thrust < 30% or impact
    │                                    ▼
    │                                DISARMED
```

## Files Changed

| File | Changes |
|------|---------|
| `user/communicate/commander.c` | Add `TAKEOFF_SPEED`/`LAND_SPEED` defines; rewrite takeoff logic in `commanderGetSetpoint()`; rewrite `flyerAutoLand()` to velocity mode |
| `user/communicate/commander.h` | No changes expected |

## What Does NOT Change

- `control.c` `Height_Control()` — velocity PID cascade works as-is
- `TAKEOFF_HEIGHT` (60cm)
- Landing detection thresholds
- `ALTHOLD_THRUST_BASE` (44%)
- RC throttle stick behavior during position-hold
- Emergency landing (`commanderDropToGround()`)
