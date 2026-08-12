# Remote Target Angle Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add target pitch/roll angle display to the remote control LCD, with font downsizing (12x16 → 8x12) to accommodate the new data row.

**Architecture:** Extend the existing nRF24 telemetry packet from 23 to 27 bytes by appending target pitch/roll (int16×100, big-endian). Parse on remote side into new StateData fields, then render on LCD using the smaller 8x12 font.

**Tech Stack:** C, STM32, nRF24L01+, UGUI LCD, FreeRTOS

---

## File Map

| Repo | File | Action | Purpose |
|------|------|--------|---------|
| drone | `user/communicate/remotedata.c` | Modify | Append target pitch/roll to telemetry packet |
| remote | `user/communicate/remotestate.h` | Modify | Add `target_pitch`, `target_roll` to StateData |
| remote | `user/communicate/remotestate.c` | Modify | Parse new fields from received packet |
| remote | `user/menu/main_ui.c` | Modify | Font 8x12 + new target angle row + layout shift |

---

### Task 1: Drone — Append target pitch/roll to telemetry packet

**Files:**
- Modify: `D:\Code\project\user\communicate\remotedata.c:264,309-311`

> Note: `#include "control.h"` is already present at line 11. No include change needed.

- [ ] **Step 1: Extend SI24R1_TX_DATA array from 23 to 27**

Line 264, change:
```c
uint8_t SI24R1_TX_DATA[23] = {0};
```
to:
```c
uint8_t SI24R1_TX_DATA[27] = {0};
```

- [ ] **Step 2: Append target pitch and target roll encoding after target.height**

Lines 309-311 are:
```c
temp = (int16_t)(target.height * 100);
SI24R1_TX_DATA[21] = Byte1(temp);
SI24R1_TX_DATA[22] = Byte0(temp);
```

Insert after `SI24R1_TX_DATA[22]`:
```c
temp = (int16_t)(debug_target_angle_pitch * 100);
SI24R1_TX_DATA[23] = Byte1(temp);
SI24R1_TX_DATA[24] = Byte0(temp);
temp = (int16_t)(debug_target_angle_roll * 100);
SI24R1_TX_DATA[25] = Byte1(temp);
SI24R1_TX_DATA[26] = Byte0(temp);
```

- [ ] **Step 3: Commit**

```bash
git -C "D:\Code\project" add user/communicate/remotedata.c
git -C "D:\Code\project" commit -m "feat: add target pitch/roll to nRF24 telemetry packet (23→27B)"
```

---

### Task 2: Remote — Add target fields to StateData

**Files:**
- Modify: `D:\Code\fly_p\projectRemote\user\communicate\remotestate.h:43`

- [ ] **Step 1: Add target_pitch and target_roll to StateData struct**

Lines 31-43, change:
```c
typedef struct
{
	float throttle;
    float_angle angle;
	float height;
	float battery_voltage;
	float battery_current;
	uint8_t ctrlMode;
	uint8_t attitudeMode;
	uint8_t keyFlight;
	float thrustLpf;
	float targetHeight;
} StateData;
```
to:
```c
typedef struct
{
	float throttle;
    float_angle angle;
	float height;
	float battery_voltage;
	float battery_current;
	uint8_t ctrlMode;
	uint8_t attitudeMode;
	uint8_t keyFlight;
	float thrustLpf;
	float targetHeight;
	float target_pitch;
	float target_roll;
} StateData;
```

- [ ] **Step 2: Commit**

```bash
git -C "D:\Code\fly_p\projectRemote" add user/communicate/remotestate.h
git -C "D:\Code\fly_p\projectRemote" commit -m "feat: add target pitch/roll fields to StateData"
```

---

### Task 3: Remote — Parse new target angle fields

**Files:**
- Modify: `D:\Code\fly_p\projectRemote\user\communicate\remotestate.c:99`

- [ ] **Step 1: Update length check and parse new fields**

Line 99: change `len >= 23` to `len >= 27`.

After line 112 (`state.targetHeight = ...`), insert:
```c
state.target_pitch = ((int16_t)((data[23] << 8) | data[24])) / 100.0f;
state.target_roll  = ((int16_t)((data[25] << 8) | data[26])) / 100.0f;
```

- [ ] **Step 2: Commit**

```bash
git -C "D:\Code\fly_p\projectRemote" add user/communicate/remotestate.c
git -C "D:\Code\fly_p\projectRemote" commit -m "feat: parse target pitch/roll from nRF24 telemetry packet"
```

---

### Task 4: Remote — Font change and LCD display

**Files:**
- Modify: `D:\Code\fly_p\projectRemote\user\menu\main_ui.c`

- [ ] **Step 1: Update constants**

Lines 44-45, change:
```c
#define MAIN_UI_ROWS 12
#define MAIN_UI_LINE_CHARS 27
```
to:
```c
#define MAIN_UI_ROWS 13
#define MAIN_UI_LINE_CHARS 39
```

- [ ] **Step 2: Update row height in main_ui_put_line**

Line 240, change:
```c
uint16_t y = row * 16;
```
to:
```c
uint16_t y = row * 12;
```

- [ ] **Step 3: Change font in main_ui_put_line**

Line 257, change:
```c
LCD_PutStr(0, y, line, FONT_12X16, color, C_BLACK);
```
to:
```c
LCD_PutStr(0, y, line, FONT_8X12, color, C_BLACK);
```

- [ ] **Step 4: Add target angle display row (new row 9)**

After line 120 (current `"PIT:%6.1f ROL:%6.1f"` display), insert:
```c
sprintf(buff, "T PIT:%5.1f T ROL:%5.1f", state.target_pitch, state.target_roll);
main_ui_put_line(9, buff, C_GREEN_YELLOW);
```

- [ ] **Step 5: Shift subsequent rows down by 1**

Existing row 9 → row 10:
```c
sprintf(buff, "RX CTRL:%d ATTI:%d", state.ctrlMode, state.attitudeMode);
main_ui_put_line(10, buff, C_WHITE);
```

Existing row 10 (flight_cmd_hint) → row 11:
```c
main_ui_put_line(11, flight_cmd_hint ? "CMD SENT" : "",
                 flight_cmd_hint ? C_CYAN : C_WHITE);
```

Existing row 11 (hint) → row 12:
```c
main_ui_put_line(12, atti_flag ? "ATTI CMD SENT" : "L1 CTRL L2 ATTI L3 FLY",
                 atti_flag ? C_GREEN_YELLOW : C_WHITE);
```

- [ ] **Step 6: Verify final UI layout matches**

```c
// Row 0:  "RoboFly RC        LOCK/UNLOCK"  [unchanged]
// Row 1:  "LINK:OK BAT:x.xV"               [unchanged]
// Row 2:  "CTRL:ALTHOLD SPD:LOW"           [unchanged]
// Row 3:  "ATTI:AIR    SET:AIR"            [unchanged]
// Row 4:  "FLY:FLYING  KEY:1"             [unchanged]
// Row 5:  "THR:x.x  TL:x.x"               [unchanged]
// Row 6:  "H:x.x S:x.x"                   [unchanged]
// Row 7:  "YAW:x.x"                       [unchanged]
// Row 8:  "PIT:x.x ROL:x.x"               [unchanged]
// Row 9:  "T PIT:x.x T ROL:x.x"           [NEW - target angles]
// Row 10: "RX CTRL:x ATTI:x"              [was row 9]
// Row 11: "CMD SENT"                      [was row 10, conditional]
// Row 12: "L1 CTRL L2 ATTI L3 FLY"       [was row 11]
```

- [ ] **Step 7: Commit**

```bash
git -C "D:\Code\fly_p\projectRemote" add user/menu/main_ui.c
git -C "D:\Code\fly_p\projectRemote" commit -m "feat: switch to 8x12 font, add target angle display row"
```

---

### Task 5: Verify

- [ ] **Step 1: Check nRF24 packet size**

Search for max payload: `nRF24L01P` has 32-byte max payload. 27 ≤ 32 — OK.

- [ ] **Step 2: Confirm FONT_8X12 is enabled**

File `D:\Code\fly_p\projectRemote\user\module\UGUI\ugui_config.h` line 32 has `#define UGUI_USE_FONT_8X12` — OK.

- [ ] **Step 3: Review diff for correctness**

```bash
git -C "D:\Code\project" diff HEAD~1 -- user/communicate/remotedata.c
git -C "D:\Code\fly_p\projectRemote" diff HEAD~3..HEAD
```

Confirm:
- Byte encoding uses same pattern as existing fields (big-endian, int16 × 100)
- Row indices are consecutive (0-12)
- Nothing is accidentally deleted
