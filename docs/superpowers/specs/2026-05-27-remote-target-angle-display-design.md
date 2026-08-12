# Remote Control Target Angle Display

## Summary

Add target pitch and target roll angle display to the remote control LCD, with a font size reduction to accommodate the new data.

## Data Flow

```
drone control.c                      drone remotedata.c           nRF24           remote remotestate.c        remote main_ui.c
debug_target_angle_pitch ────→ encode as int16×100 ────→ [27B] ────→ decode to StateData ────→ LCD display
debug_target_angle_roll  ────→ append to packet tail
```

## Changes

### Drone: `user/communicate/remotedata.c`

- `#include "control.h"` for `debug_target_angle_pitch` and `debug_target_angle_roll`
- Extend `SI24R1_TX_DATA` from `[23]` to `[27]` (nRF24 max payload is 32 bytes)
- After `target.height` (data[21-22]), append:
  - data[23-24]: `int16_t(debug_target_angle_pitch * 100)` big-endian
  - data[25-26]: `int16_t(debug_target_angle_roll * 100)` big-endian

### Remote: `user/communicate/remotestate.h`

- Add to `StateData` struct: `float target_pitch; float target_roll;`

### Remote: `user/communicate/remotestate.c`

- `RemoteState_RecieveHandler`: change `len >= 23` → `len >= 27`
- Parse data[23-26] into `state.target_pitch` and `state.target_roll`

### Remote: `user/menu/main_ui.c`

- Font: `FONT_12X16` → `FONT_8X12`
- Row height: `row * 16` → `row * 12`
- `MAIN_UI_ROWS`: update for new layout (current 12 → 13)
- `MAIN_UI_LINE_CHARS`: increase from 27 to ~40 (8px wide, 320px screen)
- Add row 9: `"T PIT:%6.1f T ROL:%6.1f"` with `state.target_pitch`, `state.target_roll`
- Shift subsequent rows down by 1

### UI Layout (FONT_8X12, 320×240, ~20 rows available)

| Row | Content |
|-----|---------|
| 0 | `RoboFly RC        LOCK/UNLOCK` |
| 1 | `LINK:OK BAT:xx.xV` |
| 2 | `CTRL:ALTHOLD SPD:LOW` |
| 3 | `ATTI:AIR    SET:AIR` |
| 4 | `FLY:FLYING  KEY:1` |
| 5 | `THR:xx.x  TL:xx.x` |
| 6 | `H:xx.x S:xx.x` |
| 7 | `YAW:xx.x` |
| 8 | `PIT:xx.x ROL:xx.x` (measured) |
| 9 | `T PIT:xx.x T ROL:xx.x` (target, new) |
| 10 | `RX CTRL:x ATTI:x` |
| 11 | `CMD SENT` (conditional) |
| 12 | `L1 CTRL L2 ATTI L3 FLY` |

13 rows × 12px = 156px used out of 240px height.

## Backwards Compatibility

- Existing nRF24 packets (23 bytes) will produce stale/missing target angle data on old remotes if drone is updated first, but `len >= 27` check means old-format packets are silently dropped on new remote firmware
- Old drone + new remote: no display issue, packet length < 27 so simply no state update (existing behavior for malformed packets)

## Verification

- nRF24 packet size: 27 ≤ 32 bytes max payload — OK
- Big-endian encoding matches existing pattern in both drone and remote code
- `FONT_8X12` is already enabled in `ugui_config.h`
