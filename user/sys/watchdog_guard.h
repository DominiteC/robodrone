#ifndef __WATCHDOG_GUARD_H__
#define __WATCHDOG_GUARD_H__

#include <stdbool.h>
#include <stdint.h>

void WatchdogGuard_Init(uint32_t timeout_ms);
void WatchdogGuard_ControlHeartbeat(void);
void WatchdogGuard_FeedNow(void);
bool WatchdogGuard_WasIwdgReset(void);
void WatchdogGuard_EnterLongAction(uint32_t allow_ms);
void WatchdogGuard_ExitLongAction(void);

#endif
