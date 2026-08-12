#ifndef __MYDELAY_H_
#define __MYDELAY_H_
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "globalTime.h"

#define myDelay(id,delayTime) myDelay_label(id, delayTime, __LINE__)

void Mydelay_Init(void);
bool myDelay_label(uint32_t id,uint32_t delayTime, int16_t label);
void deleteMyDelay(uint32_t id);
#endif // !__MYDELAY_H_
