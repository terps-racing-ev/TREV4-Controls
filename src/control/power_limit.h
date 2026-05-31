#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H

#include "IO_Constants.h"

typedef struct {
    bool   enabled;
    ubyte4 voltage_mv;
    ubyte2 dcl_amps;
    ubyte2 ccl_amps;
} PowerLimit_Data_t;

void PowerLimit_Init(void);
void PowerLimit_Update(void);
const PowerLimit_Data_t* PowerLimit_GetData(void);

/* TX scheduling: transmit the current-limit frame only while enabled. */
bool PowerLimit_TxTrigger(void);

#endif // POWER_LIMIT_H
