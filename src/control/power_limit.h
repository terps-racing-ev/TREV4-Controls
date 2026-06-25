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

/* Currently-active power cap (kW): the fixed runtime-config cap in Normal, or
 * the SoC-derated cap in Endurance. Used both for the DCL math and the config
 * readback so telemetry reflects what is actually being commanded. */
ubyte2 PowerLimit_GetActivePowerCapKw(void);

/* TX scheduling: transmit the current-limit frame only while enabled. */
bool PowerLimit_TxTrigger(void);

#endif // POWER_LIMIT_H
