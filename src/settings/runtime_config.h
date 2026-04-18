#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include "IO_Constants.h"

/*
 * RuntimeConfig
 * - Holds configurable parameters in RAM
 * - Loads/saves them to EEPROM
 * - Updated via CAN config-set messages
 */

typedef enum {
    RUNTIME_PARAM_MAX_TORQUE = 1,
} RuntimeParamId_t;

void RuntimeConfig_Init(void);
void RuntimeConfig_Task(void);

/* Generic parameter access (all parameters stored as i32). */
bool RuntimeConfig_SetI32(const ubyte2 param_id, const sbyte4 value);
bool RuntimeConfig_GetI32(const ubyte2 param_id, sbyte4* const out_value);

/* Convenience getter for torque controller usage (clamped). */
sbyte2 RuntimeConfig_GetMaxTorque(void);

/*
 * Settings broadcast helper.
 * Returns TRUE if a parameter was provided.
 * The intent is that the VCU periodically (slowly) transmits one parameter at a time,
 * and also transmits immediately after any successful SET.
 */
bool RuntimeConfig_GetNextBroadcastParam(ubyte2* const out_param_id, sbyte4* const out_value);

#endif // RUNTIME_CONFIG_H
