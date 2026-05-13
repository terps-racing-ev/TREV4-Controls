#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include "IO_Constants.h"

// MAX 255
typedef enum {
    RUNTIME_PARAM_MAX_TORQUE,
    RUNTIME_PARAM_MOTOR_DIRECTION,
    RUNTIME_PARAM_REGEN_ENABLED,
    RUNTIME_PARAM_COUNT,
} RuntimeParamId_t;

void RuntimeConfig_Init(void);
void RuntimeConfig_Task(void);
bool RuntimeConfig_IsEepromLoadComplete(void);

/* Generic parameter access (all parameters stored as i16). */
bool RuntimeConfig_Set(const RuntimeParamId_t param_id, const sbyte2 value);
bool RuntimeConfig_GetI32(const RuntimeParamId_t param_id, sbyte2* const out_value);

/* Convenience getter for torque controller usage (clamped). */
ubyte1 RuntimeConfig_GetMaxTorque(void);
bool RuntimeConfig_GetMotorDirection(void);
bool RuntimeConfig_GetRegenEnabled(void);

/*
 * Broadcast trigger helpers.
 * Used by CAN TX scheduling so settings only transmit on demand.
 */
bool RuntimeConfig_ConfigTxTrigger(void);

#endif // RUNTIME_CONFIG_H
