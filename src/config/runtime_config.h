#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include "IO_Constants.h"

// MAX 255
typedef enum {
    RUNTIME_PARAM_MAX_TORQUE,
    RUNTIME_PARAM_MOTOR_DIRECTION,
    RUNTIME_PARAM_REGEN_ENABLED,
    RUNTIME_PARAM_DEBUG_DEFINES,
    RUNTIME_PARAM_WHEEL_DIAMETER,
    RUNTIME_PARAM_COUNT,
} RuntimeParamId_t;

/* Bit positions for runtime-configurable debug flags stored in
 * RUNTIME_PARAM_DEBUG_DEFINES (i16 bitmask). Defaults = 0 (off).
 */
#define DEBUG_BIT_ECHO_DAQ_TX_TO_CONTROLS      (1 << 0)
#define DEBUG_BIT_IGNORE_RTD_SWITCH            (1 << 1)
#define DEBUG_BIT_IGNORE_RTD_BRAKES            (1 << 2)
#define DEBUG_BIT_USE_APPS1_ONLY               (1 << 3)
#define DEBUG_BIT_USE_APPS2_ONLY               (1 << 4)
#define DEBUG_BIT_IGNORE_APPS_ERRORS           (1 << 5)
#define DEBUG_BIT_IGNORE_BSE_ERRORS            (1 << 6)
#define DEBUG_BIT_IGNORE_SDC                   (1 << 7)
#define DEBUG_BIT_IGNORE_BRAKE_PLAUSIBILITY    (1 << 8)
#define DEBUG_BIT_ALWAYS_GREEN                 (1 << 9)

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
bool RuntimeConfig_ConsumeImmediateConfigTxParam(RuntimeParamId_t* const out_param_id);

#endif // RUNTIME_CONFIG_H
