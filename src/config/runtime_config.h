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
    RUNTIME_PARAM_TRACTION_CONTROL_ENABLED,
    RUNTIME_PARAM_TRACTION_CONTROL_TARGET_SLIP_X1000,
    RUNTIME_PARAM_TRACTION_CONTROL_KP_X1000,
    RUNTIME_PARAM_TRACTION_CONTROL_KI_X1000,
    RUNTIME_PARAM_TRACTION_CONTROL_KD_X1000,
    RUNTIME_PARAM_TRACTION_CONTROL_MIN_FRONT_RPM,
    RUNTIME_PARAM_POWER_LIMIT_ENABLED,
    RUNTIME_PARAM_POWER_CAP_KW,
    RUNTIME_PARAM_REGEN_MAX_TORQUE,
    RUNTIME_PARAM_REGEN_MIN_TORQUE,
    RUNTIME_PARAM_REGEN_MIN_BSE_REAR_PSI,
    RUNTIME_PARAM_REGEN_MIN_BSE_FRONT_PSI,
    RUNTIME_PARAM_REGEN_MAX_BSE_REAR_PSI,
    RUNTIME_PARAM_REGEN_MAX_BSE_FRONT_PSI,
    RUNTIME_PARAM_REGEN_MIN_SPEED,
    RUNTIME_PARAM_REGEN_MAX_SOC,
    RUNTIME_PARAM_REGEN_STRATEGY,
    /* IDs 22..38 were the old Launch-Control "learning" + uploaded-curve params.
     * They were removed when launch control was rewritten to a time-based torque
     * curve (June 2026). The IDs are intentionally left RESERVED -- the params
     * below keep explicit ids so existing EEPROM records and DBC mux indices do
     * not shift. APPEND-ONLY past this point; bump RUNTIME_CFG_VERSION on change. */
    RUNTIME_PARAM_REGEN_SOC_GATE_ENABLED = 39,
    RUNTIME_PARAM_REGEN_MAX_APPS,            /* 40 */
    RUNTIME_PARAM_REGEN_RYDER_MU_X1000,      /* 41 */

    /* Time-based launch control: runtime-tunable torque set-points (Nm). */
    RUNTIME_PARAM_LAUNCH_TORQUE_OFFTHELINE,  /* 42 */
    RUNTIME_PARAM_LAUNCH_TORQUE_INIT,        /* 43 */
    RUNTIME_PARAM_LAUNCH_TORQUE_FINAL,       /* 44 */

    /* Time-based launch control: runtime-tunable curve shape / trigger thresholds. */
    RUNTIME_PARAM_LAUNCH_OFFTHELINE_TIME_MS,    /* 45 - off-the-line hold window in ms */
    RUNTIME_PARAM_LAUNCH_CURVE_DURATION_MS,     /* 46 - parabolic ramp duration in ms */
    RUNTIME_PARAM_LAUNCH_TRIGGER_APPS_PERCENT,  /* 47 - APPS % that triggers launch */
    RUNTIME_PARAM_LAUNCH_END_APPS_PERCENT,      /* 48 - APPS % at/below which launch ends */

    RUNTIME_PARAM_COUNT,                         /* 49 */
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
