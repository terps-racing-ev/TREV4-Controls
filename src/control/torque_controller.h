#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

#include "IO_Constants.h"

#define REGEN_STATUS_FLAG_ENABLED        (1U << 0)
#define REGEN_STATUS_FLAG_DRIVING        (1U << 1)
#define REGEN_STATUS_FLAG_SPEED_OK       (1U << 2)
#define REGEN_STATUS_FLAG_FRONT_VALID    (1U << 3)
#define REGEN_STATUS_FLAG_REAR_VALID     (1U << 4)
#define REGEN_STATUS_FLAG_RYDER_ACTIVE   (1U << 5)
#define REGEN_STATUS_FLAG_REGEN_ACTIVE   (1U << 6)
#define REGEN_STATUS_FLAG_SOC_VALID      (1U << 7)

typedef enum {
    REGEN_BLOCK_NONE = 0,
    REGEN_BLOCK_DISABLED,
    REGEN_BLOCK_NOT_DRIVING,
    REGEN_BLOCK_BELOW_MIN_SPEED,
    REGEN_BLOCK_FRONT_INVALID,
    REGEN_BLOCK_REAR_INVALID,
    REGEN_BLOCK_FRONT_PRESSURE_HIGH,
    REGEN_BLOCK_RYDER_TABLE_ZERO,
    REGEN_BLOCK_RYDER_BALANCE_ZERO,
    REGEN_BLOCK_LEGACY_PRESSURE_INVALID,
    REGEN_BLOCK_ZERO_AFTER_CLAMPS,
    REGEN_BLOCK_SOC_HIGH,
    REGEN_BLOCK_APPS_ACTIVE,
} RegenBlockReason_t;

typedef struct {
    sbyte2 apps_torque;
    sbyte2 regen_torque;
    sbyte2 regen_front_table_torque;
    sbyte2 regen_balance_torque;
    sbyte2 regen_final_torque;
    ubyte2 regen_front_pressure_psi;
    ubyte2 regen_rear_pressure_psi_x10;
    ubyte2 regen_speed_rpm_abs;
    ubyte1 regen_strategy;
    ubyte1 regen_status_flags;
    ubyte1 regen_block_reason;
    bool regen_soc_gate_enabled;
    
    sbyte2 inv_torque_scaled;
    bool inv_enable;
    bool inv_direction;
    bool inv_speed_mode;

    ubyte4 speed_mph_x100;  /* Vehicle speed in mph, scaled x100 for integer math */
} TorqueController_Data_T;

void TorqueController_Init(void);
void TorqueController_Update(void);

const TorqueController_Data_T* TorqueController_GetData(void);

#endif // TORQUE_CONTROLLER_H
