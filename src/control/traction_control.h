#ifndef TRACTION_CONTROL_H
#define TRACTION_CONTROL_H

#include "IO_Constants.h"

typedef struct {
    bool enabled;
    bool active;
    bool front_left_valid;
    bool front_right_valid;

    ubyte2 rear_wheel_rpm;
    ubyte2 avg_front_rpm;
    ubyte2 slip_ratio_x1000;
    sbyte2 torque_limit_nm;
} TractionControl_Data_t;

void TractionControl_Init(void);
sbyte2 TractionControl_ApplyLimit(sbyte2 requested_torque_nm, sbyte2 motor_rpm);
const TractionControl_Data_t* TractionControl_GetData(void);

#endif // TRACTION_CONTROL_H
