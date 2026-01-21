#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

#include "IO_Constants.h"

typedef struct {
    sbyte2 apps_torque;
    sbyte2 regen_torque;
    
    sbyte2 inv_torque_scaled;
    bool inv_enable;
    bool inv_direction;
    bool inv_speed_mode;

} TorqueController_Data_T;

void TorqueController_Init(void);
void TorqueController_Update(void);

const TorqueController_Data_T* TorqueController_GetData(void);

#endif // TORQUE_CONTROLLER_H
