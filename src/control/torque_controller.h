#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

#include "IO_Constants.h"

typedef struct {

    sbyte2 apps_torque;
    sbyte2 regen_torque;
    
    sbyte2 inv_torque; // work on these namings...
    bool inv_enable;
    bool inv_direction;

} TorqueController_Data_T;

void TorqueController_Init(void);
void TorqueController_Update(void);

const TorqueController_Data_T* TorqueController_GetData(void);

#endif // TORQUE_CONTROLLER_H
