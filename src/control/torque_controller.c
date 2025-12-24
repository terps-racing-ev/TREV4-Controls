#include "IO_Constants.h"
#include "config/torque_config.h"
#include "config/apps_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "state_machine.h"
#include "torque_controller.h"


static TorqueController_Data_T torque_data;

static sbyte2 PedalTravelToTorque(ubyte2 pedal_travel)
{
    const ubyte2 pedal_travel_for_max_torque = (ubyte2)((APPS_RESOLUTION * PERCENT_TRAVEL_FOR_MAX_TORQUE) / 100);

    if (pedal_travel < APPS_DEADZONE) {
        return 0;
    }

    if (pedal_travel > pedal_travel_for_max_torque) {
        return (sbyte2)MAX_TORQUE;
    }

    return (sbyte2)(((sbyte4)(pedal_travel - APPS_DEADZONE) * (sbyte4)MAX_TORQUE) /
                    (sbyte4)(pedal_travel_for_max_torque - APPS_DEADZONE));
}

void TorqueController_Init(void)
{
    torque_data.inv_direction = MOTOR_DIRECTION;
    torque_data.inv_enable = INVERTER_DISABLE;
}

void TorqueController_Update(void)
{
    VCU_State_t state = StateMachine_GetState();
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();

    if (state != VCU_STATE_DRIVING) {
        torque_data.inv_direction = MOTOR_DIRECTION;
        torque_data.inv_enable = INVERTER_DISABLE;
        return;
    }

    torque_data.apps_torque = PedalTravelToTorque(apps->apps_value);
    // TODO should we check errors again?

}

const TorqueController_Data_T* TorqueController_GetData(void)
{
    return &torque_data;
}