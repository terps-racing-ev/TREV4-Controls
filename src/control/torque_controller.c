#include "IO_Constants.h"
#include "config/torque_config.h"
#include "config/apps_config.h"
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "state_machine.h"
#include "torque_controller.h"


static TorqueController_Data_T torque_data;

static sbyte2 PedalTravelToTorque(ubyte2 pedal_travel)
{
    const ubyte2 pedal_travel_for_max_torque = (ubyte2)((((ubyte4)APPS_RESOLUTION) * (ubyte4)PERCENT_TRAVEL_FOR_MAX_TORQUE) / 100);
    const ubyte1 max_torque = MAX_TORQUE_DEFAULT;//RuntimeConfig_GetMaxTorque();

    if (pedal_travel < APPS_DEADZONE) {
        return 0;
    }

    if (pedal_travel > pedal_travel_for_max_torque) {
        return max_torque;
    }

    return (sbyte2)(((sbyte4)(pedal_travel - APPS_DEADZONE) * (sbyte4)max_torque) /
                    (sbyte4)(pedal_travel_for_max_torque - APPS_DEADZONE));
}

static sbyte2 PSIToTorque(ubyte2 psi)
{
    // TODO piece wise linear, def make a universal function for mapping a range to another range...
    return 0;
}

static ubyte4 ComputeSpeedMPHx100(sbyte2 motor_rpm, sbyte2 wheel_diameter_in)
{
    /* Compute vehicle speed in mph x100 from motor RPM and wheel diameter (inches).
     * Formula: speed_mph = (motor_rpm / GEAR_RATIO) * pi * diameter_in / (60 * 12)
     * With fixed-point x100 and integer arithmetic:
     *   speed_mph_x100 = (motor_rpm * diameter_in * 12627) / 100000
     *   where 12627 ≈ (3.14159 * 10000 / 3.4545) / 7.2
     * Note: Speed is unsigned (always positive/magnitude).
     */
    const ubyte2 SPEED_SCALE_NUM = 12627U;
    const ubyte4 SPEED_SCALE_DENOM = 100000UL;
    
    ubyte4 rpm_u = (ubyte4)motor_rpm;
    ubyte4 diameter_u = (ubyte4)wheel_diameter_in;
    
    /* Compute with intermediate values to prevent overflow. */
    ubyte4 temp = rpm_u * diameter_u;
    temp = (temp * (ubyte4)SPEED_SCALE_NUM);
    ubyte4 result = temp / SPEED_SCALE_DENOM;
    
    /* Saturate to ubyte2 max (65535 mph x100 = 655.35 mph). */
    if (result > 65535UL) {
        result = 65535UL;
    }
    
    return result;
}

void TorqueController_Init(void)
{
    torque_data.inv_torque_scaled = 0;
    torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
    torque_data.inv_enable = INVERTER_DISABLE;
    torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
    torque_data.speed_mph_x100 = 0;
}

void TorqueController_Update(void)
{
    const VCU_State_t state = StateMachine_GetState();
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();
    const InverterHighSpeed_RX_Data_t* inv_data = CAN_RX_GetInverterHighSpeedData();

    /* Compute vehicle speed from inverter RPM and wheel diameter config. */
    sbyte2 wheel_diameter = WHEEL_DIAMETER_DEFAULT;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_WHEEL_DIAMETER, &wheel_diameter);
    torque_data.speed_mph_x100 = ComputeSpeedMPHx100(inv_data->motor_speed, wheel_diameter);

    if (state != VCU_STATE_DRIVING) {
        torque_data.inv_torque_scaled = 0;
        torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
        torque_data.inv_enable = INVERTER_DISABLE;
        torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
        return;
    }

    // TODO should we compute this always for debug reasons or put it in the else?
    torque_data.apps_torque = PedalTravelToTorque(apps->apps_value);


    // TODO all this logic will have to be improved with launch control
    if (RuntimeConfig_GetRegenEnabled() && bse->brakes_engaged && 
        inv_data->motor_speed > MIN_RPM_FOR_REGEN) {
        torque_data.regen_torque = PSIToTorque(bse->psi);
        torque_data.inv_torque_scaled = torque_data.regen_torque * 10;
    } else {
        torque_data.inv_torque_scaled = torque_data.apps_torque * 10;
    }

    // TODO should we check errors again? since statemachine is one cycle behind
    torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
    torque_data.inv_enable = INVERTER_ENABLE;
    torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;

}

const TorqueController_Data_T* TorqueController_GetData(void)
{
    return &torque_data;
}