#include "IO_Constants.h"
#include "config/torque_config.h"
#include "config/apps_config.h"
#include "can/can_manager.h"
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "state_machine.h"
#include "torque_controller.h"
#include "traction_control.h"


static TorqueController_Data_T torque_data;

static sbyte2 GetParam(RuntimeParamId_t param_id)
{
    sbyte2 value = 0;
    (void)RuntimeConfig_GetI32(param_id, &value);
    return value;
}

static sbyte2 PedalTravelToTorque(ubyte2 pedal_travel)
{
    const ubyte2 pedal_travel_for_max_torque = (ubyte2)((((ubyte4)APPS_RESOLUTION) * (ubyte4)PERCENT_TRAVEL_FOR_MAX_TORQUE) / 100);
    const ubyte1 max_torque = RuntimeConfig_GetMaxTorque(); // MAX_TORQUE_DEFAULT;

    if (pedal_travel < APPS_DEADZONE) {
        return 0;
    }

    if (pedal_travel > pedal_travel_for_max_torque) {
        return max_torque;
    }

    return (sbyte2)(((sbyte4)(pedal_travel - APPS_DEADZONE) * (sbyte4)max_torque) /
                    (sbyte4)(pedal_travel_for_max_torque - APPS_DEADZONE));
}

static sbyte2 InterpolateRegenTorque(const ubyte2 pressure_psi,
                                     const sbyte2 min_pressure_psi,
                                     const sbyte2 max_pressure_psi)
{
    sbyte2 min_torque = GetParam(RUNTIME_PARAM_REGEN_MIN_TORQUE);
    sbyte2 max_torque = GetParam(RUNTIME_PARAM_REGEN_MAX_TORQUE);

    if (max_torque < min_torque) {
        const sbyte2 configured_min_torque = min_torque;
        min_torque = max_torque;
        max_torque = configured_min_torque;
    }

    if (pressure_psi < (ubyte2)min_pressure_psi) {
        return 0;
    }

    if (max_pressure_psi <= min_pressure_psi) {
        return max_torque;
    }

    if (pressure_psi >= (ubyte2)max_pressure_psi) {
        return max_torque;
    }

    return (sbyte2)(min_torque +
                    (((sbyte4)(pressure_psi - (ubyte2)min_pressure_psi) *
                      (sbyte4)(max_torque - min_torque)) /
                     (sbyte4)(max_pressure_psi - min_pressure_psi)));
}

static ubyte2 RearBrakePressurePsi(const MOBO_PowerTelemetry_RX_Data_t* const mobo_power)
{
    return (ubyte2)(((ubyte4)mobo_power->rear_brake_pressure_psi_x10 + 5U) / 10U);
}

static bool GetRegenPressure(const BSE_Data_t* const front_bse,
                             const MOBO_PowerTelemetry_RX_Data_t* const mobo_power,
                             ubyte2* const out_pressure_psi,
                             sbyte2* const out_min_pressure_psi,
                             sbyte2* const out_max_pressure_psi)
{
    if ((front_bse == NULL) || (mobo_power == NULL) ||
        (out_pressure_psi == NULL) || (out_min_pressure_psi == NULL) ||
        (out_max_pressure_psi == NULL)) {
        return FALSE;
    }

    sbyte2 strategy = GetParam(RUNTIME_PARAM_REGEN_STRATEGY);
    const ubyte2 front_pressure = front_bse->psi;
    const ubyte2 rear_pressure = RearBrakePressurePsi(mobo_power);
    const sbyte2 front_min = GetParam(RUNTIME_PARAM_REGEN_MIN_BSE_FRONT_PSI);
    const sbyte2 rear_min = GetParam(RUNTIME_PARAM_REGEN_MIN_BSE_REAR_PSI);
    const sbyte2 front_max = GetParam(RUNTIME_PARAM_REGEN_MAX_BSE_FRONT_PSI);
    const sbyte2 rear_max = GetParam(RUNTIME_PARAM_REGEN_MAX_BSE_REAR_PSI);
    const bool rear_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_MOBO_POWER_TELEMETRY);

    if (strategy > REGEN_STRATEGY_AVERAGED) {
        strategy = REGEN_STRATEGY_DEFAULT;
    }

    if (strategy == REGEN_STRATEGY_FRONT_ONLY) {
        if (!front_bse->valid) {
            return FALSE;
        }

        *out_pressure_psi = front_pressure;
        *out_min_pressure_psi = front_min;
        *out_max_pressure_psi = front_max;
        return TRUE;
    }

    if (strategy == REGEN_STRATEGY_REAR_ONLY) {
        if (!rear_valid) {
            return FALSE;
        }

        *out_pressure_psi = rear_pressure;
        *out_min_pressure_psi = rear_min;
        *out_max_pressure_psi = rear_max;
        return TRUE;
    }

    if (!front_bse->valid || !rear_valid) {
        return FALSE;
    }

    *out_pressure_psi = (ubyte2)(((ubyte4)front_pressure + (ubyte4)rear_pressure) / 2U);
    *out_min_pressure_psi = (sbyte2)(((sbyte4)front_min + (sbyte4)rear_min) / 2);
    *out_max_pressure_psi = (sbyte2)(((sbyte4)front_max + (sbyte4)rear_max) / 2);
    return TRUE;
}

static sbyte2 CalculateRegenTorque(const BSE_Data_t* const front_bse,
                                   const MOBO_PowerTelemetry_RX_Data_t* const mobo_power,
                                   const sbyte2 motor_speed)
{
    ubyte2 pressure_psi = 0;
    sbyte2 min_pressure_psi = 0;
    sbyte2 max_pressure_psi = 0;
    const sbyte2 min_speed = GetParam(RUNTIME_PARAM_REGEN_MIN_SPEED);
    const HVCSOC_RX_Data_t* const hvc_soc = CAN_RX_GetHVCSOCData();
    const bool hvc_soc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SOC);
    const ubyte4 regen_max_soc_x100 = (ubyte4)(GetParam(RUNTIME_PARAM_REGEN_MAX_SOC) * 100);

    if (!RuntimeConfig_GetRegenEnabled()) {
        return 0;
    }

    if (hvc_soc_valid && ((ubyte4)hvc_soc->pack_soc_percent_x100 > regen_max_soc_x100)) {
        return 0;
    }

    if (motor_speed <= min_speed) {
        return 0;
    }

    if (!GetRegenPressure(front_bse, mobo_power,
                          &pressure_psi,
                          &min_pressure_psi,
                          &max_pressure_psi)) {
        return 0;
    }

    return (sbyte2)(-InterpolateRegenTorque(pressure_psi,
                                            min_pressure_psi,
                                            max_pressure_psi));
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
    torque_data.regen_torque = 0;
    TractionControl_Init();
}

void TorqueController_Update(void)
{
    const VCU_State_t state = StateMachine_GetState();
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();
    const InverterHighSpeed_RX_Data_t* inv_data = CAN_RX_GetInverterHighSpeedData();
    const MOBO_PowerTelemetry_RX_Data_t* mobo_power = CAN_RX_GetMOBO_PowerTelemetryData(); // Rear BSE PSI is here

    /* Compute vehicle speed from inverter RPM and wheel diameter config. */
    sbyte2 wheel_diameter = WHEEL_DIAMETER_DEFAULT;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_WHEEL_DIAMETER, &wheel_diameter);
    torque_data.speed_mph_x100 = ComputeSpeedMPHx100(inv_data->motor_speed, wheel_diameter);

    if (state != VCU_STATE_DRIVING) {
        (void)TractionControl_ApplyLimit(0, inv_data->motor_speed);
        torque_data.inv_torque_scaled = 0;
        torque_data.regen_torque = 0;
        torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
        torque_data.inv_enable = INVERTER_DISABLE;
        torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
        return;
    }

    // TODO should we compute this always for debug reasons or put it in the else?
    torque_data.apps_torque = PedalTravelToTorque(apps->apps_value);


    // TODO all this logic will have to be improved with launch control
    torque_data.regen_torque = CalculateRegenTorque(bse, mobo_power, inv_data->motor_speed);
    if (torque_data.regen_torque < 0) {
        (void)TractionControl_ApplyLimit(0, inv_data->motor_speed);
        torque_data.inv_torque_scaled = torque_data.regen_torque * 10;
    } else {
        const sbyte2 limited_torque = TractionControl_ApplyLimit(torque_data.apps_torque,
                                                                 inv_data->motor_speed);
        torque_data.regen_torque = 0;
        torque_data.inv_torque_scaled = limited_torque * 10;
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
