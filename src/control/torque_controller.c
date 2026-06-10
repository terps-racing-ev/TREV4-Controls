#include "IO_Constants.h"
#include "config/torque_config.h"
#include "config/apps_config.h"
#include "config/bse_config.h"
#include "can/can_manager.h"
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "state_machine.h"
#include "torque_controller.h"
#include "traction_control.h"
#include "launch_control.h"


static TorqueController_Data_T torque_data;

static const sbyte4 ryder_regen_torque_x1000[REGEN_RYDER_FRONT_TABLE_POINTS] = REGEN_RYDER_FRONT_TORQUE_X1000;

static sbyte2 GetParam(RuntimeParamId_t param_id)
{
    sbyte2 value = 0;
    (void)RuntimeConfig_GetI32(param_id, &value);
    return value;
}

static ubyte2 AbsS16ToU16(const sbyte2 value)
{
    if (value >= 0) {
        return (ubyte2)value;
    }

    if (value == (sbyte2)-32768) {
        return 32767U;
    }

    return (ubyte2)(-value);
}

static ubyte1 GetConfiguredRegenStrategy(void)
{
    sbyte2 strategy = GetParam(RUNTIME_PARAM_REGEN_STRATEGY);

    if (strategy > REGEN_STRATEGY_RYDER) {
        strategy = REGEN_STRATEGY_DEFAULT;
    }

    if (strategy < REGEN_STRATEGY_FRONT_ONLY) {
        strategy = REGEN_STRATEGY_DEFAULT;
    }

    return (ubyte1)strategy;
}

static ubyte2 GetConfiguredRegenMaxApps(void)
{
    sbyte2 max_apps_percent = GetParam(RUNTIME_PARAM_REGEN_MAX_APPS);

    if (max_apps_percent < 0) {
        max_apps_percent = 0;
    }

    return (ubyte2)((ubyte2)max_apps_percent * 10U);
}

static float4 GetConfiguredRyderMu(void)
{
    sbyte2 mu_x1000 = GetParam(RUNTIME_PARAM_REGEN_RYDER_MU_X1000);

    if (mu_x1000 < 0) {
        mu_x1000 = 0;
    }

    return ((float4)mu_x1000) / 1000.0f;
}

static bool IgnoreBrakePlausibility(void)
{
    const sbyte2 dbg_bits = GetParam(RUNTIME_PARAM_DEBUG_DEFINES);
    return ((dbg_bits & DEBUG_BIT_IGNORE_BRAKE_PLAUSIBILITY) != 0);
}

static bool BrakeThrottleCutActive(const BSE_Data_t* const front_bse)
{
    if (IgnoreBrakePlausibility()) {
        return FALSE;
    }

    if ((front_bse == NULL) || !front_bse->valid) {
        return FALSE;
    }

    return (front_bse->psi > BRAKE_THROTTLE_CUT_THRESHOLD);
}

static void ResetRegenDebugDerived(void)
{
    torque_data.regen_front_table_torque = 0;
    torque_data.regen_balance_torque = 0;
    torque_data.regen_final_torque = 0;
    torque_data.regen_block_reason = REGEN_BLOCK_NONE;
}

static void UpdateRegenDebugInputs(const VCU_State_t state,
                                   const BSE_Data_t* const front_bse,
                                   const MOBO_PowerTelemetry_RX_Data_t* const mobo_power,
                                   const sbyte2 motor_speed)
{
    const bool regen_enabled = RuntimeConfig_GetRegenEnabled();
    const bool rear_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_MOBO_POWER_TELEMETRY);
    const bool soc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SOC);
    const sbyte2 min_speed = GetParam(RUNTIME_PARAM_REGEN_MIN_SPEED);

    torque_data.regen_strategy = GetConfiguredRegenStrategy();
    torque_data.regen_soc_gate_enabled = (GetParam(RUNTIME_PARAM_REGEN_SOC_GATE_ENABLED) != 0);
    torque_data.regen_speed_rpm_abs = AbsS16ToU16(motor_speed);
    torque_data.regen_front_pressure_psi = ((front_bse != NULL) ? front_bse->psi : 0U);
    torque_data.regen_rear_pressure_psi_x10 = ((mobo_power != NULL) ? mobo_power->rear_brake_pressure_psi_x10 : 0U);
    torque_data.regen_status_flags = 0;

    if (regen_enabled) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_ENABLED;
    }

    if (state == VCU_STATE_DRIVING) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_DRIVING;
    }

    if (torque_data.regen_speed_rpm_abs > (ubyte2)min_speed) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_SPEED_OK;
    }

    if ((front_bse != NULL) && front_bse->valid) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_FRONT_VALID;
    }

    if (rear_valid) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_REAR_VALID;
    }

    if (torque_data.regen_strategy == REGEN_STRATEGY_RYDER) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_RYDER_ACTIVE;
    }

    if (soc_valid) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_SOC_VALID;
    }

    ResetRegenDebugDerived();
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

static sbyte2 LookupRyderFrontTorque(const ubyte2 front_pressure_psi)
{
    if (front_pressure_psi > REGEN_RYDER_MAX_FRONT_PSI) {
        return 0;
    }

    const ubyte2 lower_idx = (ubyte2)(front_pressure_psi / REGEN_RYDER_FRONT_PSI_STEP);
    if (lower_idx >= (REGEN_RYDER_FRONT_TABLE_POINTS - 1)) {
        return (sbyte2)((ryder_regen_torque_x1000[REGEN_RYDER_FRONT_TABLE_POINTS - 1] + 500L) / 1000L);
    }

    const ubyte2 lower_psi = (ubyte2)(lower_idx * REGEN_RYDER_FRONT_PSI_STEP);
    const ubyte2 delta_psi = (ubyte2)(front_pressure_psi - lower_psi);
    const sbyte4 lower_torque = ryder_regen_torque_x1000[lower_idx];
    const sbyte4 upper_torque = ryder_regen_torque_x1000[lower_idx + 1];
    const sbyte4 interpolated = lower_torque +
                                (((sbyte4)delta_psi * (upper_torque - lower_torque)) /
                                 (sbyte4)REGEN_RYDER_FRONT_PSI_STEP);

    if (interpolated <= 0) {
        return 0;
    }

    return (sbyte2)((interpolated + 500L) / 1000L);
}

static sbyte2 CalculateRyderBalanceTorque(const ubyte2 front_pressure_psi,
                                          const ubyte2 rear_pressure_psi_x10)
{
    const float4 x = (float4)front_pressure_psi;
    const float4 y = ((float4)rear_pressure_psi_x10) / 10.0f;
    const float4 front_term = REGEN_RYDER_FRONT_FORCE_C0 +
                              (x * REGEN_RYDER_FRONT_FORCE_C1) -
                              (x * x * REGEN_RYDER_FRONT_FORCE_C2) +
                              (x * x * x * REGEN_RYDER_FRONT_FORCE_C3);
    const float4 rear_term = y * REGEN_RYDER_REAR_FORCE_C1;
    const float4 torque = GetConfiguredRyderMu() * (front_term - rear_term) * REGEN_RYDER_TORQUE_SCALE;

    if (torque <= 0.0f) {
        return 0;
    }

    return (sbyte2)(torque + 0.5f);
}

static sbyte2 CalculateRyderRegenTorque(const BSE_Data_t* const front_bse,
                                        const MOBO_PowerTelemetry_RX_Data_t* const mobo_power)
{
    if ((front_bse == NULL) || (mobo_power == NULL)) {
        torque_data.regen_block_reason = REGEN_BLOCK_FRONT_INVALID;
        return 0;
    }

    if (!front_bse->valid) {
        torque_data.regen_block_reason = REGEN_BLOCK_FRONT_INVALID;
        return 0;
    }

    if (!CAN_Manager_RX_Data_Valid(CAN_RX_MSG_MOBO_POWER_TELEMETRY)) {
        torque_data.regen_block_reason = REGEN_BLOCK_REAR_INVALID;
        return 0;
    }

    if (front_bse->psi > REGEN_RYDER_MAX_FRONT_PSI) {
        torque_data.regen_block_reason = REGEN_BLOCK_FRONT_PRESSURE_HIGH;
        return 0;
    }

    const sbyte2 front_table_torque = LookupRyderFrontTorque(front_bse->psi);
    const sbyte2 balance_torque = CalculateRyderBalanceTorque(front_bse->psi,
                                                              mobo_power->rear_brake_pressure_psi_x10);
    sbyte2 requested_torque = balance_torque;
    const sbyte2 configured_max_torque = GetParam(RUNTIME_PARAM_REGEN_MAX_TORQUE);

    torque_data.regen_front_table_torque = front_table_torque;
    torque_data.regen_balance_torque = balance_torque;

    if (front_table_torque <= 0) {
        torque_data.regen_block_reason = REGEN_BLOCK_RYDER_TABLE_ZERO;
        return 0;
    }

    if (balance_torque <= 0) {
        torque_data.regen_block_reason = REGEN_BLOCK_RYDER_BALANCE_ZERO;
        return 0;
    }

    if (requested_torque > front_table_torque) {
        requested_torque = front_table_torque;
    }

    if (requested_torque > configured_max_torque) {
        requested_torque = configured_max_torque;
    }

    if (requested_torque < 0) {
        requested_torque = 0;
    }

    torque_data.regen_final_torque = requested_torque;

    if (requested_torque == 0) {
        torque_data.regen_block_reason = REGEN_BLOCK_ZERO_AFTER_CLAMPS;
    }

    return requested_torque;
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

    sbyte2 strategy = torque_data.regen_strategy;
    const ubyte2 front_pressure = front_bse->psi;
    const ubyte2 rear_pressure = RearBrakePressurePsi(mobo_power);
    const sbyte2 front_min = GetParam(RUNTIME_PARAM_REGEN_MIN_BSE_FRONT_PSI);
    const sbyte2 rear_min = GetParam(RUNTIME_PARAM_REGEN_MIN_BSE_REAR_PSI);
    const sbyte2 front_max = GetParam(RUNTIME_PARAM_REGEN_MAX_BSE_FRONT_PSI);
    const sbyte2 rear_max = GetParam(RUNTIME_PARAM_REGEN_MAX_BSE_REAR_PSI);
    const bool rear_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_MOBO_POWER_TELEMETRY);

    if (strategy > REGEN_STRATEGY_RYDER) {
        strategy = REGEN_STRATEGY_DEFAULT;
    }

    if (strategy == REGEN_STRATEGY_RYDER) {
        return FALSE;
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

static sbyte2 CalculateRegenTorque(const APPS_Data_t* const apps,
                                   const BSE_Data_t* const front_bse,
                                   const MOBO_PowerTelemetry_RX_Data_t* const mobo_power,
                                   const sbyte2 motor_speed)
{
    (void)motor_speed;

    ubyte2 pressure_psi = 0;
    sbyte2 min_pressure_psi = 0;
    sbyte2 max_pressure_psi = 0;
    const ubyte1 strategy = torque_data.regen_strategy;
    const HVCSOC_RX_Data_t* const hvc_soc = CAN_RX_GetHVCSOCData();
    const bool hvc_soc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SOC);
    const ubyte4 regen_max_soc_x100 = (ubyte4)(GetParam(RUNTIME_PARAM_REGEN_MAX_SOC) * 100);

    if (!RuntimeConfig_GetRegenEnabled()) {
        torque_data.regen_block_reason = REGEN_BLOCK_DISABLED;
        return 0;
    }

    if (apps->apps_value > GetConfiguredRegenMaxApps()) {
        torque_data.regen_block_reason = REGEN_BLOCK_APPS_ACTIVE;
        return 0;
    }

    if (torque_data.regen_soc_gate_enabled &&
        hvc_soc_valid &&
        ((ubyte4)hvc_soc->pack_soc_percent_x100 > regen_max_soc_x100)) {
        torque_data.regen_block_reason = REGEN_BLOCK_SOC_HIGH;
        return 0;
    }

    if ((torque_data.regen_status_flags & REGEN_STATUS_FLAG_SPEED_OK) == 0U) {
        torque_data.regen_block_reason = REGEN_BLOCK_BELOW_MIN_SPEED;
        return 0;
    }

    if (strategy == REGEN_STRATEGY_RYDER) {
        const sbyte2 positive_torque = CalculateRyderRegenTorque(front_bse, mobo_power);
        if (positive_torque > 0) {
            torque_data.regen_status_flags |= REGEN_STATUS_FLAG_REGEN_ACTIVE;
        }
        return (sbyte2)(-positive_torque);
    }

    if (!GetRegenPressure(front_bse, mobo_power,
                          &pressure_psi,
                          &min_pressure_psi,
                          &max_pressure_psi)) {
        torque_data.regen_block_reason = REGEN_BLOCK_LEGACY_PRESSURE_INVALID;
        return 0;
    }

    const sbyte2 positive_torque = InterpolateRegenTorque(pressure_psi,
                                                          min_pressure_psi,
                                                          max_pressure_psi);
    torque_data.regen_final_torque = positive_torque;

    if (positive_torque > 0) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_REGEN_ACTIVE;
    }
    else {
        torque_data.regen_block_reason = REGEN_BLOCK_ZERO_AFTER_CLAMPS;
    }

    return (sbyte2)(-positive_torque);
}

static ubyte4 ComputeSpeedMPHx100(sbyte2 motor_rpm)
{
    /* Fixed conversion: 1 RPM = 0.0152 MPH.
     * Store as mph x100, so the scale factor becomes 1.52.
     */
    return ((ubyte4)AbsS16ToU16(motor_rpm) * 152UL) / 100UL;
}

void TorqueController_Init(void)
{
    torque_data.inv_torque_scaled = 0;
    torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
    torque_data.inv_enable = INVERTER_DISABLE;
    torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
    torque_data.speed_mph_x100 = 0;
    torque_data.regen_torque = 0;
    UpdateRegenDebugInputs(VCU_STATE_NOT_READY, NULL, NULL, 0);
    TractionControl_Init();
    LaunchControl_Init();
}

void TorqueController_Update(void)
{
    const VCU_State_t state = StateMachine_GetState();
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();
    const InverterHighSpeed_RX_Data_t* inv_data = CAN_RX_GetInverterHighSpeedData();
    const InverterMotorPosition_RX_Data_t* inv_position = CAN_RX_GetInverterMotorPositionData();
    const MOBO_PowerTelemetry_RX_Data_t* mobo_power = CAN_RX_GetMOBO_PowerTelemetryData(); // Rear BSE PSI is here
    const sbyte2 regen_motor_speed = inv_position->motor_speed;

    /* Compute vehicle speed from inverter motor RPM using a fixed mph/rpm ratio. */
    torque_data.speed_mph_x100 = ComputeSpeedMPHx100(inv_data->motor_speed);
    UpdateRegenDebugInputs(state, bse, mobo_power, regen_motor_speed);

    if (state != VCU_STATE_DRIVING) {
        (void)TractionControl_ApplyLimit(0, inv_data->motor_speed);
        torque_data.inv_torque_scaled = 0;
        torque_data.regen_torque = 0;
        torque_data.regen_block_reason = REGEN_BLOCK_NOT_DRIVING;
        torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
        torque_data.inv_enable = INVERTER_DISABLE;
        torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
        return;
    }

    // TODO should we compute this always for debug reasons or put it in the else?
    torque_data.apps_torque = PedalTravelToTorque(apps->apps_value);


    // TODO all this logic will have to be improved with launch control
    torque_data.regen_torque = CalculateRegenTorque(apps, bse, mobo_power, regen_motor_speed);
    if (torque_data.regen_torque < 0) {
        (void)TractionControl_ApplyLimit(0, inv_data->motor_speed);
        torque_data.inv_torque_scaled = torque_data.regen_torque * 10;
    } else {
        sbyte2 launch_torque = 0;
        sbyte2 limited_torque;
        if (BrakeThrottleCutActive(bse)) {
            (void)TractionControl_ApplyLimit(0, inv_data->motor_speed);
            limited_torque = 0;
        } else if (LaunchControl_GetTorque(torque_data.apps_torque,
                                           &launch_torque)) {
            /* Launch control owns torque this cycle (already pedal-bounded).
             * Refresh traction-control telemetry but use the launch curve value
             * directly so the limiter does not fight the launch curve. */
            (void)TractionControl_ApplyLimit(launch_torque, inv_data->motor_speed);
            limited_torque = launch_torque;
        } else {
            limited_torque = TractionControl_ApplyLimit(torque_data.apps_torque,
                                                        inv_data->motor_speed);
        }
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
