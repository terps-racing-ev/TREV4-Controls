#include "IO_Constants.h"
#include "config/torque_config.h"
#include "config/endurance_config.h"
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

    /* Regen is RYDER-only, so this flag is always set while regen is configured. */
    torque_data.regen_status_flags |= REGEN_STATUS_FLAG_RYDER_ACTIVE;

    if (soc_valid) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_SOC_VALID;
    }

    ResetRegenDebugDerived();
}

static sbyte2 PedalTravelToTorque(ubyte2 pedal_travel)
{
    const ubyte2 pedal_travel_for_max_torque = (ubyte2)((((ubyte4)APPS_RESOLUTION) * (ubyte4)PERCENT_TRAVEL_FOR_MAX_TORQUE) / 100);
    ubyte1 max_torque = RuntimeConfig_GetMaxTorque(); // MAX_TORQUE_DEFAULT;

    /* Endurance caps drive torque to a fixed lower value, bounded by Normal's
     * cap so it can never request more torque than Normal. */
    if ((RuntimeConfig_GetDriveMode() == DRIVE_MODE_ENDURANCE) &&
        ((ubyte1)ENDURANCE_TORQUE_CAP_NM < max_torque)) {
        max_torque = (ubyte1)ENDURANCE_TORQUE_CAP_NM;
    }

    if (pedal_travel < APPS_DEADZONE) {
        return 0;
    }

    if (pedal_travel > pedal_travel_for_max_torque) {
        return max_torque;
    }

    return (sbyte2)(((sbyte4)(pedal_travel - APPS_DEADZONE) * (sbyte4)max_torque) /
                    (sbyte4)(pedal_travel_for_max_torque - APPS_DEADZONE));
}

static ubyte2 RearBrakePressurePsi(const MOBO_PowerTelemetry_RX_Data_t* const mobo_power)
{
    return (ubyte2)(((ubyte4)mobo_power->rear_brake_pressure_psi_x10 + 5U) / 10U);
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

/*
 * RYDER regen: torque comes purely from the brake-force balance equation,
 * gated by an adjustable front/rear PSI activation window and clamped to the
 * adjustable [REGEN_MIN_TORQUE, REGEN_MAX_TORQUE] envelope.
 *
 *   - Below MIN front/rear PSI: not braking enough -> no regen.
 *   - Above MAX front/rear PSI: brake-too-hard cutoff (keeps the cubic in its
 *     fitted domain and preserves brake plausibility) -> no regen.
 *   - Inside the window: torque = clamp(equation, MIN_TORQUE, MAX_TORQUE).
 *
 * A non-positive equation result (rear braking exceeds front) yields no regen
 * rather than being floored to MIN_TORQUE, so we never command rear regen the
 * balance model says is not warranted.
 */
static sbyte2 CalculateRyderRegenTorque(const BSE_Data_t* const front_bse,
                                        const MOBO_PowerTelemetry_RX_Data_t* const mobo_power)
{
    if ((front_bse == NULL) || (mobo_power == NULL) || !front_bse->valid) {
        torque_data.regen_block_reason = REGEN_BLOCK_FRONT_INVALID;
        return 0;
    }

    if (!CAN_Manager_RX_Data_Valid(CAN_RX_MSG_MOBO_POWER_TELEMETRY)) {
        torque_data.regen_block_reason = REGEN_BLOCK_REAR_INVALID;
        return 0;
    }

    const ubyte2 front_psi = front_bse->psi;
    const ubyte2 rear_psi = RearBrakePressurePsi(mobo_power);
    const sbyte2 min_front_psi = GetParam(RUNTIME_PARAM_REGEN_MIN_BSE_FRONT_PSI);
    const sbyte2 max_front_psi = GetParam(RUNTIME_PARAM_REGEN_MAX_BSE_FRONT_PSI);
    const sbyte2 min_rear_psi = GetParam(RUNTIME_PARAM_REGEN_MIN_BSE_REAR_PSI);
    const sbyte2 max_rear_psi = GetParam(RUNTIME_PARAM_REGEN_MAX_BSE_REAR_PSI);

    /* Front activation window (front is the plausibility-relevant circuit). */
    if (front_psi < (ubyte2)min_front_psi) {
        torque_data.regen_block_reason = REGEN_BLOCK_FRONT_PRESSURE_LOW;
        return 0;
    }

    if ((max_front_psi > 0) && (front_psi > (ubyte2)max_front_psi)) {
        torque_data.regen_block_reason = REGEN_BLOCK_FRONT_PRESSURE_HIGH;
        return 0;
    }

    /* Rear activation window. */
    if (rear_psi < (ubyte2)min_rear_psi) {
        torque_data.regen_block_reason = REGEN_BLOCK_REAR_PRESSURE_LOW;
        return 0;
    }

    if ((max_rear_psi > 0) && (rear_psi > (ubyte2)max_rear_psi)) {
        torque_data.regen_block_reason = REGEN_BLOCK_REAR_PRESSURE_HIGH;
        return 0;
    }

    const sbyte2 balance_torque = CalculateRyderBalanceTorque(front_psi,
                                                              mobo_power->rear_brake_pressure_psi_x10);
    torque_data.regen_balance_torque = balance_torque;

    if (balance_torque <= 0) {
        torque_data.regen_block_reason = REGEN_BLOCK_BALANCE_ZERO;
        return 0;
    }

    sbyte2 min_torque = GetParam(RUNTIME_PARAM_REGEN_MIN_TORQUE);
    sbyte2 max_torque = GetParam(RUNTIME_PARAM_REGEN_MAX_TORQUE);

    if (max_torque < min_torque) {
        const sbyte2 configured_min_torque = min_torque;
        min_torque = max_torque;
        max_torque = configured_min_torque;
    }

    sbyte2 requested_torque = balance_torque;

    if (requested_torque > max_torque) {
        requested_torque = max_torque;
    }

    if (requested_torque < min_torque) {
        requested_torque = min_torque;
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

static sbyte2 CalculateRegenTorque(const APPS_Data_t* const apps,
                                   const BSE_Data_t* const front_bse,
                                   const MOBO_PowerTelemetry_RX_Data_t* const mobo_power,
                                   const sbyte2 motor_speed)
{
    (void)motor_speed;

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

    const sbyte2 positive_torque = CalculateRyderRegenTorque(front_bse, mobo_power);
    if (positive_torque > 0) {
        torque_data.regen_status_flags |= REGEN_STATUS_FLAG_REGEN_ACTIVE;
    }

    return (sbyte2)(-positive_torque);
}

static float4 GetMPHx100PerMotorRPM(void)
{
    const sbyte2 diameter_inches = GetParam(RUNTIME_PARAM_WHEEL_DIAMETER);
    /* Wheel circumference is pi * diameter. 63360 inches/mile, 60 min/hour.
     * Runtime configuration clamps diameter to 8..30 inches. */
    return (3.14159265358979323846f * (float4)diameter_inches * 6000.0f) /
           (63360.0f * GEAR_RATIO);
}

static ubyte4 ComputeSpeedMPHx100(sbyte2 motor_rpm)
{
    /* Widen before negating to include the signed RPM minimum (-32768). */
    const sbyte4 rpm = (sbyte4)motor_rpm;
    const ubyte4 rpm_abs = (ubyte4)((rpm < 0) ? -rpm : rpm);
    return (ubyte4)((float4)rpm_abs * GetMPHx100PerMotorRPM());
}

static sbyte2 ComputeMotorRPMFromMPH(sbyte2 speed_mph)
{
    const float4 rpm = ((float4)speed_mph * 100.0f) / GetMPHx100PerMotorRPM();
    /* Small wheels can require more RPM than the signed CAN field supports. */
    if (rpm >= 32767.0f) {
        return 32767;
    }
    return (sbyte2)(rpm + 0.5f);
}

void TorqueController_Init(void)
{
    torque_data.inv_torque_scaled = 0;
    torque_data.inv_direction = MOTOR_FORWARDS;//RuntimeConfig_GetMotorDirection();
    torque_data.inv_enable = INVERTER_DISABLE;
    torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
    torque_data.inv_speed_rpm = 0;
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

    /* Compute vehicle speed from inverter motor RPM using configured wheel diameter and gear ratio. */
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
        torque_data.inv_speed_rpm = 0;
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
    /* Latch only after exceeding the configured VCU speed. Falling below the
     * threshold does not release speed mode; releasing the pedal does.
     * Existing brake/sensor/state inhibitions must also cancel the override. */
    const sbyte2 max_speed_mph = GetParam(RUNTIME_PARAM_MAX_SPEED_MPH);
    if ((max_speed_mph <= 0) || (apps->apps_value == 0U) ||
        !apps->valid || BrakeThrottleCutActive(bse) ||
        !CAN_Manager_RX_Data_Valid(CAN_RX_MSG_INV_HIGH_SPEED) ||
        (torque_data.regen_torque < 0)) {
        torque_data.inv_speed_mode = INVERTER_SPEED_DISABLE;
    } else if (torque_data.speed_mph_x100 > ((ubyte4)max_speed_mph * 100UL)) {
        torque_data.inv_speed_mode = TRUE;
    }

    torque_data.inv_speed_rpm = 0;
    if (torque_data.inv_speed_mode) {
        /* Inverse of ComputeSpeedMPHx100, rounded to the nearest RPM. */
        torque_data.inv_speed_rpm = ComputeMotorRPMFromMPH(max_speed_mph);
        /* In speed mode this field is regulator feedforward, not a torque request. */
        torque_data.inv_torque_scaled = 0;
    }

}

const TorqueController_Data_T* TorqueController_GetData(void)
{
    return &torque_data;
}
