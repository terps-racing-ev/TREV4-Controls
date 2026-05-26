#include "IO_Constants.h"
#include "IO_RTC.h"

#include "traction_control.h"

#include "can/can_manager.h"
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "config/torque_config.h"

static TractionControl_Data_t traction_data;

static float4 error_sum;
static float4 last_error;
static ubyte4 last_error_timestamp;
static bool timestamp_initialized;

static sbyte2 GetParam(const RuntimeParamId_t param_id)
{
    sbyte2 value = 0;
    (void)RuntimeConfig_GetI32(param_id, &value);
    return value;
}

static float4 GetParamX1000(const RuntimeParamId_t param_id)
{
    return ((float4)GetParam(param_id)) / 1000.0f;
}

static void ResetPid(void)
{
    error_sum = 0.0f;
    last_error = 0.0f;
    last_error_timestamp = 0;
    timestamp_initialized = FALSE;
}

static ubyte2 ClampU16FromFloat(const float4 value)
{
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 65535.0f) {
        return 65535;
    }
    return (ubyte2)value;
}

void TractionControl_Init(void)
{
    traction_data = (TractionControl_Data_t){0};
    ResetPid();
}

sbyte2 TractionControl_ApplyLimit(const sbyte2 requested_torque_nm,
                                  const sbyte2 motor_rpm)
{
    const FrontWheelRpm_RX_Data_t* const fl = CAN_RX_GetFrontLeftRpmData();
    const FrontWheelRpm_RX_Data_t* const fr = CAN_RX_GetFrontRightRpmData();

    const sbyte2 enabled_param = GetParam(RUNTIME_PARAM_TRACTION_CONTROL_ENABLED);
    const sbyte2 min_front_rpm = GetParam(RUNTIME_PARAM_TRACTION_CONTROL_MIN_FRONT_RPM);

    const sbyte4 motor_rpm_abs = (motor_rpm < 0) ? (-(sbyte4)motor_rpm) : (sbyte4)motor_rpm;
    const float4 rear_wheel_rpm_f = ((float4)motor_rpm_abs) / GEAR_RATIO;
    const ubyte2 avg_front_rpm = (ubyte2)(((ubyte4)fl->rpm + (ubyte4)fr->rpm) / 2UL);

    traction_data.enabled = (enabled_param != 0);
    traction_data.front_left_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_FRONT_LEFT_RPM);
    traction_data.front_right_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_FRONT_RIGHT_RPM);
    traction_data.rear_wheel_rpm = ClampU16FromFloat(rear_wheel_rpm_f);
    traction_data.avg_front_rpm = avg_front_rpm;
    traction_data.torque_limit_nm = requested_torque_nm;

    const bool inputs_ready = traction_data.front_left_valid &&
                              traction_data.front_right_valid;
    const bool should_run = traction_data.enabled &&
                            inputs_ready &&
                            (requested_torque_nm > 0) &&
                            (avg_front_rpm >= (ubyte2)min_front_rpm);

    if (!should_run) {
        traction_data.active = FALSE;
        traction_data.slip_ratio_x1000 = 0;
        ResetPid();
        return requested_torque_nm;
    }

    const float4 slip_ratio = rear_wheel_rpm_f / (float4)avg_front_rpm;
    const float4 target_slip = GetParamX1000(RUNTIME_PARAM_TRACTION_CONTROL_TARGET_SLIP_X1000);
    const float4 kp = GetParamX1000(RUNTIME_PARAM_TRACTION_CONTROL_KP_X1000);
    const float4 ki = GetParamX1000(RUNTIME_PARAM_TRACTION_CONTROL_KI_X1000);
    const float4 kd = GetParamX1000(RUNTIME_PARAM_TRACTION_CONTROL_KD_X1000);
    const float4 error = target_slip - slip_ratio;

    float4 integral_term = 0.0f;
    float4 derivative_term = 0.0f;

    if (timestamp_initialized) {
        const ubyte4 dt_us = IO_RTC_GetTimeUS(last_error_timestamp);
        if (dt_us > 0) {
            const float4 dt_s = ((float4)dt_us) / 1000000.0f;
            error_sum += error * dt_s;
            integral_term = ki * error_sum;
            derivative_term = kd * ((error - last_error) / dt_s);
        }
    }
    else {
        timestamp_initialized = TRUE;
    }

    last_error = error;
    IO_RTC_StartTime(&last_error_timestamp);

    const float4 pid_output = (kp * error) + integral_term + derivative_term;
    float4 torque_limit_f = ((float4)requested_torque_nm) + pid_output;

    if (torque_limit_f < 0.0f) {
        torque_limit_f = 0.0f;
    }
    if (torque_limit_f > (float4)requested_torque_nm) {
        torque_limit_f = (float4)requested_torque_nm;
    }

    traction_data.active = TRUE;
    traction_data.slip_ratio_x1000 = ClampU16FromFloat(slip_ratio * 1000.0f);
    traction_data.torque_limit_nm = (sbyte2)torque_limit_f;

    return traction_data.torque_limit_nm;
}

const TractionControl_Data_t* TractionControl_GetData(void)
{
    return &traction_data;
}
