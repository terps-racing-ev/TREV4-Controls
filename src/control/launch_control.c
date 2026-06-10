#include "IO_Constants.h"
#include "IO_RTC.h"

#include "launch_control.h"

#include "config/apps_config.h"
#include "config/runtime_config.h"
#include "config/torque_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "state_machine.h"

static LaunchControl_Data_t launch_data;
static ubyte4 launch_start_ts;

static sbyte2 GetParam(const RuntimeParamId_t param_id)
{
    sbyte2 value = 0;
    (void)RuntimeConfig_GetI32(param_id, &value);
    return value;
}

/* Evaluate the preprogrammed torque-vs-time curve at the given elapsed time. */
static sbyte2 ComputeCurveTorque(const ubyte4 elapsed_us)
{
    const sbyte2 torque_offtheline = GetParam(RUNTIME_PARAM_LAUNCH_TORQUE_OFFTHELINE);
    const sbyte2 torque_init = GetParam(RUNTIME_PARAM_LAUNCH_TORQUE_INIT);
    const sbyte2 torque_final = GetParam(RUNTIME_PARAM_LAUNCH_TORQUE_FINAL);
    const ubyte1 max_torque = RuntimeConfig_GetMaxTorque();

    const float4 offtheline_s = (float4)GetParam(RUNTIME_PARAM_LAUNCH_OFFTHELINE_TIME_MS) / 1000.0f;
    const float4 duration_s   = (float4)GetParam(RUNTIME_PARAM_LAUNCH_CURVE_DURATION_MS)  / 1000.0f;

    const float4 t_s = ((float4)elapsed_us) / 1000000.0f;
    float4 torque_f;

    if (t_s < offtheline_s) {
        /* Fixed off-the-line torque for the initial hold window. */
        torque_f = (float4)torque_offtheline;
    }
    else {
        /* Parabolic ramp: torque = init + t^2 * (final - init) / duration^2,
         * held flat at torque_final once past the curve duration. */
        float4 t_curve = t_s;
        if (t_curve > duration_s) {
            t_curve = duration_s;
        }
        torque_f = (float4)torque_init +
                   ((t_curve * t_curve) * (float4)(torque_final - torque_init)) /
                   (duration_s * duration_s);
    }

    if (torque_f < 0.0f) {
        torque_f = 0.0f;
    }
    if (torque_f > (float4)max_torque) {
        torque_f = (float4)max_torque;
    }

    return (sbyte2)(torque_f + 0.5f);
}

void LaunchControl_Init(void)
{
    launch_data = (LaunchControl_Data_t){0};
    launch_data.state = LAUNCH_STATE_DISARMED;
    launch_start_ts = 0;
}

void LaunchControl_HandleCommand(const ubyte1 cmd)
{
    switch ((LaunchCommand_t)cmd) {
        case LAUNCH_CMD_ARM:
            /* Only arm from a clean idle so an in-progress launch is never
             * re-armed mid-run. */
            if (launch_data.state == LAUNCH_STATE_DISARMED) {
                launch_data.state = LAUNCH_STATE_ARMED;
                launch_data.active = FALSE;
                launch_data.elapsed_ms = 0;
                launch_data.curve_torque_nm = 0;
            }
            break;

        case LAUNCH_CMD_DISARM:
            launch_data.state = LAUNCH_STATE_DISARMED;
            launch_data.active = FALSE;
            launch_data.elapsed_ms = 0;
            launch_data.curve_torque_nm = 0;
            break;

        case LAUNCH_CMD_NONE:
        default:
            break;
    }

    launch_data.armed = (launch_data.state != LAUNCH_STATE_DISARMED);
}

void LaunchControl_Update(void)
{
    const VCU_State_t vcu_state = StateMachine_GetState();
    const APPS_Data_t* const apps = APPS_GetData();
    const BSE_Data_t* const bse = BSE_GetData();

    const ubyte2 trigger_threshold =
        (ubyte2)(((ubyte4)APPS_RESOLUTION *
                  (ubyte4)GetParam(RUNTIME_PARAM_LAUNCH_TRIGGER_APPS_PERCENT)) / 100UL);
    const ubyte2 end_threshold =
        (ubyte2)(((ubyte4)APPS_RESOLUTION *
                  (ubyte4)GetParam(RUNTIME_PARAM_LAUNCH_END_APPS_PERCENT)) / 100UL);

    const bool driving = (vcu_state == VCU_STATE_DRIVING);
    const bool brakes_on = bse->brakes_engaged;
    const bool apps_ok = apps->valid;

    switch (launch_data.state) {
        case LAUNCH_STATE_DISARMED:
            launch_data.active = FALSE;
            launch_data.elapsed_ms = 0;
            launch_data.curve_torque_nm = 0;
            break;

        case LAUNCH_STATE_ARMED:
            launch_data.active = FALSE;
            launch_data.elapsed_ms = 0;
            launch_data.curve_torque_nm = 0;
            /* Trigger once the driver is off the brakes with valid pedal > 25%. */
            if (driving && apps_ok && !brakes_on &&
                (apps->apps_value > trigger_threshold)) {
                IO_RTC_StartTime(&launch_start_ts);
                launch_data.state = LAUNCH_STATE_LAUNCHING;
                launch_data.active = TRUE;
            }
            break;

        case LAUNCH_STATE_LAUNCHING:
            /* End the run on brake, pedal lift below 5%, or loss of DRIVING. */
            if (!driving || !apps_ok || brakes_on ||
                (apps->apps_value < end_threshold)) {
                launch_data.state = LAUNCH_STATE_DISARMED;
                launch_data.active = FALSE;
                launch_data.elapsed_ms = 0;
                launch_data.curve_torque_nm = 0;
            }
            else {
                const ubyte4 elapsed_us = IO_RTC_GetTimeUS(launch_start_ts);
                launch_data.elapsed_ms = (ubyte4)(elapsed_us / 1000UL);
                launch_data.curve_torque_nm = ComputeCurveTorque(elapsed_us);
                launch_data.active = TRUE;
            }
            break;

        default:
            launch_data.state = LAUNCH_STATE_DISARMED;
            break;
    }

    launch_data.armed = (launch_data.state != LAUNCH_STATE_DISARMED);
}

bool LaunchControl_GetTorque(const sbyte2 pedal_torque_nm, sbyte2* const out_torque)
{
    if (out_torque == NULL) {
        return FALSE;
    }

    if (launch_data.state != LAUNCH_STATE_LAUNCHING) {
        return FALSE;
    }

    sbyte2 torque = launch_data.curve_torque_nm;

    /* Pedal bound: launch torque can never exceed the driver's pedal request. */
    if (torque > pedal_torque_nm) {
        torque = pedal_torque_nm;
    }
    if (torque < 0) {
        torque = 0;
    }

    *out_torque = torque;
    return TRUE;
}

const LaunchControl_Data_t* LaunchControl_GetData(void)
{
    return &launch_data;
}
