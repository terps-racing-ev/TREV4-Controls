#include "IO_Constants.h"
#include "IO_RTC.h"

#include "traction_control_state_machine.h"

#include "learning_mode.h"
#include "can/can_manager.h"
#include "can/can_rx.h"
#include "config/apps_config.h"
#include "config/runtime_config.h"
#include "config/torque_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "state_machine.h"

static TrcStateMachine_Data_t trc_data;
static ubyte4 run_start_ts;
static bool launch_run_started;

static const RuntimeParamId_t uploaded_launch_rpm_param_ids[LAUNCH_CURVE_POINTS] = {
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_RPM_0,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_RPM_1,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_RPM_2,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_RPM_3,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_RPM_4,
};

static const RuntimeParamId_t uploaded_launch_torque_param_ids[LAUNCH_CURVE_POINTS] = {
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_TORQUE_0,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_TORQUE_1,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_TORQUE_2,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_TORQUE_3,
    RUNTIME_PARAM_TRC_LAUNCH_ACTUAL_TORQUE_4,
};

/* Compile-time launch torque-vs-(motor)speed curves. */
static const ubyte2 launch_curve_rpm[LAUNCH_CURVE_POINTS] = LAUNCH_CURVE_RPM_BREAKPOINTS;
static const sbyte2 launch_curve_a[LAUNCH_CURVE_POINTS] = LAUNCH_CURVE_A_TORQUE_NM;
static const sbyte2 launch_curve_b[LAUNCH_CURVE_POINTS] = LAUNCH_CURVE_B_TORQUE_NM;
static const sbyte2 launch_curve_c[LAUNCH_CURVE_POINTS] = LAUNCH_CURVE_C_TORQUE_NM;

static sbyte2 GetParam(const RuntimeParamId_t param_id)
{
    sbyte2 value = 0;
    (void)RuntimeConfig_GetI32(param_id, &value);
    return value;
}

static ubyte2 PedalStartThreshold(void)
{
    return (ubyte2)(((ubyte4)APPS_RESOLUTION *
                     (ubyte4)LAUNCH_START_PEDAL_PERCENT) / 100UL);
}

static sbyte2 InterpolateLaunchCurve(const sbyte2 motor_rpm,
                                     const ubyte2* const rpm_points,
                                     const sbyte2* const torque_points)
{
    const sbyte4 rpm_abs = (motor_rpm < 0) ? (-(sbyte4)motor_rpm) : (sbyte4)motor_rpm;
    const ubyte2 rpm = (ubyte2)rpm_abs; /* |sbyte2| <= 32768, always fits ubyte2 */

    if (rpm <= rpm_points[0]) {
        return torque_points[0];
    }
    if (rpm >= rpm_points[LAUNCH_CURVE_POINTS - 1]) {
        return torque_points[LAUNCH_CURVE_POINTS - 1];
    }

    for (ubyte1 i = 1; i < LAUNCH_CURVE_POINTS; i++) {
        if (rpm <= rpm_points[i]) {
            const ubyte2 rpm_lo = rpm_points[i - 1];
            const ubyte2 rpm_hi = rpm_points[i];
            const sbyte2 t_lo = torque_points[i - 1];
            const sbyte2 t_hi = torque_points[i];
            const sbyte4 span = (sbyte4)rpm_hi - (sbyte4)rpm_lo;
            if (span <= 0) {
                return t_lo;
            }
            return (sbyte2)((sbyte4)t_lo +
                            (((sbyte4)t_hi - (sbyte4)t_lo) *
                             ((sbyte4)rpm - (sbyte4)rpm_lo)) / span);
        }
    }

    return torque_points[LAUNCH_CURVE_POINTS - 1];
}

/* Linear interpolation of the selected launch curve at the given motor RPM. */
static sbyte2 LaunchCurveLookup(const sbyte2 motor_rpm, const ubyte1 curve)
{
    const sbyte2* table = launch_curve_a;

    if (curve == 1) {
        table = launch_curve_b;
    }
    else if (curve == 2) {
        table = launch_curve_c;
    }

    return InterpolateLaunchCurve(motor_rpm, launch_curve_rpm, table);
}

static void LoadUploadedLaunchCurve(ubyte2* const rpm_points,
                                    sbyte2* const torque_points)
{
    for (ubyte1 i = 0; i < LAUNCH_CURVE_POINTS; i++) {
        const sbyte2 rpm_value = GetParam(uploaded_launch_rpm_param_ids[i]);
        sbyte2 torque_value = GetParam(uploaded_launch_torque_param_ids[i]);

        if ((i > 0) && (rpm_value < (sbyte2)rpm_points[i - 1])) {
            rpm_points[i] = rpm_points[i - 1];
        }
        else if (rpm_value < 0) {
            rpm_points[i] = 0;
        }
        else {
            rpm_points[i] = (ubyte2)rpm_value;
        }

        if (torque_value < 0) {
            torque_value = 0;
        }
        torque_points[i] = torque_value;
    }
}

static sbyte2 UploadedLaunchCurveLookup(const sbyte2 motor_rpm)
{
    ubyte2 rpm_points[LAUNCH_CURVE_POINTS];
    sbyte2 torque_points[LAUNCH_CURVE_POINTS];

    LoadUploadedLaunchCurve(rpm_points, torque_points);
    return InterpolateLaunchCurve(motor_rpm, rpm_points, torque_points);
}

static bool IsLearningState(const TrcState_t state)
{
    return (state >= TRC_STATE_LEARNING_IDLE) &&
           (state <= TRC_STATE_LEARNING_ABORTED);
}

static bool IsActualLaunchState(const TrcState_t state)
{
    return (state == TRC_STATE_LAUNCH_IDLE) ||
           (state == TRC_STATE_LAUNCH);
}

static ubyte1 GetConfiguredLaunchCurve(void)
{
    sbyte2 curve = TRC_LAUNCH_ACTIVE_CURVE_DEFAULT;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_TRC_LAUNCH_ACTIVE_CURVE, &curve);

    if (curve < 0) {
        return 0;
    }
    if (curve > 3) {
        return 3;
    }

    return (ubyte1)curve;
}

static void EnterAborted(void)
{
    LearningMode_StopRun();
    trc_data.state = TRC_STATE_LEARNING_ABORTED;
    trc_data.launch_torque_active = FALSE;
    trc_data.launch_mode_active = FALSE;
    trc_data.current_run = 0;
    launch_run_started = FALSE;
}

static void CaptureResults(void)
{
    const LearningMode_Data_t* const ld = LearningMode_GetData();
    trc_data.best_curve = ld->best_curve;
    trc_data.grip_score_a_x1000 = ld->grip_score_x1000[0];
    trc_data.grip_score_b_x1000 = ld->grip_score_x1000[1];
    trc_data.grip_score_c_x1000 = ld->grip_score_x1000[2];
    trc_data.recommended_slip_x1000 = ld->recommended_slip_x1000;
}

void TractionControlSM_Init(void)
{
    trc_data = (TrcStateMachine_Data_t){0};
    trc_data.state = TRC_STATE_OFF;
    run_start_ts = 0;
    launch_run_started = FALSE;
    LearningMode_Init();
}

/* Once armed by CAN, launch starts from throttle only (with brakes released). */
static bool LaunchStartReady(const VCU_State_t vcu_state)
{
    const APPS_Data_t* const apps = APPS_GetData();
    const BSE_Data_t* const bse = BSE_GetData();

    if (vcu_state != VCU_STATE_DRIVING) {
        return FALSE;
    }
    if (!apps->valid) {
        return FALSE;
    }
    if (bse->brakes_engaged) {
        return FALSE;
    }

    if (apps->apps_value < PedalStartThreshold()) {
        return FALSE;
    }

    return TRUE;
}

static void StartLearningRun(const ubyte1 run_number,
                             const ubyte1 curve,
                             const TrcState_t running_state)
{
    LearningMode_StartRun(run_number);
    IO_RTC_StartTime(&run_start_ts);
    trc_data.current_run = run_number;
    trc_data.selected_curve = curve;
    trc_data.state = running_state;
}

static void StopActualLaunch(void)
{
    trc_data.state = TRC_STATE_LAUNCH_IDLE;
    trc_data.current_run = 0;
    trc_data.launch_torque_active = FALSE;
    trc_data.launch_mode_active = FALSE;
    launch_run_started = FALSE;
}

static void CancelActualLaunch(void)
{
    trc_data.state = TRC_STATE_OFF;
    trc_data.current_run = 0;
    trc_data.launch_torque_active = FALSE;
    trc_data.launch_mode_active = FALSE;
    launch_run_started = FALSE;
}

static void ServiceActualLaunch(const VCU_State_t vcu_state,
                                const sbyte2 motor_rpm)
{
    const BSE_Data_t* const bse = BSE_GetData();
    const sbyte2 end_rpm = GetParam(RUNTIME_PARAM_TRC_LAUNCH_END_RPM);
    const ubyte4 timeout_us =
        (ubyte4)GetParam(RUNTIME_PARAM_TRC_LAUNCH_TIMEOUT_MS) * 1000UL;
    const sbyte4 rpm_abs = (motor_rpm < 0) ? (-(sbyte4)motor_rpm) : (sbyte4)motor_rpm;

    if (vcu_state != VCU_STATE_DRIVING) {
        CancelActualLaunch();
        return;
    }

    if (!launch_run_started) {
        if (LaunchStartReady(vcu_state)) {
            IO_RTC_StartTime(&run_start_ts);
            launch_run_started = TRUE;
        }
        return;
    }

    trc_data.launch_torque_active = TRUE;

    if (bse->brakes_engaged ||
        (rpm_abs >= (sbyte4)end_rpm) ||
        (IO_RTC_GetTimeUS(run_start_ts) >= timeout_us)) {
        StopActualLaunch();
    }
}

/* Sample the active run and decide whether it should end (or abort). */
static void ServiceRunningRun(const VCU_State_t vcu_state,
                              const sbyte2 motor_rpm,
                              const TrcState_t complete_state)
{
    const BSE_Data_t* const bse = BSE_GetData();

    /* Safety: lose DRIVING -> abort immediately. */
    if (vcu_state != VCU_STATE_DRIVING) {
        EnterAborted();
        return;
    }

    const ubyte2 slip_x1000 = LearningMode_Update(motor_rpm);

    const sbyte2 max_slip = GetParam(RUNTIME_PARAM_TRC_LAUNCH_MAX_SLIP_X1000);
    const sbyte2 end_rpm = GetParam(RUNTIME_PARAM_TRC_LAUNCH_END_RPM);
    const ubyte4 timeout_us =
        (ubyte4)GetParam(RUNTIME_PARAM_TRC_LAUNCH_TIMEOUT_MS) * 1000UL;

    const sbyte4 rpm_abs = (motor_rpm < 0) ? (-(sbyte4)motor_rpm) : (sbyte4)motor_rpm;
    const ubyte4 elapsed_us = IO_RTC_GetTimeUS(run_start_ts);

    /* Over-slip safety abort. */
    if ((slip_x1000 != 0) && (slip_x1000 > (ubyte2)max_slip)) {
        EnterAborted();
        return;
    }

    /* Normal end conditions: reached target speed, driver braked, or timeout. */
    if ((rpm_abs >= (sbyte4)end_rpm) ||
        bse->brakes_engaged ||
        (elapsed_us >= timeout_us)) {
        LearningMode_StopRun();
        trc_data.state = complete_state;
        trc_data.launch_torque_active = FALSE;
        trc_data.current_run = 0;
    }
}

void TractionControlSM_Update(void)
{
    const VCU_State_t vcu_state = StateMachine_GetState();
    const InverterHighSpeed_RX_Data_t* const inv = CAN_RX_GetInverterHighSpeedData();
    const sbyte2 motor_rpm = inv->motor_speed;

    trc_data.launch_torque_active = FALSE;

    switch (trc_data.state) {
        case TRC_STATE_OFF:
        case TRC_STATE_ACTIVE:
        case TRC_STATE_FAULT:
            /* Nothing time-driven; transitions handled by commands. */
            break;

        case TRC_STATE_LAUNCH_IDLE:
            trc_data.launch_mode_active = TRUE;
            trc_data.selected_curve = GetConfiguredLaunchCurve();
            break;

        case TRC_STATE_LAUNCH:
            trc_data.launch_mode_active = TRUE;
            trc_data.selected_curve = GetConfiguredLaunchCurve();
            ServiceActualLaunch(vcu_state, motor_rpm);
            break;

        case TRC_STATE_LEARNING_IDLE:
        case TRC_STATE_LEARNING_COMPLETE_RUN1:
        case TRC_STATE_LEARNING_COMPLETE_RUN2:
        case TRC_STATE_LEARNING_RESULTS_READY:
        case TRC_STATE_LEARNING_ABORTED:
            /* Waiting for a driver/service command. */
            break;

        case TRC_STATE_LEARNING_READY_RUN1:
            trc_data.selected_curve = 0; /* Curve A */
            if (LaunchStartReady(vcu_state)) {
                StartLearningRun(1, 0, TRC_STATE_LEARNING_RUNNING_RUN1);
            }
            break;

        case TRC_STATE_LEARNING_RUNNING_RUN1:
            trc_data.launch_torque_active = TRUE;
            ServiceRunningRun(vcu_state, motor_rpm, TRC_STATE_LEARNING_COMPLETE_RUN1);
            break;

        case TRC_STATE_LEARNING_READY_RUN2:
            trc_data.selected_curve = 1; /* Curve B */
            if (LaunchStartReady(vcu_state)) {
                StartLearningRun(2, 1, TRC_STATE_LEARNING_RUNNING_RUN2);
            }
            break;

        case TRC_STATE_LEARNING_RUNNING_RUN2:
            trc_data.launch_torque_active = TRUE;
            ServiceRunningRun(vcu_state, motor_rpm, TRC_STATE_LEARNING_COMPLETE_RUN2);
            break;

        case TRC_STATE_LEARNING_READY_RUN3:
            trc_data.selected_curve = 2; /* Curve C */
            if (LaunchStartReady(vcu_state)) {
                StartLearningRun(3, 2, TRC_STATE_LEARNING_RUNNING_RUN3);
            }
            break;

        case TRC_STATE_LEARNING_RUNNING_RUN3:
            trc_data.launch_torque_active = TRUE;
            ServiceRunningRun(vcu_state, motor_rpm, TRC_STATE_LEARNING_PROCESSING);
            break;

        case TRC_STATE_LEARNING_PROCESSING:
            LearningMode_ProcessResults();
            CaptureResults();
            trc_data.state = TRC_STATE_LEARNING_RESULTS_READY;
            break;

        default:
            trc_data.state = TRC_STATE_OFF;
            break;
    }

    trc_data.learning_active = IsLearningState(trc_data.state);
    if (!IsActualLaunchState(trc_data.state)) {
        trc_data.launch_mode_active = FALSE;
    }
}

void TractionControlSM_HandleCommand(const TrcCommand_t cmd)
{
    switch (cmd) {
        case TRC_CMD_SET_ACTIVE:
            if (trc_data.state == TRC_STATE_OFF) {
                trc_data.state = TRC_STATE_ACTIVE;
            }
            break;

        case TRC_CMD_SET_OFF:
            LearningMode_StopRun();
            trc_data.state = TRC_STATE_OFF;
            trc_data.current_run = 0;
            trc_data.launch_torque_active = FALSE;
            trc_data.launch_mode_active = FALSE;
            launch_run_started = FALSE;
            break;

        case TRC_CMD_ENTER_LAUNCH:
            if ((trc_data.state == TRC_STATE_OFF) ||
                (trc_data.state == TRC_STATE_ACTIVE)) {
                LearningMode_StopRun();
                trc_data.current_run = 0;
                trc_data.selected_curve = GetConfiguredLaunchCurve();
                trc_data.state = TRC_STATE_LAUNCH_IDLE;
                trc_data.launch_mode_active = FALSE;
                trc_data.launch_torque_active = FALSE;
                launch_run_started = FALSE;
            }
            break;

        case TRC_CMD_ARM_LAUNCH:
            if (trc_data.state == TRC_STATE_LAUNCH_IDLE) {
                trc_data.selected_curve = GetConfiguredLaunchCurve();
                trc_data.state = TRC_STATE_LAUNCH;
                trc_data.launch_mode_active = TRUE;
                trc_data.launch_torque_active = FALSE;
                launch_run_started = FALSE;
            }
            break;

        case TRC_CMD_ENTER_LEARNING:
            if ((trc_data.state == TRC_STATE_OFF) ||
                (trc_data.state == TRC_STATE_ACTIVE)) {
                launch_run_started = FALSE;
                LearningMode_Init();
                trc_data.current_run = 0;
                trc_data.selected_curve = 0;
                trc_data.launch_mode_active = FALSE;
                trc_data.state = TRC_STATE_LEARNING_IDLE;
            }
            break;

        case TRC_CMD_ARM:
            if (trc_data.state == TRC_STATE_LEARNING_IDLE) {
                trc_data.selected_curve = 0;
                trc_data.state = TRC_STATE_LEARNING_READY_RUN1;
            }
            else if (trc_data.state == TRC_STATE_LEARNING_COMPLETE_RUN1) {
                trc_data.selected_curve = 1;
                trc_data.state = TRC_STATE_LEARNING_READY_RUN2;
            }
            else if (trc_data.state == TRC_STATE_LEARNING_COMPLETE_RUN2) {
                trc_data.selected_curve = 2;
                trc_data.state = TRC_STATE_LEARNING_READY_RUN3;
            }
            break;

        case TRC_CMD_ABORT:
            if (IsActualLaunchState(trc_data.state)) {
                CancelActualLaunch();
            }
            else {
                EnterAborted();
            }
            break;

        case TRC_CMD_EXIT_LEARNING:
            if (IsLearningState(trc_data.state)) {
                LearningMode_StopRun();
                trc_data.state = TRC_STATE_OFF;
                trc_data.current_run = 0;
                trc_data.launch_torque_active = FALSE;
                trc_data.launch_mode_active = FALSE;
            }
            break;

        case TRC_CMD_EXIT_LAUNCH:
            if (IsActualLaunchState(trc_data.state)) {
                CancelActualLaunch();
            }
            break;

        case TRC_CMD_NONE:
        default:
            break;
    }
}

bool TractionControlSM_GetLaunchTorque(const sbyte2 requested_torque_nm,
                                       const sbyte2 motor_rpm,
                                       sbyte2* const out_torque)
{
    if (out_torque == NULL) {
        return FALSE;
    }

    if ((trc_data.state != TRC_STATE_LEARNING_RUNNING_RUN1) &&
        (trc_data.state != TRC_STATE_LEARNING_RUNNING_RUN2) &&
        (trc_data.state != TRC_STATE_LEARNING_RUNNING_RUN3) &&
        !((trc_data.state == TRC_STATE_LAUNCH) && launch_run_started)) {
        return FALSE;
    }

    if (requested_torque_nm <= 0) {
        *out_torque = 0;
        return TRUE;
    }

    sbyte2 curve_torque;

    if (trc_data.selected_curve == 3) {
        curve_torque = UploadedLaunchCurveLookup(motor_rpm);
    }
    else {
        curve_torque = LaunchCurveLookup(motor_rpm, trc_data.selected_curve);
    }

    /* Pedal bound: launch torque can never exceed the driver's pedal request. */
    if (curve_torque > requested_torque_nm) {
        curve_torque = requested_torque_nm;
    }
    if (curve_torque < 0) {
        curve_torque = 0;
    }

    *out_torque = curve_torque;
    return TRUE;
}

const TrcStateMachine_Data_t* TractionControlSM_GetData(void)
{
    return &trc_data;
}
