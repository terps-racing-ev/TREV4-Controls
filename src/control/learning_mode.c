#include "IO_Constants.h"

#include "learning_mode.h"

#include "can/can_manager.h"
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "config/torque_config.h"

static LearningMode_Data_t learning_data;

/* Compute the slip ratio (x1000) from motor speed and front wheel speeds.
 * Mirrors the metric used in traction_control.c:
 *   rear_wheel_rpm = |motor_rpm| / GEAR_RATIO
 *   slip_ratio     = rear_wheel_rpm / avg_front_rpm
 * Returns 0 when inputs are not usable (standstill / invalid wheel data). */
static ubyte2 ComputeSlipX1000(const sbyte2 motor_rpm)
{
    const bool fl_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_FRONT_LEFT_RPM);
    const bool fr_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_FRONT_RIGHT_RPM);

    if (!fl_valid || !fr_valid) {
        return 0;
    }

    const FrontWheelRpm_RX_Data_t* const fl = CAN_RX_GetFrontLeftRpmData();
    const FrontWheelRpm_RX_Data_t* const fr = CAN_RX_GetFrontRightRpmData();

    const ubyte2 avg_front_rpm = (ubyte2)(((ubyte4)fl->rpm + (ubyte4)fr->rpm) / 2UL);
    if (avg_front_rpm == 0) {
        return 0;
    }

    const sbyte4 motor_rpm_abs = (motor_rpm < 0) ? (-(sbyte4)motor_rpm) : (sbyte4)motor_rpm;
    const float4 rear_wheel_rpm_f = ((float4)motor_rpm_abs) / GEAR_RATIO;
    const float4 slip_ratio = rear_wheel_rpm_f / (float4)avg_front_rpm;

    float4 slip_x1000_f = slip_ratio * 1000.0f;
    if (slip_x1000_f < 0.0f) {
        slip_x1000_f = 0.0f;
    }
    if (slip_x1000_f > 65535.0f) {
        slip_x1000_f = 65535.0f;
    }

    return (ubyte2)slip_x1000_f;
}

void LearningMode_Init(void)
{
    learning_data = (LearningMode_Data_t){0};
}

void LearningMode_StartRun(const ubyte1 run_number)
{
    if ((run_number < 1) || (run_number > LEARNING_RUN_COUNT)) {
        return;
    }

    const ubyte1 idx = (ubyte1)(run_number - 1);
    learning_data.peak_slip_x1000[idx] = 0;
    learning_data.avg_slip_x1000[idx] = 0;
    learning_data.sample_count[idx] = 0;
    learning_data.slip_sum_x1000[idx] = 0;
    learning_data.current_run = run_number;
    learning_data.active = TRUE;
}

void LearningMode_StopRun(void)
{
    const ubyte1 run = learning_data.current_run;
    if ((run >= 1) && (run <= LEARNING_RUN_COUNT)) {
        const ubyte1 idx = (ubyte1)(run - 1);
        if (learning_data.sample_count[idx] > 0) {
            learning_data.avg_slip_x1000[idx] =
                (ubyte2)(learning_data.slip_sum_x1000[idx] /
                         learning_data.sample_count[idx]);
        }
    }

    learning_data.active = FALSE;
    learning_data.current_run = 0;
}

ubyte2 LearningMode_Update(const sbyte2 motor_rpm)
{
    if (!learning_data.active) {
        return 0;
    }

    const ubyte1 run = learning_data.current_run;
    if ((run < 1) || (run > LEARNING_RUN_COUNT)) {
        return 0;
    }
    const ubyte1 idx = (ubyte1)(run - 1);

    const ubyte2 slip_x1000 = ComputeSlipX1000(motor_rpm);
    learning_data.last_slip_x1000 = slip_x1000;

    if (slip_x1000 == 0) {
        return 0;
    }

    if (slip_x1000 > learning_data.peak_slip_x1000[idx]) {
        learning_data.peak_slip_x1000[idx] = slip_x1000;
    }

    learning_data.slip_sum_x1000[idx] += (ubyte4)slip_x1000;
    learning_data.sample_count[idx]++;
    learning_data.avg_slip_x1000[idx] =
        (ubyte2)(learning_data.slip_sum_x1000[idx] /
                 learning_data.sample_count[idx]);

    return slip_x1000;
}

/* Relative grip score (x1000). Higher = better (less average wheelspin).
 * score = 1000 - (avg_slip_ratio - 1.0), clamped to [0, 1000].
 * This is a RELATIVE indicator between the tested curves, not an absolute mu. */
static sbyte2 GripScoreX1000(const ubyte1 idx)
{
    if (learning_data.sample_count[idx] == 0) {
        return 0;
    }

    const sbyte4 avg = (sbyte4)learning_data.avg_slip_x1000[idx];
    sbyte4 excess = avg - 1000; /* slip above 1.0 */
    if (excess < 0) {
        excess = 0;
    }

    sbyte4 score = 1000 - excess;
    if (score < 0) {
        score = 0;
    }
    return (sbyte2)score;
}

void LearningMode_ProcessResults(void)
{
    ubyte1 best_idx = 0;
    bool any_curve_ran = FALSE;
    bool found_safe_curve = FALSE;

    sbyte2 max_slip_x1000 = TRC_LAUNCH_MAX_SLIP_X1000_DEFAULT;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_TRC_LAUNCH_MAX_SLIP_X1000, &max_slip_x1000);

    for (ubyte1 i = 0; i < LEARNING_RUN_COUNT; i++) {
        learning_data.grip_score_x1000[i] = GripScoreX1000(i);

        if (learning_data.sample_count[i] == 0) {
            continue;
        }

        if (!any_curve_ran) {
            any_curve_ran = TRUE;
            best_idx = i;
        }

        if (learning_data.peak_slip_x1000[i] <= (ubyte2)max_slip_x1000) {
            best_idx = i;
            found_safe_curve = TRUE;
        }
    }

    if (!any_curve_ran) {
        best_idx = 0;
    } else if (!found_safe_curve) {
        /* No tested curve stayed below the safety ceiling; retain the first run
         * as a conservative fallback instead of selecting a more aggressive one. */
        best_idx = 0;
    }

    learning_data.best_curve = best_idx;

    ubyte2 recommended = learning_data.avg_slip_x1000[best_idx];
    if (recommended < 1000) {
        recommended = 1000;
    }
    learning_data.recommended_slip_x1000 = (sbyte2)recommended;

    (void)RuntimeConfig_Set(RUNTIME_PARAM_TRC_LAUNCH_BEST_CURVE,
                            (sbyte2)learning_data.best_curve);
    (void)RuntimeConfig_Set(RUNTIME_PARAM_TRC_LAUNCH_RECOMMENDED_SLIP_X1000,
                            learning_data.recommended_slip_x1000);
}

const LearningMode_Data_t* LearningMode_GetData(void)
{
    return &learning_data;
}
