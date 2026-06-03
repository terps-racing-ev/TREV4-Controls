#ifndef TRACTION_CONTROL_STATE_MACHINE_H
#define TRACTION_CONTROL_STATE_MACHINE_H

#include "IO_Constants.h"

/*
 * Traction Control State Machine (TRC layer).
 *
 * An independent overlay above the core VCU_STATE machine. It owns all
 * traction-related modes, including the Launch Control Learning sub-mode.
 * The core state_machine.c is left untouched; this layer only ever REDUCES or
 * shapes torque and is always bounded by the driver pedal request.
 */

typedef enum {
    TRC_STATE_OFF = 0,                  /* traction layer idle, pure pass-through */
    TRC_STATE_ACTIVE,                   /* closed-loop traction (existing PID limiter) */
    TRC_STATE_LAUNCH_IDLE,              /* actual launch mode entered; waiting for dedicated arm */
    TRC_STATE_LAUNCH,                   /* actual launch armed/active using configured curve */
    TRC_STATE_LEARNING_IDLE,            /* learning entered; waiting for CAN arm of run 1 */
    TRC_STATE_LEARNING_READY_RUN1,      /* curve A selected; armed and waiting for throttle start */
    TRC_STATE_LEARNING_RUNNING_RUN1,
    TRC_STATE_LEARNING_COMPLETE_RUN1,
    TRC_STATE_LEARNING_READY_RUN2,      /* curve B selected; armed and waiting for throttle start */
    TRC_STATE_LEARNING_RUNNING_RUN2,
    TRC_STATE_LEARNING_COMPLETE_RUN2,
    TRC_STATE_LEARNING_READY_RUN3,      /* curve C selected; armed and waiting for throttle start */
    TRC_STATE_LEARNING_RUNNING_RUN3,
    TRC_STATE_LEARNING_PROCESSING,
    TRC_STATE_LEARNING_RESULTS_READY,
    TRC_STATE_LEARNING_ABORTED,
    TRC_STATE_FAULT,
} TrcState_t;

/* Commands delivered over CAN via the SET_VCU_CONFIG mux (no dedicated RX id). */
typedef enum {
    TRC_CMD_NONE = 0,
    TRC_CMD_SET_ACTIVE,        /* OFF -> ACTIVE */
    TRC_CMD_SET_OFF,           /* ACTIVE/learning -> OFF */
    TRC_CMD_ENTER_LEARNING,    /* OFF/ACTIVE -> LEARNING_IDLE */
    TRC_CMD_ARM,               /* IDLE -> READY_RUN1, COMPLETE_RUN1 -> READY_RUN2, COMPLETE_RUN2 -> READY_RUN3 */
    TRC_CMD_ABORT,             /* any -> ABORTED */
    TRC_CMD_EXIT_LEARNING,     /* RESULTS_READY/ABORTED -> OFF */
    TRC_CMD_ENTER_LAUNCH,      /* OFF/ACTIVE -> LAUNCH_IDLE */
    TRC_CMD_ARM_LAUNCH,        /* LAUNCH_IDLE -> LAUNCH */
    TRC_CMD_EXIT_LAUNCH,       /* LAUNCH_IDLE/LAUNCH -> OFF */
} TrcCommand_t;

typedef struct {
    TrcState_t state;
    ubyte1 current_run;            /* 0 none, 1, 2, 3 */
    ubyte1 selected_curve;         /* 0 = Curve A, 1 = Curve B, 2 = Curve C, 3 = uploaded curve */
    bool   learning_active;
    bool   launch_torque_active;   /* TRUE while applying a launch curve this cycle */
    bool   launch_mode_active;     /* TRUE while actual launch mode is armed or active */

    ubyte1 best_curve;
    sbyte2 grip_score_a_x1000;
    sbyte2 grip_score_b_x1000;
    sbyte2 grip_score_c_x1000;
    sbyte2 recommended_slip_x1000;
} TrcStateMachine_Data_t;

void TractionControlSM_Init(void);
void TractionControlSM_Update(void);
void TractionControlSM_HandleCommand(TrcCommand_t cmd);
const TrcStateMachine_Data_t* TractionControlSM_GetData(void);

/*
 * Torque-path hook (called from torque_controller).
 * When launch control is actively running a run, writes the pedal-bounded launch
 * torque to *out_torque and returns TRUE. Otherwise returns FALSE and the caller
 * uses its normal torque path (e.g. TractionControl_ApplyLimit).
 */
bool TractionControlSM_GetLaunchTorque(sbyte2 requested_torque_nm,
                                       sbyte2 motor_rpm,
                                       sbyte2* out_torque);

#endif // TRACTION_CONTROL_STATE_MACHINE_H
