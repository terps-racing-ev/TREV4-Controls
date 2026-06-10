#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include "IO_Constants.h"

/*
 * Launch Control (time-based torque curve).
 *
 * The car no longer has front wheel-speed sensors, so launch torque is no
 * longer slip-controlled. Instead launch follows a fixed, preprogrammed
 * torque-vs-time curve:
 *
 *   t < LAUNCH_OFFTHELINE_TIME_S : torque = torque_offtheline   (e.g. 100 Nm)
 *   t in [offtheline .. 3.0] s   : torque = torque_init + t^2 * (torque_final - torque_init) / 3^2
 *   t > 3.0 s                    : torque = torque_final         (e.g. 183 Nm)
 *
 * torque_offtheline, torque_init and torque_final are runtime-tunable (Nm).
 *
 * Sequence:
 *   1. The pit/dash sends an ARM command over CAN (SET_VCU_CONFIG launch mux).
 *   2. Once ARMED, a launch triggers the moment the driver is OFF the brakes
 *      and APPS rises above LAUNCH_TRIGGER_APPS_PERCENT (25%).
 *   3. While LAUNCHING the curve drives torque (bounded by the pedal request).
 *   4. The launch ends (and must be re-armed) when the driver brakes, lifts
 *      below LAUNCH_END_APPS_PERCENT (5%), or the car leaves DRIVING.
 *
 * The curve output is always bounded by the driver's pedal-mapped torque, so
 * APPS / FSAE torque-plausibility safety is preserved.
 */

typedef enum {
    LAUNCH_STATE_DISARMED = 0,  /* idle; pure pass-through, needs a CAN arm */
    LAUNCH_STATE_ARMED,         /* armed; waiting for off-brakes + throttle > 25% */
    LAUNCH_STATE_LAUNCHING,     /* launch in progress, following the time curve */
} LaunchState_t;

/* Commands delivered over CAN via the SET_VCU_CONFIG launch-command mux. */
typedef enum {
    LAUNCH_CMD_NONE = 0,
    LAUNCH_CMD_ARM = 1,         /* DISARMED -> ARMED */
    LAUNCH_CMD_DISARM = 2,      /* any -> DISARMED */
} LaunchCommand_t;

typedef struct {
    LaunchState_t state;
    bool   armed;               /* TRUE in ARMED or LAUNCHING */
    bool   active;              /* TRUE while the curve is driving torque */
    ubyte4 elapsed_ms;          /* time since launch trigger (0 unless LAUNCHING) */
    sbyte2 curve_torque_nm;     /* most recent curve torque (pre pedal-bound) */
} LaunchControl_Data_t;

void LaunchControl_Init(void);

/* Handle a launch command (LaunchCommand_t) delivered over CAN. */
void LaunchControl_HandleCommand(ubyte1 cmd);

/* Run once per cycle, before the torque controller. Advances the launch state
 * machine and recomputes the curve torque. */
void LaunchControl_Update(void);

/*
 * Torque-path hook (called from torque_controller).
 * When a launch is active this cycle, writes the pedal-bounded curve torque to
 * *out_torque and returns TRUE. Otherwise returns FALSE and the caller uses its
 * normal pedal-based torque path.
 */
bool LaunchControl_GetTorque(sbyte2 pedal_torque_nm, sbyte2* out_torque);

const LaunchControl_Data_t* LaunchControl_GetData(void);

#endif // LAUNCH_CONTROL_H
