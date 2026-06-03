#ifndef TORQUE_CONFIG_H
#define TORQUE_CONFIG_H

#include "IO_Constants.h"


// TODO THESE DONT BELONG HERE OR DO THEY
#define MOTOR_FORWARDS 1
#define MOTOR_REVERSE 0

#define INVERTER_DISABLE 0
#define INVERTER_ENABLE 1

#define INVERTER_SPEED_ENABLE 1
#define INVERTER_SPEED_DISABLE 0

/* Drivetrain geometry (fixed). */
#define GEAR_RATIO 3.4545f
#define WHEEL_DIAMETER_DEFAULT 16

/* Default values for runtime-configurable parameters (EEPROM-backed). */
#define MAX_TORQUE_DEFAULT 220

#define PERCENT_TRAVEL_FOR_MAX_TORQUE 90

#define MOTOR_DIRECTION_DEFAULT MOTOR_FORWARDS // backwards for dyno testing

typedef enum {
	REGEN_STRATEGY_FRONT_ONLY = 0,
	REGEN_STRATEGY_REAR_ONLY,
	REGEN_STRATEGY_AVERAGED,
	REGEN_STRATEGY_RYDER,
} RegenStrategy_t;

/* Regen defaults (runtime-configurable, EEPROM-backed). */
#define REGEN_ENABLED_DEFAULT TRUE
#define REGEN_MAX_TORQUE_DEFAULT 80
#define REGEN_MIN_TORQUE_DEFAULT 10
#define REGEN_MIN_BSE_REAR_PSI_DEFAULT 20
#define REGEN_MIN_BSE_FRONT_PSI_DEFAULT 20
#define REGEN_MAX_BSE_REAR_PSI_DEFAULT 900
#define REGEN_MAX_BSE_FRONT_PSI_DEFAULT 900
#define REGEN_MIN_SPEED_DEFAULT 250
#define REGEN_MAX_SOC_DEFAULT 85
#define REGEN_SOC_GATE_ENABLED_DEFAULT TRUE
#define REGEN_STRATEGY_DEFAULT REGEN_STRATEGY_RYDER

/* Ryder regen strategy: front-pressure lookup plus rear-pressure balancing. */
#define REGEN_RYDER_MU                         1.5f
#define REGEN_RYDER_FRONT_TABLE_POINTS         19
#define REGEN_RYDER_FRONT_PSI_STEP             50
#define REGEN_RYDER_MAX_FRONT_PSI              900
#define REGEN_RYDER_FRONT_TORQUE_X1000         { 0, 14936, 25332, 32316, 36706, 39108, 39971, 39638, 38369, 36366, 33784, 30747, 27351, 23673, 19773, 15702, 11497, 7192, 2811 }

#define REGEN_RYDER_FRONT_FORCE_C0             183.0f
#define REGEN_RYDER_FRONT_FORCE_C1             8.7696f
#define REGEN_RYDER_FRONT_FORCE_C2             0.0141145f
#define REGEN_RYDER_FRONT_FORCE_C3             0.0000068383f
#define REGEN_RYDER_REAR_FORCE_C1              5.6077f
#define REGEN_RYDER_TORQUE_SCALE               0.032704f

/* Traction control defaults (runtime-configurable, fractional values x1000). */
#define TRACTION_CONTROL_ENABLED_DEFAULT             FALSE
#define TRACTION_CONTROL_TARGET_SLIP_X1000_DEFAULT   1100
#define TRACTION_CONTROL_KP_X1000_DEFAULT            5000
#define TRACTION_CONTROL_KI_X1000_DEFAULT            0
#define TRACTION_CONTROL_KD_X1000_DEFAULT            0
#define TRACTION_CONTROL_MIN_FRONT_RPM_DEFAULT       10

/****************************************************************************
 * Launch Control Learning Mode
 *
 * Semi-automated launch characterization. The driver performs two full-throttle
 * runs using two compile-time torque-vs-(motor)speed curves; the VCU measures
 * wheel slip and recommends the better curve + a slip target. All launch torque
 * is pedal-bounded (never exceeds the pedal-mapped request), so existing APPS /
 * FSAE torque-plausibility safety is preserved.
 *
 * RATIONALE FOR COMPILE-TIME CURVES (TTC60 EEPROM constraint):
 *   A full 2-curve torque-vs-speed map would consume far more EEPROM than the
 *   ~10 free i16 param slots allow. The curve *shapes* are therefore fixed at
 *   compile time; only the small result set (best curve, recommended slip) and a
 *   few tunable thresholds are persisted as runtime params.
 ****************************************************************************/

/* Number of breakpoints in each launch torque-vs-speed curve. */
#define LAUNCH_CURVE_POINTS                 5

/* Motor-speed (RPM) breakpoints shared by both curves (ascending). */
#define LAUNCH_CURVE_RPM_BREAKPOINTS        { 0, 340, 680, 1020, 1360 }

/* Curve A (conservative) torque (Nm) at each breakpoint. */
#define LAUNCH_CURVE_A_TORQUE_NM            { 100, 130, 160, 190, 220 }

/* Curve B (aggressive) torque (Nm) at each breakpoint. */
#define LAUNCH_CURVE_B_TORQUE_NM            {100, 160, 220, 220, 220}

/* Curve C placeholder torque (Nm) at each breakpoint. */
#define LAUNCH_CURVE_C_TORQUE_NM            {100, 220, 220, 220, 220}

/* Run-start readiness (compile-time; not persisted).
 * A CAN command arms launch mode first; once armed, launch starts from pedal
 * position only. No RPM or torque threshold is used to begin a run.
 */
#define LAUNCH_START_PEDAL_PERCENT         80     /* pedal travel % that starts a launch once armed */

/* Launch learning runtime-param defaults (EEPROM-backed, i16). */
#define TRC_LAUNCH_ENABLED_DEFAULT                  FALSE
#define TRC_LAUNCH_END_RPM_DEFAULT                  6000   /* motor RPM that ends a run */
#define TRC_LAUNCH_TIMEOUT_MS_DEFAULT               15000  /* per-run safety timeout */
#define TRC_LAUNCH_MAX_SLIP_X1000_DEFAULT           1300   /* slip ratio that aborts a run */
#define TRC_LAUNCH_BEST_CURVE_DEFAULT               0      /* 0 = Curve A, 1 = Curve B, 2 = Curve C */
#define TRC_LAUNCH_ACTIVE_CURVE_DEFAULT             3      /* 0 = A, 1 = B, 2 = C, 3 = uploaded curve */
#define TRC_LAUNCH_RECOMMENDED_SLIP_X1000_DEFAULT   1100

/* EEPROM-backed uploaded actual-launch curve (used when ACTIVE_CURVE = 3). */
#define TRC_LAUNCH_ACTUAL_RPM_0_DEFAULT             0
#define TRC_LAUNCH_ACTUAL_RPM_1_DEFAULT             340
#define TRC_LAUNCH_ACTUAL_RPM_2_DEFAULT             680
#define TRC_LAUNCH_ACTUAL_RPM_3_DEFAULT             1020
#define TRC_LAUNCH_ACTUAL_RPM_4_DEFAULT             1360

#define TRC_LAUNCH_ACTUAL_TORQUE_0_DEFAULT          40
#define TRC_LAUNCH_ACTUAL_TORQUE_1_DEFAULT          50
#define TRC_LAUNCH_ACTUAL_TORQUE_2_DEFAULT          65
#define TRC_LAUNCH_ACTUAL_TORQUE_3_DEFAULT          80
#define TRC_LAUNCH_ACTUAL_TORQUE_4_DEFAULT          90

#endif // TORQUE_CONFIG_H
