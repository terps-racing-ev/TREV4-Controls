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
#define REGEN_MAX_APPS_DEFAULT 5
#define REGEN_MAX_TORQUE_DEFAULT 80
#define REGEN_MIN_TORQUE_DEFAULT 1
#define REGEN_MIN_BSE_REAR_PSI_DEFAULT 20
#define REGEN_MIN_BSE_FRONT_PSI_DEFAULT 20
#define REGEN_MAX_BSE_REAR_PSI_DEFAULT 900
#define REGEN_MAX_BSE_FRONT_PSI_DEFAULT 900
#define REGEN_MIN_SPEED_DEFAULT 250
#define REGEN_MAX_SOC_DEFAULT 85
#define REGEN_SOC_GATE_ENABLED_DEFAULT TRUE
#define REGEN_STRATEGY_DEFAULT REGEN_STRATEGY_RYDER

/* Ryder regen strategy: front-pressure lookup plus rear-pressure balancing. */
#define REGEN_RYDER_MU_X1000_DEFAULT           2250
#define REGEN_RYDER_FRONT_TABLE_POINTS         19
#define REGEN_RYDER_FRONT_PSI_STEP             50
#define REGEN_RYDER_MAX_FRONT_PSI              900
/* Previous regen table x1000: { 0, 14936, 25332, 32316, 36706, 39108, 39971, 39638, 38369, 36366, 33784, 30747, 27351, 23673, 19773, 15702, 11497, 7192, 2811 } */
#define REGEN_RYDER_FRONT_TORQUE_X1000         { 0, 22404, 37998, 48474, 55059, 58662, 59957, 59457, 57554, 54549, 50676, 46121, 41027, 35510, 29660, 23553, 17246, 10788, 4217 }

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
 * Launch Control (time-based torque curve)
 *
 * The car no longer has front wheel-speed sensors, so launch is no longer
 * slip-controlled. Launch torque follows a fixed, preprogrammed curve of
 * torque vs. time-since-trigger:
 *
 *   t < LAUNCH_OFFTHELINE_TIME_S : torque = torque_offtheline
 *   t in [offtheline .. duration]: torque = torque_init + t^2*(torque_final - torque_init)/duration^2
 *   t > duration                 : torque = torque_final
 *
 * Launch is armed over CAN, then triggers when the driver is off the brakes and
 * APPS rises above LAUNCH_TRIGGER_APPS_PERCENT. The curve is always bounded by
 * the driver's pedal-mapped torque so APPS / FSAE plausibility is preserved.
 ****************************************************************************/

/* Runtime-tunable torque set-points (Nm, EEPROM-backed i16). */
#define LAUNCH_TORQUE_OFFTHELINE_DEFAULT    100   /* held over the first offtheline window */
#define LAUNCH_TORQUE_INIT_DEFAULT          150   /* curve value at t = 0 s */
#define LAUNCH_TORQUE_FINAL_DEFAULT         183   /* curve value at t = duration_s */

/* Runtime-tunable curve shape (EEPROM-backed i16, stored as integer ms or percent). */
#define LAUNCH_OFFTHELINE_TIME_MS_DEFAULT   200   /* off-the-line hold window, ms (default 0.2 s) */
#define LAUNCH_CURVE_DURATION_MS_DEFAULT    3000  /* duration of the parabolic ramp, ms (default 3 s) */

/* Runtime-tunable trigger / end thresholds, as a percent of APPS travel. */
#define LAUNCH_TRIGGER_APPS_PERCENT_DEFAULT 25    /* throttle % that triggers a launch once armed */
#define LAUNCH_END_APPS_PERCENT_DEFAULT     5     /* throttle % at/below which a launch ends */

#endif // TORQUE_CONFIG_H
