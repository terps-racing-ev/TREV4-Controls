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

/* Ryder regen strategy: rear-axle regen torque from the front/rear brake-force
 * balance equation, gated by an adjustable front/rear PSI activation window and
 * clamped to [REGEN_MIN_TORQUE, REGEN_MAX_TORQUE]. */
#define REGEN_RYDER_MU_X1000_DEFAULT           2250

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
