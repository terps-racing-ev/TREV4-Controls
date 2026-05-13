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


/* Default values for runtime-configurable parameters (EEPROM-backed). */
#define MAX_TORQUE_DEFAULT 5

#define PERCENT_TRAVEL_FOR_MAX_TORQUE 90

#define MOTOR_DIRECTION_DEFAULT MOTOR_FORWARDS // backwards for dyno testing

/* REGEN */
#define REGEN_ENABLED_DEFAULT FALSE
#define BRAKE_PRESSURE_FOR_MAX_REGEN 500
#define MIN_RPM_FOR_REGEN 500 // Depends on ratio and wheel
#define REGEN_TORQUE_MAX 230 // UNUSED

#define REGEN_PARABOLA_CONST 0.00025
#define REGEN_LINEAR_CONST 0.114

#endif // TORQUE_CONFIG_H
