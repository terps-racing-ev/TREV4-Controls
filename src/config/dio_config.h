#ifndef DIO_CONFIG_H
#define DIO_CONFIG_H

#include "IO_Constants.h"
#include "util/units.h"

/* Buzzer */
#define IO_PIN_BUZZER IO_DO_12
#define RTD_SOUND_DURATION_US MsToUs(1500)

/* Brake Light */
#define BRAKE_LIGHT_PIN IO_DO_02

/* TSSI */
#define TSSI_RED_PIN IO_DO_00
#define TSSI_GREEN_PIN IO_DO_01

#define TSSI_BLINK_PERIOD_US MsToUs(250)

/* RTD -> pin 263 (aka digital in 0) (switched to ground with pullup) */
#define IO_PIN_RTD IO_DI_00
#define RTD_DB_THRESHOLD MsToCycles(40)

#endif