#ifndef APPS_CONFIG_H
#define APPS_CONFIG_H

#include "util/units.h"

/*******************************************/
/*             Sensor Pin                  */
/*******************************************/

/* BSE -> pin 137 (aka adc 5V 7)*/
#define IO_PIN_BSE IO_ADC_5V_07
#define IO_BSE_SUPPLY IO_ADC_SENSOR_SUPPLY_1


/*******************************************/
/*                Limits                   */
/*******************************************/

#define BSE_FILTER_WINDOW_SIZE MsToCycles(40)

#define BSE_MAX_VOLTAGE 4500
#define BSE_MIN_VOLTAGE 500

/* Mappings */
#define BSE_MAX_PSI 3000
#define BSE_MIN_PSI 0

#define BRAKES_ENGAGED_THRESHOLD 40 //PSI

/* Voltage range above max and below min that doesn't count as an error */
#define BSE_VOLTAGE_TOLERANCE 250

/*******************************************/
/*                Faults                   */
/*******************************************/

/* EV.4.7.1 - Threshold for "Hard" Braking */
#define BRAKE_PLAUSIBILITY_THRESHOLD 500 //PSI

#define APPS_MAX_STALENESS MsToUs(100ul)

#endif // APPS_CONFIG_H
