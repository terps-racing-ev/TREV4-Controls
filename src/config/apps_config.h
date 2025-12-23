#ifndef APPS_CONFIG_H
#define APPS_CONFIG_H

#include "util/units.h"

/*******************************************/
/*             Sensor Pins                 */
/*******************************************/

/* APPS 1 -> pin 152 (ADC 5V 0)*/
#define IO_PIN_APPS_1 IO_ADC_5V_00
#define IO_APPS_1_SUPPLY IO_ADC_SENSOR_SUPPLY_0

/* APPS 2 -> pin 140 (ADC 5V 1)*/
#define IO_PIN_APPS_2 IO_ADC_5V_01
#define IO_APPS_2_SUPPLY IO_ADC_SENSOR_SUPPLY_0


/*******************************************/
/*                Limits                   */
/*******************************************/

#define APPS_RESOLUTION 1000
#define APPS_FILTER_WINDOW_SIZE MsToCycles(40)

/* APPS 1 Bounds */
#define APPS_1_MAX_VOLTAGE 4400
#define APPS_1_MIN_VOLTAGE 1039

/* APPS 2 Bounds */
#define APPS_2_MAX_VOLTAGE 4016
#define APPS_2_MIN_VOLTAGE 660

/* Voltage range above max and below min that doesn't count as an error */
#define APPS_VOLTAGE_TOLERANCE 250

/* Travel below 1 percent will count as 0 */
#define APPS_DEADZONE (APPS_RESOLUTION / 100)


/*******************************************/
/*                Faults                   */
/*******************************************/

/* T.4.2.4 */
#define APPS_IMPLAUSIBILITY_DEVIATION (APPS_RESOLUTION / 10)

/* T.4.2.5 */
#define APPS_IMPLAUSIBILITY_PERSISTENCE_PERIOD MsToUs(100ul)

/* EV.4.7.1 - The APPS signals more than 25% Pedal Travel */
#define APPS_BRAKE_PLAUSIBILITY_THRESHOLD (APPS_RESOLUTION / 4)

/* EV.4.7.2.b - The Motor shut down must stay active until the 
APPS signals less than 5% Pedal Travel, with or without brake operation */
#define APPS_THRESHHOLD_REESTABLISH_PLAUSIBILITY (APPS_RESOLUTION / 20)

#define APPS_MAX_STALENESS MsToUs(100ul)

#endif // APPS_CONFIG_H