#ifndef APPS_H
#define APPS_H

#include "IO_Constants.h"
#include "IO_ADC.h"

typedef struct {
    ubyte2  value;
    ubyte2  raw_mv;
    ubyte2  filt_mv;
    
    bool    valid;      // !out_of_range && !adc_err

    bool    out_of_range;    
    bool    adc_err;
    bool    stale;

} APPS_Sensor_t;

typedef struct {
    ubyte2  apps_value;

    APPS_Sensor_t   apps1;
    APPS_Sensor_t   apps2;

    bool    implausible;
    /*
    APPS may not be valid for any of the following reasons:
    - one of the ADCs is old
    - one of the ADCs errored
    - one of the APPS is out of range
    - Implausible
    */
    bool    valid;    // apps1.valid && apps2.valid && !implausible

     /* Brake-Accel Plausibility */
    bool    above_bap_threshold;           /* apps_value > APPS_BAP_THRESHOLD */
    bool    below_bap_reestablish_threshold; /* apps_value < APPS_BAP_REESTABLISH_THRESHOLD */

} APPS_Data_t;

void APPS_Init(void);
void APPS_Update(void);

const APPS_Data_t* APPS_GetData(void);


#endif // APPS_H
