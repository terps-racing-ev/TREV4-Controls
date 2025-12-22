#ifndef APPS_H
#define APPS_H

#include "IO_Constants.h"
#include "IO_ADC.h"

typedef struct {
    ubyte2 value;
    ubyte2 raw_mv;
    ubyte2 filt_mv;
    
    bool   valid;  // !out_of_range && !stale && !adc_err

    bool   out_of_range;    
    bool   stale;
    bool   adc_err;

    ubyte4 last_update_us;
} APPS_Sensor_t;

typedef struct {
    ubyte2 apps_value;

    APPS_Sensor_t apps1;
    APPS_Sensor_t apps2;

    /*
    APPS may not be valid for any of the following reasons:
    - one of the ADCs timed out
    - one of the ADCs errored
    - one of the APPS is out of range
    - Implausible
    */
    bool valid;    // apps1.valid && apps2.valid && !implausible

    bool implausible;
} APPS_Data_t;

void APPS_Init(void);
void APPS_Update(void);

const APPS_Data_t* APPS_GetData(void);


#endif // APPS_H