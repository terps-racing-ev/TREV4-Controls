#ifndef APPS_H
#define APPS_H

#include "IO_Constants.h"
#include "IO_ADC.h"

typedef struct {
    ubyte2 value;
    ubyte2 raw_mv;
    ubyte2 filt_mv;
    bool   out_of_range;
    bool   data_valid;
    ubyte4 last_update_us;
} APPS_Sensor_t;

typedef struct {
    ubyte2 apps_value;

    APPS_Sensor_t apps1;
    APPS_Sensor_t apps2;

    bool apps_valid;
    bool apps_implausible;
} APPS_Data_t;

void APPS_Init(void);
void APPS_Update(void);

const APPS_Data_t* APPS_GetData(void);


#endif // APPS_H