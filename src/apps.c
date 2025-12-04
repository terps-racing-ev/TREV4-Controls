#include "IO_Constants.h"
#include "IO_ADC.h"
#include "IO_RTC.h"

#include "apps.h"
#include "apps_config.h"
#include "moving_average.h"

static APPS_Data_t apps_data;
static MovingAverage_Data_t apps1_ma;
static MovingAverage_Data_t apps2_ma;

static ubyte2 APPS_VoltageToPedalTravel(ubyte2 mv, ubyte2 min_mv, ubyte2 max_mv)
{
    if (mv < min_mv) {
        return 0;
    }
    
    if (mv > max_mv) {
        return APPS_RESOLUTION;
    }
    
    return (ubyte2)(((ubyte4)(mv - min_mv) * APPS_RESOLUTION) / 
                    (max_mv - min_mv));
}


static void APPS_UpdateChannel(ubyte1 adc_channel, MovingAverage_Data_t* ma, APPS_Sensor_t* sensor,
                               ubyte2 min_mv, ubyte2 max_mv)
{
    IO_ErrorType err;
    ubyte4 current_time;
    bool data_fresh;

    sensor->data_valid = FALSE;

    // Read raw ADC
    err = IO_ADC_Get(adc_channel, &sensor->raw_mv, &data_fresh);
    if (err != IO_E_OK) {
        return;
    }

    // Only update moving average if data is fresh
    if (data_fresh) {
        err = MovingAverage_Update(ma, sensor->raw_mv, &sensor->filt_mv);
        if (err != IO_E_OK) {
            return;
        }
        sensor->last_update_us = IO_RTC_GetTimeUS(0);
    }
    
    // Check staleness, this way data doesn't have to be perfectly fresh before we mark invalid
    current_time = IO_RTC_GetTimeUS(0);
    if (current_time - sensor->last_update_us > APPS_MAX_STALENESS) {
        return;
    }

    // Bounds check, filtered mv is used for leniency
    if (sensor->filt_mv < min_mv - APPS_VOLTAGE_TOLERANCE ||
        sensor->filt_mv > max_mv + APPS_VOLTAGE_TOLERANCE) {
        sensor->out_of_range = TRUE;
        return;
    }
    
    sensor->out_of_range = FALSE;
    sensor->data_valid = TRUE;
    
    // Scale filtered voltage to pedal travel
    sensor->value = APPS_VoltageToPedalTravel(sensor->filt_mv, min_mv, max_mv);
}

void APPS_Init(void)
{
    IO_ADC_ChannelInit(IO_PIN_APPS_1, IO_ADC_RATIOMETRIC,0, 0, IO_APPS_1_SUPPLY, NULL);
    IO_ADC_ChannelInit(IO_PIN_APPS_2, IO_ADC_RATIOMETRIC, 0, 0, IO_APPS_2_SUPPLY, NULL);

    MovingAverage_Init(&apps1_ma, APPS_FILTER_WINDOW_SIZE);
    MovingAverage_Init(&apps2_ma, APPS_FILTER_WINDOW_SIZE);
    
    // Initialize sensor structs
    apps_data.apps1.last_update_us = 0;
    apps_data.apps2.last_update_us = 0;
    apps_data.apps1.data_valid = FALSE;
    apps_data.apps2.data_valid = FALSE;
}

void APPS_Update(void)
{
    APPS_UpdateChannel(IO_PIN_APPS_1, &apps1_ma, &apps_data.apps1,
                       APPS_1_MIN_VOLTAGE, APPS_1_MAX_VOLTAGE);
    APPS_UpdateChannel(IO_PIN_APPS_2, &apps2_ma, &apps_data.apps2,
                       APPS_2_MIN_VOLTAGE, APPS_2_MAX_VOLTAGE);
    
    if (apps_data.apps1.data_valid && apps_data.apps2.data_valid) {
        ubyte2 diff = (apps_data.apps1.value > apps_data.apps2.value) ?
                      (apps_data.apps1.value - apps_data.apps2.value) :
                      (apps_data.apps2.value - apps_data.apps1.value);
        
        if (diff > APPS_IMPLAUSIBILITY_DEVIATION) {
            apps_data.apps_implausible = TRUE;
        } else {
            apps_data.apps_implausible = FALSE;
        }
        
        // Combined value: average of both if plausible
        if (!apps_data.apps_implausible) {
            apps_data.apps_value = (ubyte2)((ubyte4)apps_data.apps1.value + apps_data.apps2.value) / 2;
        } else {
            apps_data.apps_value = 0;
        }
    } else {
        apps_data.apps_implausible = TRUE;
        apps_data.apps_value = 0;
    }
}

const APPS_Data_t* APPS_GetData(void)
{
    return &apps_data;
}
