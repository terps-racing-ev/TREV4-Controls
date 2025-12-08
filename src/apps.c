#include "IO_Constants.h"
#include "IO_ADC.h"
#include "IO_RTC.h"

#include "apps.h"
#include "apps_config.h"
#include "moving_average.h"
#include "debug_defines.h"

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

    sensor->valid = FALSE;
    sensor->adc_err = FALSE;
    sensor->stale = FALSE;
    sensor->out_of_range = FALSE;

    // Read raw ADC
    err = IO_ADC_Get(adc_channel, &sensor->raw_mv, &data_fresh);
    #ifndef IGNORE_APPS_ERRORS
    if (err != IO_E_OK) {
        sensor->adc_err = TRUE;
        return;
    }
    #endif

    // Only update moving average if data is fresh
    if (data_fresh) {
        MovingAverage_Update(ma, sensor->raw_mv, &sensor->filt_mv);
        sensor->last_update_us = IO_RTC_GetTimeUS(0);
    }
    
    // Check staleness, this way data doesn't have to be perfectly fresh before we mark invalid
    current_time = IO_RTC_GetTimeUS(0);
    #ifndef IGNORE_APPS_ERRORS
    if (current_time - sensor->last_update_us > APPS_MAX_STALENESS) {
        sensor->stale = TRUE;
        return;
    }
    #endif

    // Bounds check, filtered mv is used for leniency
    #ifndef IGNORE_APPS_ERRORS
    if (sensor->filt_mv < min_mv - APPS_VOLTAGE_TOLERANCE ||
        sensor->filt_mv > max_mv + APPS_VOLTAGE_TOLERANCE) {
        sensor->out_of_range = TRUE;
        return;
    }
    #endif
    
    // Sensor reading is all good
    sensor->out_of_range = FALSE;
    sensor->valid = TRUE;
    
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
    apps_data.apps1.valid = FALSE;
    apps_data.apps2.valid = FALSE;
}

void APPS_Update(void)
{
    apps_data.valid = FALSE;
    apps_data.apps_value = 0;
    apps_data.implausible = FALSE;

    APPS_UpdateChannel(IO_PIN_APPS_1, &apps1_ma, &apps_data.apps1,
                       APPS_1_MIN_VOLTAGE, APPS_1_MAX_VOLTAGE);
    APPS_UpdateChannel(IO_PIN_APPS_2, &apps2_ma, &apps_data.apps2,
                       APPS_2_MIN_VOLTAGE, APPS_2_MAX_VOLTAGE);
    
    // Individual sensor check
    #ifndef IGNORE_APPS_ERRORS
    if (!(apps_data.apps1.valid && apps_data.apps2.valid)) {
        return;
    }
    #endif

    ubyte2 diff = (apps_data.apps1.value > apps_data.apps2.value) ?
                    (apps_data.apps1.value - apps_data.apps2.value) :
                    (apps_data.apps2.value - apps_data.apps1.value);
    #ifndef IGNORE_APPS_ERRORS
    if (diff > APPS_IMPLAUSIBILITY_DEVIATION) {
        apps_data.implausible = TRUE;
        return;
    }
    #endif

    // APPS is all good
    apps_data.valid = TRUE;
    apps_data.implausible = FALSE;
    
    apps_data.apps_value = (ubyte2)((ubyte4)apps_data.apps1.value + apps_data.apps2.value) / 2;
    return;
}

const APPS_Data_t* APPS_GetData(void)
{
    return &apps_data;
}
