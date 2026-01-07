#include "IO_Constants.h"
#include "IO_ADC.h"
#include "IO_RTC.h"

#include "apps.h"

#include "config/apps_config.h"
#include "util/moving_average.h"
#include "debug_defines.h"

static APPS_Data_t apps_data;
static MovingAverage_Data_t apps1_ma;
static MovingAverage_Data_t apps2_ma;

static ubyte2 VoltageToPedalTravel(ubyte2 mv, ubyte2 min_mv, ubyte2 max_mv)
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


static void UpdateChannel(ubyte1 adc_channel, MovingAverage_Data_t* ma, APPS_Sensor_t* sensor,
                          ubyte2 min_mv, ubyte2 max_mv)
{
    /* Local Variables */
    IO_ErrorType err;       // error for function calls
    bool data_fresh;        // staleness check

    if (ma == NULL || sensor == NULL) {
        return;
    }

    sensor->valid = FALSE;
    sensor->adc_err = FALSE;
    sensor->stale = FALSE;
    sensor->out_of_range = FALSE;
    
    // Read raw ADC
    err = IO_ADC_Get(adc_channel, &sensor->raw_mv, &data_fresh);
    #if !IGNORE_APPS_ERRORS
        if (err != IO_E_OK) {
            sensor->adc_err = TRUE;
            return;
        }
        /* This is a harsh check since APPS_Update preserves its value
            and the caller is expected to provide some tolerance */
        if (!data_fresh) {
            sensor->stale = TRUE;
            return;
        }
    #endif

    sensor->filt_mv = MovingAverage_Update(ma, sensor->raw_mv);

    // Bounds check, filtered mv is used for leniency
    #if !IGNORE_APPS_ERRORS
        if (sensor->filt_mv < min_mv - APPS_VOLTAGE_TOLERANCE ||
            sensor->filt_mv > max_mv + APPS_VOLTAGE_TOLERANCE) {
            sensor->out_of_range = TRUE;
            return;
        }
    #endif
    
    // Sensor reading is all good
    sensor->valid = TRUE;
    
    // Scale filtered voltage to pedal travel
    sensor->value = VoltageToPedalTravel(sensor->filt_mv, min_mv, max_mv);
}

void APPS_Init(void)
{
    IO_ADC_ChannelInit(IO_PIN_APPS_1, IO_ADC_RATIOMETRIC, 0, 0, IO_APPS_1_SUPPLY, NULL);
    IO_ADC_ChannelInit(IO_PIN_APPS_2, IO_ADC_RATIOMETRIC, 0, 0, IO_APPS_2_SUPPLY, NULL);

    MovingAverage_Init(&apps1_ma, APPS_FILTER_WINDOW_SIZE);
    MovingAverage_Init(&apps2_ma, APPS_FILTER_WINDOW_SIZE);
    
    apps_data.apps_value = 0;
    apps_data.valid = FALSE;

    // Initialize sensor structs
    apps_data.apps1.valid = FALSE;
    apps_data.apps2.valid = FALSE;
}

/*
Preserves the last valid APPS value even upon error.
It is up to the caller to decide when to actually invalidate reading.
*/
void APPS_Update(void)
{

    /* Local Variables */
    ubyte2 diff; // used for implausibility

    apps_data.valid = FALSE;
    apps_data.implausible = FALSE;

    UpdateChannel(IO_PIN_APPS_1, &apps1_ma, &apps_data.apps1,
                  APPS_1_MIN_VOLTAGE, APPS_1_MAX_VOLTAGE);
    UpdateChannel(IO_PIN_APPS_2, &apps2_ma, &apps_data.apps2,
                  APPS_2_MIN_VOLTAGE, APPS_2_MAX_VOLTAGE);
    
    // Individual sensor check
    #if !IGNORE_APPS_ERRORS
    if (!(apps_data.apps1.valid && apps_data.apps2.valid)) {
        return;
    }
    #endif

    // Absolute difference between both values
    diff = (apps_data.apps1.value > apps_data.apps2.value) ?
                    (apps_data.apps1.value - apps_data.apps2.value) :
                    (apps_data.apps2.value - apps_data.apps1.value);

    // More than 10 percent?
    #if !IGNORE_APPS_ERRORS
    if (diff > APPS_IMPLAUSIBILITY_DEVIATION) {
        apps_data.implausible = TRUE;
        return;
    }
    #endif

    // APPS is all good
    apps_data.valid = TRUE;
    apps_data.implausible = FALSE;
    
    // Final averaged value
    apps_data.apps_value = (ubyte2)((ubyte4)apps_data.apps1.value + apps_data.apps2.value) / 2;

    // Set additional flags
    apps_data.above_bap_threshold = apps_data.apps_value > APPS_BAP_THRESHOLD;
    apps_data.below_bap_reestablish_threshold = apps_data.apps_value < APPS_BAP_REESTABLISH_THRESHOLD;

}

const APPS_Data_t* APPS_GetData(void)
{
    return &apps_data;
}
