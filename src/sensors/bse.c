#include "IO_Constants.h"
#include "IO_ADC.h"
#include "IO_RTC.h"

#include "bse.h"

#include "config/bse_config.h"
#include "util/moving_average.h"
#include "debug_defines.h"

static BSE_Data_t bse_data;
static MovingAverage_Data_t bse_ma;

static ubyte2 VoltageToPSI(ubyte2 mv)
{
    if (mv < BSE_MIN_VOLTAGE) {
        return BSE_MIN_PSI;
    }

    if (mv > BSE_MAX_VOLTAGE) {
        return BSE_MAX_PSI;
    }

    return (ubyte2)(((ubyte4)(mv - BSE_MIN_VOLTAGE) *
                     (BSE_MAX_PSI - BSE_MIN_PSI)) /
                    (BSE_MAX_VOLTAGE - BSE_MIN_VOLTAGE) +
                    BSE_MIN_PSI);
}

void BSE_Init(void)
{
    IO_ADC_ChannelInit(IO_PIN_BSE, IO_ADC_RATIOMETRIC, 0, 0, IO_BSE_SUPPLY, NULL);

    MovingAverage_Init(&bse_ma, BSE_FILTER_WINDOW_SIZE);
    
    bse_data.psi = 0;
    bse_data.valid = FALSE;
    bse_data.brakes_engaged = FALSE;
}

void BSE_Update(void)
{
    /* Local Variables */
    IO_ErrorType err;       // error for function calls
    bool data_fresh;        // staleness check

    bse_data.valid = FALSE;
    bse_data.out_of_range = FALSE;
    bse_data.adc_err = FALSE;
    bse_data.stale = FALSE;

    // Read raw ADC
    err = IO_ADC_Get(IO_PIN_BSE, &bse_data.raw_mv, &data_fresh);
    #if !IGNORE_BSE_ERRORS
    if (err != IO_E_OK) {
        bse_data.adc_err = TRUE;
        return;
    }
    if (!data_fresh) {
        bse_data.stale = TRUE;
        return;
    }
    #endif

    bse_data.filt_mv = MovingAverage_Update(&bse_ma, bse_data.raw_mv);

    // Bounds check
    #if !IGNORE_BSE_ERRORS
    if (bse_data.filt_mv < BSE_MIN_VOLTAGE - BSE_VOLTAGE_TOLERANCE ||
        bse_data.filt_mv > BSE_MAX_VOLTAGE + BSE_VOLTAGE_TOLERANCE) {
        bse_data.out_of_range = TRUE;
        return;
    }
    #endif

    // Sensor reading is all good
    bse_data.valid = TRUE;

    // Scale filtered voltage to psi
    bse_data.psi = VoltageToPSI(bse_data.filt_mv);

    /* Perform brake level checks */
    bse_data.brakes_engaged = (bse_data.psi > BRAKES_ENGAGED_THRESHOLD);
    bse_data.hard_braking = (bse_data.psi > BRAKE_PLAUSIBILITY_THRESHOLD);
}

const BSE_Data_t* BSE_GetData(void)
{
    return &bse_data;
}
