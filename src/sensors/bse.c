#include "IO_Constants.h"
#include "IO_ADC.h"
#include "IO_RTC.h"

#include "bse.h"

#include "config/bse_config.h"
#include "util/moving_average.h"
#include "debug_defines.h"

static BSE_Data_t bse_data;
static MovingAverage_Data_t bse_ma;