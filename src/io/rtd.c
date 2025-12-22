#include "IO_Constants.h"
#include "IO_DIO.h"

#include "rtd.h"

static bool rtd_active;

void RTD_Init(void)
{
    rtd_active = FALSE;
    IO_DI_Init( IO_PIN_RTD,
                IO_DI_PU_10K );
}

void RTD_Update(void)
{
    // TODO error checking and debouncing
    // TODO separate define (maybe enum?) for rtd state to isolate from hardware
    IO_DI_Get(IO_PIN_RTD, &rtd_active);
}

bool RTD_IsActive(void)
{
    return rtd_active;
}