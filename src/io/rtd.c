#include "IO_Constants.h"
#include "IO_DIO.h"

#include "rtd.h"

#include "util/debounce.h"

static RTD_Data_t rtd;
static Debounce_Data_t db;

void RTD_Init(void)
{
    IO_DI_Init( IO_PIN_RTD,
                IO_DI_PU_10K );

    Debounce_Init(&db, RTD_OFF, RTD_DB_THRESHOLD);

    rtd.state = RTD_OFF;
    rtd.raw_state = RTD_OFF;

    rtd.valid = FALSE;
}

/*
Preserves the last valid RTD value even upon error.
It is up to the caller to decide when to actually invalidate reading.
*/
void RTD_Update(void)
{
    /* Local Variables */
    bool raw_val;       // raw input
    IO_ErrorType err;   // error for function call

    err = IO_DI_Get(IO_PIN_RTD, &raw_val);
    if (err != IO_E_OK) {
        rtd.valid = FALSE;
        return;
    }

    rtd.valid = TRUE;
    rtd.raw_state = raw_val;
    rtd.state = Debounce_Update(&db, raw_val);
}

bool RTD_IsActive(void)
{
    #ifndef IGNORE_RTD_SWITCH
    return (rtd.state == RTD_ON);
    #else
    return TRUE;
    #endif
}