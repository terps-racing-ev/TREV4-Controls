#include "IO_Constants.h"
#include "IO_DIO.h"
#include "IO_RTC.h"

#include "io/lights.h"
#include "sensors/bse.h"
#include "config/dio_config.h"
#include "can/can_manager.h"

static bool red_car;
static bool red_light_state;
static ubyte4 last_blink;

void Lights_Init(void)
{
    IO_DO_Init( BRAKE_LIGHT_PIN );

    IO_DO_Init( TSSI_GREEN_PIN );
    IO_DO_Init( TSSI_RED_PIN );

    IO_RTC_StartTime(&last_blink);
    red_light_state = FALSE;
    red_car = FALSE;
}

void Lights_Update(void)
{
    const BSE_Data_t* bse = BSE_GetData();
    const HVCSummary_RX_Data_t* hvc = CAN_Manager_GetHVCSummaryData();

    /* Brake Light */
    if (bse->brakes_engaged) {
        IO_DO_Set(BRAKE_LIGHT_PIN, TRUE);
    } else {
        IO_DO_Set(BRAKE_LIGHT_PIN, FALSE);
    }

    // TODO need init timeout and can timeout (valid check ig)
    /* Logic for Red Car */
    if (!hvc->imd_ok || !hvc->bms_ok) {
        red_car = TRUE;
    } 
    // TODO link rules here weird logic but true
    else if (hvc->imd_ok && hvc->bms_ok && hvc->sdc_ok) {
        red_car = FALSE;
    }

    /* Actuation for TSSI */
    if (red_car) {
        if (IO_RTC_GetTimeUS(last_blink) > TSSI_BLINK_PERIOD_US) {
            red_light_state = !red_light_state;
            // Reset timer
            IO_RTC_StartTime(&last_blink);
        }
    } else {
        red_light_state = FALSE;
    }

    IO_DO_Set(TSSI_GREEN_PIN, !red_car);
    IO_DO_Set(TSSI_RED_PIN, red_light_state);
}