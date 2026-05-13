#include "IO_Constants.h"
#include "IO_DIO.h"
#include "IO_RTC.h"

#include "io/lights.h"
#include "sensors/bse.h"
#include "config/dio_config.h"
#include "can/can_rx.h"
#include "can/can_manager.h"

static bool red_car;
static bool red_light_state;
static ubyte4 last_blink;
static ubyte4 hvc_grace_start;

void Lights_Init(void)
{
    IO_DO_Init( BRAKE_LIGHT_PIN );

    IO_DO_Init( TSSI_GREEN_PIN );
    IO_DO_Init( TSSI_RED_PIN );

    IO_RTC_StartTime(&last_blink);
    IO_RTC_StartTime(&hvc_grace_start);
    red_light_state = FALSE;
    red_car = FALSE;
}

void Lights_Update(void)
{
    const BSE_Data_t* bse = BSE_GetData();
    // the rx structs only get updated when it gets received so sus idk if worth 1000th rework tho
    const HVCSummary_RX_Data_t* hvc = CAN_RX_GetHVCSummaryData();
    const bool hvc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SUMMARY);
    const bool in_hvc_grace = (IO_RTC_GetTimeUS(hvc_grace_start) < HVC_STARTUP_GRACE_US);

    /* Brake Light */
    if (bse->brakes_engaged) {
        IO_DO_Set(BRAKE_LIGHT_PIN, TRUE);
    } else {
        IO_DO_Set(BRAKE_LIGHT_PIN, FALSE);
    }

    /* Logic for Red Car */
    if (!hvc_valid) {
        /* Allow HVC time to come online at boot. */
        red_car = FALSE;//;in_hvc_grace ? FALSE : TRUE;
    }
    else if (!hvc->imd_ok || !hvc->bms_ok) {
        red_car = FALSE;//TRUE;
    }
    // Red car can only clear if sdc is good
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

bool Lights_isRedCar(void)
{
    return red_car;
}