#include "IO_Constants.h"
#include "IO_DIO.h"
#include "IO_RTC.h"

#include "io/lights.h"
#include "sensors/bse.h"
#include "config/dio_config.h"
#include "can/can_rx.h"
#include "can/can_manager.h"
#include "config/runtime_config.h"

static bool red_car;
static bool red_light_state;
static ubyte4 last_blink;
static ubyte4 red_car_grace_start;

void Lights_Init(void)
{
    IO_DO_Init( BRAKE_LIGHT_PIN );

    IO_DO_Init( TSSI_GREEN_PIN );
    IO_DO_Init( TSSI_RED_PIN );

    IO_RTC_StartTime(&last_blink);
    IO_RTC_StartTime(&red_car_grace_start);
    red_light_state = FALSE;
    red_car = FALSE;
}

void Lights_Update(void)
{
    const BSE_Data_t* bse = BSE_GetData();
    // the rx structs only get updated when it gets received so sus idk if worth 1000th rework tho
    const HVCSummary_RX_Data_t* hvc = CAN_RX_GetHVCSummaryData();
    const bool hvc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SUMMARY);
    const bool in_red_car_grace = (IO_RTC_GetTimeUS(red_car_grace_start) < RED_CAR_STARTUP_GRACE_US);

    /* Brake Light */
    if (bse->brakes_engaged) {
        IO_DO_Set(BRAKE_LIGHT_PIN, TRUE);
    } else {
        IO_DO_Set(BRAKE_LIGHT_PIN, FALSE);
    }

    sbyte2 dbg_bits = 0;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_DEBUG_DEFINES, &dbg_bits);
    const bool always_green = (dbg_bits & DEBUG_BIT_ALWAYS_GREEN);

    if (always_green || in_red_car_grace) {
        /* Debug override / startup grace forces green and clears any latch. */
        red_car = FALSE;
    }
    else if (!hvc_valid || !hvc->imd_ok || !hvc->bms_ok) {
        /* An IMD or BMS fault (or lost HVC) latches the TSSI red. Either fault
         * opens the Shutdown Circuit automatically. */
        red_car = TRUE;
    }
    else if (hvc->sdc_ok) {
        /* Stay latched red until the Shutdown Circuit is confirmed closed again,
         * even if the IMD/BMS fault flags have already cleared. */
        red_car = FALSE;
    }
    /* else: no active fault but SDC not yet confirmed OK -> hold latched red. */

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