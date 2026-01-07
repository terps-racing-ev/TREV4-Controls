#include "IO_Constants.h"
#include "IO_DIO.h"
#include "IO_RTC.h"

#include "buzzer.h"
#include "config/dio_config.h"


// TODO make this a struct, maybe add more states

static Buzzer_State_t buzzer_state;
static ubyte4 buzzer_timestamp;

void Buzzer_Init(void)
{
    IO_DO_Init( IO_PIN_BUZZER );
    buzzer_state = BUZZER_STATE_INACTIVE;
}

// TODO sus, i feel like you could get stuck in infinite buzzer glitch
Buzzer_State_t Buzzer_Play(void)
{
    if (BUZZER_STATE_INACTIVE) {
        // when this state has just been entered, turn on the buzzer and a timer
        IO_RTC_StartTime(&buzzer_timestamp);
        buzzer_state = BUZZER_STATE_IN_PROG;

        // TODO we can make this fancy af
        IO_DO_Set(IO_PIN_BUZZER, TRUE);

    } else if (IO_RTC_GetTimeUS(buzzer_timestamp) > RTD_SOUND_DURATION_US) {
        buzzer_state = BUZZER_STATE_INACTIVE;
        IO_DO_Set(IO_PIN_BUZZER, FALSE);
    }

    return buzzer_state;
}

void Buzzer_Stop(void)
{
    buzzer_state = BUZZER_STATE_INACTIVE;
    IO_DO_Set(IO_PIN_BUZZER, FALSE);
}