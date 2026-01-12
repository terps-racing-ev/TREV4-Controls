#include "IO_Constants.h"
#include "IO_DIO.h"
#include "IO_RTC.h"

#include "state_machine.h"
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

void Buzzer_Update(void)
{
    const VCU_State_t vcu_state = StateMachine_GetState();

    if (vcu_state == VCU_STATE_PLAYING_RTD_SOUND) {
        switch (buzzer_state) {
            case BUZZER_STATE_INACTIVE:
                // when this state has just been entered, turn on the buzzer and a timer
                buzzer_state = BUZZER_STATE_IN_PROG;
                IO_RTC_StartTime(&buzzer_timestamp);
                
                // TODO we can make this fancy af
                IO_DO_Set(IO_PIN_BUZZER, TRUE);
                break;
            
            case BUZZER_STATE_IN_PROG:
                IO_DO_Set(IO_PIN_BUZZER, TRUE);
                if (IO_RTC_GetTimeUS(buzzer_timestamp) > RTD_SOUND_DURATION_US) {
                    buzzer_state = BUZZER_STATE_INACTIVE;
                    IO_DO_Set(IO_PIN_BUZZER, FALSE);
                }
                break;

            case BUZZER_STATE_DONE:
                IO_DO_Set(IO_PIN_BUZZER, FALSE);
                buzzer_state = BUZZER_STATE_INACTIVE;
                break;

            default:
                break;
        }
    } else {
        buzzer_state = BUZZER_STATE_INACTIVE;
        IO_DO_Set(IO_PIN_BUZZER, FALSE);
    }
}

Buzzer_State_t Buzzer_GetState(void)
{
    return buzzer_state;
}