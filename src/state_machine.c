#include "IO_Constants.h"

#include "state_machine.h"

#include "sensors/apps.h"
#include "sensors/bse.h"
#include "io/rtd.h"

static VCU_State_t current_state;

// TODO add HVC summary struct, sdc, shoud rtd go in here?
static bool HardFaultPresent(const APPS_Data_t* apps, const BSE_Data_t* bse) 
{
    return (!apps->valid || !bse->valid);
}

static bool SoftFaultPresent(const APPS_Data_t* apps) {
    return (apps->implausible);
}

void StateMachine_Init(void)
{
    current_state = VCU_STATE_NOT_READY;
}

void StateMachine_Update(void)
{
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();
    const bool rtd_active = RTD_IsActive();
    // TODO read all the messages we need

    switch (current_state) {

        case VCU_STATE_NOT_READY:
            if (rtd_active && bse->brakes_engaged) {
                if (HardFaultPresent(apps, bse) == FALSE) {
                    current_state = VCU_STATE_PLAYING_RTD_SOUND;
                }
            }
            // TODO set some output structs
            
            break;

        case VCU_STATE_PLAYING_RTD_SOUND:
            // TODO wait for buzzer to do his thang, make cancellable
            current_state = VCU_STATE_DRIVING;
            break;

        case VCU_STATE_DRIVING:
            if (HardFaultPresent(apps, bse) == TRUE) {
                current_state = VCU_STATE_HARD_FAULT;
            } else if (SoftFaultPresent(apps) == TRUE) {
                current_state = VCU_STATE_SOFT_FAULT;
            }
            break;

        case VCU_STATE_SOFT_FAULT:
            if (SoftFaultPresent(apps) == FALSE) {
                current_state = VCU_STATE_DRIVING;
            }
            break;

        case VCU_STATE_HARD_FAULT:
            if (rtd_active == FALSE) {
                current_state = VCU_STATE_NOT_READY;
            }
            break;

        default:
            break;
    }
}

VCU_State_t StateMachine_GetState(void)
{
    return current_state;
}
