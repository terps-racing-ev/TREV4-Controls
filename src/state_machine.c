#include "IO_Constants.h"

#include "state_machine.h"

#include "sensors/apps.h"
#include "sensors/bse.h"
#include "io/rtd.h"

static VCU_State_t current_state;

// TODO add HVC summary struct
static bool NoFaults(APPS_Data_t* apps, BSE_Data_t* bse_data) 
{
    return (apps->valid && bse_data->valid);
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
                if (NoFaults(apps, bse)) {
                    current_state = VCU_STATE_PLAYING_RTD_SOUND;
                }
            }
            // TODO set some output structs
            
            break;
        case VCU_STATE_PLAYING_RTD_SOUND:
            break;
        case VCU_STATE_DRIVING:
            break;
        case VCU_STATE_SOFT_ERROR:
            break;
        case VCU_STATE_HARD_ERROR:
            break;
        default:
            break;
    }
}