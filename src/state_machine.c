#include "IO_Constants.h"

#include "state_machine.h"

#include "sensors/apps.h"
#include "sensors/bse.h"
#include "io/rtd.h"

static VCU_State_t current_state;

void StateMachine_Update(void)
{
    const APPS_Data_t* apps = APPS_GetData();
    //const BSE_Data_t* bse = BSE_GetData();
    const bool rtd_active = RTD_IsActive();

    switch (current_state) {

        case VCU_STATE_NOT_READY:
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