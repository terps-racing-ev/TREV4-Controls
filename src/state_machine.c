#include "IO_Constants.h"

#include "state_machine.h"

#include "sensors/apps.h"
#include "sensors/bse.h"
#include "config/apps_config.h"
#include "config/bse_config.h"
#include "io/rtd.h"
#include "io/buzzer.h"
#include "can/can_manager.h"

static VCU_State_t current_state;

void StateMachine_Init(void)
{
    current_state = VCU_STATE_NOT_READY;
}

void StateMachine_Update(void)
{
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();
    const bool rtd_active = RTD_IsActive();
    const Buzzer_State_t buzzer_state = Buzzer_GetState();
    
    // TODO read all the messages we need
    const HVCSummary_RX_Data_t* hvc_summary = CAN_Manager_GetHVCSummaryData();

    const bool hard_fault = (!apps->valid || !bse->valid || !hvc_summary->sdc_ok);
    const bool bap_fault = (apps->above_bap_threshold && bse->hard_braking);

    switch (current_state) {

        case VCU_STATE_NOT_READY:
            if (rtd_active && bse->brakes_engaged) {
                if (!hard_fault && !bap_fault) {
                    current_state = VCU_STATE_PLAYING_RTD_SOUND;
                }
            }            
            break;

        case VCU_STATE_PLAYING_RTD_SOUND:
            if (!rtd_active) {
                current_state = VCU_STATE_NOT_READY;
            }
            else if (hard_fault) {
                current_state = VCU_STATE_HARD_FAULT;
            }
            else if (buzzer_state == BUZZER_STATE_DONE) {
                current_state = VCU_STATE_DRIVING;
            }
            break;

        case VCU_STATE_DRIVING:
            if (!rtd_active) {
                current_state = VCU_STATE_NOT_READY;   
            }
            else if (hard_fault) {
                current_state = VCU_STATE_HARD_FAULT;
            } 
            else if (bap_fault) {
                current_state = VCU_STATE_BAP_FAULT;
            }
            break;

        case VCU_STATE_BAP_FAULT:
            if (!rtd_active) {
                current_state = VCU_STATE_NOT_READY;
            }
            else if (hard_fault) {
                current_state = VCU_STATE_HARD_FAULT;
            } 
            else if (apps->below_bap_reestablish_threshold) {
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
