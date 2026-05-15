#include "IO_Constants.h"

#include "state_machine.h"

#include "sensors/apps.h"
#include "sensors/bse.h"
#include "config/apps_config.h"
#include "config/bse_config.h"
#include "config/runtime_config.h"
#include "io/rtd.h"
#include "io/buzzer.h"
#include "io/lights.h"
#include "can/can_rx.h"

static VCU_State_t current_state;

static bool dead_car_tx_pending;
static bool clear_faults_tx_pending;
static bool was_red_car; // since red car isn't a state rn (should it be?)

void StateMachine_Init(void)
{
    current_state = VCU_STATE_LOADING;
    clear_faults_tx_pending = FALSE;
    dead_car_tx_pending = FALSE;
    was_red_car = FALSE;
}

void StateMachine_Update(void)
{
    const APPS_Data_t* apps = APPS_GetData();
    const BSE_Data_t* bse = BSE_GetData();
    const bool rtd_active = RTD_IsActive();
    const bool is_red_car = Lights_isRedCar();
    const Buzzer_State_t buzzer_state = Buzzer_GetState();
    
    // TODO read all the messages we need
    const HVCSummary_RX_Data_t* hvc_summary = CAN_RX_GetHVCSummaryData();

    /*
    hvc better always be honest and reflect hardware. no imd or bms hiccups allowed.
    red car check should be covered by sdc in theory, but this is in case to avoid sus red car driving situations
    */
    const bool hard_fault = (!apps->valid || !bse->valid || !hvc_summary->sdc_ok || is_red_car);
    const bool bap_fault = (apps->above_bap_threshold && bse->hard_braking);

    bool ready_to_drive = TRUE;
    ready_to_drive &= rtd_active;
    {
        sbyte2 dbg_bits = 0;
        (void)RuntimeConfig_GetI32(RUNTIME_PARAM_DEBUG_DEFINES, &dbg_bits);
        const bool ignore_rtd_brakes = (dbg_bits & DEBUG_BIT_IGNORE_RTD_BRAKES);

        if (!ignore_rtd_brakes) {
            ready_to_drive &= bse->brakes_engaged;
        }
    }

    /* red car check since it isn't a state rn */
    if (!was_red_car && is_red_car) {
        dead_car_tx_pending = TRUE;
    }
    was_red_car = is_red_car;

    switch (current_state) {

        case VCU_STATE_LOADING:
            if (RuntimeConfig_IsEepromLoadComplete()) {
                current_state = VCU_STATE_NOT_READY;
            }
            break;

        case VCU_STATE_NOT_READY:
            if (ready_to_drive) {
                if (!hard_fault && !bap_fault) {
                    current_state = VCU_STATE_PLAYING_RTD_SOUND;
                    clear_faults_tx_pending = TRUE;
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
                dead_car_tx_pending = TRUE;
                current_state = VCU_STATE_HARD_FAULT;
            } 
            else if (bap_fault) {
                dead_car_tx_pending = TRUE;
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

// TODO if this is geeking then just spam clear faults during buzzer like last year
bool StateMachine_ClearFaultsTxTrigger(void)
{
    if (clear_faults_tx_pending) {
        clear_faults_tx_pending = FALSE;
        return TRUE;
    }

    return FALSE;
}

bool StateMachine_DeadCarTxTrigger(void)
{
    if (dead_car_tx_pending) {
        dead_car_tx_pending = FALSE;
        return TRUE;
    }

    return FALSE;
}