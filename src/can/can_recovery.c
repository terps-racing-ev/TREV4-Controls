#include "IO_Constants.h"
#include "IO_CAN.h"

#include "can_recovery.h"
#include "config/can_config.h"

/*
CAN Recovery was mostly AI generated with little caleb review.
hopefully this code never runs.
*/

static CAN_Recovery_State_t recovery_state;

void CAN_Recovery_Init(void)
{
    recovery_state = (CAN_Recovery_State_t){0};
    recovery_state.can_ready = TRUE;
}

void CAN_Recovery_DeInit(void)
{
    recovery_state.can_ready = FALSE;
}

void CAN_Recovery_UpdateRecoveryState(IO_ErrorType controls_status, IO_ErrorType daq_status)
{
    if (recovery_state.cooldown_cycles > 0) {
        recovery_state.cooldown_cycles--;
    }

    const bool any_bus_off = (controls_status == IO_E_CAN_BUS_OFF) ||
                             (daq_status == IO_E_CAN_BUS_OFF);
    const bool any_non_ok = (controls_status != IO_E_OK) ||
                            (daq_status != IO_E_OK);

    if (any_bus_off) {
        recovery_state.bus_off_cycles++;
    }
    else {
        recovery_state.bus_off_cycles = 0;
    }

    if (any_non_ok) {
        recovery_state.non_ok_cycles++;
    }
    else {
        recovery_state.non_ok_cycles = 0;
        recovery_state.recovery_attempts = 0;
    }

    if ((recovery_state.cooldown_cycles == 0) &&
        ((recovery_state.bus_off_cycles >= CAN_RECOVERY_BUS_OFF_TRIGGER_CYCLES) ||
         (recovery_state.non_ok_cycles >= CAN_RECOVERY_NON_OK_TRIGGER_CYCLES))) {
        recovery_state.recovery_requested = TRUE;
    }
}

void CAN_Recovery_RunRecovery(void (*init_callback)(void), void (*deinit_callback)(void))
{
    if (!recovery_state.recovery_requested) {
        return;
    }

    recovery_state.recovery_requested = FALSE;
    recovery_state.non_ok_cycles = 0;
    recovery_state.bus_off_cycles = 0;
    recovery_state.recovery_attempts++;

    deinit_callback();
    init_callback();

    recovery_state.cooldown_cycles = CAN_RECOVERY_COOLDOWN_CYCLES;
}

const CAN_Recovery_State_t* CAN_Recovery_GetState(void)
{
    return &recovery_state;
}

bool CAN_Recovery_IsReady(void)
{
    return recovery_state.can_ready;
}

void CAN_Recovery_MarkHealthFault(bool set_tx_fault, bool set_rx_fault)
{
    if (!recovery_state.can_ready) {
        // Recovery will mark health faults after init fails
        // This is handled in can_manager.c after calling recovery
    }
}
