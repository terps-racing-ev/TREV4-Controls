#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "IO_Constants.h"

typedef enum {
    VCU_STATE_LOADING = 0,
    VCU_STATE_NOT_READY = 1,
    VCU_STATE_PLAYING_RTD_SOUND = 2,
    VCU_STATE_DRIVING = 3,
    VCU_STATE_BAP_FAULT = 4, // Brake Accel Plausibility
    VCU_STATE_HARD_FAULT = 5
} VCU_State_t;

void StateMachine_Init(void);
void StateMachine_Update(void);

VCU_State_t StateMachine_GetState(void);
bool StateMachine_ClearFaultsTxTrigger(void);
bool StateMachine_DeadCarTxTrigger(void);

#endif // STATE_MACHINE_H
