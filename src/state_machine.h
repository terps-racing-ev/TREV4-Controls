#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "IO_Constants.h"

typedef enum {
    VCU_STATE_NOT_READY = 0,
    VCU_STATE_PLAYING_RTD_SOUND = 1,
    VCU_STATE_DRIVING = 2,
    VCU_STATE_SOFT_FAULT = 3,
    VCU_STATE_HARD_FAULT = 4
} VCU_State_t;

void StateMachine_Init(void);
void StateMachine_Update(void);

VCU_State_t StateMachine_GetState(void);

#endif // STATE_MACHINE_H
