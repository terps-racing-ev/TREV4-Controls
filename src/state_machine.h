#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "IO_Constants.h"

typedef enum {
    VCU_STATE_NOT_READY = 0,
    VCU_STATE_PLAYING_RTD_SOUND = 1,
    VCU_STATE_DRIVING = 2,
    VCU_STATE_SOFT_ERROR = 3,
    VCU_STATE_HARD_ERROR = 4
} VCU_State_t;

void StateMachine_Init(void);
void StateMachine_Update(void);

#endif // STATE_MACHINE_H