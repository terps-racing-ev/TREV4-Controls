#ifndef BUZZER_H
#define BUZZER_H

#include "IO_Constants.h"

// TODO maybe we can make a song using this
typedef enum {
    BUZZER_STATE_INACTIVE = 0,
    BUZZER_STATE_IN_PROG = 1,
    BUZZER_STATE_DONE = 2,
} Buzzer_State_t;

void Buzzer_Init(void);
void Buzzer_Update(void);
Buzzer_State_t Buzzer_GetState(void);


#endif // BUZZER_H