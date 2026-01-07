#ifndef BUZZER_H
#define BUZZER_H

#include "IO_Constants.h"

// TODO maybe we can make a song using this
typedef enum {
    BUZZER_STATE_INACTIVE = 0,
    BUZZER_STATE_IN_PROG = 1
} Buzzer_State_t;

void Buzzer_Init(void);
Buzzer_State_t Buzzer_Play(void); // TODO maybe make get state or is that overkill
void Buzzer_Stop(void);


#endif // BUZZER_H