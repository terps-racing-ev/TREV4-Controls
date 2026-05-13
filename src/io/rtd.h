#ifndef RTD_H
#define RTD_H

#include "IO_Constants.h"
// TODO CHEKC THE CAN IO BUFFERS WHAT WE NEED
#define RTD_ON FALSE
#define RTD_OFF TRUE

typedef struct {
    bool    state;
    bool    raw_state;
    
    bool    valid;
} RTD_Data_t;

void RTD_Init(void);
void RTD_Update(void);

// TODO use same approach with struct like apps?
bool RTD_IsActive(void);

#endif // RTD_H
