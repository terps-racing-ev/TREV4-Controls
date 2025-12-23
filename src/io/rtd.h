#ifndef RTD_H
#define RTD_H

#include "IO_Constants.h"

#include "util/units.h"

/* RTD -> pin 263 (aka digital in 0) (switched to ground with pullup) */
#define IO_PIN_RTD IO_DI_00 // TODO put this and apps in one hardware file?

#define RTD_DB_THRESHOLD MsToCycles(40)

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