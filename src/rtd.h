#ifndef RTD_H
#define RTD_H

#include "IO_Constants.h"

/* RTD -> pin 263 (aka digital in 0) (switched to ground with pullup) */
#define IO_PIN_RTD IO_DI_00 // TODO put this and apps in one hardware file


void RTD_Init(void);
void RTD_Update(void);

// TODO use same approach with struct like apps?
bool RTD_IsActive(void);

#endif // RTD_H