#ifndef BSE_H
#define BSE_H

#include "IO_Constants.h"
#include "IO_ADC.h"

typedef struct {
    ubyte2  psi;
    bool    brakes_engaged;
                    // psi > BRAKES_ENGAGED_THRESHOLD
    bool    hard_braking;
                    // psi > BRAKE_PLAUSIBILITY_THRESHOLD

    ubyte2  raw_mv;
    ubyte2  filt_mv;
    
    bool    valid;  // !out_of_range && !stale && !adc_err

    bool    out_of_range;    
    bool    stale;
    bool    adc_err;

    ubyte4 last_update_us;
} BSE_Data_t;

void BSE_Init(void);
void BSE_Update(void);

const BSE_Data_t* BSE_GetData(void);

#endif // BSE_H
