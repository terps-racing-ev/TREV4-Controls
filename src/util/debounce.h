#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include "IO_Constants.h"

typedef struct {
    bool current_state;

    // number of cycles needed to switch states
    ubyte2 switch_threshold;

    // number of successive inputs differing from current state
    ubyte2 count;
} Debounce_Data_t;

void Debounce_Init(Debounce_Data_t* db, bool starting_state, ubyte2 threshold);

bool Debounce_Update(Debounce_Data_t* db, bool new_val);

#endif // DEBOUNCE_H
