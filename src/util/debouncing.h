#ifndef DEBOUNCING_H_INCLUDED
#define DEBOUNCING_H_INCLUDED

#include "APDB.h"

#define THRESHOLD_100_MS 20 // 20 cycles -> 100 ms
#define THRESHOLD_500_MS 100 // 100 cycles -> 500 ms

struct debouncing_info {
    bool current_state;

    // number of cycles needed to switch states
    ubyte2 switch_threshold;

    // number of successive inputs differing from current state
    ubyte2 count;
};

void initialize_debouncing_struct(struct debouncing_info* info, bool starting_state, ubyte2 thresh);

bool debounce_input(struct debouncing_info* info, bool input);

#endif