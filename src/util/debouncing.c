#include "debouncing.h"

void initialize_debouncing_struct(struct debouncing_info* info, bool starting_state, ubyte2 thresh) {
    info->current_state = starting_state;

    info->switch_threshold = thresh;

    info->count = 0;
}

bool debounce_input(struct debouncing_info* info, bool input) {
    //check if input is different from current state
    if (input != info->current_state) {
        info->count++;
    } else {
        info->count = 0;
    }

    //if number of successive new inputs >= the threshold
    if (info->count >= info->switch_threshold) {
        // flip state to new input
        info->current_state = input;
        
        //reset counter
        info->count = 0;
    }

    // return current state
    return info->current_state;
}