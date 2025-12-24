#include "IO_Constants.h"

#include "debounce.h"

void Debounce_Init(Debounce_Data_t* db, bool starting_state, ubyte2 threshold) {
    db->current_state = starting_state;

    db->switch_threshold = threshold;

    db->count = 0;
}

bool Debounce_Update(Debounce_Data_t* db, bool new_val) {

    if (db == NULL) {
        return db->current_state;
    }

    //check if input is different from current state
    if (new_val != db->current_state) {
        db->count++;
    } else {
        db->count = 0;
    }

    //if number of successive new inputs >= the threshold
    if (db->count >= db->switch_threshold) {
        // flip state to new input
        db->current_state = new_val;
        
        //reset counter
        db->count = 0;
    }

    // return current state
    return db->current_state;
}
