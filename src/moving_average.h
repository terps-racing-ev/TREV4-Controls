#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include "IO_Constants.h"

#define MAX_WINDOW_SIZE 32

typedef struct {
    ubyte2 window[MAX_WINDOW_SIZE];
    ubyte4 sum;
    ubyte1 index;
    ubyte1 window_size;
} MovingAverage_Data_t;

IO_ErrorType MovingAverage_Init(MovingAverage_Data_t* ma, ubyte1 window_size);

IO_ErrorType MovingAverage_Update(MovingAverage_Data_t* ma, ubyte2 new_val, ubyte2* filtered_val);


#endif // MOVING_AVERAGE_H
