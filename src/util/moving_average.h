#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include "IO_Constants.h"

#define MAX_WINDOW_SIZE 32

#define EWMA_WEIGHT_1S 0.1f TODO

#define EWMA(cur_value, new_value, weight) \
    (curr_val*(1.0f-weight) + new_val*weight)

typedef struct {
    ubyte2 window[MAX_WINDOW_SIZE];
    ubyte4 sum;
    ubyte1 index;
    ubyte1 window_size;
} MovingAverage_Data_t;

void MovingAverage_Init(MovingAverage_Data_t* ma, ubyte1 window_size);

ubyte2 MovingAverage_Update(MovingAverage_Data_t* ma, ubyte2 new_val);

float4 MovingAverage_EWMA(float4 curr_val, float4 new_val, float4 weight);


#endif // MOVING_AVERAGE_H
