#include "IO_Constants.h"

#include "moving_average.h"

void MovingAverage_Init(MovingAverage_Data_t* ma, ubyte1 window_size)
{
    if (ma == NULL || window_size == 0 || window_size > MAX_WINDOW_SIZE) {
        return;
    }

    ma->window_size = window_size;
    ma->sum = 0;
    ma->index = 0;
    for (ubyte1 i = 0; i < MAX_WINDOW_SIZE; i++) {
        ma->window[i] = 0;
    }
}

ubyte2 MovingAverage_Update(MovingAverage_Data_t* ma, ubyte2 new_val)
{
    if (ma == NULL) {
        return new_val;
    }
    // Swap current value
    ma->sum -= ma->window[ma->index];
    ma->window[ma->index] = new_val;
    ma->sum += new_val;

    // Increase index
    ma->index++;
    if (ma->index >= ma->window_size) {
        ma->index = 0;
    }

    return (ubyte2)(ma->sum / ma->window_size);
}

float4 MovingAverage_EWMA(float4 curr_val, float4 new_val, float4 weight)
{
    return curr_val*(1.0f-weight) + new_val*weight;
}