#include "IO_Constants.h"

#include "moving_average.h"

IO_ErrorType MovingAverage_Init(MovingAverage_Data_t* ma, ubyte1 window_size)
{
    ubyte1 i;   // Index
    
    if (ma == NULL) {
        return IO_E_NULL_POINTER;
    }
    if (window_size == 0 || window_size > MAX_WINDOW_SIZE) {
        return IO_E_INVALID_PARAMETER;
    }
    ma->window_size = window_size;
    ma->sum = 0;
    ma->index = 0;
    for (i = 0; i < MAX_WINDOW_SIZE; i++) {
        ma->window[i] = 0;
    }
    return IO_E_OK;
}

IO_ErrorType MovingAverage_Update(MovingAverage_Data_t* ma, ubyte2 new_val, ubyte2* filtered_val)
{
    if (ma == NULL || filtered_val == NULL) {
        return IO_E_NULL_POINTER;
    }

    // Swap current value
    ma->sum -= ma->window[ma->index];
    ma->window[ma->index] = new_val;
    ma->sum += new_val;

    // Update average
    *filtered_val = (ubyte2)(ma->sum / ma->window_size);

    // Increase index
    ma->index++;
    if (ma->index >= ma->window_size) {
        ma->index = 0;
    }
    return IO_E_OK;
}
