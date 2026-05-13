#ifndef CAN_UTIL_H
#define CAN_UTIL_H

#include "IO_Constants.h"
#include "IO_CAN.h"

void CAN_Util_ClearData(IO_CAN_DATA_FRAME* frame);

/*
    Compact CAN status values for logging / telemetry.
*/
typedef enum {
    CAN_STATUS_OK = 0,
    CAN_STATUS_BUSY = 1,
    CAN_STATUS_OLD_DATA = 2,
    CAN_STATUS_OVERFLOW = 3,
    CAN_STATUS_FIFO_FULL = 4,
    CAN_STATUS_INVALID_DATA = 5,
    CAN_STATUS_ERROR_PASSIVE = 6,
    CAN_STATUS_BUS_OFF = 7,
    CAN_STATUS_WRONG_HANDLE = 8,
    CAN_STATUS_CHANNEL_NOT_CONFIGURED = 9,
    CAN_STATUS_INVALID_PARAMETER = 10,
    CAN_STATUS_NULL_POINTER = 11,
    CAN_STATUS_INVALID_CHANNEL_ID = 12,
    CAN_STATUS_MAX_MO_REACHED = 13,
    CAN_STATUS_MAX_HANDLES_REACHED = 14,
    CAN_STATUS_UNKNOWN = 15,
} CAN_Status_t;

CAN_Status_t CAN_Util_TranslateStatus(IO_ErrorType status);

/*
    Reads one frame(if any) from a given RX FIFO Buffer
*/
IO_ErrorType CAN_Util_ReadFIFO(ubyte1 handle, IO_CAN_DATA_FRAME* dest_data_frame, bool* msg_received);

/*
    Adds one frame to a TX FIFO Buffer
*/
IO_ErrorType CAN_Util_WriteFIFO(ubyte1 handle, const IO_CAN_DATA_FRAME* src_data_frame);

#endif // CAN_UTIL_H
