#ifndef CAN_UTIL_H
#define CAN_UTIL_H

#include "APDB.h"
#include "IO_CAN.h"

void CAN_Util_ClearData(IO_CAN_DATA_FRAME* frame);

/*
    Reads one frame(if any) from a given RX FIFO Buffer
*/
IO_ErrorType CAN_Util_ReadFIFO(ubyte1 handle, IO_CAN_DATA_FRAME* dest_data_frame, bool* msg_received);

/*
    Adds one frame to a TX FIFO Buffer
*/
IO_ErrorType CAN_Util_WriteFIFO(ubyte1 handle, const IO_CAN_DATA_FRAME* src_data_frame);

#endif