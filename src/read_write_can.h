#ifndef READ_WRITE_CAN_H_INCLUDED
#define READ_WRITE_CAN_H_INCLUDED

#include "IO_Driver.h"
#include "IO_CAN.h"
#include "APDB.h"

void clear_can_frame(IO_CAN_DATA_FRAME* frame);
void read_can_msg(ubyte1* handle_r, IO_CAN_DATA_FRAME* dst_data_frame, bool *msg_received, ubyte4 id, ubyte1 channel, ubyte1 frame_type);
void write_can_msg(ubyte1 handle, IO_CAN_DATA_FRAME* src_data_frame);

#endif