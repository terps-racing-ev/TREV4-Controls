#include "IO_Driver.h"
#include "IO_CAN.h"
#include "APDB.h"

#include "read_write_can.h"

void clear_can_frame(IO_CAN_DATA_FRAME* frame) {
    for (int i = 0; i < 8; i++) {
        frame->data[i] = 0;
    }
}


void read_can_msg(ubyte1* handle_r, IO_CAN_DATA_FRAME* dst_data_frame, bool *msg_received, ubyte4 id, ubyte1 channel, ubyte1 frame_type) {
    IO_ErrorType can_read_error;

    *msg_received = FALSE;

    can_read_error = IO_CAN_MsgStatus(*handle_r);

    if (can_read_error == IO_E_OK) {
        IO_CAN_ReadMsg(*handle_r, dst_data_frame);
        *msg_received = TRUE;
    } else if (can_read_error == IO_E_CAN_OVERFLOW) {
        IO_CAN_DeInitHandle(*handle_r);

        IO_CAN_ConfigMsg( handle_r
                , channel
                , IO_CAN_MSG_READ
                , frame_type
                , id
                , 0x7FF);
    }

}

void write_can_msg(ubyte1 handle, IO_CAN_DATA_FRAME* src_data_frame) {
    IO_ErrorType can_write_error = IO_CAN_FIFOStatus(handle);

    IO_CAN_WriteFIFO(handle, src_data_frame, 1);
}
