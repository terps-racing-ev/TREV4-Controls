#include "IO_Constants.h"
#include "IO_CAN.h"

#include "can_util.h"

/**************************************************************************
*                   P U B L I C    F U N C T I O N S
**************************************************************************/

void CAN_Util_ClearData(IO_CAN_DATA_FRAME* frame) {
    for (int i = 0; i < 8; i++) {
        frame->data[i] = 0;
    }
}


IO_ErrorType CAN_Util_ReadFIFO(ubyte1 handle, IO_CAN_DATA_FRAME* dest_data_frame, bool* msg_received)
{
    IO_ErrorType status;
    ubyte1 rxed_frames = 0;

    // Validate inputs
    if (dest_data_frame == NULL || msg_received == NULL) {
        return IO_E_INVALID_PARAMETER;
    }

    *msg_received = FALSE;

    status = IO_CAN_FIFOStatus(handle);

    if (status == IO_E_OK) {
        status = IO_CAN_ReadFIFO(handle, dest_data_frame, 1, &rxed_frames);
        if (status == IO_E_OK && rxed_frames > 0) {
            *msg_received = TRUE;
        }
    }
    else if (status == IO_E_CAN_OLD_DATA) {
        return IO_E_CAN_OLD_DATA;
    }

    return status;

}

IO_ErrorType CAN_Util_WriteFIFO(ubyte1 handle, const IO_CAN_DATA_FRAME* src_data_frame)
{
    IO_ErrorType status;
    
    // Validate inputs
    if (src_data_frame == NULL) {
        return IO_E_INVALID_PARAMETER;
    }
    
    /* For TX FIFOs, IO_CAN_FIFOStatus() can return IO_E_BUSY simply because
       transmission is ongoing. That does not imply the FIFO is full.
       Allow enqueue while BUSY and let IO_CAN_WriteFIFO() report FIFO_FULL
       when capacity is exhausted. */
    status = IO_CAN_FIFOStatus(handle);

    if ((status == IO_E_OK) || (status == IO_E_BUSY) || (status == IO_E_CAN_OLD_DATA)) {
        status = IO_CAN_WriteFIFO(handle, src_data_frame, 1);
    }
    
    return status;
}
