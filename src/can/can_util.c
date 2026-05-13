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

CAN_Status_t CAN_Util_TranslateStatus(IO_ErrorType status)
{
    switch (status) {
    case IO_E_OK:
        return CAN_STATUS_OK;
    case IO_E_BUSY:
        return CAN_STATUS_BUSY;
    case IO_E_CAN_OLD_DATA:
        return CAN_STATUS_OLD_DATA;
    case IO_E_CAN_OVERFLOW:
        return CAN_STATUS_OVERFLOW;
    case IO_E_CAN_FIFO_FULL:
        return CAN_STATUS_FIFO_FULL;
    case IO_E_CAN_INVALID_DATA:
        return CAN_STATUS_INVALID_DATA;
    case IO_E_CAN_ERROR_PASSIVE:
        return CAN_STATUS_ERROR_PASSIVE;
    case IO_E_CAN_BUS_OFF:
        return CAN_STATUS_BUS_OFF;
    case IO_E_CAN_WRONG_HANDLE:
        return CAN_STATUS_WRONG_HANDLE;
    case IO_E_CHANNEL_NOT_CONFIGURED:
        return CAN_STATUS_CHANNEL_NOT_CONFIGURED;
    case IO_E_INVALID_PARAMETER:
        return CAN_STATUS_INVALID_PARAMETER;
    case IO_E_NULL_POINTER:
        return CAN_STATUS_NULL_POINTER;
    case IO_E_INVALID_CHANNEL_ID:
        return CAN_STATUS_INVALID_CHANNEL_ID;
    case IO_E_CAN_MAX_MO_REACHED:
        return CAN_STATUS_MAX_MO_REACHED;
    case IO_E_CAN_MAX_HANDLES_REACHED:
        return CAN_STATUS_MAX_HANDLES_REACHED;
    default:
        return CAN_STATUS_UNKNOWN;
    }
}

IO_ErrorType CAN_Util_ReadFIFO(ubyte1 handle, IO_CAN_DATA_FRAME* dest_data_frame, bool* msg_received)
{
    // Validate inputs
    if (dest_data_frame == NULL || msg_received == NULL) {
        return IO_E_INVALID_PARAMETER;
    }

    *msg_received = FALSE;

    IO_ErrorType fifo_status = IO_CAN_FIFOStatus(handle);
    bool saw_overflow = FALSE;

    while ((fifo_status == IO_E_OK) || (fifo_status == IO_E_CAN_OVERFLOW)) {
        if (fifo_status == IO_E_CAN_OVERFLOW) {
            saw_overflow = TRUE;
        }

        ubyte1 rxed_frames = 0;
        const IO_ErrorType read_status = IO_CAN_ReadFIFO(handle, dest_data_frame, 1, &rxed_frames);

        if (read_status != IO_E_OK) {
            return read_status;
        }

        if (rxed_frames > 0) {
            *msg_received = TRUE;
        }

        fifo_status = IO_CAN_FIFOStatus(handle);
    }

    if (fifo_status == IO_E_CAN_OLD_DATA) {
        if (saw_overflow) {
            return IO_E_CAN_OVERFLOW;
        }

        if (*msg_received == TRUE) {
            return IO_E_OK;
        }

        return IO_E_CAN_OLD_DATA;
    }

    return fifo_status;

}

IO_ErrorType CAN_Util_WriteFIFO(ubyte1 handle, const IO_CAN_DATA_FRAME* src_data_frame)
{
    // Validate inputs
    if (src_data_frame == NULL) {
        return IO_E_INVALID_PARAMETER;
    }
    
    IO_ErrorType status = IO_CAN_FIFOStatus(handle);

    if ((status == IO_E_OK) || (status == IO_E_BUSY) || (status == IO_E_CAN_OLD_DATA)) {
        status = IO_CAN_WriteFIFO(handle, src_data_frame, 1);
    }
    
    return status;
}
