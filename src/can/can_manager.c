#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "can_tx_messages.h"
#include "can_rx_messages.h"
#include "config/can_config.h"

/* TX FIFO handles
    Only need one FIFO for each channel */
static ubyte1 controls_tx_fifo_handle;
static ubyte1 telemetry_tx_fifo_handle;

// Error counters TODO implement
ubyte1 controls_tx_error_ctr;
ubyte1 controls_rx_error_ctr;

ubyte1 telemetry_tx_error_ctr;
ubyte1 telemetry_rx_error_ctr;


/**************************************************************************
* RX Messages
**************************************************************************/
static InverterStatus_RX_Data_t inverter_status_rx_data = {0};
static InverterHighSpeed_RX_Data_t inverter_high_speed_rx_data = {0};
static HVCSummary_RX_Data_t hvc_summary_rx_data = {0};

static CAN_RX_Message_t rx_messages[] = {
    {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_INV_STATUS,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &inverter_status_rx_data,
        .unpack_fn = CAN_RX_UnpackInverterStatus
    },
    {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_INV_HIGH_SPEED,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &inverter_high_speed_rx_data,
        .unpack_fn = CAN_RX_UnpackInverterHighSpeed
    },
    {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = 0, //TODO
        .timeout_us = MSG_TIMEOUT_US,
        .data = &hvc_summary_rx_data,
        .unpack_fn = NULL
    }
};

#define NUM_RX_MESSAGES (sizeof(rx_messages) / sizeof(rx_messages[0]))

void CAN_Manager_Init(void)
{
    ubyte1 i;
    CAN_RX_Message_t* msg;

    /* initialize CAN channel and FIFO buffer */
    IO_CAN_Init( CONTROLS_CAN_CHANNEL
               , BAUD_RATE
               , 0
               , 0
               , 0);

    IO_CAN_Init( TELEMETRY_CAN_CHANNEL
               , BAUD_RATE
               , 0
               , 0
               , 0);
    

    /* Initialize FIFOs for EACH MESSAGE we expect to RX */
    for (i = 0; i < NUM_RX_MESSAGES; i++) {
        msg = &rx_messages[i];

        IO_CAN_ConfigFIFO( &msg->handle,
                            msg->channel,
                            RX_FIFO_BUFFER_SIZE,
                            IO_CAN_MSG_READ,
                            msg->id_format,
                            msg->id,
                            0x1FFFFFFF);
    }

    /* Initialize FIFOs for Both TX Channels */
    IO_CAN_ConfigFIFO( &controls_tx_fifo_handle
    				 , CONTROLS_CAN_CHANNEL
    				 , TX_FIFO_BUFFER_SIZE
    				 , IO_CAN_MSG_WRITE
    				 , IO_CAN_EXT_FRAME
    				 , 0
    				 , 0);

    IO_CAN_ConfigFIFO( &telemetry_tx_fifo_handle
                    , TELEMETRY_CAN_CHANNEL
                    , TX_FIFO_BUFFER_SIZE
                    , IO_CAN_MSG_WRITE
                    , IO_CAN_EXT_FRAME
                    , 0
                    , 0);
}

// TODO add deinit and reinit on failure
void CAN_Manager_ProcessRxMessages(void)
{
    IO_CAN_DATA_FRAME rx_frame;
    bool received;
    ubyte4 curr_time;
    ubyte1 i;
    CAN_RX_Message_t* msg;

    curr_time = IO_RTC_GetTimeUS(0);

    for (i = 0; i < NUM_RX_MESSAGES; i++) {
        msg = &rx_messages[i];
        
        // Skip if unpack function not configured
        if (msg->unpack_fn == NULL) {
            continue;
        }
        
        CAN_Util_ReadFIFO(msg->handle, &rx_frame, &received);
        if (received) {
            msg->unpack_fn(&rx_frame, msg->data);
            msg->data_vld = TRUE;
            msg->last_rx_timestamp = curr_time;
        }
        else if (curr_time - msg->last_rx_timestamp > msg->timeout_us) {
            msg->data_vld = FALSE;
        }
    }
}

void CAN_Manager_ProcessTxMessages(void)
{
    // Reuse one frame for every tx
    IO_CAN_DATA_FRAME tx_frame;

    // TODO maybe this could be an array

    /* Torque */
    CAN_TX_PackTorqueCommand(&tx_frame);
    CAN_Util_WriteFIFO(controls_tx_fifo_handle, &tx_frame);

    /* APPS */
    CAN_TX_PackAPPSValues(&tx_frame);
    CAN_Util_WriteFIFO(controls_tx_fifo_handle, &tx_frame);

    CAN_TX_PackAPPSVoltages(&tx_frame);
    CAN_Util_WriteFIFO(controls_tx_fifo_handle, &tx_frame);
}

void CAN_Manager_Print(ubyte4 can_id, ubyte2 data)
{
    IO_CAN_DATA_FRAME debug_frame;
    debug_frame.id = can_id;
    debug_frame.id_format = IO_CAN_EXT_FRAME;
    debug_frame.length = 8;
    CAN_Util_ClearData(&debug_frame);
    
    debug_frame.data[0] = data & 0xFF;
    debug_frame.data[1] = data >> 8;

    CAN_Util_WriteFIFO(controls_tx_fifo_handle, &debug_frame);
    CAN_Util_WriteFIFO(telemetry_tx_fifo_handle, &debug_frame);
}

/* Getters */

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void)
{
    return &inverter_status_rx_data;
}

const InverterHighSpeed_RX_Data_t* CAN_Manager_GetInverterHighSpeedData(void)
{
    return &inverter_high_speed_rx_data;
}

const HVCSummary_RX_Data_t* CAN_Manager_GetHVCSummaryData(void)
{
    return &hvc_summary_rx_data;
}