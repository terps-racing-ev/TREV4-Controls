#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "can_tx_pack.h"
#include "can_rx_unpack.h"
#include "config/can_config.h"
#include "state_machine.h"

/* TX FIFO handles
    Only need one FIFO for each channel */
static ubyte1 controls_tx_fifo_handle;
static ubyte1 daq_tx_fifo_handle;

// Error counters TODO implement
ubyte1 controls_tx_error_ctr;
ubyte1 controls_rx_error_ctr;

ubyte1 daq_tx_error_ctr;
ubyte1 daq_rx_error_ctr;


/**************************************************************************
* RX Messages
**************************************************************************/
static InverterStatus_RX_Data_t inverter_status_rx_data = {0};
static InverterHighSpeed_RX_Data_t inverter_high_speed_rx_data = {0};
static HVCSummary_RX_Data_t hvc_summary_rx_data = {0};

static CAN_RX_Message_t rx_messages[CAN_RX_MSG_COUNT] = {
    {
        // Inverter RX Data
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_STATUS,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &inverter_status_rx_data,
        .unpack_fn = CAN_RX_UnpackInverterStatus
    },
    {
        // Inverter High Speed Message
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_HIGH_SPEED,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &inverter_high_speed_rx_data,
        .unpack_fn = CAN_RX_UnpackInverterHighSpeed
    },
    {
        // HVC Summary
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = 0, //TODO
        .timeout_us = MSG_TIMEOUT_US,
        .data = &hvc_summary_rx_data,
        .unpack_fn = CAN_RX_UnpackHVCSummary
    }
};

/**************************************************************************
* TX Messages
**************************************************************************/

static const CAN_TX_Message_t tx_messages[CAN_TX_MSG_COUNT] = {
    {
        // Inverter Torque Command
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_TORQUE_COMMAND,
        .rate_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackInvTorqueCommand
    },
    {
        // Inverter Read/Write
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_READ_WRITE,
        .rate_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackInvReadWrite
    },
    {
        // APPS Values
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_APPS_VALUES,
        .rate_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackAPPSValues
    },
    {
        // APPS Voltages
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_APPS_VOLTAGES,
        .rate_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackAPPSVoltages
    }
};


/**************************************************************************
* Public Functions
**************************************************************************/

static void CAN_Manager_ProcessTxMessage(const CAN_TX_Message_t* msg, const VCU_State_t vcu_state, ubyte4 tx_elapsed_us)
{
    IO_CAN_DATA_FRAME tx_frame;
    ubyte1 tx_fifo_handle;

    if ((msg == NULL) || (msg->pack_fn == NULL) || (msg->rate_us == 0)) {
        return;
    }

    // TODO better system for this
    if ((tx_elapsed_us % msg->rate_us) != 0) {
        return;
    }

    if ((msg->id == CAN_ID_INV_READ_WRITE) && (vcu_state != VCU_STATE_PLAYING_RTD_SOUND)) {
        return;
    }

    if (msg->channel == CONTROLS_CAN_CHANNEL) {
        tx_fifo_handle = controls_tx_fifo_handle;
    }
    else if (msg->channel == DAQ_CAN_CHANNEL) {
        tx_fifo_handle = daq_tx_fifo_handle;
    }
    else {
        return;
    }

    msg->pack_fn(&tx_frame);
    tx_frame.id_format = msg->id_format;
    tx_frame.id = msg->id;
    CAN_Util_WriteFIFO(tx_fifo_handle, &tx_frame);
}

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

    IO_CAN_Init( DAQ_CAN_CHANNEL
               , BAUD_RATE
               , 0
               , 0
               , 0);
    

    /* Initialize FIFOs for EACH MESSAGE we expect to RX */
    for (i = 0; i < CAN_RX_MSG_COUNT; i++) {
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

    IO_CAN_ConfigFIFO( &daq_tx_fifo_handle
                    , DAQ_CAN_CHANNEL
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

    for (i = 0; i < CAN_RX_MSG_COUNT; i++) {
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
    static ubyte4 tx_elapsed_us;
    const VCU_State_t vcu_state = StateMachine_GetState();
    ubyte1 i;

    for (i = 0; i < CAN_TX_MSG_COUNT; i++) {
        CAN_Manager_ProcessTxMessage(&tx_messages[i], vcu_state, tx_elapsed_us);
    }

    tx_elapsed_us += CAN_TX_PROCESS_CYCLE_US;
    if (tx_elapsed_us >= CAN_TX_RATE_1000MS) {
        tx_elapsed_us = 0;
    }
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
    CAN_Util_WriteFIFO(daq_tx_fifo_handle, &debug_frame);
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