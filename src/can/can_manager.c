/*
 * CAN Manager
 * - Owns RX/TX message tables
 * - Initializes CAN channels + FIFOs
 * - Schedules TX messages
 */

#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "can_tx_pack.h"
#include "can_rx_unpack.h"
#include "config/can_config.h"
#include "state_machine.h"

/*
 * Internal message-slot IDs and message table structs.
 * These are intentionally private to this module.
 */

typedef enum {
    CAN_RX_MSG_INV_STATUS,
    CAN_RX_MSG_INV_HIGH_SPEED,
    CAN_RX_MSG_HVC_SUMMARY,
    CAN_RX_MSG_VCU_CONFIG_SET,

    CAN_RX_MSG_COUNT
} CAN_RX_MessageId_t;

typedef struct {
    const ubyte1 channel;
    const ubyte1 id_format;
    const ubyte4 id;
    ubyte1 handle;

    const ubyte4 timeout_us;
    void* data; // Pointer to an RX_data struct
    void (*unpack_fn)(IO_CAN_DATA_FRAME*, void*);
    ubyte4 last_rx_timestamp;
    bool data_vld;
} CAN_RX_Message_t;

typedef enum {
    CAN_TX_MSG_INV_TORQUE_COMMAND,
    CAN_TX_MSG_INV_READ_WRITE,
    CAN_TX_MSG_APPS_VALUES,
    CAN_TX_MSG_APPS_VOLTAGES,
    CAN_TX_MSG_VCU_SUMMARY,
    CAN_TX_MSG_BSE,
    CAN_TX_MSG_VCU_SETTINGS,

    CAN_TX_MSG_COUNT
} CAN_TX_MessageId_t;

typedef struct {
    const ubyte1 channel;
    const ubyte1 id_format;
    const ubyte4 id;
    const ubyte4 period_us;
    void (*pack_fn)(IO_CAN_DATA_FRAME*);
} CAN_TX_Message_t;

/* Mirror all DAQ (telemetry) TX frames onto the controls bus as well. */
#define CAN_DEBUG_MIRROR_DAQ_TX_TO_CONTROLS 1

/**************************************************************************
*      P R I V A T E    F U N C T I O N    D E C L A R A T I O N S
**************************************************************************/

static void CAN_Manager_ProcessTxMessage(const CAN_TX_MessageId_t msg_id,
                                        const CAN_TX_Message_t* msg,
                                        const VCU_State_t vcu_state);

/**************************************************************************
*          P R I V A T E    D A T A    D E F I N I T I O N S
**************************************************************************/

/* TX FIFO handles
   Note: on XC2000 MultiCAN message objects, a FIFO's ID format (STD vs EXT)
   is configured at FIFO creation time. Keep separate FIFOs per format so
   STD frames aren't pushed through an EXT-configured FIFO (and vice versa). */

#define CONTROLS_TX_STD_FIFO_BUFFER_SIZE (TX_FIFO_BUFFER_SIZE / 2)
#define CONTROLS_TX_EXT_FIFO_BUFFER_SIZE (TX_FIFO_BUFFER_SIZE / 2)

static ubyte1 controls_tx_std_fifo_handle;
static ubyte1 controls_tx_ext_fifo_handle;
static ubyte1 daq_tx_ext_fifo_handle;

// Error counters TODO implement
static ubyte1 controls_tx_error_ctr;
static ubyte1 controls_rx_error_ctr;

static ubyte1 daq_tx_error_ctr;
static ubyte1 daq_rx_error_ctr;

/**************
* TX scheduling
**************/
static ubyte4 tx_last_timestamp[CAN_TX_MSG_COUNT];
static bool tx_timer_initialized[CAN_TX_MSG_COUNT];

/* Force-send requests */
static bool vcu_settings_tx_requested;


/**************************************************************************
* RX Messages
**************************************************************************/
static InverterStatus_RX_Data_t inverter_status_rx_data = {0};
static InverterHighSpeed_RX_Data_t inverter_high_speed_rx_data = {0};
static HVCSummary_RX_Data_t hvc_summary_rx_data = {0};
static VCUConfigSet_RX_Data_t vcu_config_set_rx_data = {0};

static CAN_RX_Message_t rx_messages[CAN_RX_MSG_COUNT] = {
    [CAN_RX_MSG_INV_STATUS] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_STATUS,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &inverter_status_rx_data,
        .unpack_fn = CAN_RX_UnpackInverterStatus
    },
    [CAN_RX_MSG_INV_HIGH_SPEED] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_HIGH_SPEED,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &inverter_high_speed_rx_data,
        .unpack_fn = CAN_RX_UnpackInverterHighSpeed
    },
    [CAN_RX_MSG_HVC_SUMMARY] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = 0, //TODO
        .timeout_us = MSG_TIMEOUT_US,
        .data = &hvc_summary_rx_data,
        .unpack_fn = CAN_RX_UnpackHVCSummary
    },
    [CAN_RX_MSG_VCU_CONFIG_SET] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_VCU_CONFIG_SET,
        .timeout_us = MSG_TIMEOUT_US,
        .data = &vcu_config_set_rx_data,
        .unpack_fn = CAN_RX_UnpackVCUConfigSet
    },
};


/**************************************************************************
* TX Messages
**************************************************************************/

static const CAN_TX_Message_t tx_messages[CAN_TX_MSG_COUNT] = {
    [CAN_TX_MSG_INV_TORQUE_COMMAND] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_TORQUE_COMMAND,
        .period_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackInvTorqueCommand
    },
    [CAN_TX_MSG_INV_READ_WRITE] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_READ_WRITE,
        .period_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackInvReadWrite
    },
    [CAN_TX_MSG_APPS_VALUES] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_APPS_VALUES,
        .period_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackAPPSValues
    },
    [CAN_TX_MSG_APPS_VOLTAGES] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_APPS_VOLTAGES,
        .period_us = CAN_TX_RATE_5MS,
        .pack_fn = CAN_TX_PackAPPSVoltages
    },
    [CAN_TX_MSG_VCU_SUMMARY] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_VCU_SUMMARY,
        .period_us = CAN_TX_RATE_100MS,
        .pack_fn = CAN_TX_PackVCUSummary
    },
    [CAN_TX_MSG_BSE] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_BSE,
        .period_us = CAN_TX_RATE_10MS,
        .pack_fn = CAN_TX_PackBSE
    },
    [CAN_TX_MSG_VCU_SETTINGS] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_VCU_SETTINGS,
        .period_us = CAN_TX_RATE_1000MS,
        .pack_fn = CAN_TX_PackVCUSettings
    },
};

void CAN_Manager_Init(void)
{
    ubyte1 i;
    CAN_RX_Message_t* msg;

    vcu_settings_tx_requested = FALSE;
    for (i = 0; i < CAN_TX_MSG_COUNT; i++) {
        tx_timer_initialized[i] = FALSE;
    }

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

    /* Controls bus uses both STD and EXT frames. */
    IO_CAN_ConfigFIFO( &controls_tx_std_fifo_handle
                     , CONTROLS_CAN_CHANNEL
                     , CONTROLS_TX_STD_FIFO_BUFFER_SIZE
                     , IO_CAN_MSG_WRITE
                     , IO_CAN_STD_FRAME
                     , 0
                     , 0);

    IO_CAN_ConfigFIFO( &controls_tx_ext_fifo_handle
                     , CONTROLS_CAN_CHANNEL
                     , CONTROLS_TX_EXT_FIFO_BUFFER_SIZE
                     , IO_CAN_MSG_WRITE
                     , IO_CAN_EXT_FRAME
                     , 0
                     , 0);

    /* DAQ channel is currently EXT-only. */
    IO_CAN_ConfigFIFO( &daq_tx_ext_fifo_handle
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
    const VCU_State_t vcu_state = StateMachine_GetState();
    ubyte1 i;

    for (i = 0; i < CAN_TX_MSG_COUNT; i++) {
        CAN_Manager_ProcessTxMessage((CAN_TX_MessageId_t)i, &tx_messages[i], vcu_state);
    }
}

void CAN_Manager_RequestVcuSettingsTx(void)
{
    vcu_settings_tx_requested = TRUE;
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

    CAN_Util_WriteFIFO(controls_tx_ext_fifo_handle, &debug_frame);
    CAN_Util_WriteFIFO(daq_tx_ext_fifo_handle, &debug_frame);
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


/**************************************************************************
*                  P R I V A T E    F U N C T I O N S
**************************************************************************/

static void CAN_Manager_ProcessTxMessage(const CAN_TX_MessageId_t msg_id,
                                        const CAN_TX_Message_t* msg,
                                        const VCU_State_t vcu_state)
{
    IO_CAN_DATA_FRAME tx_frame;
    ubyte1 tx_fifo_handle;
    bool due;

    if ((msg == NULL) || (msg->pack_fn == NULL) || (msg->period_us == 0)) {
        return;
    }

    if ((ubyte1)msg_id >= CAN_TX_MSG_COUNT) {
        return;
    }

    if (!tx_timer_initialized[msg_id]) {
        /* Send immediately the first time this message is processed. */
        due = TRUE;
    }
    else {
        due = (IO_RTC_GetTimeUS(tx_last_timestamp[msg_id]) >= msg->period_us);
    }

    /* Allow an on-demand send for VCU settings (e.g., right after a config SET). */
    if ((msg_id == CAN_TX_MSG_VCU_SETTINGS) && vcu_settings_tx_requested) {
        due = TRUE;
    }

    if (!due) {
        return;
    }

    if ((msg->id == CAN_ID_INV_READ_WRITE) && (vcu_state != VCU_STATE_PLAYING_RTD_SOUND)) {
        return;
    }

    if (msg->channel == CONTROLS_CAN_CHANNEL) {
        tx_fifo_handle = (msg->id_format == IO_CAN_STD_FRAME)
                         ? controls_tx_std_fifo_handle
                         : controls_tx_ext_fifo_handle;
    }
    else if (msg->channel == DAQ_CAN_CHANNEL) {
        tx_fifo_handle = daq_tx_ext_fifo_handle;
    }
    else {
        return;
    }

    msg->pack_fn(&tx_frame);

    tx_frame.id_format = msg->id_format;
    tx_frame.id = msg->id;
    CAN_Util_WriteFIFO(tx_fifo_handle, &tx_frame);

#if CAN_DEBUG_MIRROR_DAQ_TX_TO_CONTROLS
    /* Debug-only: mirror telemetry/DAQ traffic onto the controls bus. */
    if (msg->channel == DAQ_CAN_CHANNEL) {
        CAN_Util_WriteFIFO(controls_tx_ext_fifo_handle, &tx_frame);
    }
#endif

    if (msg_id == CAN_TX_MSG_VCU_SETTINGS) {
        vcu_settings_tx_requested = FALSE;
    }

    IO_RTC_StartTime(&tx_last_timestamp[msg_id]);
    tx_timer_initialized[msg_id] = TRUE;
}