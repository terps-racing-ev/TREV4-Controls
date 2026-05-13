#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "can_tx.h"
#include "can_rx.h"
#include "config/can_config.h"
#include "config/runtime_config.h"
#include "state_machine.h"

/**************************************************************************
*          P R I V A T E    D A T A    D E F I N I T I O N S
**************************************************************************/

static ubyte1 controls_tx_std_fifo_handle;
static ubyte1 controls_tx_ext_fifo_handle;
// currently no std messages being send on daq
static ubyte1 daq_tx_ext_fifo_handle;

static CAN_Manager_Health_t can_health;
static CAN_RX_Message_t rx_messages[CAN_RX_MSG_COUNT];
static CAN_TX_Message_t tx_messages[CAN_TX_MSG_COUNT];

typedef struct {
    ubyte2 non_ok_cycles;
    ubyte2 bus_off_cycles;
    ubyte2 cooldown_cycles;
    ubyte2 recovery_attempts;
    bool recovery_requested;
    bool can_ready;
} CAN_Manager_Recovery_t;

static CAN_Manager_Recovery_t can_recovery;





static void CAN_Manager_RunRecovery(void);

static void CAN_Manager_UpdateRecoveryStateFromStatus(IO_ErrorType controls_status,
                                                      IO_ErrorType daq_status)
{
    if (can_recovery.cooldown_cycles > 0) {
        can_recovery.cooldown_cycles--;
    }

    const bool any_bus_off = (controls_status == IO_E_CAN_BUS_OFF) ||
                             (daq_status == IO_E_CAN_BUS_OFF);
    const bool any_non_ok = (controls_status != IO_E_OK) ||
                            (daq_status != IO_E_OK);

    if (any_bus_off) {
        can_recovery.bus_off_cycles++;
    }
    else {
        can_recovery.bus_off_cycles = 0;
    }

    if (any_non_ok) {
        can_recovery.non_ok_cycles++;
    }
    else {
        can_recovery.non_ok_cycles = 0;
        can_recovery.recovery_attempts = 0;
    }

    if ((can_recovery.cooldown_cycles == 0) &&
        ((can_recovery.bus_off_cycles >= CAN_RECOVERY_BUS_OFF_TRIGGER_CYCLES) ||
         (can_recovery.non_ok_cycles >= CAN_RECOVERY_NON_OK_TRIGGER_CYCLES))) {
        can_recovery.recovery_requested = TRUE;
    }
}

static void CAN_Manager_RunRecovery(void)
{
    if (!can_recovery.recovery_requested) {
        return;
    }

    can_recovery.recovery_requested = FALSE;
    can_recovery.non_ok_cycles = 0;
    can_recovery.bus_off_cycles = 0;
    can_recovery.recovery_attempts++;

    CAN_Manager_DeInit();
    CAN_Manager_Init();
    
    if (!can_recovery.can_ready) {
        can_health.controls_tx_fault_seen = TRUE;
        can_health.controls_rx_fault_seen = TRUE;
        can_health.daq_tx_fault_seen = TRUE;
        can_health.daq_rx_fault_seen = TRUE;
    }

    can_recovery.cooldown_cycles = CAN_RECOVERY_COOLDOWN_CYCLES;
}

static void CAN_Manager_RecordFifoStatus(ubyte1 fifo_idx, IO_ErrorType status)
{
    CAN_HealthFifoData_t* const fifo_data = &can_health.fifos[fifo_idx];

    fifo_data->last_status_raw = (ubyte1)status;

    if (status == IO_E_CAN_FIFO_FULL) {
        fifo_data->fifo_full_count++;
    }
    else if (status == IO_E_CAN_OVERFLOW) {
        fifo_data->overflow_count++;
    }
    else if (status == IO_E_CAN_INVALID_DATA) {
        fifo_data->invalid_data_count++;
    }
    else if ((status != IO_E_OK) && (status != IO_E_BUSY) && (status != IO_E_CAN_OLD_DATA)) {
        fifo_data->other_error_count++;
    }

    switch (fifo_idx) {
    case CAN_HEALTH_TX_FIFO_CONTROLS_STD:
    case CAN_HEALTH_TX_FIFO_CONTROLS_EXT:
        if ((status != IO_E_OK) && (status != IO_E_BUSY)) {
            can_health.controls_tx_fault_seen = TRUE;
        }
        break;
    case CAN_HEALTH_TX_FIFO_DAQ_EXT:
        if ((status != IO_E_OK) && (status != IO_E_BUSY)) {
            can_health.daq_tx_fault_seen = TRUE;
        }
        break;
    default:
        if ((status != IO_E_OK) && (status != IO_E_CAN_OLD_DATA)) {
            can_health.controls_rx_fault_seen = TRUE;
        }
        break;
    }
}

static void CAN_Manager_UpdateChannelHealth(void)
{
    ubyte1 rx_error_counter = can_health.controls_rx_error_counter;
    ubyte1 tx_error_counter = can_health.controls_tx_error_counter;

    IO_ErrorType controls_status = IO_CAN_Status(CONTROLS_CAN_CHANNEL,
                                                 &rx_error_counter,
                                                 &tx_error_counter);

    can_health.controls_rx_error_counter = rx_error_counter;
    can_health.controls_tx_error_counter = tx_error_counter;
    can_health.controls_status_ret_raw = (ubyte1)controls_status;
    can_health.controls_error_passive = (controls_status == IO_E_CAN_ERROR_PASSIVE);
    can_health.controls_bus_off = (controls_status == IO_E_CAN_BUS_OFF);
    if (tx_error_counter > 0) {
        can_health.controls_tx_fault_seen = TRUE;
    }
    if (rx_error_counter > 0) {
        can_health.controls_rx_fault_seen = TRUE;
    }

    rx_error_counter = can_health.daq_rx_error_counter;
    tx_error_counter = can_health.daq_tx_error_counter;

    IO_ErrorType daq_status = IO_CAN_Status(DAQ_CAN_CHANNEL,
                                            &rx_error_counter,
                                            &tx_error_counter);

    can_health.daq_rx_error_counter = rx_error_counter;
    can_health.daq_tx_error_counter = tx_error_counter;
    can_health.daq_status_ret_raw = (ubyte1)daq_status;
    can_health.daq_error_passive = (daq_status == IO_E_CAN_ERROR_PASSIVE);
    can_health.daq_bus_off = (daq_status == IO_E_CAN_BUS_OFF);
    if (tx_error_counter > 0) {
        can_health.daq_tx_fault_seen = TRUE;
    }
    if (rx_error_counter > 0) {
        can_health.daq_rx_fault_seen = TRUE;
    }

    CAN_Manager_UpdateRecoveryStateFromStatus(controls_status, daq_status);
}

static CAN_HealthTxFifoId_t CAN_Manager_GetTxHealthFifoId(const CAN_TX_Message_t* msg)
{
    if (msg->channel == CONTROLS_CAN_CHANNEL) {
        if (msg->id_format == IO_CAN_STD_FRAME) {
            return CAN_HEALTH_TX_FIFO_CONTROLS_STD;
        }

        return CAN_HEALTH_TX_FIFO_CONTROLS_EXT;
    }

    return CAN_HEALTH_TX_FIFO_DAQ_EXT;
}

/**************************************************************************
* RX Messages
**************************************************************************/
static CAN_RX_Message_t rx_messages[CAN_RX_MSG_COUNT] = {
    [CAN_RX_MSG_INV_STATUS] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_STATUS,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackInverterStatus
    },
    [CAN_RX_MSG_INV_HIGH_SPEED] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_HIGH_SPEED,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackInverterHighSpeed
    },
    [CAN_RX_MSG_HVC_SUMMARY] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_HVC_SUMMARY,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackHVCSummary
    },
    [CAN_RX_MSG_SET_VCU_CONFIG] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_SET_VCU_CONFIG,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackSetVCUConfig
    },
};


/**************************************************************************
* TX Messages
**************************************************************************/

static CAN_TX_Message_t tx_messages[CAN_TX_MSG_COUNT] = {
    [CAN_TX_MSG_INV_TORQUE_COMMAND] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_TORQUE_COMMAND,
        .period_cycles = CAN_TX_RATE_10MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackInvTorqueCommand
    },
    [CAN_TX_MSG_INV_READ_WRITE] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_READ_WRITE,
        .period_cycles = CAN_TX_RATE_NON_PERIODIC,
        .tx_trigger_fn = StateMachine_ClearFaultsTxTrigger,
        .pack_fn = CAN_TX_PackInvReadWrite
    },
    [CAN_TX_MSG_APPS_VALUES] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_APPS_VALUES,
        .period_cycles = CAN_TX_RATE_10MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackAPPSValues
    },
    [CAN_TX_MSG_APPS_VOLTAGES] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_APPS_VOLTAGES,
        .period_cycles = CAN_TX_RATE_10MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackAPPSVoltages
    },
    [CAN_TX_MSG_VCU_SUMMARY] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_VCU_SUMMARY,
        .period_cycles = CAN_TX_RATE_10MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackVCUSummary
    },
    [CAN_TX_MSG_BSE] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_BSE,
        .period_cycles = CAN_TX_RATE_10MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackBSE
    },
    [CAN_TX_MSG_CONFIG] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_CONFIG,
        .period_cycles = CAN_TX_RATE_1000MS,
        .tx_trigger_fn = RuntimeConfig_ConfigTxTrigger,
        .pack_fn = CAN_TX_PackConfig
    },
    [CAN_TX_MSG_CAN_HEALTH] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_CAN_HEALTH,
        .period_cycles = CAN_TX_RATE_100MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackCANHealth
    },
    [CAN_TX_MSG_CAN_HEALTH_FIFO] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_CAN_HEALTH_FIFO,
        .period_cycles = CAN_TX_RATE_100MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackCANHealthFifo
    },
    [CAN_TX_MSG_DEAD_CAR] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_DEAD_CAR,
        .period_cycles = CAN_TX_RATE_NON_PERIODIC,
        .tx_trigger_fn = StateMachine_DeadCarTxTrigger,
        .pack_fn = CAN_TX_PackDeadCar
    },
};

void CAN_Manager_Init(void)
{
    can_health = (CAN_Manager_Health_t){0};

    for (ubyte1 i = 0; i < CAN_TX_MSG_COUNT; i++) {
        if (tx_messages[i].period_cycles == CAN_TX_RATE_NON_PERIODIC) {
            tx_messages[i].cycles_until_tx = 0;
        }
        else {
            tx_messages[i].cycles_until_tx = tx_messages[i].period_cycles;
        }
    }

    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        (void)IO_RTC_StartTime(&rx_messages[i].last_rx_timestamp);
        rx_messages[i].data_vld = FALSE;
    }

    /* initialize CAN channels */
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
    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        CAN_RX_Message_t* const msg = &rx_messages[i];

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
                     , TX_FIFO_BUFFER_SIZE
                     , IO_CAN_MSG_WRITE
                     , IO_CAN_STD_FRAME
                     , 0
                     , 0);

    IO_CAN_ConfigFIFO( &controls_tx_ext_fifo_handle
                     , CONTROLS_CAN_CHANNEL
                     , TX_FIFO_BUFFER_SIZE
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

    can_recovery = (CAN_Manager_Recovery_t){0};
    can_recovery.can_ready = TRUE;
}

void CAN_Manager_DeInit(void)
{
    can_recovery.can_ready = FALSE;

    /* Deinit all RX message handles */
    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        CAN_RX_Message_t* const msg = &rx_messages[i];

        (void)IO_CAN_DeInitHandle(msg->handle);
    }

    /* Deinit TX FIFO handles for both channels */

    /* Controls bus uses both STD and EXT frames. */
    (void)IO_CAN_DeInitHandle(controls_tx_std_fifo_handle);

    (void)IO_CAN_DeInitHandle(controls_tx_ext_fifo_handle);

    /* DAQ channel is currently EXT-only. */
    (void)IO_CAN_DeInitHandle(daq_tx_ext_fifo_handle);

    /* Deinit both CAN channels */
    (void)IO_CAN_DeInit(CONTROLS_CAN_CHANNEL);
    (void)IO_CAN_DeInit(DAQ_CAN_CHANNEL);
}

void CAN_Manager_ProcessRxMessages(void)
{
    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        CAN_RX_Message_t* const msg = &rx_messages[i];

        if (!can_recovery.can_ready) {
            msg->data_vld = FALSE;
            continue;
        }
        
        // Skip if decode function not configured
        if (msg->decode_fn == NULL) {
            continue;
        }

        IO_CAN_DATA_FRAME rx_frame;
        bool received = FALSE;
        const IO_ErrorType read_status = CAN_Util_ReadFIFO(msg->handle, &rx_frame, &received);
        
        if (received) {
            msg->decode_fn(&rx_frame);
            msg->data_vld = TRUE;
            IO_RTC_StartTime(&msg->last_rx_timestamp);
        }

        CAN_Manager_RecordFifoStatus(CAN_HEALTH_FIFO_FROM_RX_MSG(i), read_status);

        if (IO_RTC_GetTimeUS(msg->last_rx_timestamp) > msg->timeout_us) {
            msg->data_vld = FALSE;
        }
    }
}

void CAN_Manager_ProcessTxMessages(void)
{
    const VCU_State_t vcu_state = StateMachine_GetState();
    CAN_Manager_UpdateChannelHealth();
    CAN_Manager_RunRecovery();

    (void)vcu_state;

    for (ubyte1 msg_id = 0; msg_id < CAN_TX_MSG_COUNT; msg_id++) {

        CAN_TX_Message_t* const msg = &tx_messages[msg_id];

        if (!can_recovery.can_ready) {
            continue;
        }

        IO_CAN_DATA_FRAME tx_frame;
        ubyte1 tx_fifo_handle;
        
        const bool trigger_tx = (msg->tx_trigger_fn != NULL) && msg->tx_trigger_fn();

        if ((msg->period_cycles != CAN_TX_RATE_NON_PERIODIC) &&
            (msg->cycles_until_tx > 0)) {
            msg->cycles_until_tx--;
        }

        const bool periodic_due = (msg->period_cycles != CAN_TX_RATE_NON_PERIODIC) &&
                                  (msg->cycles_until_tx == 0);

        bool mark_for_tx = FALSE;

        mark_for_tx |= trigger_tx;
        mark_for_tx |= periodic_due;

        if (mark_for_tx) {
            // TODO want to get rid of this bs
            if (msg->channel == CONTROLS_CAN_CHANNEL) {
                tx_fifo_handle = (msg->id_format == IO_CAN_STD_FRAME)
                                ? controls_tx_std_fifo_handle
                                : controls_tx_ext_fifo_handle;
            }
            else if (msg->channel == DAQ_CAN_CHANNEL) {
                tx_fifo_handle = daq_tx_ext_fifo_handle;
            }
            else {
                continue;
            }

            if (msg->pack_fn == NULL) {
                continue;
            }

            tx_frame.length = 8;
            CAN_Util_ClearData(&tx_frame);
            tx_frame.id_format = msg->id_format;
            tx_frame.id = msg->id;
            
            msg->pack_fn(&tx_frame);

            const IO_ErrorType write_status = CAN_Util_WriteFIFO(tx_fifo_handle, &tx_frame);
            CAN_Manager_RecordFifoStatus(CAN_Manager_GetTxHealthFifoId(msg), write_status);

#if CAN_DEBUG_MIRROR_DAQ_TX_TO_CONTROLS
            /* mirror DAQ traffic onto the controls bus. */
            if (msg->channel == DAQ_CAN_CHANNEL) {
                CAN_Util_WriteFIFO(controls_tx_ext_fifo_handle, &tx_frame);
            }
#endif

            if (periodic_due) {
                msg->cycles_until_tx = msg->period_cycles;
            }
        }
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

    CAN_Util_WriteFIFO(controls_tx_ext_fifo_handle, &debug_frame);
    CAN_Util_WriteFIFO(daq_tx_ext_fifo_handle, &debug_frame);
}

/*
this kinda ruined my hopes and dreams of beautiful can manager but idk how else to dewit
*/
bool CAN_Manager_RX_Data_Valid(CAN_RX_MessageId_t msg_id)
{
    return rx_messages[msg_id].data_vld;
}

const CAN_Manager_Health_t* CAN_Manager_GetHealthData(void)
{
    return &can_health;
}
