#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "can_tx.h"
#include "can_rx.h"
#include "can_recovery.h"
#include "config/can_config.h"
#include "config/runtime_config.h"
#include "control/power_limit.h"
#include "state_machine.h"

/**************************************************************************
*          P R I V A T E    D A T A    D E F I N I T I O N S
**************************************************************************/

static ubyte1 controls_tx_std_fifo_handle;
static ubyte1 controls_tx_ext_fifo_handle;

// currently no std messages being send on daq
static ubyte1 daq_tx_ext_fifo_handle;

// TODO combine this with can recovery to be one health module
static CAN_Manager_Health_t can_health;

/**************************************************************************
* RX Messages - ADD MESSAGES HERE
**************************************************************************/
static CAN_RX_Message_t rx_messages[CAN_RX_MSG_COUNT] = {
    [CAN_RX_MSG_INV_MOTOR_POSITION] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_MOTOR_POSITION,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackInverterMotorPosition
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
    [CAN_RX_MSG_HVC_SOC] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_HVC_SOC,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackHVCSOC
    },
    [CAN_RX_MSG_HVC_VSENSE] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_HVC_VSENSE,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackHVCVSense
    },
    [CAN_RX_MSG_MOBO_POWER_TELEMETRY] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_MOBO_POWER_TELEMETRY,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackMOBOPowerTelemetry
    },
    [CAN_RX_MSG_SET_VCU_CONFIG] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_SET_VCU_CONFIG,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackSetVCUConfig
    },
    [CAN_RX_MSG_FRONT_LEFT_RPM] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_FRONT_LEFT_RPM,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackFrontLeftRpm
    },
    [CAN_RX_MSG_FRONT_RIGHT_RPM] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_FRONT_RIGHT_RPM,
        .timeout_us = MSG_TIMEOUT_US,
        .decode_fn = CAN_RX_UnpackFrontRightRpm
    },
};

/**************************************************************************
* TX Messages - ADD MESSAGES HERE
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
    [CAN_TX_MSG_INV_CURRENT_LIMIT] = {
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_STD_FRAME,
        .id = CAN_ID_INV_CURRENT_LIMIT,
        .period_cycles = CAN_TX_RATE_NON_PERIODIC,
        .tx_trigger_fn = PowerLimit_TxTrigger,
        .pack_fn = CAN_TX_PackInverterCurrentLimit
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
        .channel = CONTROLS_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_CONFIG,
        .period_cycles = CAN_TX_RATE_250MS,
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
    [CAN_TX_MSG_VCU_REGEN_DEBUG] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_VCU_REGEN_DEBUG,
        .period_cycles = CAN_TX_RATE_100MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackVCURegenDebug
    },
    [CAN_TX_MSG_TRACTION_CONTROL] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_TRACTION_CONTROL,
        .period_cycles = CAN_TX_RATE_10MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackTractionControl
    },
    [CAN_TX_MSG_VCU_LAUNCH_STATE] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_VCU_LAUNCH_STATE,
        .period_cycles = CAN_TX_RATE_100MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackVCULaunchState
    },
    [CAN_TX_MSG_CAN_READBACK] = {
        .channel = DAQ_CAN_CHANNEL,
        .id_format = IO_CAN_EXT_FRAME,
        .id = CAN_ID_CAN_READBACK,
        .period_cycles = CAN_TX_RATE_100MS,
        .tx_trigger_fn = NULL,
        .pack_fn = CAN_TX_PackCANReadback
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

#define CAN_RX_EXACT_MASK 0x1FFFFFFF
/* Shared Controls EXT RX FIFO for HVC Summary/SOC/VSense and MOBO power.
 * Mask off the ID bits that vary across 0x004001F0, 0x004001F4,
 * 0x004001F7, and 0x00200010; dispatch later by the received frame ID. */
#define CAN_RX_CONTROLS_TELEMETRY_MASK 0x1F9FFE18

static void CAN_Manager_RunRecovery(void);

static bool CAN_Manager_RxMessageOwnsFifo(CAN_RX_MessageId_t msg_id)
{
    switch (msg_id) {
    case CAN_RX_MSG_HVC_SOC:
    case CAN_RX_MSG_HVC_VSENSE:
    case CAN_RX_MSG_MOBO_POWER_TELEMETRY:
        return FALSE;
    default:
        return TRUE;
    }
}

static ubyte4 CAN_Manager_GetRxFifoMask(CAN_RX_MessageId_t msg_id)
{
    if (msg_id == CAN_RX_MSG_HVC_SUMMARY) {
        return CAN_RX_CONTROLS_TELEMETRY_MASK;
    }

    return CAN_RX_EXACT_MASK;
}

static CAN_RX_MessageId_t CAN_Manager_FindRxMessageForFrame(CAN_RX_MessageId_t fifo_msg_id,
                                                            const IO_CAN_DATA_FRAME* frame)
{
    const CAN_RX_Message_t* const fifo_msg = &rx_messages[fifo_msg_id];

    if (CAN_Manager_GetRxFifoMask(fifo_msg_id) == CAN_RX_EXACT_MASK) {
        return fifo_msg_id;
    }

    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        const CAN_RX_Message_t* const msg = &rx_messages[i];

        if ((msg->channel == fifo_msg->channel) &&
            (msg->id == frame->id)) {
            return (CAN_RX_MessageId_t)i;
        }
    }

    return CAN_RX_MSG_COUNT;
}

static void CAN_Manager_DispatchRxFrame(CAN_RX_MessageId_t fifo_msg_id,
                                        IO_CAN_DATA_FRAME* frame)
{
    const CAN_RX_MessageId_t rx_msg_id = CAN_Manager_FindRxMessageForFrame(fifo_msg_id, frame);

    if (rx_msg_id >= CAN_RX_MSG_COUNT) {
        return;
    }

    CAN_RX_Message_t* const msg = &rx_messages[rx_msg_id];

    if (msg->decode_fn == NULL) {
        return;
    }

    msg->decode_fn(frame);
    msg->data_vld = TRUE;
    IO_RTC_StartTime(&msg->last_rx_timestamp);
}

static IO_ErrorType CAN_Manager_ReadRxFifo(CAN_RX_MessageId_t fifo_msg_id)
{
    const CAN_RX_Message_t* const fifo_msg = &rx_messages[fifo_msg_id];
    IO_CAN_DATA_FRAME rx_frame;
    bool saw_overflow = FALSE;
    bool received = FALSE;
    IO_ErrorType fifo_status = IO_CAN_FIFOStatus(fifo_msg->handle);

    while ((fifo_status == IO_E_OK) || (fifo_status == IO_E_CAN_OVERFLOW)) {
        if (fifo_status == IO_E_CAN_OVERFLOW) {
            saw_overflow = TRUE;
        }

        ubyte1 rxed_frames = 0;
        const IO_ErrorType read_status = IO_CAN_ReadFIFO(fifo_msg->handle, &rx_frame, 1, &rxed_frames);

        if (read_status != IO_E_OK) {
            return read_status;
        }

        if (rxed_frames > 0) {
            received = TRUE;
            CAN_Manager_DispatchRxFrame(fifo_msg_id, &rx_frame);
        }

        fifo_status = IO_CAN_FIFOStatus(fifo_msg->handle);
    }

    if (fifo_status == IO_E_CAN_OLD_DATA) {
        if (saw_overflow) {
            return IO_E_CAN_OVERFLOW;
        }

        if (received) {
            return IO_E_OK;
        }

        return IO_E_CAN_OLD_DATA;
    }

    return fifo_status;
}

static void CAN_Manager_UpdateRecoveryStateFromStatus(IO_ErrorType controls_status,
                                                      IO_ErrorType daq_status)
{
    CAN_Recovery_UpdateRecoveryState(controls_status, daq_status);
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

static void CAN_Manager_RunRecovery(void)
{
    if (!CAN_RECOVERY_ENABLE) {
        return;
    }
    
        // If recovery is disabled

    CAN_Recovery_RunRecovery(CAN_Manager_Init, CAN_Manager_DeInit);

    if (!CAN_Recovery_IsReady()) {
        can_health.controls_tx_fault_seen = TRUE;
        can_health.controls_rx_fault_seen = TRUE;
        can_health.daq_tx_fault_seen = TRUE;
        can_health.daq_rx_fault_seen = TRUE;
    }
}

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

    /* Initialize FIFOs for each RX filter. HVC/MOBO telemetry share one Controls EXT FIFO. */
    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        CAN_RX_Message_t* const msg = &rx_messages[i];

        if (!CAN_Manager_RxMessageOwnsFifo((CAN_RX_MessageId_t)i)) {
            continue;
        }

        IO_CAN_ConfigFIFO( &msg->handle,
                            msg->channel,
                            RX_FIFO_BUFFER_SIZE,
                            IO_CAN_MSG_READ,
                            msg->id_format,
                            msg->id,
                            CAN_Manager_GetRxFifoMask((CAN_RX_MessageId_t)i));
    }

    CAN_Recovery_Init();
}

void CAN_Manager_DeInit(void)
{
    CAN_Recovery_DeInit();

    /* Deinit all RX message handles */
    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        CAN_RX_Message_t* const msg = &rx_messages[i];

        if (!CAN_Manager_RxMessageOwnsFifo((CAN_RX_MessageId_t)i)) {
            continue;
        }

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

        if (!CAN_Recovery_IsReady()) {
            msg->data_vld = FALSE;
        }

        if (IO_RTC_GetTimeUS(msg->last_rx_timestamp) > msg->timeout_us) {
            msg->data_vld = FALSE;
        }
    }

    if (!CAN_Recovery_IsReady()) {
        return;
    }

    for (ubyte1 i = 0; i < CAN_RX_MSG_COUNT; i++) {
        if (!CAN_Manager_RxMessageOwnsFifo((CAN_RX_MessageId_t)i)) {
            continue;
        }

        const IO_ErrorType read_status = CAN_Manager_ReadRxFifo((CAN_RX_MessageId_t)i);

        CAN_Manager_RecordFifoStatus(CAN_HEALTH_FIFO_FROM_RX_MSG(i), read_status);
    }
}

void CAN_Manager_ProcessTxMessages(void)
{
    const VCU_State_t vcu_state = StateMachine_GetState();
    CAN_Manager_UpdateChannelHealth();
    CAN_Manager_RunRecovery();

    (void)vcu_state;

    sbyte2 dbg_bits = 0;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_DEBUG_DEFINES, &dbg_bits);

    for (ubyte1 msg_id = 0; msg_id < CAN_TX_MSG_COUNT; msg_id++) {

        CAN_TX_Message_t* const msg = &tx_messages[msg_id];

        if (!CAN_Recovery_IsReady()) {
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

            /* echo DAQ traffic onto the controls bus (runtime controlled). */
            if ((dbg_bits & DEBUG_BIT_ECHO_DAQ_TX_TO_CONTROLS) && (msg->channel == DAQ_CAN_CHANNEL)) {
                CAN_Util_WriteFIFO(controls_tx_ext_fifo_handle, &tx_frame);
            }

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
