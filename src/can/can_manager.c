#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "util/units.h"
#include "config/can_config.h"

/* RX FIFO handles
    You need one FIFO for each frame */
static ubyte1 inverter_status_rx_handle;

/* TX FIFO handles
    Only need one FIFO for each channel */
static ubyte1 controls_tx_fifo_handle;
static ubyte1 telemetry_tx_fifo_handle;

// Error counters
ubyte1 controls_tx_error_ctr;
ubyte1 controls_rx_error_ctr;

ubyte1 telemetry_tx_error_ctr;
ubyte1 telemetry_rx_error_ctr;


/**************************************************************************
* Private RX Data
**************************************************************************/

static InverterStatus_RX_Data_t inverter_status_rx_data = {0};

/**************************************************************************
* Private TX Data
**************************************************************************/


void CAN_Manager_Init(void)
{
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
    

    /* Initialize FIFOs for txing messages */
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

    /* Initialize FIFOs for rxing messages */
    IO_CAN_ConfigFIFO( &inverter_status_rx_handle
                    , CONTROLS_CAN_CHANNEL
                    , RX_FIFO_BUFFER_SIZE
                    , IO_CAN_MSG_READ
                    , IO_CAN_EXT_FRAME
                    , CAN_ID_INV_STATUS
                    , 0x1FFFFFFF);
}

void CAN_Manager_ProcessRxMessages(void)
{
    // Reuse one frame for every rx, since we'll store the data in our private structs
    IO_CAN_DATA_FRAME rx_frame;
    bool received;
    ubyte4 curr_time = IO_RTC_GetTimeUS(0);

    CAN_Util_ReadFIFO(inverter_status_rx_handle, &rx_frame, &received);
    if (received) {
        // worry about this later
        inverter_status_rx_data.last_rx_timestamp = curr_time;
        inverter_status_rx_data.data_vld = TRUE;
    }
    if (curr_time - inverter_status_rx_data.last_rx_timestamp > MsToUs(1000)) {
        inverter_status_rx_data.data_vld = FALSE;
    }
}

void CAN_Manager_ProcessTxMessages(void)
{
    // Reuse one frame for every tx, since WriteFIFO copies it anyways
    IO_CAN_DATA_FRAME tx_frame;

    // TODO figure out the best way to store this data...
    // pack the tx frame
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

static void PackAPPSVoltages(IO_CAN_DATA_FRAME* apps_voltages_frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();
    
    apps_voltages_frame->id = CAN_ID_APPS_VOLTAGES;
    apps_voltages_frame->id_format = IO_CAN_EXT_FRAME;
    apps_voltages_frame->length = 8;
    CAN_Util_ClearData(apps_voltages_frame);
    
    apps_voltages_frame->data[0] = apps_data->apps2.filt_mv & 0xFF;
    apps_voltages_frame->data[1] = apps_data->apps2.filt_mv >> 8;
    apps_voltages_frame->data[2] = apps_data->apps2.raw_mv & 0xFF;
    apps_voltages_frame->data[3] = apps_data->apps2.raw_mv >> 8;
    apps_voltages_frame->data[4] = apps_data->apps1.filt_mv & 0xFF;
    apps_voltages_frame->data[5] = apps_data->apps1.filt_mv >> 8;
    apps_voltages_frame->data[6] = apps_data->apps1.raw_mv & 0xFF;
    apps_voltages_frame->data[7] = apps_data->apps1.raw_mv >> 8;
    
}

static void PackAPPSValues(IO_CAN_DATA_FRAME* apps_values_frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();
    
    apps_values_frame->id = CAN_ID_APPS_VALUES;
    apps_values_frame->id_format = IO_CAN_EXT_FRAME;
    apps_values_frame->length = 8;
    CAN_Util_ClearData(apps_frame1);

    apps_values_frame->data[0] = (apps_data->apps1.valid << 7) |
                                 (apps_data->apps1.out_of_range << 6) |
                                 (apps_data->apps1.adc_err << 5) |
                                 (apps_data->apps1.stale << 4) |
                                 (apps_data->apps2.valid << 3) |
                                 (apps_data->apps2.out_of_range << 2) |
                                 (apps_data->apps2.adc_err << 1) |
                                 apps_data->apps2.stale;
    apps_values_frame->data[1] = apps_data->apps2.value & 0xFF;
    apps_values_frame->data[2] = apps_data->apps2.value >> 8;
    apps_values_frame->data[3] = apps_data->apps1.value & 0xFF;
    apps_values_frame->data[4] = apps_data->apps1.value >> 8;
    apps_values_frame->data[5] = (apps_data->valid << 4) | apps_data->implausible;
    apps_values_frame->data[6] = apps_data->apps_value & 0xFF;
    apps_values_frame->data[7] = apps_data->apps_value >> 8;

}

/* Getters */

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void)
{
    return &inverter_status_rx_data;
}