#include "IO_Constants.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "can_manager.h"
#include "can_util.h"
#include "util/units.h"

#define CONTROLS_CAN_CHANNEL IO_CAN_CHANNEL_0
#define TELEMETRY_CAN_CHANNEL IO_CAN_CHANNEL_1

// Buffer size for every rx message, will pretty much always be at 0 or 1
// since we poll (200hz) faster than we will receive (50hz)
#define RX_FIFO_BUFFER_SIZE 8

// Buffer size for an entire tx channel
#define TX_FIFO_BUFFER_SIZE 32


// RX Message IDs
#define CAN_ID_INV_STATUS       0x000000AB

// TX Message IDs
#define CAN_ID_TORQUE_COMMAND   0x000000C0

// RX FIFO handles
static ubyte1 inverter_status_rx_handle;

// TX FIFO handles
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

//getters for rx
const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void)
{
    return &inverter_status_rx_data;
}

//setters for tx TODO
void CAN_Manager_SetTorqueCommandData(sbyte2 torque_nm, bool enable)
{

}