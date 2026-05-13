#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H


#include "IO_Constants.h"
#include "IO_CAN.h"

typedef struct {
    const ubyte1 channel;
    const ubyte1 id_format;
    const ubyte4 id;
    const ubyte4 timeout_us;
    void (*const decode_fn)(IO_CAN_DATA_FRAME*);

    ubyte1 handle;  // each rx message needs its own queue
    ubyte4 last_rx_timestamp;
    bool data_vld;
} CAN_RX_Message_t;

typedef struct {
    const ubyte1 channel;
    const ubyte1 id_format;
    const ubyte4 id;
    const ubyte4 period_cycles;
    bool (*const tx_trigger_fn)(void);
    void (*const pack_fn)(IO_CAN_DATA_FRAME*);
    
    ubyte4 cycles_until_tx;
} CAN_TX_Message_t;


/********************
* ADD MESSAGES HERE
********************/
typedef enum {
    CAN_RX_MSG_INV_STATUS,
    CAN_RX_MSG_INV_HIGH_SPEED,
    CAN_RX_MSG_HVC_SUMMARY,
    CAN_RX_MSG_SET_VCU_CONFIG,

    CAN_RX_MSG_COUNT
} CAN_RX_MessageId_t;

typedef enum {
    CAN_TX_MSG_INV_TORQUE_COMMAND,
    CAN_TX_MSG_INV_READ_WRITE,
    CAN_TX_MSG_APPS_VALUES,
    CAN_TX_MSG_APPS_VOLTAGES,
    CAN_TX_MSG_VCU_SUMMARY,
    CAN_TX_MSG_BSE,
    CAN_TX_MSG_CONFIG,
    CAN_TX_MSG_CAN_HEALTH,
    CAN_TX_MSG_CAN_HEALTH_FIFO,
    CAN_TX_MSG_DEAD_CAR,

    CAN_TX_MSG_COUNT
} CAN_TX_MessageId_t;

typedef enum {
    CAN_HEALTH_TX_FIFO_CONTROLS_STD = 0,
    CAN_HEALTH_TX_FIFO_CONTROLS_EXT,
    CAN_HEALTH_TX_FIFO_DAQ_EXT,

    CAN_HEALTH_TX_FIFO_COUNT
} CAN_HealthTxFifoId_t;

#define CAN_HEALTH_FIFO_RX_OFFSET CAN_HEALTH_TX_FIFO_COUNT
#define CAN_HEALTH_FIFO_COUNT (CAN_HEALTH_TX_FIFO_COUNT + CAN_RX_MSG_COUNT)
#define CAN_HEALTH_FIFO_FROM_RX_MSG(msg_id) (CAN_HEALTH_FIFO_RX_OFFSET + (msg_id))

typedef struct {
    ubyte1 last_status_raw;
    ubyte2 fifo_full_count;
    ubyte2 overflow_count;
    ubyte2 invalid_data_count;
    ubyte2 other_error_count;
} CAN_HealthFifoData_t;

typedef struct {
    ubyte1 controls_tx_error_counter;
    ubyte1 controls_rx_error_counter;
    ubyte1 daq_tx_error_counter;
    ubyte1 daq_rx_error_counter;

    ubyte1 controls_status_ret_raw;
    ubyte1 daq_status_ret_raw;

    bool controls_error_passive;
    bool controls_bus_off;
    bool daq_error_passive;
    bool daq_bus_off;

    bool controls_tx_fault_seen;
    bool controls_rx_fault_seen;
    bool daq_tx_fault_seen;
    bool daq_rx_fault_seen;

    CAN_HealthFifoData_t fifos[CAN_HEALTH_FIFO_COUNT];
} CAN_Manager_Health_t;

void CAN_Manager_Init(void);
void CAN_Manager_DeInit(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);
void CAN_Manager_Print(ubyte4 CAN_id, ubyte2 data);

bool CAN_Manager_RX_Data_Valid(CAN_RX_MessageId_t msg_id);
const CAN_Manager_Health_t* CAN_Manager_GetHealthData(void);

#endif // CAN_MANAGER_H
