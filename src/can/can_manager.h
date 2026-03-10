#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "IO_Constants.h"
#include "can_rx_unpack.h"

typedef enum {
    CAN_RX_MSG_INV_STATUS,
    CAN_RX_MSG_INV_HIGH_SPEED,
    CAN_RX_MSG_HVC_SUMMARY,

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
    bool    data_vld;
} CAN_RX_Message_t;

typedef enum {
    CAN_TX_MSG_INV_TORQUE_COMMAND,
    CAN_TX_MSG_INV_READ_WRITE,
    CAN_TX_MSG_APPS_VALUES,
    CAN_TX_MSG_APPS_VOLTAGES,

    CAN_TX_MSG_COUNT
} CAN_TX_MessageId_t;

typedef struct {
    const ubyte1 channel;
    const ubyte1 id_format;
    const ubyte4 id;
    const ubyte4 rate_us;
    void (*pack_fn)(IO_CAN_DATA_FRAME*);
} CAN_TX_Message_t;


void CAN_Manager_Init(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);
void CAN_Manager_Print(ubyte4 CAN_id, ubyte2 data);

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void);
const InverterHighSpeed_RX_Data_t* CAN_Manager_GetInverterHighSpeedData(void);
const HVCSummary_RX_Data_t* CAN_Manager_GetHVCSummaryData(void);

#endif // CAN_MANAGER_H
