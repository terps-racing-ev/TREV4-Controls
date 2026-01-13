#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "IO_Constants.h"
#include "can_rx_messages.h"

typedef struct {
    const ubyte1 channel;
    const ubyte1 id_format;
    const ubyte1 id;
    ubyte1 handle;

    const ubyte4 timeout_us;
    void* data; // Pointer to an RX_data struct
    void (*unpack_fn)(IO_CAN_DATA_FRAME*, void*);
    ubyte4 last_rx_timestamp;
    bool    data_vld;
} CAN_RX_Message_t;


void CAN_Manager_Init(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);
void CAN_Manager_Print(ubyte4 CAN_id, ubyte2 data);

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void);
const InverterHighSpeed_RX_Data_t* CAN_Manager_GetInverterHighSpeedData(void);
const HVCSummary_RX_Data_t* CAN_Manager_GetHVCSummaryData(void);

#endif // CAN_MANAGER_H
