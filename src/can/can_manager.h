#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "IO_Constants.h"


void CAN_Manager_Init(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);

typedef struct {
    ubyte4  run_faults;
    ubyte4  post_faults;

    ubyte4  last_rx_timestamp;
    bool    data_vld;
} InverterStatus_RX_Data_t;

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void);

#endif // CAN_MANAGER_H