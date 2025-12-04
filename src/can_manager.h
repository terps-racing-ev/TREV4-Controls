#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "IO_Constants.h"
#include "can_util.h"

#define BAUD_RATE 500

typedef struct {
    ubyte4  run_faults;
    ubyte4  post_faults;

    ubyte4  last_rx_timestamp;
    bool    data_vld;
} InverterStatus_RX_Data_t;

void CAN_Manager_Init(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);

#endif // CAN_MANAGER_H