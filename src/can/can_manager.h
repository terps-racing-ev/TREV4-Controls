#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H


#include "IO_Constants.h"
#include "can_rx_unpack.h"


void CAN_Manager_Init(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);
void CAN_Manager_Print(ubyte4 CAN_id, ubyte2 data);

/* Forces a VCU settings transmit on the next CAN_Manager_ProcessTxMessages() call. */
void CAN_Manager_RequestVcuSettingsTx(void);

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void);
const InverterHighSpeed_RX_Data_t* CAN_Manager_GetInverterHighSpeedData(void);
const HVCSummary_RX_Data_t* CAN_Manager_GetHVCSummaryData(void);

#endif // CAN_MANAGER_H
