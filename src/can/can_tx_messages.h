#ifndef CAN_TX_MESSAGES_H
#define CAN_TX_MESSAGES_H

#include "IO_Constants.h"
#include "IO_CAN.h"

void CAN_TX_PackAPPSVoltages(IO_CAN_DATA_FRAME* frame);
void CAN_TX_PackAPPSValues(IO_CAN_DATA_FRAME* frame);
void CAN_TX_PackInvTorqueCommand(IO_CAN_DATA_FRAME* frame);
void CAN_TX_PackInvReadWrite(IO_CAN_DATA_FRAME* frame);

#endif // CAN_TX_MESSAGES_H
