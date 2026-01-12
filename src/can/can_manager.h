#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "IO_Constants.h"


void CAN_Manager_Init(void);
void CAN_Manager_ProcessRxMessages(void);
void CAN_Manager_ProcessTxMessages(void);
void CAN_Manager_Print(ubyte4 CAN_id, ubyte2 data);

// TODO figure out timeouts for each message and stale...
typedef struct {
    ubyte4  run_faults;
    ubyte4  post_faults;

    ubyte4  last_rx_timestamp;

    bool    data_vld;
} InverterStatus_RX_Data_t;

typedef struct {
    sbyte2  torque_cmd;     // x10
    sbyte2  torque_feedback;    // x10
    sbyte2  motor_speed;    // x1
    sbyte2  dc_bus_voltage; // x10

    ubyte4  last_rx_timestamp;

    bool    data_vld;
} InverterHighSpeed_RX_Data_t;

typedef struct {
    bool    sdc_ok;
    bool    imd_ok;
    bool    bms_ok;

    sbyte2  pack_voltage;
    sbyte2  pack_current;
    sbyte2  pack_soc;

    bool    data_vld;
} HVCSummary_RX_Data_t;

const InverterStatus_RX_Data_t* CAN_Manager_GetInverterStatusData(void);
const InverterHighSpeed_RX_Data_t* CAN_Manager_GetInverterHighSpeedData(void);
const HVCSummary_RX_Data_t* CAN_Manager_GetHVCSummaryData(void);

#endif // CAN_MANAGER_H
