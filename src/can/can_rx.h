#ifndef CAN_RX_H
#define CAN_RX_H

#include "IO_Constants.h"
#include "IO_CAN.h"

typedef struct {
    ubyte4  run_faults;
    ubyte4  post_faults;
} InverterStatus_RX_Data_t;

typedef struct {
    sbyte2  torque_cmd;         // x10
    sbyte2  torque_feedback;    // x10
    sbyte2  motor_speed;        // x1
    sbyte2  dc_bus_voltage;     // x10
} InverterHighSpeed_RX_Data_t;

typedef struct {
    bool    sdc_ok;
    bool    imd_ok;
    bool    bms_ok;

    sbyte2  pack_voltage;
    sbyte2  pack_current;
    sbyte2  pack_soc;
} HVCSummary_RX_Data_t;

/* RX message entrypoints (frame -> module-owned struct). */
void CAN_RX_UnpackInverterStatus(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackInverterHighSpeed(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackHVCSummary(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackSetVCUConfig(IO_CAN_DATA_FRAME* frame);

/* RX message storage + public getters. */
const InverterStatus_RX_Data_t* CAN_RX_GetInverterStatusData(void);
const InverterHighSpeed_RX_Data_t* CAN_RX_GetInverterHighSpeedData(void);
const HVCSummary_RX_Data_t* CAN_RX_GetHVCSummaryData(void);

#endif // CAN_RX_H
