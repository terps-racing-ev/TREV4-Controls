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

typedef struct {
    ubyte4  inv_voltage_mv;
} HVCVSense_RX_Data_t;

typedef struct {
    ubyte2  pack_soc_percent_x100;
} HVCSOC_RX_Data_t;

typedef struct {
    ubyte2  rear_brake_pressure_psi_x10;
} MOBO_PowerTelemetry_RX_Data_t;

typedef struct {
    ubyte2 rpm;
} FrontWheelRpm_RX_Data_t;

/* RX message entrypoints (frame -> module-owned struct). */
void CAN_RX_UnpackInverterStatus(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackInverterHighSpeed(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackHVCSummary(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackHVCSOC(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackHVCVSense(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackMOBOPowerTelemetry(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackSetVCUConfig(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackFrontLeftRpm(IO_CAN_DATA_FRAME* frame);
void CAN_RX_UnpackFrontRightRpm(IO_CAN_DATA_FRAME* frame);

/* RX message storage + public getters. */
const InverterStatus_RX_Data_t* CAN_RX_GetInverterStatusData(void);
const InverterHighSpeed_RX_Data_t* CAN_RX_GetInverterHighSpeedData(void);
const HVCSummary_RX_Data_t* CAN_RX_GetHVCSummaryData(void);
const HVCSOC_RX_Data_t* CAN_RX_GetHVCSOCData(void);
const HVCVSense_RX_Data_t* CAN_RX_GetHVCVSenseData(void);
const MOBO_PowerTelemetry_RX_Data_t* CAN_RX_GetMOBO_PowerTelemetryData(void);
const FrontWheelRpm_RX_Data_t* CAN_RX_GetFrontLeftRpmData(void);
const FrontWheelRpm_RX_Data_t* CAN_RX_GetFrontRightRpmData(void);

#endif // CAN_RX_H
