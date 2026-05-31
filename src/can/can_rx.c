#include "can_rx.h"

#include "config/runtime_config.h"

static InverterStatus_RX_Data_t inverter_status_rx_data = {0};
static InverterHighSpeed_RX_Data_t inverter_high_speed_rx_data = {0};
static HVCSummary_RX_Data_t hvc_summary_rx_data = {0};
static HVCSummary_RX_Data_t hvc_summary_effective_data = {0};
static HVCVSense_RX_Data_t hvc_vsense_rx_data = {0};
static FrontWheelRpm_RX_Data_t front_left_rpm_rx_data = {0};
static FrontWheelRpm_RX_Data_t front_right_rpm_rx_data = {0};


const InverterStatus_RX_Data_t* CAN_RX_GetInverterStatusData(void)
{
    return &inverter_status_rx_data;
}

const InverterHighSpeed_RX_Data_t* CAN_RX_GetInverterHighSpeedData(void)
{
    return &inverter_high_speed_rx_data;
}

const FrontWheelRpm_RX_Data_t* CAN_RX_GetFrontLeftRpmData(void)
{
    return &front_left_rpm_rx_data;
}

const FrontWheelRpm_RX_Data_t* CAN_RX_GetFrontRightRpmData(void)
{
    return &front_right_rpm_rx_data;
}

const HVCSummary_RX_Data_t* CAN_RX_GetHVCSummaryData(void)
{
    hvc_summary_effective_data = hvc_summary_rx_data;

    // TODO are override locations good?
    sbyte2 dbg_bits = 0;
    (void)RuntimeConfig_GetI32(RUNTIME_PARAM_DEBUG_DEFINES, &dbg_bits);

    if (dbg_bits & DEBUG_BIT_IGNORE_SDC) {
        hvc_summary_effective_data.sdc_ok = TRUE;
    }

    /* technically covered in red car but just in case something uses this */
    if (dbg_bits & DEBUG_BIT_ALWAYS_GREEN) {
        hvc_summary_effective_data.imd_ok = TRUE;
        hvc_summary_effective_data.bms_ok = TRUE;
    }

    return &hvc_summary_effective_data;
}

const HVCVSense_RX_Data_t* CAN_RX_GetHVCVSenseData(void)
{
    return &hvc_vsense_rx_data;
}

void CAN_RX_UnpackInverterStatus(IO_CAN_DATA_FRAME* frame)
{
    // TODO
    (void)frame;
}

void CAN_RX_UnpackInverterHighSpeed(IO_CAN_DATA_FRAME* frame)
{
    if (frame == NULL) {
        return;
    }

    ubyte2 w;

    w = (ubyte2)(frame->data[0] | ((ubyte2)frame->data[1] << 8));
    inverter_high_speed_rx_data.torque_cmd = (sbyte2)w;

    w = (ubyte2)(frame->data[2] | ((ubyte2)frame->data[3] << 8));
    inverter_high_speed_rx_data.torque_feedback = (sbyte2)w;

    w = (ubyte2)(frame->data[4] | ((ubyte2)frame->data[5] << 8));
    inverter_high_speed_rx_data.motor_speed = (sbyte2)w;

    w = (ubyte2)(frame->data[6] | ((ubyte2)frame->data[7] << 8));
    inverter_high_speed_rx_data.dc_bus_voltage = (sbyte2)w;
}

void CAN_RX_UnpackHVCSummary(IO_CAN_DATA_FRAME* frame)
{
    if (frame == NULL) {
        return;
    }

    hvc_summary_rx_data.sdc_ok = (bool)(((frame->data[0] >> 0) & 1) == 0);
    hvc_summary_rx_data.imd_ok = (bool)(((frame->data[0] >> 1) & 1) == 0);
    hvc_summary_rx_data.bms_ok = (bool)(((frame->data[0] >> 2) & 1) == 0);
}

void CAN_RX_UnpackHVCVSense(IO_CAN_DATA_FRAME* frame)
{
    if (frame == NULL) {
        return;
    }

    /* HVC IO_VSense: Inv_Voltage_mV is a 32-bit LE value at byte offset 4. */
    hvc_vsense_rx_data.inv_voltage_mv = (ubyte4)((ubyte4)frame->data[4] |
                                                 ((ubyte4)frame->data[5] << 8) |
                                                 ((ubyte4)frame->data[6] << 16) |
                                                 ((ubyte4)frame->data[7] << 24));
}

void CAN_RX_UnpackSetVCUConfig(IO_CAN_DATA_FRAME* frame)
{
    if (frame == NULL) {
        return;
    }

    if (frame->length < 3) {
        return;
    }

    const ubyte1 mux = frame->data[0];
    const ubyte2 raw_value = (ubyte2)((ubyte2)frame->data[1] |
                                      ((ubyte2)frame->data[2] << 8));
    const sbyte4 value = (sbyte4)((sbyte2)raw_value);

    (void)RuntimeConfig_Set(mux, value);
}

static void CAN_RX_UnpackFrontRpm(IO_CAN_DATA_FRAME* frame,
                                  FrontWheelRpm_RX_Data_t* const out_data)
{
    if ((frame == NULL) || (out_data == NULL) || (frame->length < 7)) {
        return;
    }

    out_data->rpm = (ubyte2)((ubyte2)frame->data[5] |
                             ((ubyte2)frame->data[6] << 8));
}

void CAN_RX_UnpackFrontLeftRpm(IO_CAN_DATA_FRAME* frame)
{
    CAN_RX_UnpackFrontRpm(frame, &front_left_rpm_rx_data);
}

void CAN_RX_UnpackFrontRightRpm(IO_CAN_DATA_FRAME* frame)
{
    CAN_RX_UnpackFrontRpm(frame, &front_right_rpm_rx_data);
}
