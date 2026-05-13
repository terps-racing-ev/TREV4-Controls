#include "can_rx.h"

#include "config/runtime_config.h"

static InverterStatus_RX_Data_t inverter_status_rx_data = {0};
static InverterHighSpeed_RX_Data_t inverter_high_speed_rx_data = {0};
static HVCSummary_RX_Data_t hvc_summary_rx_data = {0};


const InverterStatus_RX_Data_t* CAN_RX_GetInverterStatusData(void)
{
    return &inverter_status_rx_data;
}

const InverterHighSpeed_RX_Data_t* CAN_RX_GetInverterHighSpeedData(void)
{
    return &inverter_high_speed_rx_data;
}

const HVCSummary_RX_Data_t* CAN_RX_GetHVCSummaryData(void)
{
    return &hvc_summary_rx_data;
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
    //TODO for now say everything is good
    (void)frame;

    hvc_summary_rx_data.bms_ok = TRUE;
    hvc_summary_rx_data.imd_ok = TRUE;
    hvc_summary_rx_data.sdc_ok = TRUE;
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