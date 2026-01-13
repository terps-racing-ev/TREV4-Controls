#include "can_rx_messages.h"
#include "IO_Constants.h"


void CAN_RX_UnpackInverterStatus(IO_CAN_DATA_FRAME* frame, void* data)
{
    InverterStatus_RX_Data_t* rx_data = (InverterStatus_RX_Data_t*)data;
    // TODO
    (void)frame;
    (void)rx_data;
}

void CAN_RX_UnpackInverterHighSpeed(IO_CAN_DATA_FRAME* frame, void* data)
{
    InverterHighSpeed_RX_Data_t* rx_data = (InverterHighSpeed_RX_Data_t*)data;
    ubyte2 w;
    
    w = (ubyte2)(frame->data[0] | ((ubyte2)frame->data[1] << 8));
    rx_data->torque_cmd = (sbyte2)w;

    w = (ubyte2)(frame->data[2] | ((ubyte2)frame->data[3] << 8));
    rx_data->torque_feedback = (sbyte2)w;

    w = (ubyte2)(frame->data[4] | ((ubyte2)frame->data[5] << 8));
    rx_data->motor_speed = (sbyte2)w;

    w = (ubyte2)(frame->data[6] | ((ubyte2)frame->data[7] << 8));
    rx_data->dc_bus_voltage = (sbyte2)w;
}

void CAN_RX_UnpackHVCSummary(IO_CAN_DATA_FRAME* frame, void* data)
{
    HVCSummary_RX_Data_t* rx_data = (HVCSummary_RX_Data_t*)data;
    //TODO
    (void)frame;
    (void)rx_data;
}
