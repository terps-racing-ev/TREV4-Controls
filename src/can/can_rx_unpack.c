#include "can_rx_unpack.h"
#include "IO_Constants.h"

#include "settings/runtime_config.h"
#include "can_manager.h"


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
    //TODO for now say everything is good
    (void)frame;
    rx_data->bms_ok = TRUE;
    rx_data->imd_ok = TRUE;
    rx_data->sdc_ok = TRUE;
}

void CAN_RX_UnpackVCUConfigSet(IO_CAN_DATA_FRAME* frame, void* data)
{
    VCUConfigSet_RX_Data_t* rx_data = (VCUConfigSet_RX_Data_t*)data;
    ubyte2 pid;
    ubyte4 raw;
    sbyte4 val;

    if ((frame == NULL) || (rx_data == NULL)) {
        return;
    }

    /* Payload:
       bytes0-1: param_id (u16)
       bytes4-7: value (i32)
       bytes2-3: reserved
    */
    pid = (ubyte2)((ubyte2)frame->data[0] | ((ubyte2)frame->data[1] << 8));
    raw = (ubyte4)((ubyte4)frame->data[4] |
                   ((ubyte4)frame->data[5] << 8) |
                   ((ubyte4)frame->data[6] << 16) |
                   ((ubyte4)frame->data[7] << 24));
    val = (sbyte4)raw;

    rx_data->param_id = pid;
    rx_data->value = val;

    if (RuntimeConfig_SetI32(pid, val)) {
        /* Immediately re-broadcast settings after a successful update. */
        CAN_Manager_RequestVcuSettingsTx();
    }
}
