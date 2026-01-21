#include "can_tx_messages.h"
#include "can_util.h"
#include "config/can_config.h"
#include "sensors/apps.h"
#include "control/torque_controller.h"

void CAN_TX_PackAPPSVoltages(IO_CAN_DATA_FRAME* frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();
    
    frame->id = CAN_ID_APPS_VOLTAGES;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);
    
    frame->data[0] = apps_data->apps2.filt_mv & 0xFF;
    frame->data[1] = apps_data->apps2.filt_mv >> 8;
    frame->data[2] = apps_data->apps2.raw_mv & 0xFF;
    frame->data[3] = apps_data->apps2.raw_mv >> 8;
    frame->data[4] = apps_data->apps1.filt_mv & 0xFF;
    frame->data[5] = apps_data->apps1.filt_mv >> 8;
    frame->data[6] = apps_data->apps1.raw_mv & 0xFF;
    frame->data[7] = apps_data->apps1.raw_mv >> 8;
}

void CAN_TX_PackAPPSValues(IO_CAN_DATA_FRAME* frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();
    
    frame->id = CAN_ID_APPS_VALUES;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);

    frame->data[0] = (apps_data->apps1.valid << 7) |
                                 (apps_data->apps1.out_of_range << 6) |
                                 (apps_data->apps1.adc_err << 5) |
                                 (apps_data->apps1.stale << 4) |
                                 (apps_data->apps2.valid << 3) |
                                 (apps_data->apps2.out_of_range << 2) |
                                 (apps_data->apps2.adc_err << 1) |
                                 apps_data->apps2.stale;
    frame->data[1] = apps_data->apps2.value & 0xFF;
    frame->data[2] = apps_data->apps2.value >> 8;
    frame->data[3] = apps_data->apps1.value & 0xFF;
    frame->data[4] = apps_data->apps1.value >> 8;
    frame->data[5] = (apps_data->valid << 4) | apps_data->implausible;
    frame->data[6] = apps_data->apps_value & 0xFF;
    frame->data[7] = apps_data->apps_value >> 8;
}

void CAN_TX_PackInvTorqueCommand(IO_CAN_DATA_FRAME* frame)
{
    const TorqueController_Data_T* torque_data = TorqueController_GetData();

    frame->id = CAN_ID_INV_TORQUE_COMMAND;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);

    frame->data[0] = torque_data->inv_torque_scaled & 0xFF;
    frame->data[1] = torque_data->inv_torque_scaled >> 8;
    // TODO speed command?
    frame->data[2] = 0;
    frame->data[3] = 0;
    frame->data[4] = torque_data->inv_direction;
    frame->data[5] = torque_data->inv_enable;
    frame->data[6] = torque_data->inv_speed_mode;
    // TODO torque limit. useful prolly not?
    frame->data[7] = 0;
}

void CAN_TX_PackInvReadWrite(IO_CAN_DATA_FRAME* frame)
{
    // TODO rn this only does clear faults but thats allw e need it to do rly
    frame->id = CAN_ID_INV_READ_WRITE;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);

    frame->data[0] = 20;
    frame->data[1] = 0;
    frame->data[2] = 1;
    frame->data[3] = 0;
    frame->data[4] = 0;
    frame->data[5] = 0;
    frame->data[6] = 0;
    frame->data[7] = 0;
}