#include "can_tx_messages.h"
#include "can_util.h"
#include "config/can_config.h"
#include "sensors/apps.h"
#include "control/torque_controller.h"

void CAN_TX_PackAPPSVoltages(IO_CAN_DATA_FRAME* apps_voltages_frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();
    
    apps_voltages_frame->id = CAN_ID_APPS_VOLTAGES;
    apps_voltages_frame->id_format = IO_CAN_EXT_FRAME;
    apps_voltages_frame->length = 8;
    CAN_Util_ClearData(apps_voltages_frame);
    
    apps_voltages_frame->data[0] = apps_data->apps2.filt_mv & 0xFF;
    apps_voltages_frame->data[1] = apps_data->apps2.filt_mv >> 8;
    apps_voltages_frame->data[2] = apps_data->apps2.raw_mv & 0xFF;
    apps_voltages_frame->data[3] = apps_data->apps2.raw_mv >> 8;
    apps_voltages_frame->data[4] = apps_data->apps1.filt_mv & 0xFF;
    apps_voltages_frame->data[5] = apps_data->apps1.filt_mv >> 8;
    apps_voltages_frame->data[6] = apps_data->apps1.raw_mv & 0xFF;
    apps_voltages_frame->data[7] = apps_data->apps1.raw_mv >> 8;
}

void CAN_TX_PackAPPSValues(IO_CAN_DATA_FRAME* apps_values_frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();
    
    apps_values_frame->id = CAN_ID_APPS_VALUES;
    apps_values_frame->id_format = IO_CAN_EXT_FRAME;
    apps_values_frame->length = 8;
    CAN_Util_ClearData(apps_values_frame);

    apps_values_frame->data[0] = (apps_data->apps1.valid << 7) |
                                 (apps_data->apps1.out_of_range << 6) |
                                 (apps_data->apps1.adc_err << 5) |
                                 (apps_data->apps1.stale << 4) |
                                 (apps_data->apps2.valid << 3) |
                                 (apps_data->apps2.out_of_range << 2) |
                                 (apps_data->apps2.adc_err << 1) |
                                 apps_data->apps2.stale;
    apps_values_frame->data[1] = apps_data->apps2.value & 0xFF;
    apps_values_frame->data[2] = apps_data->apps2.value >> 8;
    apps_values_frame->data[3] = apps_data->apps1.value & 0xFF;
    apps_values_frame->data[4] = apps_data->apps1.value >> 8;
    apps_values_frame->data[5] = (apps_data->valid << 4) | apps_data->implausible;
    apps_values_frame->data[6] = apps_data->apps_value & 0xFF;
    apps_values_frame->data[7] = apps_data->apps_value >> 8;
}

void CAN_TX_PackTorqueCommand(IO_CAN_DATA_FRAME* torque_command_frame)
{
    const TorqueController_Data_T* torque_data = TorqueController_GetData();

    torque_command_frame->id = CAN_ID_TORQUE_COMMAND;
    torque_command_frame->id_format = IO_CAN_EXT_FRAME;
    torque_command_frame->length = 8;
    CAN_Util_ClearData(torque_command_frame);

    torque_command_frame->data[0] = torque_data->inv_torque_scaled & 0xFF;
    torque_command_frame->data[1] = torque_data->inv_torque_scaled >> 8;
    // TODO speed command?
    torque_command_frame->data[2] = 0;
    torque_command_frame->data[3] = 0;
    torque_command_frame->data[4] = torque_data->inv_direction;
    torque_command_frame->data[5] = torque_data->inv_enable;
    torque_command_frame->data[6] = torque_data->inv_speed_mode;
    // TOOD torque limit. useful prolly not?
    torque_command_frame->data[7] = 0;
}
