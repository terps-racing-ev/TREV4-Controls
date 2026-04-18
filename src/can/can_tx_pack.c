#include "can_tx_pack.h"
#include "can_util.h"
#include "config/can_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "control/torque_controller.h"

#include "state_machine.h"
#include "io/rtd.h"
#include "io/buzzer.h"
#include "can_manager.h"
#include "settings/runtime_config.h"

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
    frame->id_format = IO_CAN_STD_FRAME;
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
    frame->id_format = IO_CAN_STD_FRAME;
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

void CAN_TX_PackVCUSummary(IO_CAN_DATA_FRAME* frame)
{

    static ubyte1 heartbeat;
    const VCU_State_t state = StateMachine_GetState();
    const InverterHighSpeed_RX_Data_t* inv = CAN_Manager_GetInverterHighSpeedData();
    const bool rtd_active = RTD_IsActive();
    const Buzzer_State_t buzzer_state = Buzzer_GetState();
    sbyte2 motor_speed;
    ubyte2 speed;

    frame->id = CAN_ID_VCU_SUMMARY;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);

    motor_speed = inv->motor_speed;
    if (motor_speed < 0) {
        motor_speed = (sbyte2)(-motor_speed);
    }
    speed = (ubyte2)motor_speed;

    /* DAQBus.dbc / VCU_Summary
       byte0: VCU_Heartbeat
       byte1: VCU_State
         byte2-3: VCU_Speed (currently inverter motor speed, abs(rpm))
         byte4 bit0: VCU_RTD_Active
         byte4 bit1: VCU_Buzzer_State (1 if not inactive)
     */
    frame->data[0] = heartbeat++;
    frame->data[1] = (ubyte1)state;
    frame->data[2] = speed & 0xFF;
    frame->data[3] = speed >> 8;
     frame->data[4] = (ubyte1)(((ubyte1)rtd_active << 0) |
                    ((ubyte1)(buzzer_state != BUZZER_STATE_INACTIVE) << 1));
}

void CAN_TX_PackBSE(IO_CAN_DATA_FRAME* frame)
{
    const BSE_Data_t* bse_data = BSE_GetData();
    ubyte4 psi_x10_32;
    ubyte2 psi_x10;

    frame->id = CAN_ID_BSE;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);

    /* Byte0 flags */
    frame->data[0] = (ubyte1)((bse_data->stale << 0) |
                             (bse_data->adc_err << 1) |
                             (bse_data->out_of_range << 2) |
                             (bse_data->valid << 3));

    /* Byte1-2: PSI x10 */
    psi_x10_32 = (ubyte4)bse_data->psi * 10U;
    psi_x10 = (psi_x10_32 > 0xFFFFU) ? (ubyte2)0xFFFFU : (ubyte2)psi_x10_32;
    frame->data[1] = (ubyte1)(psi_x10 & 0xFF);
    frame->data[2] = (ubyte1)(psi_x10 >> 8);

    /* Byte3-4: filtered mV */
    frame->data[3] = (ubyte1)(bse_data->filt_mv & 0xFF);
    frame->data[4] = (ubyte1)(bse_data->filt_mv >> 8);

    /* Byte5-6: raw mV */
    frame->data[5] = (ubyte1)(bse_data->raw_mv & 0xFF);
    frame->data[6] = (ubyte1)(bse_data->raw_mv >> 8);
}



void CAN_TX_PackVCUSettings(IO_CAN_DATA_FRAME* frame)
{
    static ubyte2 seq;
    ubyte2 pid;
    sbyte4 val;

    frame->id = CAN_ID_VCU_SETTINGS;
    frame->id_format = IO_CAN_EXT_FRAME;
    frame->length = 8;
    CAN_Util_ClearData(frame);

    if (!RuntimeConfig_GetNextBroadcastParam(&pid, &val)) {
        return;
    }

    /* Payload:
       bytes0-1: param_id (u16)
       bytes2-3: sequence (u16)
       bytes4-7: value (i32)
    */
    frame->data[0] = pid & 0xFF;
    frame->data[1] = pid >> 8;

    frame->data[2] = seq & 0xFF;
    frame->data[3] = seq >> 8;
    seq++;

    frame->data[4] = (ubyte1)((ubyte4)val & 0xFF);
    frame->data[5] = (ubyte1)(((ubyte4)val >> 8) & 0xFF);
    frame->data[6] = (ubyte1)(((ubyte4)val >> 16) & 0xFF);
    frame->data[7] = (ubyte1)(((ubyte4)val >> 24) & 0xFF);
}

void CAN_TX_PackVCUExitDriveReason(IO_CAN_DATA_FRAME* frame)
{
    
}