#include "can_tx.h"
#include "can_util.h"
#include "can_manager.h"
#include "config/can_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "control/torque_controller.h"

#include "state_machine.h"
#include "io/rtd.h"
#include "io/buzzer.h"
#include "io/lights.h"
#include "can_rx.h"
#include "config/runtime_config.h"

void CAN_TX_PackAPPSVoltages(IO_CAN_DATA_FRAME* frame)
{
    const APPS_Data_t* apps_data = APPS_GetData();

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
    const InverterHighSpeed_RX_Data_t* inv = CAN_RX_GetInverterHighSpeedData();
    const bool rtd_active = RTD_IsActive();
    const bool is_red_car = Lights_isRedCar();
    const Buzzer_State_t buzzer_state = Buzzer_GetState();

    sbyte2 motor_speed = inv->motor_speed;
    if (motor_speed < 0) {
        motor_speed = (sbyte2)(-motor_speed);
    }
    const ubyte2 speed = (ubyte2)motor_speed;

    /* DAQBus.dbc / VCU_Summary
       byte0: VCU_Heartbeat
       byte1: VCU_State
         byte2-3: VCU_Speed (currently inverter motor speed, abs(rpm))
         byte4 bit0: VCU_RTD_Active
         byte45 VCU_Buzzer_State
     */
    frame->data[0] = heartbeat++;
    frame->data[1] = (ubyte1)state;
    frame->data[2] = speed & 0xFF;
    frame->data[3] = speed >> 8;
    frame->data[4] = rtd_active;
    frame->data[4] |= is_red_car << 1;
    frame->data[5] = (ubyte1)buzzer_state;
}

void CAN_TX_PackBSE(IO_CAN_DATA_FRAME* frame)
{
    const BSE_Data_t* bse_data = BSE_GetData();

    /* Byte0 flags */
    frame->data[0] = (ubyte1)((bse_data->stale << 0) |
                             (bse_data->adc_err << 1) |
                             (bse_data->out_of_range << 2) |
                             (bse_data->valid << 3));

    /* Byte1-2: PSI x10 */
    const ubyte4 psi_x10_32 = (ubyte4)bse_data->psi * 10U;
    const ubyte2 psi_x10 = (psi_x10_32 > 0xFFFFU) ? (ubyte2)0xFFFFU : (ubyte2)psi_x10_32;
    frame->data[1] = (ubyte1)(psi_x10 & 0xFF);
    frame->data[2] = (ubyte1)(psi_x10 >> 8);

    /* Byte3-4: filtered mV */
    frame->data[3] = (ubyte1)(bse_data->filt_mv & 0xFF);
    frame->data[4] = (ubyte1)(bse_data->filt_mv >> 8);

    /* Byte5-6: raw mV */
    frame->data[5] = (ubyte1)(bse_data->raw_mv & 0xFF);
    frame->data[6] = (ubyte1)(bse_data->raw_mv >> 8);
}

void CAN_TX_PackConfig(IO_CAN_DATA_FRAME* frame)
{
    static ubyte1 config_tx_cycle_idx = 0;
    sbyte2 param_value = 0;

    config_tx_cycle_idx++;
    if (config_tx_cycle_idx >= RUNTIME_PARAM_COUNT) {
        config_tx_cycle_idx = 0;
    }

    (void)RuntimeConfig_GetI32((RuntimeParamId_t)config_tx_cycle_idx, &param_value);

    const ubyte2 packed_value = (ubyte2)(param_value);

    frame->data[0] = (ubyte1)config_tx_cycle_idx;
    frame->data[1] = (ubyte1)(packed_value & 0xFF);
    frame->data[2] = (ubyte1)(packed_value >> 8);
}

void CAN_TX_PackCANHealth(IO_CAN_DATA_FRAME* frame)
{
    static ubyte1 health_heartbeat = 0;
    const CAN_Manager_Health_t* health = CAN_Manager_GetHealthData();

    frame->data[0] = health_heartbeat++;
    frame->data[1] = health->controls_tx_error_counter;
    frame->data[2] = health->controls_rx_error_counter;
    frame->data[3] = health->daq_tx_error_counter;
    frame->data[4] = health->daq_rx_error_counter;

    frame->data[5] = health->controls_error_passive;
    frame->data[5] |= health->controls_bus_off << 1;
    frame->data[5] |= health->daq_error_passive << 2;
    frame->data[5] |= health->daq_bus_off << 3;
    frame->data[5] |= health->controls_tx_fault_seen << 4;
    frame->data[5] |= health->controls_rx_fault_seen << 5;
    frame->data[5] |= health->daq_tx_fault_seen << 6;
    frame->data[5] |= health->daq_rx_fault_seen << 7;

    frame->data[6] = CAN_Util_TranslateStatus((IO_ErrorType)health->controls_status_ret_raw);
    frame->data[7] = CAN_Util_TranslateStatus((IO_ErrorType)health->daq_status_ret_raw);
}

void CAN_TX_PackCANHealthFifo(IO_CAN_DATA_FRAME* frame)
{
    static ubyte1 fifo_health_heartbeat = 0;
    static ubyte1 fifo_mux_idx = 0;

    const CAN_Manager_Health_t* health = CAN_Manager_GetHealthData();
    const ubyte1 idx = fifo_mux_idx;
    const CAN_HealthFifoData_t* fifo_data = &health->fifos[idx];

    frame->data[0] = fifo_health_heartbeat++;
    frame->data[1] = idx;
    frame->data[2] = CAN_Util_TranslateStatus((IO_ErrorType)fifo_data->last_status_raw);
    frame->data[3] = (ubyte1)(fifo_data->invalid_data_count & 0xFF);
    frame->data[4] = (ubyte1)(fifo_data->fifo_full_count & 0xFF);
    frame->data[5] = (ubyte1)(fifo_data->overflow_count & 0xFF);
    frame->data[6] = (ubyte1)(fifo_data->other_error_count & 0xFF);
    frame->data[7] = 0;

    fifo_mux_idx++;
    if (fifo_mux_idx >= CAN_HEALTH_FIFO_COUNT) {
        fifo_mux_idx = 0;
    }
}

void CAN_TX_PackDeadCar(IO_CAN_DATA_FRAME* frame)
{
    // i mean technically this should work i think but maybe more robust to have statemachine give ground zero truth
    // nah this should work
    const HVCSummary_RX_Data_t* hvc = CAN_RX_GetHVCSummaryData();
    const bool hvc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SUMMARY);
    const bool imd_ok = hvc->imd_ok;
    const bool bms_ok = hvc->bms_ok;
    const bool sdc_ok = hvc->sdc_ok;
    // will do exit drive shit later idk. this is all so goober because they should be getting sent anyways hmmm

    frame->data[0] = hvc_valid;
    frame->data[0] |= imd_ok << 1;
    frame->data[0] |= bms_ok << 2;
    frame->data[0] |= sdc_ok << 3;

}
