#include "can_tx.h"
#include "can_util.h"
#include "can_manager.h"
#include "config/can_config.h"
#include "sensors/apps.h"
#include "sensors/bse.h"
#include "control/torque_controller.h"
#include "control/traction_control.h"
#include "control/launch_control.h"
#include "control/power_limit.h"

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

void CAN_TX_PackInverterCurrentLimit(IO_CAN_DATA_FRAME* frame)
{
    const PowerLimit_Data_t* limit = PowerLimit_GetData();

    /* byte0-1: discharge current limit (DCL) in Amps, LE
       byte2-3: charge current limit (CCL) in Amps, LE */
    frame->data[0] = (ubyte1)(limit->dcl_amps & 0xFF);
    frame->data[1] = (ubyte1)(limit->dcl_amps >> 8);
    frame->data[2] = (ubyte1)(limit->ccl_amps & 0xFF);
    frame->data[3] = (ubyte1)(limit->ccl_amps >> 8);
    frame->data[4] = 0;
    frame->data[5] = 0;
    frame->data[6] = 0;
    frame->data[7] = 0;
}

void CAN_TX_PackVCURegenDebug(IO_CAN_DATA_FRAME* frame)
{
    static ubyte1 regen_debug_mux = 0;
    const TorqueController_Data_T* torque = TorqueController_GetData();

    frame->data[0] = regen_debug_mux;

    switch (regen_debug_mux) {
    case 0:
        frame->data[1] = (ubyte1)((torque->regen_soc_gate_enabled ? 1U : 0U) << 2);
        frame->data[2] = torque->regen_status_flags;
        frame->data[3] = torque->regen_block_reason;
        frame->data[4] = (ubyte1)((ubyte2)torque->regen_torque & 0xFF);
        frame->data[5] = (ubyte1)((ubyte2)torque->regen_torque >> 8);
        frame->data[6] = (ubyte1)(torque->regen_speed_rpm_abs & 0xFF);
        frame->data[7] = (ubyte1)(torque->regen_speed_rpm_abs >> 8);
        break;

    case 1:
        frame->data[1] = 0;
        frame->data[2] = (ubyte1)(torque->regen_front_pressure_psi & 0xFF);
        frame->data[3] = (ubyte1)(torque->regen_front_pressure_psi >> 8);
        frame->data[4] = (ubyte1)(torque->regen_rear_pressure_psi_x10 & 0xFF);
        frame->data[5] = (ubyte1)(torque->regen_rear_pressure_psi_x10 >> 8);
        frame->data[6] = (ubyte1)((ubyte2)torque->inv_torque_scaled & 0xFF);
        frame->data[7] = (ubyte1)((ubyte2)torque->inv_torque_scaled >> 8);
        break;

    case 2:
    default:
        frame->data[1] = 0;
        frame->data[2] = 0;
        frame->data[3] = 0;
        frame->data[4] = (ubyte1)((ubyte2)torque->regen_balance_torque & 0xFF);
        frame->data[5] = (ubyte1)((ubyte2)torque->regen_balance_torque >> 8);
        frame->data[6] = (ubyte1)((ubyte2)torque->regen_final_torque & 0xFF);
        frame->data[7] = (ubyte1)((ubyte2)torque->regen_final_torque >> 8);
        break;
    }

    regen_debug_mux++;
    if (regen_debug_mux > 2U) {
        regen_debug_mux = 0;
    }
}

void CAN_TX_PackVCUSummary(IO_CAN_DATA_FRAME* frame)
{
    static ubyte1 heartbeat;
    const VCU_State_t state = StateMachine_GetState();
    const TorqueController_Data_T* torque = TorqueController_GetData();
    const HVCSummary_RX_Data_t* hvc_summary = CAN_RX_GetHVCSummaryData();
    const bool hvc_summary_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SUMMARY);
    const bool rtd_active = RTD_IsActive();
    const bool is_red_car = Lights_isRedCar();
    const Buzzer_State_t buzzer_state = Buzzer_GetState();

    /* Saturate speed_mph_x100 to ubyte2 max (65535 = 655.35 mph). */
    ubyte2 speed_mph_x100 = (ubyte2)(torque->speed_mph_x100);
    if (torque->speed_mph_x100 > 65535U) {
        speed_mph_x100 = 65535U;
    }

    /* DAQBus.dbc / VCU_Summary
       byte0: VCU_Heartbeat
       byte1: VCU_State
    byte2-3: VCU_Speed_MPH (vehicle speed in mph x100)
       byte4 bit0: VCU_RTD_Active
       byte4 bit1: VCU_IsRedCar
             byte4 bit2: HVC summary valid
             byte4 bit3: HVC SDC closed
             byte4 bit4: HVC IMD OK
             byte4 bit5: HVC BMS fault OK
       byte5: VCU_Buzzer_State
     */
    frame->data[0] = heartbeat++;
    frame->data[1] = (ubyte1)state;
    frame->data[2] = speed_mph_x100 & 0xFF;
    frame->data[3] = speed_mph_x100 >> 8;
    frame->data[4] = rtd_active;
    frame->data[4] |= is_red_car << 1;
    frame->data[4] |= hvc_summary_valid << 2;
    frame->data[4] |= hvc_summary->sdc_ok << 3;
    frame->data[4] |= hvc_summary->imd_ok << 4;
    frame->data[4] |= hvc_summary->bms_ok << 5;
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
    RuntimeParamId_t param_id = RUNTIME_PARAM_MAX_TORQUE;

    const bool use_immediate_mux = RuntimeConfig_ConsumeImmediateConfigTxParam(&param_id);

    if (!use_immediate_mux) {
        config_tx_cycle_idx++;
        if (config_tx_cycle_idx >= RUNTIME_PARAM_COUNT) {
            config_tx_cycle_idx = 0;
        }

        param_id = (RuntimeParamId_t)config_tx_cycle_idx;
    }

    (void)RuntimeConfig_GetI32(param_id, &param_value);

    const ubyte2 packed_value = (ubyte2)(param_value);

    frame->data[0] = (ubyte1)param_id;
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

void CAN_TX_PackTractionControl(IO_CAN_DATA_FRAME* frame)
{
    const TractionControl_Data_t* tc = TractionControl_GetData();

    frame->data[0] = (ubyte1)(tc->rear_wheel_rpm & 0xFF);
    frame->data[1] = (ubyte1)(tc->rear_wheel_rpm >> 8);
    frame->data[2] = (ubyte1)(tc->avg_front_rpm & 0xFF);
    frame->data[3] = (ubyte1)(tc->avg_front_rpm >> 8);
    frame->data[4] = (ubyte1)(tc->slip_ratio_x1000 & 0xFF);
    frame->data[5] = (ubyte1)(tc->slip_ratio_x1000 >> 8);
    frame->data[6] = (ubyte1)tc->torque_limit_nm;
    frame->data[7] = (ubyte1)((tc->enabled << 0) |
                              (tc->active << 1) |
                              (tc->front_left_valid << 2) |
                              (tc->front_right_valid << 3));
}

void CAN_TX_PackVCULaunchState(IO_CAN_DATA_FRAME* frame)
{
    const LaunchControl_Data_t* launch = LaunchControl_GetData();

    ubyte2 elapsed_ms = (ubyte2)((launch->elapsed_ms > 65535UL) ? 65535UL
                                                                : launch->elapsed_ms);
    sbyte2 curve_torque = launch->curve_torque_nm;
    if (curve_torque < 0) {
        curve_torque = 0;
    }
    if (curve_torque > 255) {
        curve_torque = 255;
    }

    frame->data[0] = (ubyte1)launch->state;
    frame->data[1] = (ubyte1)(((launch->armed ? 1U : 0U) << 0) |
                              ((launch->active ? 1U : 0U) << 1));
    frame->data[2] = (ubyte1)(elapsed_ms & 0xFF);
    frame->data[3] = (ubyte1)(elapsed_ms >> 8);
    frame->data[4] = (ubyte1)curve_torque;
    frame->data[5] = 0;
    frame->data[6] = 0;
    frame->data[7] = 0;
}

void CAN_TX_PackCANReadback(IO_CAN_DATA_FRAME* frame)
{
    static ubyte1 readback_mux = 0;

    const HVCSummary_RX_Data_t* hvc_summary = CAN_RX_GetHVCSummaryData();
    const HVCSOC_RX_Data_t* hvc_soc = CAN_RX_GetHVCSOCData();
    const HVCVSense_RX_Data_t* hvc_vsense = CAN_RX_GetHVCVSenseData();
    const MOBO_PowerTelemetry_RX_Data_t* mobo_power = CAN_RX_GetMOBO_PowerTelemetryData();
    const InverterMotorPosition_RX_Data_t* inverter_position = CAN_RX_GetInverterMotorPositionData();
    const InverterHighSpeed_RX_Data_t* inverter = CAN_RX_GetInverterHighSpeedData();
    const FrontWheelRpm_RX_Data_t* front_left = CAN_RX_GetFrontLeftRpmData();
    const FrontWheelRpm_RX_Data_t* front_right = CAN_RX_GetFrontRightRpmData();

    frame->data[0] = readback_mux;

    switch (readback_mux) {
    case 0:
        frame->data[1] = (ubyte1)((CAN_Manager_RX_Data_Valid(CAN_RX_MSG_INV_HIGH_SPEED) << 0) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SUMMARY) << 1) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SOC) << 2) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_VSENSE) << 3) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_MOBO_POWER_TELEMETRY) << 4) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_FRONT_LEFT_RPM) << 5) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_FRONT_RIGHT_RPM) << 6) |
                                  (CAN_Manager_RX_Data_Valid(CAN_RX_MSG_INV_MOTOR_POSITION) << 7));
        frame->data[2] = (ubyte1)((hvc_summary->sdc_ok << 0) |
                                  (hvc_summary->imd_ok << 1) |
                                  (hvc_summary->bms_ok << 2));
        frame->data[3] = hvc_summary->io_summary_flags;
        frame->data[4] = (ubyte1)(hvc_soc->pack_soc_percent_x100 & 0xFF);
        frame->data[5] = (ubyte1)(hvc_soc->pack_soc_percent_x100 >> 8);
        frame->data[6] = (ubyte1)(mobo_power->rear_brake_pressure_psi_x10 & 0xFF);
        frame->data[7] = (ubyte1)(mobo_power->rear_brake_pressure_psi_x10 >> 8);
        break;

    case 1:
        frame->data[1] = 0;
        frame->data[2] = (ubyte1)((ubyte2)inverter->torque_cmd & 0xFF);
        frame->data[3] = (ubyte1)((ubyte2)inverter->torque_cmd >> 8);
        frame->data[4] = (ubyte1)((ubyte2)inverter->torque_feedback & 0xFF);
        frame->data[5] = (ubyte1)((ubyte2)inverter->torque_feedback >> 8);
        frame->data[6] = (ubyte1)((ubyte2)inverter->motor_speed & 0xFF);
        frame->data[7] = (ubyte1)((ubyte2)inverter->motor_speed >> 8);
        break;

    case 2:
        frame->data[1] = 0;
        frame->data[2] = (ubyte1)((ubyte2)inverter->dc_bus_voltage & 0xFF);
        frame->data[3] = (ubyte1)((ubyte2)inverter->dc_bus_voltage >> 8);
        frame->data[4] = (ubyte1)(hvc_vsense->inv_voltage_mv & 0xFF);
        frame->data[5] = (ubyte1)((hvc_vsense->inv_voltage_mv >> 8) & 0xFF);
        frame->data[6] = (ubyte1)((hvc_vsense->inv_voltage_mv >> 16) & 0xFF);
        frame->data[7] = (ubyte1)((hvc_vsense->inv_voltage_mv >> 24) & 0xFF);
        break;

    case 3:
    default:
        frame->data[1] = 0;
        frame->data[2] = (ubyte1)(front_left->rpm & 0xFF);
        frame->data[3] = (ubyte1)(front_left->rpm >> 8);
        frame->data[4] = (ubyte1)(front_right->rpm & 0xFF);
        frame->data[5] = (ubyte1)(front_right->rpm >> 8);
        frame->data[6] = (ubyte1)((ubyte2)inverter_position->motor_speed & 0xFF);
        frame->data[7] = (ubyte1)((ubyte2)inverter_position->motor_speed >> 8);
        break;
    }

    readback_mux++;
    if (readback_mux > 3) {
        readback_mux = 0;
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
