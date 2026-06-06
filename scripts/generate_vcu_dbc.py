#!/usr/bin/env python3
"""Generate a VCU-focused DBC for the CAN message definitions used in this repo."""

from pathlib import Path

HEADER_LINES = [
    'VERSION ""',
    '',
    'NS_ :',
    '',
    'BS_:',
    '',
    'BU_: BL DBF DBL DBR FR HVC INV MOBO RPI SET VCU',
    '',
]

MESSAGE_BLOCKS = [
    """BO_ 192 VCU_INV_Command: 8 VCU
 SG_ VCU_INV_Torque_Cmd : 0|16@1- (0.1,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_INV_Speed_Cmd : 16|16@1- (1.0,0.0) [-6000.0|6000.0] "RPM" Vector__XXX
 SG_ VCU_INV_Direction_Cmd : 32|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_INV_Inverter_Enable : 40|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_INV_Speed_Enable : 42|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_INV_Discharge_Enable : 41|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_INV_Torque_Limit_Cmd : 48|16@1- (0.1,0.0) [-230.0|230.0] "Nm" Vector__XXX""",
    """BO_ 514 VCU_INV_Current_Limit: 8 VCU
 SG_ VCU_INV_Discharge_Current_Limit : 0|16@1+ (1.0,0.0) [0.0|65535.0] "A" Vector__XXX
 SG_ VCU_INV_Charge_Current_Limit : 16|16@1+ (1.0,0.0) [0.0|65535.0] "A" Vector__XXX""",
    """BO_ 2147483855 SET_VCU_Config: 8 SET
 SG_ SET_VCU_Select M : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ SET_VCU_Max_Torque m0 : 8|16@1- (0.1,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Motor_Direction m1 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Regen_Enabled m2 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_ECHO_DAQ m3 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Ignore_RTD_Switch m3 : 9|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Ignore_RTD_Brakes m3 : 10|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Use_APPS1_Only m3 : 11|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Use_APPS2_Only m3 : 12|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Ignore_APPS_Errs m3 : 13|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Ignore_BSE_Errs m3 : 14|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Ignore_SDC m3 : 15|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Ignore_Brake_Plausibility m3 : 16|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Always_Green m3 : 17|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Wheel_Diameter m4 : 8|16@1+ (1.0,0.0) [8.0|30.0] "in" Vector__XXX
 SG_ SET_VCU_TC_Enabled m5 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_TC_Target_Slip m6 : 8|16@1+ (0.001,0.0) [1.0|5.0] "" Vector__XXX
 SG_ SET_VCU_TC_Kp m7 : 8|16@1+ (0.001,0.0) [0.0|32.767] "" Vector__XXX
 SG_ SET_VCU_TC_Ki m8 : 8|16@1+ (0.001,0.0) [0.0|32.767] "" Vector__XXX
 SG_ SET_VCU_TC_Kd m9 : 8|16@1+ (0.001,0.0) [0.0|32.767] "" Vector__XXX
 SG_ SET_VCU_TC_Min_Front_RPM m10 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Power_Limit_Enabled m11 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Power_Cap_kW m12 : 8|16@1+ (1.0,0.0) [5.0|100.0] "kW" Vector__XXX
 SG_ SET_VCU_Regen_Max_Torque m13 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Regen_Min_Torque m14 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Regen_Min_BSE_Rear_PSI m15 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ SET_VCU_Regen_Min_BSE_Front_PSI m16 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ SET_VCU_Regen_Max_BSE_Rear_PSI m17 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ SET_VCU_Regen_Max_BSE_Front_PSI m18 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ SET_VCU_Regen_Min_Speed m19 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Regen_Max_SOC m20 : 8|16@1+ (1.0,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ SET_VCU_Regen_Strategy m21 : 8|8@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ SET_VCU_Launch_Enabled m22 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Launch_End_RPM m23 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Launch_Timeout_ms m24 : 8|16@1+ (1.0,0.0) [0.0|60000.0] "ms" Vector__XXX
 SG_ SET_VCU_Launch_Max_Slip m25 : 8|16@1+ (0.001,0.0) [1.0|5.0] "" Vector__XXX
 SG_ SET_VCU_Launch_Best_Curve m26 : 8|8@1+ (1.0,0.0) [0.0|2.0] "" Vector__XXX
 SG_ SET_VCU_Launch_Active_Curve m27 : 8|8@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ SET_VCU_Launch_Recommended_Slip m28 : 8|16@1+ (0.001,0.0) [1.0|5.0] "" Vector__XXX
 SG_ SET_VCU_Launch_Actual_RPM_0 m29 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Launch_Actual_RPM_1 m30 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Launch_Actual_RPM_2 m31 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Launch_Actual_RPM_3 m32 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Launch_Actual_RPM_4 m33 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ SET_VCU_Launch_Actual_Torque_0 m34 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Launch_Actual_Torque_1 m35 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Launch_Actual_Torque_2 m36 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Launch_Actual_Torque_3 m37 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Launch_Actual_Torque_4 m38 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ SET_VCU_Regen_SOC_Gate_Enabled m39 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ SET_VCU_Regen_Max_APPS m40 : 8|16@1+ (1.0,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ SET_VCU_Regen_Ryder_Mu m41 : 8|16@1+ (0.001,0.0) [0.0|10.0] "" Vector__XXX
 SG_ SET_VCU_TRC_Command m240 : 8|16@1+ (1.0,0.0) [0.0|9.0] "" Vector__XXX""",
    """BO_ 2148532431 VCU_Config: 8 VCU
 SG_ VCU_Config_Mux M : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_Max_Torque m0 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Motor_Direction m1 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Enabled m2 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_ECHO_DAQ m3 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Ignore_RTD_Switch m3 : 9|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Ignore_RTD_Brakes m3 : 10|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Use_APPS1_Only m3 : 11|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Use_APPS2_Only m3 : 12|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Ignore_APPS_Errs m3 : 13|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Ignore_BSE_Errs m3 : 14|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Ignore_SDC m3 : 15|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Ignore_Brake_Plausibility m3 : 16|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Always_Green m3 : 17|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Wheel_Diameter m4 : 8|16@1+ (1.0,0.0) [8.0|30.0] "in" Vector__XXX
 SG_ VCU_TC_Enabled m5 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TC_Target_Slip m6 : 8|16@1+ (0.001,0.0) [1.0|5.0] "" Vector__XXX
 SG_ VCU_TC_Kp m7 : 8|16@1+ (0.001,0.0) [0.0|32.767] "" Vector__XXX
 SG_ VCU_TC_Ki m8 : 8|16@1+ (0.001,0.0) [0.0|32.767] "" Vector__XXX
 SG_ VCU_TC_Kd m9 : 8|16@1+ (0.001,0.0) [0.0|32.767] "" Vector__XXX
 SG_ VCU_TC_Min_Front_RPM m10 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Power_Limit_Enabled m11 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Power_Cap_kW m12 : 8|16@1+ (1.0,0.0) [5.0|100.0] "kW" Vector__XXX
 SG_ VCU_Regen_Max_Torque m13 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_Min_Torque m14 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_Min_BSE_Rear_PSI m15 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_Regen_Min_BSE_Front_PSI m16 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_Regen_Max_BSE_Rear_PSI m17 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_Regen_Max_BSE_Front_PSI m18 : 8|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_Regen_Min_Speed m19 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Regen_Max_SOC m20 : 8|16@1+ (1.0,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ VCU_Regen_Strategy m21 : 8|8@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ VCU_Launch_Enabled m22 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Launch_End_RPM m23 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Launch_Timeout_ms m24 : 8|16@1+ (1.0,0.0) [0.0|60000.0] "ms" Vector__XXX
 SG_ VCU_Launch_Max_Slip m25 : 8|16@1+ (0.001,0.0) [1.0|5.0] "" Vector__XXX
 SG_ VCU_Launch_Best_Curve m26 : 8|8@1+ (1.0,0.0) [0.0|2.0] "" Vector__XXX
 SG_ VCU_Launch_Active_Curve m27 : 8|8@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ VCU_Launch_Recommended_Slip m28 : 8|16@1+ (0.001,0.0) [1.0|5.0] "" Vector__XXX
 SG_ VCU_Launch_Actual_RPM_0 m29 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Launch_Actual_RPM_1 m30 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Launch_Actual_RPM_2 m31 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Launch_Actual_RPM_3 m32 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Launch_Actual_RPM_4 m33 : 8|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Launch_Actual_Torque_0 m34 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Launch_Actual_Torque_1 m35 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Launch_Actual_Torque_2 m36 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Launch_Actual_Torque_3 m37 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Launch_Actual_Torque_4 m38 : 8|16@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_SOC_Gate_Enabled m39 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Max_APPS m40 : 8|16@1+ (1.0,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ VCU_Regen_Ryder_Mu m41 : 8|16@1+ (0.001,0.0) [0.0|10.0] "" Vector__XXX""",
    """BO_ 2148532928 VCU_MOBO_Command: 8 VCU
 SG_ VCU_Pump_Toggle_Cmd : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_DRS_Toggle_Cmd : 9|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Fans_Toggle_Cmd : 10|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Rad_Toggle_Cmd : 11|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX""",
    """BO_ 2366636528 VCU_Summary: 8 VCU
 SG_ VCU_Heartbeat : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_State : 8|8@1+ (1.0,0.0) [0.0|5.0] "" Vector__XXX
 SG_ VCU_Speed : 16|16@1- (1.0,0.0) [-6000.0|6000.0] "RPM" Vector__XXX
 SG_ VCU_RTD_Active : 32|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Red_Car : 33|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_HVC_Summary_Valid : 34|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_HVC_SDC_Ok : 35|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_HVC_IMD_Ok : 36|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_HVC_BMS_Ok : 37|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Buzzer_State : 40|8@1+ (1.0,0.0) [0.0|2.0] "" Vector__XXX""",
   """BO_ 2366638016 VCU_Traction_Control: 8 VCU
 SG_ VCU_TC_Rear_Wheel_RPM : 0|16@1+ (1.0,0.0) [0.0|65535.0] "RPM" Vector__XXX
 SG_ VCU_TC_Avg_Front_RPM : 16|16@1+ (1.0,0.0) [0.0|65535.0] "RPM" Vector__XXX
 SG_ VCU_TC_Slip_Ratio : 32|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX
 SG_ VCU_TC_Torque_Limit : 48|8@1+ (1.0,0.0) [0.0|230.0] "Nm" Vector__XXX
 SG_ VCU_TC_Enabled_Status : 56|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TC_Active : 57|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TC_Front_Left_Valid : 58|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TC_Front_Right_Valid : 59|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX""",
    """BO_ 2366637792 VCU_Regen_Debug: 8 VCU
 SG_ VCU_Regen_Debug_Mux M : 0|8@1+ (1.0,0.0) [0.0|2.0] "" Vector__XXX
 SG_ VCU_Regen_Strategy_Status m0 : 8|2@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ VCU_Regen_SOC_Gate_Status m0 : 10|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Enabled_Status m0 : 16|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Driving_Status m0 : 17|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Speed_OK m0 : 18|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Front_Valid m0 : 19|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Rear_Valid m0 : 20|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Ryder_Active m0 : 21|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Active m0 : 22|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_SOC_Valid m0 : 23|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Regen_Block_Reason m0 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_Regen_Torque_Request m0 : 32|16@1- (1.0,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_Speed_RPM_Abs m0 : 48|16@1+ (1.0,0.0) [0.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_Regen_Front_PSI m1 : 16|16@1+ (1.0,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_Regen_Rear_PSI m1 : 32|16@1+ (0.1,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_Regen_INV_Torque_Cmd m1 : 48|16@1- (0.1,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_Ryder_Table_Torque m2 : 16|16@1- (1.0,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_Ryder_Balance_Torque m2 : 32|16@1- (1.0,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_Regen_Final_Positive_Torque m2 : 48|16@1- (1.0,0.0) [-230.0|230.0] "Nm" Vector__XXX""",
   """BO_ 2366638272 VCU_TRC_State: 8 VCU
 SG_ VCU_TRC_State_Machine : 0|8@1+ (1.0,0.0) [0.0|16.0] "" Vector__XXX
 SG_ VCU_TRC_Current_Run : 8|2@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ VCU_TRC_Selected_Curve : 10|2@1+ (1.0,0.0) [0.0|3.0] "" Vector__XXX
 SG_ VCU_TRC_Learning_Active : 12|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TRC_Launch_Mode_Active : 13|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TRC_Best_Curve : 14|2@1+ (1.0,0.0) [0.0|2.0] "" Vector__XXX
 SG_ VCU_TRC_Grip_Score_A : 16|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX
 SG_ VCU_TRC_Grip_Score_B : 32|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX
 SG_ VCU_TRC_Recommended_Slip : 48|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX""",
    """BO_ 2366638273 VCU_TRC_Run_Data: 8 VCU
 SG_ VCU_TRC_Run_Index : 0|8@1+ (1.0,0.0) [1.0|3.0] "" Vector__XXX
 SG_ VCU_TRC_Run_Valid : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_TRC_Run_Avg_Slip : 16|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX
 SG_ VCU_TRC_Run_Peak_Slip : 32|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX
 SG_ VCU_TRC_Run_Grip_Score : 48|16@1+ (0.001,0.0) [0.0|65.535] "" Vector__XXX""",
    """BO_ 2366687998 VCU_CAN_Readback: 8 VCU
 SG_ VCU_CAN_Readback_Mux M : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_RB_INV_HighSpeed_Valid m0 : 8|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_Summary_Valid m0 : 9|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_SOC_Valid m0 : 10|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_VSense_Valid m0 : 11|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_MOBO_Power_Valid m0 : 12|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_Front_Left_RPM_Valid m0 : 13|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_Front_Right_RPM_Valid m0 : 14|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_INV_Motor_Position_Valid m0 : 15|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_SDC_Ok m0 : 16|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_IMD_Ok m0 : 17|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_BMS_Ok m0 : 18|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_RB_HVC_IO_Summary_Raw m0 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_RB_SOC_Percent m0 : 32|16@1+ (0.01,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ VCU_RB_Rear_BSE_PSI m0 : 48|16@1+ (0.1,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_RB_INV_Torque_Cmd m1 : 16|16@1- (0.1,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_RB_INV_Torque_Feedback m1 : 32|16@1- (0.1,0.0) [-230.0|230.0] "Nm" Vector__XXX
 SG_ VCU_RB_INV_Motor_Speed m1 : 48|16@1- (1.0,0.0) [-32768.0|32767.0] "RPM" Vector__XXX
 SG_ VCU_RB_INV_DC_Bus_Voltage m2 : 16|16@1- (0.1,0.0) [-3276.8|3276.7] "V" Vector__XXX
 SG_ VCU_RB_HVC_Inv_Voltage m2 : 32|32@1+ (1.0,0.0) [0.0|4294967295.0] "mV" Vector__XXX
 SG_ VCU_RB_Front_Left_RPM m3 : 16|16@1+ (1.0,0.0) [0.0|65535.0] "RPM" Vector__XXX
 SG_ VCU_RB_Front_Right_RPM m3 : 32|16@1+ (1.0,0.0) [0.0|65535.0] "RPM" Vector__XXX
 SG_ VCU_RB_INV_Motor_Position_Speed m3 : 48|16@1- (1.0,0.0) [-32768.0|32767.0] "RPM" Vector__XXX""",
    """BO_ 2366638942 VCU_BSE: 8 VCU
 SG_ VCU_BSE_PSI : 8|16@1+ (0.1,0.0) [0.0|10000.0] "PSI" Vector__XXX
 SG_ VCU_BSE_Filt_mV : 24|16@1- (0.001,0.0) [-1000.0|1000.0] "V" Vector__XXX
 SG_ VCU_BSE_Raw_mV : 40|16@1- (0.001,0.0) [-1000.0|1000.0] "V" Vector__XXX
 SG_ VCU_BSE_Stale : 0|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_BSE_ADC_Err : 1|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_BSE_Out_of_Range : 2|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_BSE_Valid : 3|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX""",
    """BO_ 2366639273 VCU_CAN_Health: 8 VCU
 SG_ VCU_Health_Heartbeat : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_Controls_TX_Errs : 8|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_Controls_RX_Errs : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_DAQ_TX_Errs : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_DAQ_RX_Errs : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_Controls_Passive_Err : 40|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Controls_Bus_Off : 41|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_DAQ_Passive_Err : 42|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_DAQ_Bus_Off : 43|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Controls_TX_Fault_Seen : 44|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Controls_RX_Fault_Seen : 45|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_DAQ_TX_Fault_Seen : 46|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_DAQ_RX_Fault_Seen : 47|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Controls_Status : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_DAQ_Status : 56|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX""",
    """BO_ 2366693037 VCU_Dead_Car: 8 VCU
 SG_ VCU_Dead_HVC_Msg_Valid : 0|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Dead_IMD_OK : 1|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Dead_BMS_OK : 2|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_Dead_SDC_OK : 3|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX""",
    """BO_ 2366697968 VCU_CAN_Health_FIFO: 8 VCU
 SG_ VCU_FIFO_Heartbeat : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ VCU_FIFO_Mux M : 8|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_STD_VCU_FIFO_Last_Status m0 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_EXT_VCU_FIFO_Last_Status m1 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_DAQ_EXT_VCU_FIFO_Last_Status m2 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_MOTOR_POSITION_VCU_FIFO_Last_Status m3 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_HIGH_SPEED_VCU_FIFO_Last_Status m4 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_HVC_SUMMARY_VCU_FIFO_Last_Status m5 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_SET_VCU_CONFIG_VCU_FIFO_Last_Status m6 : 16|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_STD_VCU_FIFO_Invld_Data_Count m0 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_EXT_VCU_FIFO_Invld_Data_Count m1 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_DAQ_EXT_VCU_FIFO_Invld_Data_Count m2 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_MOTOR_POSITION_VCU_FIFO_Invld_Data_Count m3 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_HIGH_SPEED_VCU_FIFO_Invld_Data_Count m4 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_HVC_SUMMARY_VCU_FIFO_Invld_Data_Count m5 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_SET_VCU_CONFIG_VCU_FIFO_Invld_Data_Count m6 : 24|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_STD_VCU_FIFO_Full_Count m0 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_EXT_VCU_FIFO_Full_Count m1 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_DAQ_EXT_VCU_FIFO_Full_Count m2 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_MOTOR_POSITION_VCU_FIFO_Full_Count m3 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_HIGH_SPEED_VCU_FIFO_Full_Count m4 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_HVC_SUMMARY_VCU_FIFO_Full_Count m5 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_SET_VCU_CONFIG_VCU_FIFO_Full_Count m6 : 32|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_STD_VCU_FIFO_Overflow_Count m0 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_EXT_VCU_FIFO_Overflow_Count m1 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_DAQ_EXT_VCU_FIFO_Overflow_Count m2 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_MOTOR_POSITION_VCU_FIFO_Overflow_Count m3 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_HIGH_SPEED_VCU_FIFO_Overflow_Count m4 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_HVC_SUMMARY_VCU_FIFO_Overflow_Count m5 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_SET_VCU_CONFIG_VCU_FIFO_Overflow_Count m6 : 40|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_STD_VCU_FIFO_Other_Errs_Count m0 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_CONTROLS_EXT_VCU_FIFO_Other_Errs_Count m1 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ TX_DAQ_EXT_VCU_FIFO_Other_Errs_Count m2 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_MOTOR_POSITION_VCU_FIFO_Other_Errs_Count m3 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_INV_HIGH_SPEED_VCU_FIFO_Other_Errs_Count m4 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_HVC_SUMMARY_VCU_FIFO_Other_Errs_Count m5 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX
 SG_ RX_SET_VCU_CONFIG_VCU_FIFO_Other_Errs_Count m6 : 48|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX""",
    """BO_ 2366639360 VCU_LOG_Event_Flag: 8 VCU
 SG_ VCU_LOG_Event_ID : 0|8@1+ (1.0,0.0) [0.0|255.0] "" Vector__XXX""",
    """BO_ 2367343840 VCU_APPS_Voltages: 8 VCU
 SG_ VCU_APPS2_Filt_mV : 0|16@1- (0.001,0.0) [-1000.0|1000.0] "V" Vector__XXX
 SG_ VCU_APPS2_Raw_mV : 16|16@1- (0.001,0.0) [-1000.0|1000.0] "V" Vector__XXX
 SG_ VCU_APPS1_Filt_mV : 32|16@1- (0.001,0.0) [-1000.0|1000.0] "V" Vector__XXX
 SG_ VCU_APPS1_Raw_mV : 48|16@1- (0.001,0.0) [-1000.0|1000.0] "V" Vector__XXX""",
    """BO_ 2367343841 VCU_APPS_Values: 8 VCU
 SG_ VCU_APPS2_Stale : 0|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS2_ADC_Err : 1|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS2_Out_of_Range : 2|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS2_Valid : 3|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS1_Stale : 4|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS1_ADC_Err : 5|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS1_Out_of_Range : 6|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS1_Valid : 7|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS2_Value : 8|16@1+ (0.1,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ VCU_APPS1_Value : 24|16@1+ (0.1,0.0) [0.0|100.0] "%" Vector__XXX
 SG_ VCU_APPS_Implausible : 40|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS_Valid : 44|1@1+ (1.0,0.0) [0.0|1.0] "" Vector__XXX
 SG_ VCU_APPS_Value : 48|16@1+ (0.1,0.0) [0.0|100.0] "%" Vector__XXX""",
]

VALUE_LINES = [
    'VAL_ 192 VCU_INV_Direction_Cmd 0 "REVERSE" 1 "FORWARD" ;',
    'VAL_ 192 VCU_INV_Inverter_Enable 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 192 VCU_INV_Speed_Enable 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 192 VCU_INV_Discharge_Enable 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2147483855 SET_VCU_Select 0 "MAX_TORQUE" 1 "MOTOR_DIRECTION" 2 "REGEN_ENABLED" 3 "DEBUG_DEFINES" 4 "WHEEL_DIAMETER" 5 "TRACTION_CONTROL_ENABLED" 6 "TRACTION_CONTROL_TARGET_SLIP" 7 "TRACTION_CONTROL_KP" 8 "TRACTION_CONTROL_KI" 9 "TRACTION_CONTROL_KD" 10 "TRACTION_CONTROL_MIN_FRONT_RPM" 11 "POWER_LIMIT_ENABLED" 12 "POWER_CAP_KW" 13 "REGEN_MAX_TORQUE" 14 "REGEN_MIN_TORQUE" 15 "REGEN_MIN_BSE_REAR_PSI" 16 "REGEN_MIN_BSE_FRONT_PSI" 17 "REGEN_MAX_BSE_REAR_PSI" 18 "REGEN_MAX_BSE_FRONT_PSI" 19 "REGEN_MIN_SPEED" 20 "REGEN_MAX_SOC" 21 "REGEN_STRATEGY" 22 "LAUNCH_ENABLED" 23 "LAUNCH_END_RPM" 24 "LAUNCH_TIMEOUT_MS" 25 "LAUNCH_MAX_SLIP" 26 "LAUNCH_BEST_CURVE" 27 "LAUNCH_ACTIVE_CURVE" 28 "LAUNCH_RECOMMENDED_SLIP" 29 "LAUNCH_ACTUAL_RPM_0" 30 "LAUNCH_ACTUAL_RPM_1" 31 "LAUNCH_ACTUAL_RPM_2" 32 "LAUNCH_ACTUAL_RPM_3" 33 "LAUNCH_ACTUAL_RPM_4" 34 "LAUNCH_ACTUAL_TORQUE_0" 35 "LAUNCH_ACTUAL_TORQUE_1" 36 "LAUNCH_ACTUAL_TORQUE_2" 37 "LAUNCH_ACTUAL_TORQUE_3" 38 "LAUNCH_ACTUAL_TORQUE_4" 39 "REGEN_SOC_GATE_ENABLED" 40 "REGEN_MAX_APPS" 41 "REGEN_RYDER_MU" 240 "TRC_COMMAND" ;',
    'VAL_ 2147483855 SET_VCU_Motor_Direction 0 "REVERSE" 1 "FORWARD" ;',
    'VAL_ 2147483855 SET_VCU_Regen_Enabled 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_ECHO_DAQ 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Ignore_RTD_Switch 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Ignore_RTD_Brakes 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Use_APPS1_Only 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Use_APPS2_Only 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Ignore_APPS_Errs 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Ignore_BSE_Errs 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Ignore_SDC 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Ignore_Brake_Plausibility 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2147483855 SET_VCU_Always_Green 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2147483855 SET_VCU_TC_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2147483855 SET_VCU_Power_Limit_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2147483855 SET_VCU_Regen_Strategy 0 "FRONT_ONLY" 1 "REAR_ONLY" 2 "AVERAGED" 3 "RYDER" ;',
   'VAL_ 2147483855 SET_VCU_Regen_SOC_Gate_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2147483855 SET_VCU_Launch_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2147483855 SET_VCU_Launch_Best_Curve 0 "CURVE_A" 1 "CURVE_B" 2 "CURVE_C" ;',
   'VAL_ 2147483855 SET_VCU_Launch_Active_Curve 0 "CURVE_A" 1 "CURVE_B" 2 "CURVE_C" 3 "UPLOADED" ;',
   'VAL_ 2147483855 SET_VCU_TRC_Command 0 "NONE" 1 "SET_ACTIVE" 2 "SET_OFF" 3 "ENTER_LEARNING" 4 "ARM" 5 "ABORT" 6 "EXIT_LEARNING" 7 "ENTER_LAUNCH" 8 "ARM_LAUNCH" 9 "EXIT_LAUNCH" ;',
   'VAL_ 2148532431 VCU_Config_Mux 0 "MAX_TORQUE" 1 "MOTOR_DIRECTION" 2 "REGEN_ENABLED" 3 "DEBUG_DEFINES" 4 "WHEEL_DIAMETER" 5 "TRACTION_CONTROL_ENABLED" 6 "TRACTION_CONTROL_TARGET_SLIP" 7 "TRACTION_CONTROL_KP" 8 "TRACTION_CONTROL_KI" 9 "TRACTION_CONTROL_KD" 10 "TRACTION_CONTROL_MIN_FRONT_RPM" 11 "POWER_LIMIT_ENABLED" 12 "POWER_CAP_KW" 13 "REGEN_MAX_TORQUE" 14 "REGEN_MIN_TORQUE" 15 "REGEN_MIN_BSE_REAR_PSI" 16 "REGEN_MIN_BSE_FRONT_PSI" 17 "REGEN_MAX_BSE_REAR_PSI" 18 "REGEN_MAX_BSE_FRONT_PSI" 19 "REGEN_MIN_SPEED" 20 "REGEN_MAX_SOC" 21 "REGEN_STRATEGY" 22 "LAUNCH_ENABLED" 23 "LAUNCH_END_RPM" 24 "LAUNCH_TIMEOUT_MS" 25 "LAUNCH_MAX_SLIP" 26 "LAUNCH_BEST_CURVE" 27 "LAUNCH_ACTIVE_CURVE" 28 "LAUNCH_RECOMMENDED_SLIP" 29 "LAUNCH_ACTUAL_RPM_0" 30 "LAUNCH_ACTUAL_RPM_1" 31 "LAUNCH_ACTUAL_RPM_2" 32 "LAUNCH_ACTUAL_RPM_3" 33 "LAUNCH_ACTUAL_RPM_4" 34 "LAUNCH_ACTUAL_TORQUE_0" 35 "LAUNCH_ACTUAL_TORQUE_1" 36 "LAUNCH_ACTUAL_TORQUE_2" 37 "LAUNCH_ACTUAL_TORQUE_3" 38 "LAUNCH_ACTUAL_TORQUE_4" 39 "REGEN_SOC_GATE_ENABLED" 40 "REGEN_MAX_APPS" 41 "REGEN_RYDER_MU" ;',
    'VAL_ 2148532431 VCU_Motor_Direction 0 "REVERSE" 1 "FORWARD" ;',
    'VAL_ 2148532431 VCU_Regen_Enabled 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_ECHO_DAQ 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Ignore_RTD_Switch 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Ignore_RTD_Brakes 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Use_APPS1_Only 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Use_APPS2_Only 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Ignore_APPS_Errs 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Ignore_BSE_Errs 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Ignore_SDC 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Ignore_Brake_Plausibility 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532431 VCU_Always_Green 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2148532431 VCU_TC_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2148532431 VCU_Power_Limit_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2148532431 VCU_Regen_Strategy 0 "FRONT_ONLY" 1 "REAR_ONLY" 2 "AVERAGED" 3 "RYDER" ;',
   'VAL_ 2148532431 VCU_Regen_SOC_Gate_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2148532431 VCU_Launch_Enabled 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2148532431 VCU_Launch_Best_Curve 0 "CURVE_A" 1 "CURVE_B" 2 "CURVE_C" ;',
   'VAL_ 2148532431 VCU_Launch_Active_Curve 0 "CURVE_A" 1 "CURVE_B" 2 "CURVE_C" 3 "UPLOADED" ;',
    'VAL_ 2148532928 VCU_Pump_Toggle_Cmd 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532928 VCU_DRS_Toggle_Cmd 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532928 VCU_Fans_Toggle_Cmd 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2148532928 VCU_Rad_Toggle_Cmd 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366636528 VCU_State 0 "LOADING" 1 "NOT_READY" 2 "PLAYING_RTD_SOUND" 3 "DRIVING" 4 "BAP_FAULT" 5 "HARD_FAULT" ;',
    'VAL_ 2366636528 VCU_RTD_Active 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366636528 VCU_Red_Car 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366636528 VCU_HVC_Summary_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366636528 VCU_HVC_SDC_Ok 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366636528 VCU_HVC_IMD_Ok 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366636528 VCU_HVC_BMS_Ok 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366636528 VCU_Buzzer_State 0 "INACTIVE" 1 "PLAYING" 2 "DONE" ;',
   'VAL_ 2366638016 VCU_TC_Enabled_Status 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366638016 VCU_TC_Active 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366638016 VCU_TC_Front_Left_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366638016 VCU_TC_Front_Right_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Debug_Mux 0 "STATUS" 1 "PRESSURES" 2 "RYDER_MATH" ;',
   'VAL_ 2366637792 VCU_Regen_Strategy_Status 0 "FRONT_ONLY" 1 "REAR_ONLY" 2 "AVERAGED" 3 "RYDER" ;',
   'VAL_ 2366637792 VCU_Regen_SOC_Gate_Status 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Enabled_Status 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Driving_Status 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Speed_OK 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Front_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Rear_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Ryder_Active 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Active 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_SOC_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366637792 VCU_Regen_Block_Reason 0 "NONE" 1 "DISABLED" 2 "NOT_DRIVING" 3 "BELOW_MIN_SPEED" 4 "FRONT_INVALID" 5 "REAR_INVALID" 6 "FRONT_PRESSURE_HIGH" 7 "RYDER_TABLE_ZERO" 8 "RYDER_BALANCE_ZERO" 9 "LEGACY_PRESSURE_INVALID" 10 "ZERO_AFTER_CLAMPS" 11 "SOC_HIGH" 12 "APPS_ACTIVE" ;',
   'VAL_ 2366638272 VCU_TRC_State_Machine 0 "OFF" 1 "ACTIVE" 2 "LAUNCH_IDLE" 3 "LAUNCH" 4 "LEARNING_IDLE" 5 "LEARNING_READY_RUN1" 6 "LEARNING_RUNNING_RUN1" 7 "LEARNING_COMPLETE_RUN1" 8 "LEARNING_READY_RUN2" 9 "LEARNING_RUNNING_RUN2" 10 "LEARNING_COMPLETE_RUN2" 11 "LEARNING_READY_RUN3" 12 "LEARNING_RUNNING_RUN3" 13 "LEARNING_PROCESSING" 14 "LEARNING_RESULTS_READY" 15 "LEARNING_ABORTED" 16 "FAULT" ;',
   'VAL_ 2366638272 VCU_TRC_Selected_Curve 0 "CURVE_A" 1 "CURVE_B" 2 "CURVE_C" 3 "UPLOADED" ;',
   'VAL_ 2366638272 VCU_TRC_Best_Curve 0 "CURVE_A" 1 "CURVE_B" 2 "CURVE_C" ;',
   'VAL_ 2366638272 VCU_TRC_Learning_Active 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366638272 VCU_TRC_Launch_Mode_Active 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366638273 VCU_TRC_Run_Index 1 "RUN1" 2 "RUN2" 3 "RUN3" ;',
   'VAL_ 2366638273 VCU_TRC_Run_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_CAN_Readback_Mux 0 "STATUS" 1 "INVERTER" 2 "VOLTAGE" 3 "FRONT_WHEEL_RPM" ;',
   'VAL_ 2366687998 VCU_RB_INV_HighSpeed_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_HVC_Summary_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_HVC_SOC_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_HVC_VSense_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_MOBO_Power_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_Front_Left_RPM_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_Front_Right_RPM_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_INV_Motor_Position_Valid 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_HVC_SDC_Ok 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_HVC_IMD_Ok 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366687998 VCU_RB_HVC_BMS_Ok 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366638942 VCU_BSE_Stale 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366638942 VCU_BSE_ADC_Err 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366638942 VCU_BSE_Out_of_Range 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366638942 VCU_BSE_Valid 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_Controls_Passive_Err 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_Controls_Bus_Off 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_DAQ_Passive_Err 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_DAQ_Bus_Off 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_Controls_TX_Fault_Seen 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_Controls_RX_Fault_Seen 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_DAQ_TX_Fault_Seen 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_DAQ_RX_Fault_Seen 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366639273 VCU_Controls_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366639273 VCU_DAQ_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366693037 VCU_Dead_HVC_Msg_Valid 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366693037 VCU_Dead_IMD_OK 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366693037 VCU_Dead_BMS_OK 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2366693037 VCU_Dead_SDC_OK 0 "FALSE" 1 "TRUE" ;',
   'VAL_ 2366697968 VCU_FIFO_Mux 0 "TX_CONTROLS_STD" 1 "TX_CONTROLS_EXT" 2 "TX_DAQ_EXT" 3 "RX_INV_MOTOR_POSITION" 4 "RX_INV_HIGH_SPEED" 5 "RX_HVC_SUMMARY" 6 "RX_SET_VCU_CONFIG" ;',
    'VAL_ 2366697968 TX_CONTROLS_STD_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366697968 TX_CONTROLS_EXT_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366697968 TX_DAQ_EXT_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
   'VAL_ 2366697968 RX_INV_MOTOR_POSITION_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366697968 RX_INV_HIGH_SPEED_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366697968 RX_HVC_SUMMARY_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2366697968 RX_SET_VCU_CONFIG_VCU_FIFO_Last_Status 0 "OK" 1 "BUSY" 2 "OLD_DATA" 3 "OVERFLOW" 4 "FIFO_FULL" 5 "INVALID_DATA" 6 "ERROR_PASSIVE" 7 "BUS_OFF" 8 "WRONG_HANDLE" 9 "CHANNEL_NOT_CONFIGURED" 10 "INVALID_PARAMETER" 11 "NULL_POINTER" 12 "INVALID_CHANNEL_ID" 13 "MAX_MO_REACHED" 14 "MAX_HANDLES_REACHED" 15 "UNKNOWN" ;',
    'VAL_ 2367343841 VCU_APPS2_Stale 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS2_ADC_Err 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS2_Out_of_Range 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS2_Valid 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS1_Stale 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS1_ADC_Err 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS1_Out_of_Range 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS1_Valid 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS_Implausible 0 "FALSE" 1 "TRUE" ;',
    'VAL_ 2367343841 VCU_APPS_Valid 0 "FALSE" 1 "TRUE" ;',
]


def generate_dbc() -> str:
    lines = list(HEADER_LINES)

    for block in MESSAGE_BLOCKS:
        lines.extend(block.splitlines())
        lines.append('')

    lines.extend(VALUE_LINES)
    lines.append('')

    return '\n'.join(lines)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    output_path = script_dir.parent / 'VCU.dbc'
    dbc_content = generate_dbc()
    output_path.write_text(dbc_content, encoding='ascii')

    message_count = sum(1 for line in dbc_content.splitlines() if line.startswith('BO_ '))
    value_count = sum(1 for line in dbc_content.splitlines() if line.startswith('VAL_ '))

    print(f'Generated {output_path}')
    print(f'Messages: {message_count}')
    print(f'Value tables: {value_count}')


if __name__ == '__main__':
    main()
