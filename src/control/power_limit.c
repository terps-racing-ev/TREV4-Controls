#include "IO_Constants.h"

#include "power_limit.h"

#include "can/can_manager.h"
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "config/power_config.h"
#include "config/endurance_config.h"
#include "endurance_mode.h"

static PowerLimit_Data_t power_limit_data;

static sbyte2 GetParam(const RuntimeParamId_t param_id)
{
    sbyte2 value = 0;
    (void)RuntimeConfig_GetI32(param_id, &value);
    return value;
}

static bool IsEnabled(void)
{
    return (GetParam(RUNTIME_PARAM_POWER_LIMIT_ENABLED) != 0);
}

/* Active power cap (kW). Normal uses the fixed runtime-config cap; Endurance
 * derives it from pack SoC via the endurance derate curve (fail-safe to the
 * lowest cap when SoC is invalid). */
ubyte2 PowerLimit_GetActivePowerCapKw(void)
{
    if (RuntimeConfig_GetDriveMode() == DRIVE_MODE_ENDURANCE) {
        const HVCSOC_RX_Data_t* const soc = CAN_RX_GetHVCSOCData();
        const bool soc_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_SOC);
        return EnduranceMode_GetPowerCapKw(soc->pack_soc_percent_x100, soc_valid);
    }

    return (ubyte2)GetParam(RUNTIME_PARAM_POWER_CAP_KW);
}

void PowerLimit_Init(void)
{
    power_limit_data = (PowerLimit_Data_t){0};
    power_limit_data.dcl_amps = (ubyte2)MIN_DCL_AMPS;
    power_limit_data.ccl_amps = (ubyte2)CCL_AMPS;
}

void PowerLimit_Update(void)
{
    const HVCVSense_RX_Data_t* const vsense = CAN_RX_GetHVCVSenseData();
    const bool voltage_valid = CAN_Manager_RX_Data_Valid(CAN_RX_MSG_HVC_VSENSE);

    const ubyte4 voltage_mv = vsense->inv_voltage_mv;
    const ubyte4 power_cap_kw = (ubyte4)PowerLimit_GetActivePowerCapKw();

    ubyte4 dcl_amps = (ubyte4)MIN_DCL_AMPS;

    /* Only derive a limit from voltage once a valid, non-zero sample exists.
       Otherwise fall back to the floor so we never command a tiny limit. */
    if (voltage_valid && (voltage_mv > 0)) {
        /* dcl_amps = (power_cap_kW * 1,000,000) / inv_voltage_mv */
        dcl_amps = (power_cap_kw * 1000000UL) / voltage_mv;

        if (dcl_amps < (ubyte4)MIN_DCL_AMPS) {
            dcl_amps = (ubyte4)MIN_DCL_AMPS;
        }
    }

    dcl_amps = (ubyte4)(((float4)dcl_amps * AMPS_MULTIPLIER) + 0.5f); // Apply multipler because fuck the inverter

    if (dcl_amps > 0xFFFFUL) {
        dcl_amps = 0xFFFFUL;
    }

    power_limit_data.enabled = IsEnabled();
    power_limit_data.voltage_mv = voltage_mv;
    power_limit_data.dcl_amps = (ubyte2)dcl_amps;
    power_limit_data.ccl_amps = (ubyte2)CCL_AMPS;
}

const PowerLimit_Data_t* PowerLimit_GetData(void)
{
    return &power_limit_data;
}

bool PowerLimit_TxTrigger(void)
{
    return IsEnabled();
}
