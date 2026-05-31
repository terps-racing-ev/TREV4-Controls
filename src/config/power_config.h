#ifndef POWER_CONFIG_H
#define POWER_CONFIG_H

#include "IO_Constants.h"

/* Vehicle power capping (discharge/charge current limit to inverter).

   The discharge current limit (DCL) is derived from a configurable power cap
   and the pack voltage reported by the HVC (IO_VSense / Inv_Voltage_mV):

       dcl_amps = (power_cap_kW * 1,000,000) / inv_voltage_mv

   The result is clamped to a minimum floor so a low/invalid voltage reading
   cannot command an unreasonably small (or zero) limit.
*/

/* Runtime-configurable power cap (integer kW), EEPROM-backed. */
#define POWER_CAP_KW_DEFAULT            80
#define POWER_CAP_KW_MIN               5
#define POWER_CAP_KW_MAX               100

/* Runtime-configurable enable flag (default on). */
#define POWER_LIMIT_ENABLED_DEFAULT    TRUE

/* Fixed current limits (Amps). */
#define MIN_DCL_AMPS                   20
#define CCL_AMPS                       20

#endif // POWER_CONFIG_H
