#ifndef ENDURANCE_CONFIG_H
#define ENDURANCE_CONFIG_H

#include "IO_Constants.h"

/****************************************************************************
 * Drive modes
 *
 * The VCU supports selectable driving modes, chosen at runtime over CAN via
 * RUNTIME_PARAM_DRIVE_MODE (EEPROM-backed). NORMAL preserves the historical
 * behavior (pedal mapped to max_torque, fixed power cap). ENDURANCE trades
 * peak performance for range/consistency: a fixed lower torque cap plus a
 * State-of-Charge-dependent power cap derate (see the lookup table below).
 *
 * APPEND-ONLY: new modes must be added at the end and the RUNTIME_PARAM_DRIVE_MODE
 * max_value in runtime_config.c widened to match.
 ****************************************************************************/
typedef enum {
    DRIVE_MODE_NORMAL    = 0,
    DRIVE_MODE_ENDURANCE = 1,
} DriveMode_t;

/* Boot default. Endurance by default; flip RUNTIME_PARAM_DRIVE_MODE to
 * DRIVE_MODE_NORMAL over CAN to restore the standard tune. */
#define DRIVE_MODE_DEFAULT              DRIVE_MODE_ENDURANCE
#define DRIVE_MODE_MAX                  DRIVE_MODE_ENDURANCE

/****************************************************************************
 * Endurance mode tuning
 *
 * Torque is capped to a fixed, lower value. The power cap follows a simple
 * kW-vs-SoC lookup table (endurance_mode.c) with linear interpolation between
 * breakpoints and a flat clamp outside the table:
 *
 *   SoC >= top breakpoint (60%) ............. ENDURANCE_POWER_CAP_HIGH_KW (40)
 *   between breakpoints ..................... linear interpolation
 *   SoC <= bottom breakpoint (30%) .......... ENDURANCE_POWER_CAP_LOW_KW  (19)
 *   SoC invalid / unavailable ............... ENDURANCE_POWER_CAP_LOW_KW  (fail safe)
 ****************************************************************************/

/* Fixed torque cap applied in Endurance (Nm). Bounded by the normal max_torque
 * (used as a min(), so Endurance can never command more than Normal). */
#define ENDURANCE_TORQUE_CAP_NM        120

/* Flat plateaus outside the lookup-table SoC range (kW). */
#define ENDURANCE_POWER_CAP_HIGH_KW    38
#define ENDURANCE_POWER_CAP_LOW_KW     25

/* One (SoC %, power cap kW) breakpoint. SoC is whole-percent; the live HVC
 * SoC (pack_soc_percent_x100) is divided by 100 before lookup. The table in
 * endurance_mode.c is ordered by DESCENDING SoC. */
typedef struct {
    ubyte2 soc_percent;
    ubyte2 power_cap_kw;
} EnduranceSocCapPoint_t;

#endif // ENDURANCE_CONFIG_H
