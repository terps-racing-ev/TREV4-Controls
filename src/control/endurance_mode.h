#ifndef ENDURANCE_MODE_H
#define ENDURANCE_MODE_H

#include "IO_Constants.h"

/*
 * Endurance-mode power cap lookup.
 *
 * Returns the Endurance power cap (kW) for the given pack State of Charge.
 *
 *   soc_percent_x100 : HVC SoC scaled x100 (0..10000 == 0%..100%), as stored in
 *                      HVCSOC_RX_Data_t.pack_soc_percent_x100.
 *   soc_valid        : pass FALSE when the SoC sample is stale/unavailable; the
 *                      function then fails safe to the lowest cap.
 *
 * The mapping is a flat-clamped, linearly interpolated table (see
 * config/endurance_config.h). Pure function, no internal state.
 */
ubyte2 EnduranceMode_GetPowerCapKw(const ubyte2 soc_percent_x100, const bool soc_valid);

#endif // ENDURANCE_MODE_H
