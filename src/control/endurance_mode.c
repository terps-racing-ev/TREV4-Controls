#include "IO_Constants.h"

#include "endurance_mode.h"

#include "config/endurance_config.h"

/* kW-vs-SoC derate curve, ordered by DESCENDING SoC. Outside this range the
 * cap is flat-clamped: >= the first breakpoint holds ENDURANCE_POWER_CAP_HIGH_KW,
 * <= the last breakpoint holds that last entry's value. Between breakpoints the
 * cap is linearly interpolated. Edit these points to retune the curve. */
static const EnduranceSocCapPoint_t endurance_soc_cap_table[] = {
    { 70, 38 },
    { 30, 25 },
};

#define ENDURANCE_SOC_CAP_POINTS \
    ((ubyte2)(sizeof(endurance_soc_cap_table) / sizeof(endurance_soc_cap_table[0])))

ubyte2 EnduranceMode_GetPowerCapKw(const ubyte2 soc_percent_x100, const bool soc_valid)
{
    /* Fail safe to the lowest cap if we cannot trust the SoC reading. */
    if (!soc_valid) {
        return (ubyte2)ENDURANCE_POWER_CAP_LOW_KW;
    }

    const ubyte4 soc_x100 = (ubyte4)soc_percent_x100;

    /* Flat clamp above the highest breakpoint. */
    const ubyte4 top_soc_x100 = (ubyte4)endurance_soc_cap_table[0].soc_percent * 100UL;
    if (soc_x100 >= top_soc_x100) {
        return (ubyte2)ENDURANCE_POWER_CAP_HIGH_KW;
    }

    /* Flat clamp at/below the lowest breakpoint. */
    const ubyte2 last = (ubyte2)(ENDURANCE_SOC_CAP_POINTS - 1U);
    const ubyte4 bottom_soc_x100 = (ubyte4)endurance_soc_cap_table[last].soc_percent * 100UL;
    if (soc_x100 <= bottom_soc_x100) {
        return endurance_soc_cap_table[last].power_cap_kw;
    }

    /* Find the bracketing pair (table[i] = higher SoC, table[i+1] = lower SoC)
     * and linearly interpolate in x100 units for smoothness. */
    for (ubyte2 i = 0; i < last; i++) {
        const ubyte4 hi_soc_x100 = (ubyte4)endurance_soc_cap_table[i].soc_percent * 100UL;
        const ubyte4 lo_soc_x100 = (ubyte4)endurance_soc_cap_table[i + 1U].soc_percent * 100UL;

        if ((soc_x100 <= hi_soc_x100) && (soc_x100 >= lo_soc_x100)) {
            const ubyte4 hi_cap = (ubyte4)endurance_soc_cap_table[i].power_cap_kw;
            const ubyte4 lo_cap = (ubyte4)endurance_soc_cap_table[i + 1U].power_cap_kw;
            const ubyte4 span = hi_soc_x100 - lo_soc_x100; /* > 0: descending, distinct */

            /* cap increases with SoC, so (hi_cap - lo_cap) >= 0; round to nearest. */
            const ubyte4 cap = lo_cap +
                (((soc_x100 - lo_soc_x100) * (hi_cap - lo_cap)) + (span / 2UL)) / span;

            return (ubyte2)cap;
        }
    }

    /* Unreachable for a well-formed descending table; fail safe. */
    return (ubyte2)ENDURANCE_POWER_CAP_LOW_KW;
}
