///////////////////////////////////////////////////////////////////////////////
//
// NTC package
// Part number: NXRT15XH103FA1B030, assuming a 10kOhm pull up resistor.
//
///////////////////////////////////////////////////////////////////////////////

#include "stdint.h"
#include "stdbool.h"
#include "ntc.h"


static const int16_t ntc_temperature_conversion_lut[][2] =
{
    {4088, -25 * 100},
    {3974, -20 * 100},
    {3841, -15 * 100},
    {3687, -10 * 100},
    {3515,  -5 * 100},
    {3326,   0 * 100},
    {3122,   5 * 100},
    {2908,  10 * 100},
    {2689,  15 * 100},
    {2469,  20 * 100},
    {2252,  25 * 100},
    {2042,  30 * 100},
    {1842,  35 * 100},
    {1655,  40 * 100},
    {1482,  45 * 100},
    {1323,  50 * 100},
    {1179,  55 * 100},
    {1049,  60 * 100},
    {933,   65 * 100},
    {830,   70 * 100},
    {739,   75 * 100},
    {658,   80 * 100},
    {586,   85 * 100},
    {523,   90 * 100},
    {467,   95 * 100},
    {418,  100 * 100},
    {375,  105 * 100},
    {337,  110 * 100},
    {303,  115 * 100},
    {273,  120 * 100},
    {246,  125 * 100},
    {223,  130 * 100},
    {202,  135 * 100},
    {184,  140 * 100},
};

//
// Returns NTC_INVALID_TEMPERATURE if raw_value is outside
// expected range.
//
int16_t ntc_temperature_x100_get(uint16_t raw_value)
{
    int16_t temperature_x100 = NTC_INVALID_TEMPERATURE;

    // If NTC is shorted, temperature is invalid
    if (raw_value <= ntc_temperature_conversion_lut[(sizeof(ntc_temperature_conversion_lut) / 2) - 1][0])
        return NTC_INVALID_TEMPERATURE;

    if (raw_value > ntc_temperature_conversion_lut[0][0])
        return ntc_temperature_conversion_lut[0][1];

    uint16_t i;
    for (i = 0U; i < (sizeof(ntc_temperature_conversion_lut) / 2); ++i)
    {
        if (raw_value > ntc_temperature_conversion_lut[i][0])
        {
            // Do a linear interpolation
            int32_t delta_y = (ntc_temperature_conversion_lut[i][1]
                    - ntc_temperature_conversion_lut[i - 1][1]);
            int32_t delta_x = (ntc_temperature_conversion_lut[i][0]
                    - ntc_temperature_conversion_lut[i - 1][0]);

            temperature_x100 = ntc_temperature_conversion_lut[i - 1][1]
                    + ((delta_y * ((int32_t)raw_value - ntc_temperature_conversion_lut[i - 1][0]))
                            / delta_x);
            break;
        }
    }

    return temperature_x100;
}
