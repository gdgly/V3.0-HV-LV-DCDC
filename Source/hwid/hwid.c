///////////////////////////////////////////////////////////////////////////////
//
// HW ID package
// Assuming a 10kOhm pull up resistor.
//
///////////////////////////////////////////////////////////////////////////////

#include "stdint.h"
#include "stdbool.h"
#include "hwid/hwid.h"


static const int16_t hwid_conversion_lut[HWID_MAX + 1] =
{
      30U,  // HW ID = HWID_INVALID
     232U,  // HW ID = 1
     434U,
     636U,
     838U,
    1040U,
    1242U,
    1444U,
    1646U,
    1848U,
    2050U,
    2252U,
    2454U,
    2656U,
    2858U,
    3060U,
    3262U,
    3464U,
    3666U,
    3868U,
    4070U,  // HW ID = HWID_MAX
};

//
// Returns HWID_INVALID if raw_value is outside
// expected range.
//
int16_t hwid_get(uint16_t raw_value)
{
	int16_t hw_id = HWID_INVALID;

    // If input is shorted or open, HW ID is invalid
    if ((raw_value <= hwid_conversion_lut[0])
            || (raw_value > hwid_conversion_lut[sizeof(hwid_conversion_lut) - 1U]))
        return HWID_INVALID;


    uint16_t i;
    for (i = 1U; i < sizeof(hwid_conversion_lut); ++i)
    {
        if (raw_value <= hwid_conversion_lut[i])
        {
            hw_id = i;
            break;
        }
    }

    return hw_id;
}
