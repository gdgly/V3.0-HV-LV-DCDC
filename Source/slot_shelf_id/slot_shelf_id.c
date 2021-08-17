///////////////////////////////////////////////////////////////////////////////
//
// Slot/Shelf ID package
// Assuming a 10kOhm pull up resistor.
//
///////////////////////////////////////////////////////////////////////////////

#include "stdint.h"
#include "stdbool.h"
#include "slot_shelf_id/slot_shelf_id.h"


static const int16_t slot_shelf_id_conversion_lut[SLOT_SHELF_ID_MAX + 1] =
{
      20U,  // Slot/Shelf ID = SLOT_SHELF_ID_INVALID
     425U,  // Slot/Shelf ID = 1
     830U,
    1235U,
    1640U,
    2045U,
    2450U,
    2855U,
    3260U,
    3665U,
    4070U,  // Slot/Shelf ID = SLOT_SHELF_ID_MAX
};

//
// Returns SLOT_SHELF_ID_INVALID if raw_value is outside
// expected range.
//
int16_t slot_shelf_id_get(uint16_t raw_value)
{
	int16_t slot_shelf_id = SLOT_SHELF_ID_INVALID;

    // If input is shorted or open, Slot/Shelf ID is invalid
    if ((raw_value <= slot_shelf_id_conversion_lut[0])
            || (raw_value > slot_shelf_id_conversion_lut[sizeof(slot_shelf_id_conversion_lut) - 1U]))
        return SLOT_SHELF_ID_INVALID;


    uint16_t i;
    for (i = 1U; i < sizeof(slot_shelf_id_conversion_lut); ++i)
    {
        if (raw_value <= slot_shelf_id_conversion_lut[i])
        {
            slot_shelf_id = i;
            break;
        }
    }

    return slot_shelf_id;
}
