///////////////////////////////////////////////////////////////////////////////
//
// Slot/Shelf ID package include file
// Assuming a 10kOhm pull up resistor.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SLOT_SHELF_ID_H_
#define SLOT_SHELF_ID_H_


#define SLOT_SHELF_ID_INVALID   -1
#define SLOT_SHELF_ID_MAX       10

extern int16_t slot_shelf_id_get(uint16_t raw_value);

#endif
