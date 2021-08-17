///////////////////////////////////////////////////////////////////////////////
//
// NTC package include file
// Part number: NXRT15XH103FA1B030, assuming a 10kOhm pull up resistor.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NTC_H_
#define NTC_H_

#define NTC_INVALID_TEMPERATURE     0x7FFF


extern int16_t ntc_temperature_x100_get(uint16_t raw_value);

#endif
