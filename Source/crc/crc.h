///////////////////////////////////////////////////////////////////////////////
//
// CRC package include file
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRC_H_
#define CRC_H_

extern uint16_t crc_16ccitt_calculate(const uint16_t *buffer,
                                      uint16_t buffer_size_in_words,
                                      uint16_t previous_crc);

#endif
