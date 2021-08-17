///////////////////////////////////////////////////////////////////////////////
//
// CRC package
//
///////////////////////////////////////////////////////////////////////////////

#include "stdint.h"
#include "crc.h"

#define CRC_CCITT_POLYNOMIAL	0x1021

//
//
//
static uint16_t crc_ccitt_inner_loop_calculate(uint16_t crc)
{
    uint16_t bit;
    for (bit = 0U; bit < 8U; bit++)
    {
        if (crc & (1U << 15U))
            crc = (crc << 1U) ^ CRC_CCITT_POLYNOMIAL;
        else
            crc = (crc << 1U);
    }

    return crc;
}

//
//
//
uint16_t crc_16ccitt_calculate(const uint16_t *buffer,
                               uint16_t buffer_size_in_words,
                               uint16_t previous_crc)
{
    uint16_t i;
    uint16_t crc = previous_crc;

    for (i = 0U; i < buffer_size_in_words; i++)
    {
        uint16_t high_byte = buffer[i] & 0xFFU; // Little endian flash memory structure on the F280049
        uint16_t low_byte = buffer[i] >> 8U;

        crc = crc ^ (high_byte << 8U);
        crc = crc_ccitt_inner_loop_calculate(crc);

        crc = crc ^ (low_byte << 8U);
        crc = crc_ccitt_inner_loop_calculate(crc);
    }

    return (crc & 0xFFFFU);
}
