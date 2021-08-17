///////////////////////////////////////////////////////////////////////////////
//
// Serial debug protocol package, master device
//
///////////////////////////////////////////////////////////////////////////////

#include "stdint.h"
#include "stdbool.h"
#include "sdp.h"
#include "sdp_package_scope.h"
#include "crc/crc.h"


static const struct sdp_get *sdp_get_array_p;
static uint16_t sdp_get_array_size;
static const struct sdp_set *sdp_set_array_p;
static uint16_t sdp_set_array_size;


//
//
//
static void sdp_value_send(bool error_flag, bool is_read_command, uint16_t id, uint32_t value)
{
    sdp_data[0] = id
            | (is_read_command ? (1U << 13U) : 0U)
            | (error_flag ? (1U << 14U) : 0U);
    sdp_data[1] = value >> 16U;
    sdp_data[2] = (uint16_t)value;
    sdp_data[3] = crc_16ccitt_calculate(sdp_data, 3U, 0U);

    sdp_transmission_start();
}

//
//
//
static void sdp_get_execute(uint16_t id)
{
    uint32_t value;

    uint16_t i;
    for (i = 0U; i < sdp_get_array_size; ++i)
    {
        if (sdp_get_array_p[i].id == id)
        {
            value = sdp_get_array_p[i].get_fn();
            sdp_value_send(false, true, id, value);
            return;
        }
    }

    sdp_value_send(true, true, id, SDP_INEXISTENT_ID);
}

//
//
//
static void sdp_set_execute(uint16_t id)
{
    uint32_t value = ((uint32_t)sdp_data[1] << 16) | sdp_data[2];

    uint16_t i;
    for (i = 0U; i < sdp_set_array_size; ++i)
    {
        if (sdp_set_array_p[i].id == id)
        {
            bool success = sdp_set_array_p[i].set_fn(value);

            if (success)
                sdp_value_send(false, false, id, SDP_NO_ERROR);
            else
                sdp_value_send(true, false, id, SDP_VALUE_OUT_OF_RANGE);
            return;
        }
    }

    sdp_value_send(true, false, id, SDP_INEXISTENT_ID);
}

//
//
//
void sdp_data_process(void)
{
    uint16_t id = sdp_data[0] & SDP_ID_MASK;
    bool is_read_command = sdp_data[0] & SDP_RW_MASK;

    uint16_t received_crc = sdp_data[3];
    uint16_t calculated_crc = crc_16ccitt_calculate(sdp_data, 3U, 0U);

    if (received_crc != calculated_crc)
    {
        sdp_value_send(true, is_read_command, id, SDP_CRC_ERROR);
    }
    else
    {
        if (is_read_command)
        {
            sdp_get_execute(id);
        }
        else // write command
        {
            sdp_set_execute(id);
        }
    }
}


//
//
//
void sdp_slave_init(const struct sdp_get *get_array_p, uint16_t get_array_size,
              const struct sdp_set *set_array_p, uint16_t set_array_size)
{
    sdp_get_array_p = get_array_p;
    sdp_get_array_size = get_array_size;
    sdp_set_array_p = set_array_p;
    sdp_set_array_size = set_array_size;

    sdp_data_link_init();
}
