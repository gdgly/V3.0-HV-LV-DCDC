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
#include "driverlib.h"
#include "util.h"


// TODO: How long this timeout should be?
#define SDP_TRANSFER_TIMEOUT_uS     (50L * MILLISECONDS_IN_uS)


static const struct sdp_store *sdp_store_array_p;
static uint16_t sdp_store_array_size;
static const struct sdp_fetch *sdp_fetch_array_p;
static uint16_t sdp_fetch_array_size;

static uint16_t sdp_id_last;

static uint16_t sdp_transmit_error_count;
static uint16_t sdp_receive_error_count;
static enum sdp_error_codes sdp_error_code_last;

static bool sdp_transfer_in_progress;
static int32_t sdp_transfer_timeout_us;


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
static void sdp_store_execute(uint16_t id)
{
    uint32_t value = ((uint32_t)sdp_data[1] << 16) | sdp_data[2];

    uint16_t i;
    for (i = 0; i < sdp_store_array_size; ++i)
    {
        if (sdp_store_array_p[i].id == id)
        {
            bool success = sdp_store_array_p[i].store_fn(value);
            return;
        }
    }
}

//
//
//
static void sdp_transfer_timeout_service(void)
{
    int32_t cpu_time_us = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    if (IS_CPU_TIME_AFTER(cpu_time_us, sdp_transfer_timeout_us))
    {
        sdp_transfer_in_progress = false;
        sdp_transfer_timeout_us = cpu_time_us + SDP_TRANSFER_TIMEOUT_uS;
    }
}

//
//
//
void sdp_data_process(void)
{
    // Response received, transfer no longer in progress.
    sdp_transfer_in_progress = false;

    uint16_t id = sdp_data[0] & SDP_ID_MASK;
    bool is_read_command_response = sdp_data[0] & SDP_RW_MASK;

    uint16_t received_crc = sdp_data[3];
    uint16_t calculated_crc = crc_16ccitt_calculate(sdp_data, 3U, 0U);

    if ((received_crc != calculated_crc)
            || (id != sdp_id_last))
    {
        sdp_receive_error_count++;
    }
    else
    {
        bool error_flag = sdp_data[0] & SDP_ERROR_MASK;
        if (error_flag)
        {
            sdp_transmit_error_count++;
            uint32_t value = ((uint32_t)sdp_data[1] << 16) | sdp_data[2];
            sdp_error_code_last = (enum sdp_error_codes)value;
        }
        else
        {
            if (is_read_command_response)
            {
                sdp_store_execute(id);
            }
            else // write command response
            {
                // Do nothing. If we got to this point, it means we had a successful acknowledgment.
            }
        }
    }
}


//
//
//
void sdp_master_init(const struct sdp_store *store_array_p, uint16_t store_array_size,
                     const struct sdp_fetch *fetch_array_p, uint16_t fetch_array_size)
{
    sdp_store_array_p = store_array_p;
    sdp_store_array_size = store_array_size;
    sdp_fetch_array_p = fetch_array_p;
    sdp_fetch_array_size = fetch_array_size;

    sdp_transmit_error_count = 0U;
    sdp_receive_error_count = 0U;

    sdp_error_code_last = SDP_NO_ERROR;

    sdp_transfer_in_progress = false;

    sdp_data_link_init();
}

//
//
//
bool sdp_master_read_transmit(uint16_t id)
{
    sdp_transfer_timeout_service();
    if ((!sdp_is_ready_to_transmit())
            || sdp_transfer_in_progress)
        return false;

    uint16_t i;
    for (i = 0U; i < sdp_store_array_size; ++i)
    {
        if (sdp_store_array_p[i].id == id)
        {
            sdp_id_last = id;
            sdp_value_send(false, true, id, 0UL);
            sdp_transfer_in_progress = true;
            sdp_transfer_timeout_us =
                    CPUTimer_getTimerCount(CPUTIMER0_BASE) + SDP_TRANSFER_TIMEOUT_uS;
            return true;
        }
    }

    return false;
}

//
//
//
bool sdp_master_write_transmit(uint16_t id)
{
    sdp_transfer_timeout_service();
    if ((!sdp_is_ready_to_transmit())
            || sdp_transfer_in_progress)
        return false;

    uint16_t i;
    for (i = 0U; i < sdp_fetch_array_size; ++i)
    {
        if (sdp_fetch_array_p[i].id == id)
        {
            sdp_id_last = id;
            uint32_t value = sdp_fetch_array_p[i].fetch_fn();
            sdp_value_send(false, false, id, value);
            sdp_transfer_in_progress = true;
            sdp_transfer_timeout_us =
                    CPUTimer_getTimerCount(CPUTIMER0_BASE) + SDP_TRANSFER_TIMEOUT_uS;
            return true;
        }
    }

    return false;
}

//
//
//
enum sdp_error_codes sdp_master_error_code_last_get(void)
{
    return sdp_error_code_last;
}

uint16_t sdp_master_transmit_error_count_get(void)
{
    return sdp_transmit_error_count;
}

uint16_t sdp_master_receive_error_count_get(void)
{
    return sdp_receive_error_count;
}
