///////////////////////////////////////////////////////////////////////////////
//
// Serial debug protocol package, data link layer
//
///////////////////////////////////////////////////////////////////////////////

#include "stdint.h"
#include "stdbool.h"
#include "sdp.h"
#include "sdp_package_scope.h"
#include "driverlib.h"
#include "util.h"


// TODO: How long this timeout should be?
#define SDP_COMMS_LOST_TIMEOUT_uS   (10L * SECONDS_IN_uS)


// Do not reorder these states! The code depends on it
enum sdp_rx_states
{
    SDP_WAITING_FOR_SOF,
    SDP_WAITING_FOR_RX_BYTE_1,
    SDP_WAITING_FOR_RX_BYTE_2,
    SDP_WAITING_FOR_RX_BYTE_3,
    SDP_WAITING_FOR_RX_BYTE_4,
    SDP_WAITING_FOR_RX_BYTE_5,
    SDP_WAITING_FOR_RX_BYTE_6,
    SDP_WAITING_FOR_RX_BYTE_7,
    SDP_WAITING_FOR_RX_BYTE_8,
};

enum sdp_tx_states
{
    SDP_WAITING_FOR_NEW_FRAME_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_SOF_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_1_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_2_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_3_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_4_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_5_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_6_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_7_TO_BE_TRANSMITTED,
    SDP_WAITING_FOR_BYTE_8_TO_BE_TRANSMITTED,
};


#define SDP_SOF_MASK    0x80U
#define SDP_SOF_BYTE    0x80U


static enum sdp_rx_states sdp_rx_state;
static enum sdp_tx_states sdp_tx_state;

static int32_t sd_comms_lost_timeout;
static bool sdp_is_comms_lost;


// SDP package variable
uint16_t sdp_data[4];


//
//
//
void sdp_data_link_init(void)
{
    sdp_rx_state = SDP_WAITING_FOR_SOF;
    sdp_tx_state = SDP_WAITING_FOR_NEW_FRAME_TO_BE_TRANSMITTED;
    sdp_is_comms_lost = false;
    sd_comms_lost_timeout =
                    CPUTimer_getTimerCount(CPUTIMER0_BASE) + SDP_COMMS_LOST_TIMEOUT_uS;
}

//
//
//
bool sdp_is_ready_to_transmit(void)
{
    return (sdp_tx_state == SDP_WAITING_FOR_NEW_FRAME_TO_BE_TRANSMITTED);
}

//
//
//
void sdp_transmission_start(void)
{
    sdp_tx_state = SDP_WAITING_FOR_SOF_TO_BE_TRANSMITTED;
}

//
// Stores a received character.
//
void sdp_receive_put(uint16_t c)
{
    if ((c & SDP_SOF_MASK) == SDP_SOF_BYTE)
    {
        sdp_data[0] = ((c & 0x7FU) << 8U);
        sdp_rx_state = SDP_WAITING_FOR_RX_BYTE_1;
        return;
    }

    switch (sdp_rx_state)
    {
        case SDP_WAITING_FOR_RX_BYTE_1:
            sdp_data[0] |= ((c & 0x7FU) << 1U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_2:
            sdp_data[0] |= ((c & 0x40U) >> 6U);
            sdp_data[1]  = ((c & 0x3FU) << 10U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_3:
            sdp_data[1] |= ((c & 0x7FU) << 3U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_4:
            sdp_data[1] |= ((c & 0x70U) >> 4U);
            sdp_data[2]  = ((c & 0x0FU) << 12U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_5:
            sdp_data[2] |= ((c & 0x7FU) << 5U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_6:
            sdp_data[2] |= ((c & 0x7CU) >> 2U);
            sdp_data[3]  = ((c & 0x03U) << 14U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_7:
            sdp_data[3] |= ((c & 0x7FU) << 7U);
            sdp_rx_state++;
            break;
        case SDP_WAITING_FOR_RX_BYTE_8:
            sdp_data[3] |= (c & 0x7FU);
            sdp_data_process(); // We finished reception.
            // We just received a frame, so we are not in comms lost.
            sdp_is_comms_lost = false;
            sd_comms_lost_timeout =
                    CPUTimer_getTimerCount(CPUTIMER0_BASE) + SDP_COMMS_LOST_TIMEOUT_uS;
            sdp_rx_state = SDP_WAITING_FOR_SOF;
            break;
        default:
            // Do nothing.
            break;
    }
}

//
// Checks if there is any character to be transmitted.
// Multiple calls to this function without calling
// calling sdp_transmit_done() will return the same value.
// Returns SDP_NO_CHARACTER if there is no character to transmit.
//
int16_t sdp_transmit_peek(void)
{
    int32_t cpu_time_us = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    if (IS_CPU_TIME_AFTER(cpu_time_us, sd_comms_lost_timeout))
    {
        sdp_is_comms_lost = true;
        sd_comms_lost_timeout =
                CPUTimer_getTimerCount(CPUTIMER0_BASE) + SDP_COMMS_LOST_TIMEOUT_uS;
    }

    int16_t c;

    switch (sdp_tx_state)
    {
        case SDP_WAITING_FOR_SOF_TO_BE_TRANSMITTED:
            c = ((sdp_data[0] >> 8U) & 0x7FU) | SDP_SOF_MASK;
            break;
        case SDP_WAITING_FOR_BYTE_1_TO_BE_TRANSMITTED:
            c = ((sdp_data[0] >> 1U) & 0x7FU);
            break;
        case SDP_WAITING_FOR_BYTE_2_TO_BE_TRANSMITTED:
            c  = ((sdp_data[0] & 0x01U) << 6U);
            c |= ((sdp_data[1] >> 10U) & 0x3FU);
            break;
        case SDP_WAITING_FOR_BYTE_3_TO_BE_TRANSMITTED:
            c = ((sdp_data[1] >> 3U) & 0x7FU);
            break;
        case SDP_WAITING_FOR_BYTE_4_TO_BE_TRANSMITTED:
            c  = ((sdp_data[1] & 0x07U) << 4U);
            c |= ((sdp_data[2] >> 12U) & 0x0FU);
            break;
        case SDP_WAITING_FOR_BYTE_5_TO_BE_TRANSMITTED:
            c = ((sdp_data[2] >> 5U) & 0x7FU);
            break;
        case SDP_WAITING_FOR_BYTE_6_TO_BE_TRANSMITTED:
            c  = ((sdp_data[2] & 0x1FU) << 2U);
            c |= ((sdp_data[3] >> 14U) & 0x03U);
            break;
        case SDP_WAITING_FOR_BYTE_7_TO_BE_TRANSMITTED:
            c = ((sdp_data[3] >> 7U) & 0x7FU);
            break;
        case SDP_WAITING_FOR_BYTE_8_TO_BE_TRANSMITTED:
            c = (sdp_data[3] & 0x7FU);
            break;
        default:
            c = SDP_NO_CHARACTER; // Nothing to send
            break;
    }

    return c;
}

//
// This function advances the transmission state. It should only
// be called after peeking a character to be transmitted, and
// after a successful transmission.
//
void sdp_transmit_done(void)
{
    if (sdp_tx_state >= SDP_WAITING_FOR_BYTE_8_TO_BE_TRANSMITTED) // done transmitting frame
        sdp_tx_state = SDP_WAITING_FOR_NEW_FRAME_TO_BE_TRANSMITTED;
    else if (sdp_tx_state >= SDP_WAITING_FOR_SOF_TO_BE_TRANSMITTED)
        sdp_tx_state++;
}

//
// Indicates whether comms are lost or not.
//
bool sdp_is_comms_lost_get(void)
{
    return sdp_is_comms_lost;
}
