///////////////////////////////////////////////////////////////////////////////
//
// SDP file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "dcdc.h"
#include "sdp/sdp.h"
#include "sci.h"
#include "version.h"

static uint16_t sdp_store_array_index;
static uint16_t sdp_fetch_array_index;
static bool sdp_send_read_commnad_next;


// TODO: volatile to avoid error (variable not used)
volatile static uint32_t sdp_fw_build;


// Function prototypes
static bool sdp_fw_build_store(uint32_t value);

static uint32_t sdp_mock_fetch(void);



static const struct sdp_store sdp_store_array[] =
{
    {0U,  &sdp_fw_build_store},
};

static const struct sdp_fetch sdp_fetch_array[] =
{
    {1U,  &sdp_mock_fetch},
};



//
//
//
static bool sdp_fw_build_store(uint32_t value)
{
    sdp_fw_build = value;
    return true;
}

//
//
//
static uint32_t sdp_mock_fetch(void)
{
    return 0x12345678UL;
}

//
//
//
static void dcdc_sdp_message_schedule(void)
{
    bool success;
    // TODO: Decide how to schedule messages. Maybe only one array with all the read and write commands?
    if (sdp_send_read_commnad_next)
    {
        success = sdp_master_read_transmit(sdp_store_array[sdp_store_array_index].id);

        if (success)
        {
            sdp_store_array_index++;
            if (sdp_store_array_index
                    >= (sizeof(sdp_store_array) / sizeof(sdp_store_array[0])))
                sdp_store_array_index = 0U;

            sdp_send_read_commnad_next = false;
        }
    }
    else
    {
        success = sdp_master_write_transmit(sdp_fetch_array[sdp_fetch_array_index].id);

        if (success)
        {
            sdp_fetch_array_index++;
            if (sdp_fetch_array_index
                    >= (sizeof(sdp_fetch_array) / sizeof(sdp_fetch_array[0])))
                sdp_fetch_array_index = 0U;

            sdp_send_read_commnad_next = true;
        }
    }
}



//
//
//
void dcdc_sdp_init(void)
{
    sdp_master_init(sdp_store_array,
                    sizeof(sdp_store_array) / sizeof(sdp_store_array[0]),
                    sdp_fetch_array,
                    sizeof(sdp_fetch_array) / sizeof(sdp_fetch_array[0]));

    sdp_store_array_index = 0U;
    sdp_fetch_array_index = 0U;
    sdp_send_read_commnad_next = true;
}

//
//
//
void dcdc_sdp_service(void)
{
    int16_t c;

    dcdc_sdp_message_schedule();

    if (SCI_getRxFIFOStatus(SCIA_BASE) > SCI_FIFO_RX0)
    {
        c = SCI_readCharNonBlocking(SCIA_BASE);
        sdp_receive_put(c);
    }

    c = sdp_transmit_peek();
    if (c != SDP_NO_CHARACTER)
    {
        if (SCI_getTxFIFOStatus(SCIA_BASE) < SCI_FIFO_TX16)
        {
            SCI_writeCharNonBlocking(SCIA_BASE, c);
            sdp_transmit_done();
        }
    }

    uint16_t sci_rx_status = SCI_getRxStatus(SCIA_BASE);
    if (sci_rx_status & SCI_RXSTATUS_ERROR)
        SCI_performSoftwareReset(SCIA_BASE);
}
