///////////////////////////////////////////////////////////////////////////////
//
// SPI Flash package
// Part number Adesto AT25SF041B
// The driver assumes SPI-A is used on GPIOs 8 to 11.
// (to be able to use it in boot mode)
//
///////////////////////////////////////////////////////////////////////////////

#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
#include "driverlib.h"
#include "spi_flash/spi_flash.h"


#define SPI_FLASH_CS_GPIO               11

#define SPI_FLASH_OPCODE_WRITE          0x02U
#define SPI_FLASH_OPCODE_READ           0x03U
#define SPI_FLASH_OPCODE_READ_STATUS_1  0x05U
#define SPI_FLASH_OPCODE_WRITE_ENABLE   0x06U
#define SPI_FLASH_OPCODE_ERASE_4KB      0x20U
#define SPI_FLASH_OPCODE_ERASE_CHIP     0x60U

#define SPI_FLASH_STATUS_BUSY_BIT_MASK  0x01U
#define SPI_FLASH_WRITE_PAGE_SIZE       256UL
#define SPI_FLASH_WRITE_PAGE_MASK       0xFFUL

#define SPI_FLASH_DUMMY_BYTE            0x00U


enum spi_flash_states
{
    SPI_FLASH_STATE_READY,
    SPI_FLASH_STATE_WAITING_FOR_WRITE_ENABLE,
    SPI_FLASH_STATE_WAITING_FOR_OPCODE_TO_BE_SENT,
    SPI_FLASH_STATE_WAITING_FOR_ADDRESS_TO_BE_SENT,
    SPI_FLASH_STATE_WAITING_FOR_DATA_TO_BE_SENT,
    SPI_FLASH_STATE_WAITING_FOR_DEVICE_READY,
};


static enum spi_flash_states spi_flash_state;
static uint16_t spi_flash_opcode;
static uint32_t spi_flash_address;
static const uint16_t *spi_flash_write_data;
static uint16_t *spi_flash_read_data;
static uint32_t spi_flash_data_size_in_words;
static uint32_t spi_flash_data_index;


//
//
//
static void spi_flash_cs_enable(void)
{
    GPIO_writePin(SPI_FLASH_CS_GPIO, 0);
}

//
//
//
static void spi_flash_cs_disable(void)
{
    GPIO_writePin(SPI_FLASH_CS_GPIO, 1);
}

//
//
//
static void spi_flash_write_to_spi(uint16_t data)
{
    spi_flash_cs_enable();
    uint16_t left_justified_data = data << 8U;
    SPI_writeDataNonBlocking(SPIA_BASE, left_justified_data);
}

//
//
//
static uint16_t spi_flash_read_from_spi(void)
{
    uint16_t value = SPI_readDataNonBlocking(SPIA_BASE);
    return (value & 0xFFU);
}

//
//
//
static void spi_flash_write_enable_initiate(void)
{
    spi_flash_write_to_spi(SPI_FLASH_OPCODE_WRITE_ENABLE);
    spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_WRITE_ENABLE;
}

//
//
//
static void spi_flash_opcode_transmission_initiate(void)
{
    spi_flash_write_to_spi(spi_flash_opcode);
}

//
//
//
static void spi_flash_address_transmission_initiate(void)
{
    if (spi_flash_opcode == SPI_FLASH_OPCODE_ERASE_CHIP)
        return;

    spi_flash_write_to_spi((spi_flash_address >> 16U) & 0xFFU);
    spi_flash_write_to_spi((spi_flash_address >> 8U) & 0xFFU);
    spi_flash_write_to_spi(spi_flash_address & 0xFFU);
}

//
//
//
static void spi_flash_data_transmission_initiate(void)
{
    switch (spi_flash_opcode)
    {
        case SPI_FLASH_OPCODE_WRITE:
            spi_flash_write_to_spi(spi_flash_write_data[spi_flash_data_index] >> 8U);
            spi_flash_write_to_spi(spi_flash_write_data[spi_flash_data_index] & 0xFFU);
            break;
        case SPI_FLASH_OPCODE_READ:
            spi_flash_write_to_spi(SPI_FLASH_DUMMY_BYTE);
            spi_flash_write_to_spi(SPI_FLASH_DUMMY_BYTE);
            break;
        case SPI_FLASH_OPCODE_ERASE_4KB:
        case SPI_FLASH_OPCODE_ERASE_CHIP:
        default:
            break;
    }
}

//
//
//
static void spi_flash_read_word_end(void)
{
    uint16_t msb = spi_flash_read_from_spi();
    uint16_t lsb = spi_flash_read_from_spi();
    spi_flash_read_data[spi_flash_data_index] = (msb << 8U) + lsb;
}


//
//
//
static void spi_flash_read_status_initiate(void)
{
    spi_flash_write_to_spi(SPI_FLASH_OPCODE_READ_STATUS_1);
    spi_flash_write_to_spi(SPI_FLASH_DUMMY_BYTE);
}

//
//
//
static uint16_t spi_flash_read_status_end(void)
{
    uint16_t value = spi_flash_read_from_spi(); // Discard first read
    value = spi_flash_read_from_spi();
    return value;
}



//
//
//
void spi_flash_init(void)
{
    // Make sure there is no ongoing operation
    spi_flash_cs_disable();
    spi_flash_read_status_initiate();
    spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_DEVICE_READY;

    while (spi_flash_state != SPI_FLASH_STATE_READY)
        spi_flash_service();

    spi_flash_opcode = 0U;
    spi_flash_address = 0UL;
    spi_flash_write_data = 0U;
    spi_flash_data_size_in_words = 0UL;
    spi_flash_data_index = 0UL;
}

//
//
//
void spi_flash_service(void)
{
    if (SPI_isBusy(SPIA_BASE))
        return;

    switch (spi_flash_state)
    {
        case SPI_FLASH_STATE_READY:
            // Do nothing
            break;
        case SPI_FLASH_STATE_WAITING_FOR_WRITE_ENABLE:
            spi_flash_cs_disable();
            spi_flash_opcode_transmission_initiate();
            spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_OPCODE_TO_BE_SENT;
            break;
        case SPI_FLASH_STATE_WAITING_FOR_OPCODE_TO_BE_SENT:
            spi_flash_address_transmission_initiate();
            spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_ADDRESS_TO_BE_SENT;
            break;
        case SPI_FLASH_STATE_WAITING_FOR_ADDRESS_TO_BE_SENT:
            SPI_resetRxFIFO(SPIA_BASE); // First clear the FIFO to get rid of any previous values there
            spi_flash_data_index = 0UL;
            spi_flash_data_transmission_initiate();
            spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_DATA_TO_BE_SENT;
            break;
        case SPI_FLASH_STATE_WAITING_FOR_DATA_TO_BE_SENT:
            if (spi_flash_opcode == SPI_FLASH_OPCODE_READ)
                spi_flash_read_word_end();

            ++spi_flash_data_index;
            if (spi_flash_data_index >= spi_flash_data_size_in_words)
            {
                spi_flash_cs_disable();
                spi_flash_read_status_initiate();
                spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_DEVICE_READY;
            }
            else if ((spi_flash_opcode == SPI_FLASH_OPCODE_WRITE) // if writing new page
                    && (((spi_flash_address + (spi_flash_data_index << 1U)) & SPI_FLASH_WRITE_PAGE_MASK) == 0U))
            {
                spi_flash_address += SPI_FLASH_WRITE_PAGE_SIZE;
                spi_flash_address &= ~SPI_FLASH_WRITE_PAGE_MASK;
                spi_flash_data_size_in_words -= spi_flash_data_index;
                spi_flash_cs_disable();
                spi_flash_write_enable_initiate();
            }
            else
            {
                spi_flash_data_transmission_initiate();
            }
            break;
        case SPI_FLASH_STATE_WAITING_FOR_DEVICE_READY:
        {
            uint16_t value = spi_flash_read_status_end();
            if (value & SPI_FLASH_STATUS_BUSY_BIT_MASK)
            {
                spi_flash_read_status_initiate(); // ask again
            }
            else
            {
                spi_flash_cs_disable();
                spi_flash_state = SPI_FLASH_STATE_READY;
            }
            break;
        }
        default:
            spi_flash_cs_disable();
            spi_flash_state = SPI_FLASH_STATE_READY;
            break;
    }
}

//
//
//
bool spi_flash_is_ready(void)
{
    return (spi_flash_state == SPI_FLASH_STATE_READY)
            && (!SPI_isBusy(SPIA_BASE));
}

//
// Return true if successful initiation
//
bool spi_flash_read_initiate(uint32_t address, void *data_p,
                             uint32_t data_size_in_words)
{
    if (!spi_flash_is_ready())
        return false;

    if (data_p == NULL)
        return false;

    if ((address + (data_size_in_words << 1U)) > SPI_FLASH_ADDRESS_MAX)
        return false;

    spi_flash_opcode = SPI_FLASH_OPCODE_READ;
    spi_flash_address = address & ~1UL;
    spi_flash_read_data = data_p;
    spi_flash_data_size_in_words = data_size_in_words;

    spi_flash_opcode_transmission_initiate();
    spi_flash_state = SPI_FLASH_STATE_WAITING_FOR_OPCODE_TO_BE_SENT;

    return true;
}

//
// Return true if successful initiation
//
bool spi_flash_write_initiate(uint32_t address, const void *data_p,
                              uint32_t data_size_in_words)
{
    if (!spi_flash_is_ready())
        return false;

    if (data_p == NULL)
        return false;

    if ((address + (data_size_in_words << 1U)) > SPI_FLASH_ADDRESS_MAX)
        return false;

    spi_flash_opcode = SPI_FLASH_OPCODE_WRITE;
    spi_flash_address = address & ~1UL;
    spi_flash_write_data = data_p;
    spi_flash_data_size_in_words = data_size_in_words;

    spi_flash_write_enable_initiate();

    return true;
}

//
// Return true if successful initiation
//
bool spi_flash_erase_4kb_block_initiate(uint32_t address)
{
    if (!spi_flash_is_ready())
        return false;

    if (address > SPI_FLASH_ADDRESS_MAX)
        return false;

    spi_flash_opcode = SPI_FLASH_OPCODE_ERASE_4KB;
    spi_flash_address = address;
    spi_flash_data_size_in_words = 0UL;

    spi_flash_write_enable_initiate();

    return true;
}

//
// Return true if successful initiation
//
bool spi_flash_erase_chip_initiate(void)
{
    if (!spi_flash_is_ready())
        return false;

    spi_flash_opcode = SPI_FLASH_OPCODE_ERASE_CHIP;
    spi_flash_data_size_in_words = 0UL;

    spi_flash_write_enable_initiate();

    return true;
}
