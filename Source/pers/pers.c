///////////////////////////////////////////////////////////////////////////////
//
// Persistence package
//
///////////////////////////////////////////////////////////////////////////////

#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
#include "driverlib.h"
#include "pers/pers.h"
#include "spi_flash/spi_flash.h"
#include "crc/crc.h"


#define PERS_STRUCT_SIZE                            0x0001000UL
#define PERS_STRUCT_HEADER_OFFSET                   0x0000FF0UL

#define PERS_FACTORY_STRUCT_BASE_ADDRESS_A          0x003C000UL
#define PERS_FACTORY_STRUCT_BASE_ADDRESS_B          (PERS_FACTORY_STRUCT_BASE_ADDRESS_A + PERS_STRUCT_SIZE)

#define PERS_CONFIGURATION_STRUCT_BASE_ADDRESS_A    0x003E000UL
#define PERS_CONFIGURATION_STRUCT_BASE_ADDRESS_B    (PERS_CONFIGURATION_STRUCT_BASE_ADDRESS_A + PERS_STRUCT_SIZE)


struct pers_structure_header
{
    uint16_t size_in_words;
    uint16_t crc;
};

enum pers_state
{
    PERS_STATE_READY,
    PERS_STATE_WAITING_FOR_BLOCK_ERASE_TO_FINISH,
    PERS_STATE_WAITING_FOR_BLOCK_WRITE_TO_FINISH,
    PERS_STATE_WAITING_FOR_BLOCK_HEADER_WRITE_TO_FINISH,
};


static void *pers_factory_struct_p;
static uint16_t pers_factory_struct_size_in_words;
static void *pers_configuration_struct_p;
static uint16_t pers_configuration_struct_size_in_words;

static enum pers_state pers_state;
static struct pers_structure_header pers_header;

static uint32_t pers_address_to_write;
static void *pers_struct_to_write_p;
static uint16_t pers_struct_to_write_size_in_words;
static bool pers_are_both_blocks_to_be_written;



//
// Blocking function. To be used ONLY during initialization.
//
static void pers_spi_flash_service_until_ready(void)
{
    do {
        spi_flash_service();
        SysCtl_serviceWatchdog();
    } while (!spi_flash_is_ready());
}

//
// Blocking function. To be used ONLY during initialization.
//
static bool pers_is_struct_valid(uint32_t address)
{
    // First verify structure header
    spi_flash_read_initiate((address + PERS_STRUCT_HEADER_OFFSET), &pers_header, sizeof(pers_header));
    pers_spi_flash_service_until_ready();

    // Calculate CRC of structure block
    uint16_t crc_calculated = 0U;
    uint16_t i;
    for (i = 0U; i < pers_header.size_in_words; ++i)
    {
        uint16_t buffer = 0U; // Initialize to remove MISRA error.
        spi_flash_read_initiate(address + i, &buffer, 1UL);
        pers_spi_flash_service_until_ready();
        crc_calculated = crc_16ccitt_calculate(&buffer, 1U, crc_calculated);
    }

    bool is_data_valid = (pers_header.crc == crc_calculated);

    return is_data_valid;
}

//
// Returns the size of the structure retrieved (in words).
// Blocking function. To be used ONLY during initialization.
//
static uint16_t pers_struct_retrieve(uint32_t address, void *struct_p,
                                     uint16_t struct_size_in_words)
{
    // First verify structure header
    spi_flash_read_initiate((address + PERS_STRUCT_HEADER_OFFSET), &pers_header, sizeof(pers_header));
    pers_spi_flash_service_until_ready();

    // For backward/forward compatibility, retrieve only the minimum
    // of the persistent size and the size of the structure in the application
    uint16_t size_to_retrieve =
            (pers_header.size_in_words < struct_size_in_words) ?
                    pers_header.size_in_words : struct_size_in_words;
    spi_flash_read_initiate((address), struct_p, size_to_retrieve);
    pers_spi_flash_service_until_ready();

    return size_to_retrieve;
}

//
// Blocking function. To be used ONLY during initialization.
//
static void pers_block_restore(uint32_t address, void *struct_p,
                               uint16_t struct_size_in_words)
{
    pers_address_to_write = address;
    pers_struct_to_write_p = struct_p;
    pers_struct_to_write_size_in_words = struct_size_in_words;
    pers_are_both_blocks_to_be_written = false;

    spi_flash_erase_4kb_block_initiate(address);
    pers_state = PERS_STATE_WAITING_FOR_BLOCK_ERASE_TO_FINISH;
    do {
        pers_service();
    } while (pers_is_ready());
}

//
// Returns the size of the structure retrieved (in words).
// Blocking function. To be used ONLY during initialization.
//
static uint16_t pers_retrieve_and_restore(uint32_t address_block_a, void *struct_p,
                                          uint16_t struct_size_in_words)
{
    bool is_block_a_valid = pers_is_struct_valid(address_block_a);
    uint32_t address_block_b = address_block_a + PERS_STRUCT_SIZE;
    bool is_block_b_valid = pers_is_struct_valid(address_block_b);

    uint16_t size_to_retrieve_in_words = 0U;
    if (is_block_a_valid)
        size_to_retrieve_in_words = pers_struct_retrieve(
                address_block_a, struct_p,
                struct_size_in_words);
    else if (is_block_b_valid)
        size_to_retrieve_in_words = pers_struct_retrieve(
                address_block_b, struct_p,
                struct_size_in_words);

    if (is_block_a_valid && (!is_block_b_valid))
    {
        pers_block_restore(address_block_b, struct_p,
                           size_to_retrieve_in_words);
    }
    else if ((!is_block_a_valid) && is_block_b_valid)
    {
        pers_block_restore(address_block_a, struct_p,
                           size_to_retrieve_in_words);
    }
    // else nothing to restore

    return size_to_retrieve_in_words;
}




//
// Retrieves values that persist on external flash device
// Set pointers to NULL if the structure is unused.
//
void pers_init(void *factory_struct_p,
               uint16_t factory_struct_size_in_words,
               void *configuration_struct_p,
               uint16_t configuration_struct_size_in_words)
{
    uint16_t size_retrieved_in_words;

    if (factory_struct_p != NULL)
    {
        size_retrieved_in_words = pers_retrieve_and_restore(
                PERS_FACTORY_STRUCT_BASE_ADDRESS_A, factory_struct_p,
                factory_struct_size_in_words);
        pers_factory_struct_p = factory_struct_p;
        pers_factory_struct_size_in_words = size_retrieved_in_words;
    }

    if (configuration_struct_p != NULL)
    {
        size_retrieved_in_words = pers_retrieve_and_restore(
                PERS_CONFIGURATION_STRUCT_BASE_ADDRESS_A, configuration_struct_p,
                configuration_struct_size_in_words);
        pers_configuration_struct_p = configuration_struct_p;
        pers_configuration_struct_size_in_words = size_retrieved_in_words;
    }

    pers_state = PERS_STATE_READY;
}

//
//
//
void pers_service(void)
{
    if (!spi_flash_is_ready())
        return;

    switch (pers_state)
    {
        case PERS_STATE_READY:
            // Do nothing.
            break;
        case PERS_STATE_WAITING_FOR_BLOCK_ERASE_TO_FINISH:
            spi_flash_write_initiate(pers_address_to_write,
                                     pers_struct_to_write_p,
                                     pers_struct_to_write_size_in_words);
            pers_state = PERS_STATE_WAITING_FOR_BLOCK_WRITE_TO_FINISH;
            break;
        case PERS_STATE_WAITING_FOR_BLOCK_WRITE_TO_FINISH:
            pers_header.size_in_words = pers_struct_to_write_size_in_words;
            pers_header.crc = crc_16ccitt_calculate(
                    pers_struct_to_write_p, pers_struct_to_write_size_in_words, 0U);
            spi_flash_write_initiate(pers_address_to_write + PERS_STRUCT_HEADER_OFFSET,
                                     &pers_header, sizeof(pers_header));
            pers_state = PERS_STATE_WAITING_FOR_BLOCK_HEADER_WRITE_TO_FINISH;
            break;
        case PERS_STATE_WAITING_FOR_BLOCK_HEADER_WRITE_TO_FINISH:
            if (pers_are_both_blocks_to_be_written)
            {
                pers_are_both_blocks_to_be_written = false;
                pers_address_to_write += PERS_STRUCT_SIZE;
                spi_flash_erase_4kb_block_initiate(pers_address_to_write);
                pers_state = PERS_STATE_WAITING_FOR_BLOCK_ERASE_TO_FINISH;
            }
            else
                pers_state = PERS_STATE_READY;
            break;
        default:
            pers_state = PERS_STATE_READY;
            break;
    }
}

//
//
//
bool pers_is_ready(void)
{
    return (pers_state == PERS_STATE_READY);
}

//
//
//
bool pers_factory_store_initiate(void)
{
    if (!pers_is_ready())
        return false;

    // Block A is always written first, then block B.
    pers_address_to_write = PERS_FACTORY_STRUCT_BASE_ADDRESS_A;
    pers_struct_to_write_p = pers_factory_struct_p;
    pers_struct_to_write_size_in_words = pers_factory_struct_size_in_words;
    pers_are_both_blocks_to_be_written = true;

    spi_flash_erase_4kb_block_initiate(pers_address_to_write);
    pers_state = PERS_STATE_WAITING_FOR_BLOCK_ERASE_TO_FINISH;
    return true;
}

//
//
//
bool pers_configuration_store_initiate(void)
{
    if (pers_is_ready())
        return false;

    // Block A is always written first, then block B.
    pers_address_to_write = PERS_CONFIGURATION_STRUCT_BASE_ADDRESS_A;
    pers_struct_to_write_p = pers_configuration_struct_p;
    pers_struct_to_write_size_in_words = pers_configuration_struct_size_in_words;
    pers_are_both_blocks_to_be_written = true;

    spi_flash_erase_4kb_block_initiate(pers_address_to_write);
    pers_state = PERS_STATE_WAITING_FOR_BLOCK_ERASE_TO_FINISH;
    return true;
}

