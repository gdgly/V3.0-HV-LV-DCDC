///////////////////////////////////////////////////////////////////////////////
//
// SPI Flash package include file
// Part number Adesto AT25SF041B
// The driver assumes SPI-A is used on GPIOs 8 to 11.
// (to be able to use it in boot mode)
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_


#define SPI_FLASH_ADDRESS_MAX       0x0007FFFFUL


extern void spi_flash_init(void);
extern void spi_flash_service(void);

extern bool spi_flash_is_ready(void);
extern bool spi_flash_read_initiate(uint32_t address, void *data_p,
                                    uint32_t data_size_in_words);
extern bool spi_flash_write_initiate(uint32_t address, const void *data_p,
                                     uint32_t data_size_in_words);
extern bool spi_flash_erase_4kb_block_initiate(uint32_t address);
extern bool spi_flash_erase_chip_initiate(void);

#endif
