#ifndef _spi_utils_h
#define _spi_utils_h

#include <SPI.h>

void spi_setup();
uint8_t spi_read(uint8_t reg);
void spi_read_multi(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t spi_write(uint8_t reg, uint8_t val);
uint8_t spi_write_multi(uint8_t reg, uint8_t* buf, uint8_t len);

#endif
