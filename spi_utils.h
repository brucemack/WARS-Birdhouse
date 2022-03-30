/* 
 * LoRa Birdhouse Mesh Network Project
 * Wellesley Amateur Radio Society
 * 
 * Copyright (C) 2022 Bruce MacKinnon
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef _spi_utils_h
#define _spi_utils_h

#include <SPI.h>

void spi_setup();
uint8_t spi_read(uint8_t reg);
void spi_read_multi(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t spi_write(uint8_t reg, uint8_t val);
uint8_t spi_write_multi(uint8_t reg, uint8_t* buf, uint8_t len);

#endif
