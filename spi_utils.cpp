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
#include <Arduino.h>
#include "spi_utils.h"

#define SS_PIN    5

static SPISettings spi_settings(1000000, MSBFIRST, SPI_MODE0);

void spi_setup() {
  // SPI slave select configuration
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  delay(100);
  SPI.begin();
}

uint8_t spi_read(uint8_t reg) {
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask off
  SPI.transfer(reg & ~0x80); 
  // The written value is ignored, reg value is read
  uint8_t val = SPI.transfer(0); 
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

void spi_read_multi(uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask off
  SPI.transfer(reg & ~0x80); 
  while (len--) {
    // The written value is ignored, reg value is read
    *buf = SPI.transfer(0); 
    buf++;
  }
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

// Writes one byte to SPI.  Returns whatever comes back during the transfer.
//
// NOTE: From RFM95W datasheet (pg 76):
//
// During the write access, the byte transferred from the slave to the master on the MISO line 
// is the value of the written register before the write operation.
//
uint8_t spi_write(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask on. 
  SPI.transfer(reg | 0x80);
  // Send the data, capturing the original value 
  uint8_t orig_val = SPI.transfer(val); 
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  return orig_val;
}

uint8_t spi_write_multi(uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask on
  uint8_t stat = SPI.transfer(reg | 0x80); 
  // Transfer each byte individually
  while (len--) {
    SPI.transfer(*buf);
    buf++;
  }
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  return stat;
}
