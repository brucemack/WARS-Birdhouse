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
#ifndef _StationConfig_h
#define _StationConfig_h

#include "Utils.h"

// Size is approximately 24 bytes
struct StationConfig {
    nodeaddr_t myAddr;
    uint8_t myCall[8];
    uint16_t batteryLimitMv;
    uint8_t commandMode;
    uint32_t securityToken;
    uint16_t bootCount;
    uint16_t sleepCount;
    uint8_t logLevel;
};

#endif
