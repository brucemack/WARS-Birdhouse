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
#ifndef _Instrumentation_h
#define _Instrumentation_h

#include <stdint.h>

/**
 * @brief An interface for accessing device-level instrumentation data.
 * This is done through an abstract interface to make unit testing 
 * easier.
 */
class Instrumentation {
public:

    virtual uint16_t getSoftwareVersion() const = 0;
    virtual uint16_t getDeviceClass() const = 0;
    virtual uint16_t getDeviceRevision() const = 0;
    virtual uint16_t getBatteryVoltage() const = 0;
    virtual uint16_t getPanelVoltage() const = 0;
    virtual int16_t getTemperature() const = 0;
    virtual int16_t getHumidity() const = 0;
    virtual uint16_t getBootCount() const = 0;
    virtual uint16_t getSleepCount() const = 0;
    
    virtual void sleep(uint32_t ms) = 0;
    virtual void restart() = 0;
    virtual void restartRadio() = 0;
};

#endif
