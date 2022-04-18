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
#ifndef _Configuration_h
#define _Configuration_h

#include "Utils.h"

class Configuration {
public:

    virtual CallSign getCall() const = 0;
    virtual void setCall(const CallSign& call) {};

    virtual nodeaddr_t getAddr() const = 0;
    virtual void setAddr(nodeaddr_t a) {};

    virtual bool checkPasscode(uint32_t) const = 0;
    virtual void setPasscode(uint32_t) = 0;

    virtual uint16_t getBatteryLimit() const = 0;
    virtual void setBatteryLimit(uint16_t l) {};

    virtual uint16_t getBootCount() const = 0;
    virtual void setBootCount(uint16_t l) {};

    virtual uint16_t getSleepCount() const = 0;
    virtual void setSleepCount(uint16_t l) {};

    virtual uint8_t getLogLevel() const { return 0; }
    virtual void setLogLevel(uint8_t l) {};

    virtual uint8_t getCommandMode() const { return 0; }
    virtual void setCommandMode(uint8_t l) { };

    virtual void factoryReset() = 0;
};

#endif
