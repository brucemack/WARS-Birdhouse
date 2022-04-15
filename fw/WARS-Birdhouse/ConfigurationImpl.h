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
#ifndef _ConfigurationImpl_h
#define _ConfigurationImpl_h

#include <Preferences.h>
#include "StationConfig.h"
#include "Configuration.h"

class ConfigurationImpl : public Configuration {
public:

    ConfigurationImpl(Preferences& pref);

    void begin();

    CallSign getCall() const;
    void setCall(const CallSign& call);

    nodeaddr_t getAddr() const;
    void setAddr(nodeaddr_t a);

    bool checkPasscode(uint32_t) const;
    void setPasscode(uint32_t);

    uint16_t getBatteryLimit() const;
    void setBatteryLimit(uint16_t l);

    uint16_t getBootCount() const;
    void setBootCount(uint16_t l);

    uint16_t getSleepCount() const;
    void setSleepCount(uint16_t l);

    void factoryReset();

private:

    void _load();
    void _save();

    // Handle to the ESP32 NVRAM
    Preferences& _pref;
    // A RAM-cached version of the configuration
    StationConfig _configCache;
};

#endif
