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
#include "ConfigurationImpl.h"

#include <EEPROM.h>

ConfigurationImpl::ConfigurationImpl() {
    _load();
}

CallSign ConfigurationImpl::getCall() const {
    CallSign cs;
    cs.readFrom(_configCache.myCall);
    return cs;
}

nodeaddr_t ConfigurationImpl::getAddr() const {
    return _configCache.myAddr;
}

void ConfigurationImpl::setCall(const CallSign& call) {
    call.writeTo(_configCache.myCall);
    _save();
}

void ConfigurationImpl::setAddr(nodeaddr_t a) {
    _configCache.myAddr = a;
    _save();
}

void ConfigurationImpl::_save() {
    const uint8_t* v = (const uint8_t*)&_configCache;
    for (unsigned int i = 0; i < sizeof(StationConfig); i++)
        EEPROM.write(i, v[i]);
}

void ConfigurationImpl::_load() {
    uint8_t* v = (uint8_t*)&_configCache;
    for (unsigned int i = 0; i < sizeof(StationConfig); i++)
        v[i] = EEPROM.read(i);
}


