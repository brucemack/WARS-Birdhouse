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
#include <string.h>
#include "ConfigurationImpl.h"

ConfigurationImpl::ConfigurationImpl(Preferences& pref)
:   _pref(pref) {
}

void ConfigurationImpl::begin() {
    _load();
}


CallSign ConfigurationImpl::getCall() const {
    CallSign cs;
    cs.readFrom(_configCache.myCall);
    return cs;
}

void ConfigurationImpl::setCall(const CallSign& call) {
    call.writeTo(_configCache.myCall);
    _save();
}

nodeaddr_t ConfigurationImpl::getAddr() const {
    return _configCache.myAddr;
}

void ConfigurationImpl::setAddr(nodeaddr_t a) {
    _configCache.myAddr = a;
    _save();
}

bool ConfigurationImpl::checkPasscode(uint32_t p) const {
    return _configCache.securityToken == p;
}

void ConfigurationImpl::setPasscode(uint32_t p) {
    _configCache.securityToken = p;
    _save();
}

uint16_t ConfigurationImpl::getBatteryLimit() const {
    return _configCache.batteryLimitMv;
}

void ConfigurationImpl::setBatteryLimit(uint16_t l) {
    _configCache.batteryLimitMv = l;
    _save();
}

uint16_t ConfigurationImpl::getBootCount() const {
    return _configCache.bootCount;
}

void ConfigurationImpl::setBootCount(uint16_t l) {
    _configCache.bootCount = l;
    _save();
}

uint16_t ConfigurationImpl::getSleepCount() const {
    return _configCache.sleepCount;
}

void ConfigurationImpl::setSleepCount(uint16_t l) {
    _configCache.sleepCount = l;
    _save();
}

uint8_t ConfigurationImpl::getLogLevel() const {
    return _configCache.logLevel;
}

void ConfigurationImpl::setLogLevel(uint8_t l) {
    _configCache.logLevel = l;
    _save();
}

uint8_t ConfigurationImpl::getCommandMode() const {
    return _configCache.commandMode;
}

void ConfigurationImpl::setCommandMode(uint8_t l) {
    _configCache.commandMode = l;
    _save();  
}

void ConfigurationImpl::_save() {
    const uint8_t* v = (const uint8_t*)&_configCache;
    _pref.putBytes("config", v, sizeof(StationConfig));
}

void ConfigurationImpl::_load() {
    uint8_t* v = (uint8_t*)&_configCache;
    _pref.getBytes("config", v, sizeof(StationConfig));
}

void ConfigurationImpl::factoryReset() {
    memset((uint8_t*)&_configCache, 0, sizeof(StationConfig));
    _pref.remove("config");
}
