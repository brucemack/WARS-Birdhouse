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
#include <stdint.h>
#include "RoutingTableImpl.h"

RoutingTableImpl::RoutingTableImpl(Preferences& pref) 
:   _pref(pref) {
}

void RoutingTableImpl::begin() {
    _load();
}

nodeaddr_t RoutingTableImpl::nextHop(nodeaddr_t finalDestAddr) {
    if (finalDestAddr == 0) {
        return 0;
    } else if (finalDestAddr >= 0xfff0) {
        return finalDestAddr;
    } else if (finalDestAddr >= _tableSize) {
        return NO_ROUTE;
    } else {
        return _table[finalDestAddr];
    }
}

void RoutingTableImpl::setRoute(nodeaddr_t target, nodeaddr_t nextHop) {
    if (target > 0 && target < _tableSize) {
        _table[target] = nextHop;
        _save();
    }
}

void RoutingTableImpl::clearRoutes() {
    for (unsigned int i = 0; i < _tableSize; i++)
        _table[i] = 0;
    _save();
}

void RoutingTableImpl::_load() {
    _pref.getBytes("routing", (void*)_table, _tableSize);
}

void RoutingTableImpl::_save() {
    _pref.putBytes("routing", (const void*)_table, _tableSize);
}
