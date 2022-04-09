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
#ifndef _RoutingTableImpl_h
#define _RoutingTableImpl_h

#include "RoutingTable.h"

class RoutingTableImpl : public RoutingTable {
public:
    
    RoutingTableImpl();
    nodeaddr_t nextHop(nodeaddr_t finalDestAddr);
    void setRoute(nodeaddr_t target, nodeaddr_t nextHop);
    void clearRoutes();

private:

    /**
     * @brief Initializes the route table from what is stored in 
     * NVRAM.
     */
    void _load();
    void _save();

    static const unsigned int _tableSize = 64;
    nodeaddr_t _table[64];
};

#endif
