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
#ifndef _RoutingTable_h
#define _RoutingTable_h

#include "Utils.h"

class RoutingTable {
public:

    static nodeaddr_t NO_ROUTE;

    virtual nodeaddr_t nextHop(nodeaddr_t finalDestAddr) = 0;

    virtual void setRoute(nodeaddr_t target, nodeaddr_t nextHop) = 0;

    /**
     * @brief Removes all routes from the table.
     */
    virtual void clearRoutes() = 0;
};

#endif
