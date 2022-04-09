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
#ifndef _MessageProcessor_h
#define _MessageProcessor_h

#include "Configuration.h"
#include "OutboundPacketManager.h"
#include "CircularBuffer.h"
#include "Clock.h"
#include "Instrumentation.h"
#include "RoutingTable.h"

/**
 * @brief An instance of this class is responsible for pulling 
 * packets off the node's receive queue and processing them.
 */
class MessageProcessor {
public:

    MessageProcessor(Clock& clock, CircularBuffer& rxBuffer, CircularBuffer& txBuffer,
        RoutingTable& routingTable, Instrumentation& instrumentation,
        Configuration& config,
        uint32_t txTimeoutMs, uint32_t txRetryMs);

    /**
     * @brief Call this method from the event loop.  This causes
     * things to happen.
     */
    void pump();

    bool transmitIfPossible(const Packet& packet, 
        unsigned int packetLen);

    /**
     * @brief Generates a unique message ID
     */
    unsigned int getUniqueId();

    /**
     * @brief Resets all diagnostic counters
     */
    void resetCounter();

private:

    void _process(int16_t rssi, const Packet& packet, unsigned int packetLen);

    Configuration& _config;
    const Clock& _clock;
    CircularBuffer& _rxBuffer;
    CircularBuffer& _txBuffer;
    RoutingTable& _routingTable;
    Instrumentation& _instrumentation;    
    OutboundPacketManager _opm;
    unsigned int _idCounter;
    uint32_t _startTime;
};

#endif
