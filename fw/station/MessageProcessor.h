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

struct PacketReport {
    PacketReport() : node(0), id(0) { }
    nodeaddr_t node;
    uint16_t id;
};

/**
 * @brief An instance of this class is responsible for dealing
 * with the inbound and outbound message flow.  Key responsibilities:
 *   - Keeping a queue of messages that are waiting to be 
 *     transmitted.
 *   - Keeping a copy of any message that has not been acknowledged
 *     in case it needs to be re-transmitted.
 *   - Keeping track of timeout intervals and issuing re-transmitts. 
 *   - Looking at received messages to find the acknowledgements.  Marking
 *     outbound messages when they are successfully acknowledged.
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

    /**
     * @brief Queues a message for transmission, assuming there is 
     * space in the outbound queue.
     * 
     * @param packet 
     * @param packetLen 
     * @return true 
     * @return false 
     */
    bool transmitIfPossible(const Packet& packet, 
        unsigned int packetLen);

    /**
     * @brief Generates a unique message ID
     */
    unsigned int getUniqueId();

    /**
     * @brief Get the number of packets that are in-flight (i.e.
     * waiting to be sent or waiting to be ACKd.
     */
    uint16_t getPendingCount() const;

    uint16_t getBadRxPacketCounter() const;
    uint16_t getBadRouteCounter() const;

    /**
     * @brief Resets all diagnostic counters
     */
    void resetCounters();

    uint32_t getSecondsSinceLastRx() const;

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
    uint32_t _lastRxTime;
    // Diagnostic counters
    uint16_t _rxPacketCounter;
    uint16_t _badRxPacketCounter;
    uint16_t _badRouteCounter;
    
    // Used for tracking packets to supress duplicates.  The more slots we allocate
    // the better we will be at eliminating duplicates.
    const static unsigned int _packetReportSlots = 32;
    unsigned int _packetReportPtr = 0;
    PacketReport _packetReport[_packetReportSlots];
};

#endif
