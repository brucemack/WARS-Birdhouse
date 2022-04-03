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
#ifndef _OutboundPacketManager_h
#define _OutboundPacketManager_h

#include "Clock.h"
#include "CircularBuffer.h"
#include "packets.h"
#include "OutboundPacket.h"

class OutboundPacketManager {
public:

    OutboundPacketManager(const Clock& clock, CircularBuffer& txBuffer,
        uint32_t txTimeoutMs, uint32_t txRetryMs);

    bool scheduleTransmitIfPossible(const Packet& packet, 
        unsigned int packetLen);

    void processAck(const Packet& ackPacket);

    void pump();

    /**
     * @brief Gets the number of free packets that remain.
     * 
     * @return unsigned int 
     */
    unsigned int getFreeCount() const;

private:

    static const unsigned int _packetCount = 8;
    const Clock& _clock;
    CircularBuffer& _txBuffer;
    OutboundPacket _packets[_packetCount];
    uint32_t _txTimeoutMs;
    uint32_t _txRetryMs;
};

#endif
