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
#ifndef _OutboundPacket_h
#define _OutboundPacket_h

#include "Clock.h"
#include "CircularBuffer.h"
#include "packets.h"

/**
 * @brief Used for tracking a packet that needs to the transmitted.  
 * A copy of the packet is retained inside of the OutboundPacketManager
 * in case a re-transmit is needed.  This object is als used to manage
 * retransmissions.
 */
class OutboundPacket {
public:

    OutboundPacket();

    bool isAllocated() const;
    bool isAck() const;
    
    void scheduleTransmit(const Packet& packet, unsigned int packetLen,
        uint32_t giveUpTime);

    /**
     * @brief Causes a transmit or re-transmit it the time is right.
     * 
     * @param clock 
     * @param tx_buffer 
     */
    void transmitIfReady(const Clock& clock, CircularBuffer& tx_buffer);

    /**
     * @brief Processes an ACK packet from another station, or ignores it if it
     *   is not relevant.
     * 
     * @param ackPacket The packet that was received. 
     */
    void processAckIfRelevant(const Packet& ackPacket);

private:

    void _reset();

    Packet _packet;
    // Inclusive of header
    unsigned int _packetLen;
    bool _isAllocated;
    // When we give up
    uint32_t _giveUpTime;
    // The last time a transmission was attempted
    uint32_t _lastTransmitTime;
};

#endif
