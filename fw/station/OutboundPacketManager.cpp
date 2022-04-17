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
#include "OutboundPacketManager.h"

OutboundPacketManager::OutboundPacketManager(const Clock& clock, CircularBuffer& txBuffer,
    uint32_t txTimeoutMs, uint32_t txRetryMs) 
    : _clock(clock), 
      _txBuffer(txBuffer),
      _txTimeoutMs(txTimeoutMs),
      _txRetryMs(txRetryMs) {
}

unsigned int OutboundPacketManager::getPendingCount() const {
    return _packetCount - getFreeCount();
}

bool OutboundPacketManager::scheduleTransmitIfPossible(const Packet& packet, unsigned int packetLen) {
    // Look for an unallocated packet a grab it - first come, first served.
    for (unsigned int i = 0; i < _packetCount; i++) {
        if (!_packets[i].isAllocated()) {
            _packets[i].scheduleTransmit(packet, packetLen, _clock.time() + _txTimeoutMs);
            return true;
        }
    }
    // No unallocated packets, return the failure
    return false;
}

void OutboundPacketManager::pump() {
    // Make sure the ACKs have priority in the output queue
    for (unsigned int i = 0; i < _packetCount; i++) {
        if (_packets[i].isAck())
            _packets[i].transmitIfReady(_clock, _txBuffer);
    }
    // Then do everything else
    for (unsigned int i = 0; i < _packetCount; i++) 
        _packets[i].transmitIfReady(_clock, _txBuffer);
}

void OutboundPacketManager::processAck(const Packet& ackPacket) {
    for (unsigned int i = 0; i < _packetCount; i++) 
        _packets[i].processAckIfRelevant(ackPacket);
}

unsigned int OutboundPacketManager::getFreeCount() const {
    unsigned int r = 0;
    for (unsigned int i = 0; i < _packetCount; i++) 
        if (!_packets[i].isAllocated()) 
            r++;
    return r;
}
