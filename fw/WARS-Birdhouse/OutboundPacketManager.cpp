#include "OutboundPacketManager.h"

OutboundPacketManager::OutboundPacketManager(const Clock& clock, CircularBuffer& txBuffer) 
    : _clock(clock), 
      _txBuffer(txBuffer) {
}

bool OutboundPacketManager::allocateIfPossible(const Packet& packet, unsigned int packetLen,
    bool ackRequired, uint32_t giveUpTime) {
    // Look for an unallocated packet a grab it - first come, first served.
    for (unsigned int i = 0; i < _packetCount; i++) {
        if (!_packets[i].isAllocated()) {
            _packets[i].allocate(packet, packetLen, ackRequired, giveUpTime);
            return true;
        }
    }
    // No unallocated packets, return the failure
    return false;
}

void OutboundPacketManager::pump() {
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
