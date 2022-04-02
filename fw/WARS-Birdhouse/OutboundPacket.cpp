#include "OutboundPacket.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

extern Stream& logger;

#define RETRY_INTERVAL_SECONDS 2

//using namespace std;

OutboundPacket::OutboundPacket()
: _isAllocated(false) {
}

bool OutboundPacket::isAllocated() const {
    return _isAllocated;
}

void OutboundPacket::allocate(const Packet& packet, unsigned int packetLen,
    uint32_t giveUpTime) {
    _isAllocated = true;
    ::memcpy((void*)&_packet, (const void*)&packet, packetLen);
    _packetLen = packetLen;
    _giveUpTime = giveUpTime;
}

void OutboundPacket::transmitIfReady(const Clock& clock, CircularBuffer& txBuffer) {
    if (!_isAllocated) 
        return;
    // Check for timeouts.  If we hit a timeout then reset the packet
    if (clock.time()  > _giveUpTime) {
        logger.print("WRN: TX timeout ");
        logger.print(_packet.header.id);
        logger.println();
        _reset();
        return;
    } 
    // Check to see if this packet is still pending
    if ((clock.time() - _lastTransmitTime) < RETRY_INTERVAL_SECONDS * 1000) {
        return;
    }
    // If we make it here than we are ready to transmit
    bool good = txBuffer.push(0, &_packet, _packetLen);
    if (good) {
        logger.print("INF: Queued ");
        logger.print(_packet.header.id);
        logger.println();
        if (_packet.header.isAckRequired()) {
            // If an acknowledgement is required then record the 
            // necessary information to manage the retries.
            _lastTransmitTime = clock.time();
        } else {
            // If no acknowledgement is required then we are done.
            _reset();
        }    
    } else {
        logger.println("WRN: TX queue full");
    }
}

void OutboundPacket::processAckIfRelevant(const Packet& ackPacket) {
    // Check it see if this is an ACK that we were waiting for 
    if (_isAllocated &&
        ackPacket.header.sourceAddr == _packet.header.destAddr &&
        ackPacket.header.id == _packet.header.id) {
        logger.print("INF: ACK on ");
        logger.print(_packet.header.id);
        logger.println();
        _reset();
    }
}

void OutboundPacket::_reset() {
    _isAllocated = false;
    _lastTransmitTime = 0;
    _packetLen = 0;
}
