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

    void allocate(const Packet& packet, unsigned int packetLen,
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
