#ifndef _OutboundPacket_h
#define _OutboundPacket_h

#include "CircularBuffer.h"
#include "packets.h"

static const unsigned int MAX_PAYLOAD_SIZE = 128 - sizeof(Header);

struct Packet {
    Header header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
};

class OutboundPacket {
public:

    OutboundPacket();

    /**
     * @brief Used to allocate the slot to an outbound packet.
     * 
     * @param acknowledgementRequired Set to true if this packet
     *   needs to be acknowledged by the destination station.
     * @param timeoutTime Set to the time when we should give
     *   up and fail this packet.
     */
    void setActive(bool acknowledgementRequired, int32_t timeoutTime);

    /**
     * @brief Indicates whether the slot is being used.
     * 
     * @return true 
     * @return false 
     */
    bool isActive() const;

    /**
     * @brief Transmits the packet onto the buffer.
     * 
     * @param tx_buffer 
     * @param time The current time
     */
    void transmitIfReady(CircularBuffer& tx_buffer, int32_t time);

    /**
     * @brief Processes an ACK message, or ignores it if it
     *   is not relevant to this packet.
     * 
     * @param ack_header 
     */
    void receivedAck(const Header& ack_header);

    Packet packet;
    // Inclusive of header
    unsigned int totalLen;

private:

    void _reset();

    bool _isUsed;
    bool _isAckRequired;
    // When we give up
    int32_t _timeoutTime;
    // The last time a transmission was attempted
    int32_t _lastTransmitTime;
};

#endif
