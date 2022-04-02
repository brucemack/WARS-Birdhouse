#ifndef _OutboundPacketManager_h
#define _OutboundPacketManager_h

#include "Clock.h"
#include "CircularBuffer.h"
#include "packets.h"
#include "OutboundPacket.h"

class OutboundPacketManager {
public:

        OutboundPacketManager(const Clock& clock, CircularBuffer& txBuffer);

        bool allocateIfPossible(const Packet& packet, unsigned int packetLen,
                uint32_t giveUpTime);

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
};

#endif
