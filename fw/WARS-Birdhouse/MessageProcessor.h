#ifndef _MessageProcessor_h
#define _MessageProcessor_h

#include "OutboundPacketManager.h"
#include "CircularBuffer.h"
#include "Clock.h"
#include "Instrumentation.h"
#include "RoutingTable.h"

class MessageProcessor {
public:

    MessageProcessor(Clock& clock, CircularBuffer& rxBuffer, CircularBuffer& txBuffer,
        RoutingTable& routingTable, Instrumentation& instrumentation,
        nodeaddr_t myAddr, const char* myCall);

    void pump();

private:

    void _process(int16_t rssi, const Packet& packet, unsigned int packetLen);

    unsigned int _getUniqueId();

    const Clock& _clock;
    CircularBuffer& _rxBuffer;
    CircularBuffer& _txBuffer;
    RoutingTable& _routingTable;
    Instrumentation& _instrumentation;
    nodeaddr_t _myAddr;
    char _myCall[9];
    
    OutboundPacketManager _opm;
    unsigned int _idCounter;
    uint32_t _startTime;
};

#endif
