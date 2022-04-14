#include <Arduino.h>

#include "../WARS-Birdhouse/CircularBuffer.h"
#include "../WARS-Birdhouse/packets.h"
#include "../WARS-Birdhouse/OutboundPacketManager.h"
#include "../WARS-Birdhouse/Instrumentation.h"
#include "../WARS-Birdhouse/RoutingTable.h"
#include "../WARS-Birdhouse/RoutingTableImpl.h"
#include "../WARS-Birdhouse/MessageProcessor.h"
#include "../WARS-Birdhouse/CommandProcessor.h"
#include "../WARS-Birdhouse/Configuration.h"
#include "TestClockImpl.h"

#include <iostream>
#include <assert.h>
#include <string.h>

using namespace std;

// ===== DUMMY COMPONENTS =============================================

class TestStream : public Stream {
public:

    void print(const char* m) { cout << m; }
    void print(unsigned int m) { cout << m; }
    void print(uint16_t m) { cout << m; }
    void println() { cout << endl; }
    void println(const char* m) { cout << m << endl; }
};

class TestInstrumentation : public Instrumentation {
public:
    uint16_t getSoftwareVersion() const { return 1; }
    uint16_t getDeviceClass() const { return 2; }
    uint16_t getDeviceRevision() const { return 1; }
    uint16_t getBatteryVoltage() const { return 3800; }
    uint16_t getPanelVoltage() const { return 4000; }
    int16_t getTemperature() const { return 23; }
    int16_t getHumidity() const { return  87; }
    void restart() { cout << "RESTART" << endl; }
    void restartRadio() { cout << "RESTART" << endl; }
    void sleep(uint32_t ms) { cout << "SLEEP " << ms << endl; }
};

class TestConfiguration : public Configuration {
public:

    TestConfiguration(nodeaddr_t myAddr, const char* myCall) 
    : _myAddr(myAddr), 
      _myCall(myCall),
      _bootCount(6),
      _sleepCount(1) {
    }

    nodeaddr_t getAddr() const {
        return _myAddr;
    }

    CallSign getCall() const {
        return _myCall;
    }

    uint16_t getBatteryLimit() const {
        return 3400;
    }

    uint16_t getBootCount() const {
        return _bootCount;
    }

    void setBootCount(uint16_t l) {
        _bootCount = l;
    }

    uint16_t getSleepCount() const {
        return _sleepCount;
    };

    void setSleepCount(uint16_t l) {
        _sleepCount = l;
    }

private:

    nodeaddr_t _myAddr;
    const CallSign _myCall;
    uint16_t _bootCount, _sleepCount;
};

void movePacket(CircularBuffer& from, CircularBuffer& to) {
    unsigned int packetLen = 256;
    uint8_t packet[256];
    bool got = from.popIfNotEmpty(0, packet, &packetLen);
    if (got) {
        int16_t rssi = 100;
        to.push(&rssi, packet, packetLen);
    }
}

// ===== TEST CASES ================================================

static TestStream testLogger;
Stream& logger = testLogger;
static TestClock testClock;
Clock& systemClock = testClock;

// Node #1
Preferences nvram1;
static TestConfiguration testConfig(1, "KC1FSZ");
Configuration& systemConfig = testConfig;
static TestInstrumentation testInstrumentation;
Instrumentation& systemInstrumentation = testInstrumentation;
static RoutingTableImpl testRoutingTable(nvram1);
RoutingTable& systemRoutingTable = testRoutingTable;
CircularBufferImpl<4096> txBuffer(0);
CircularBufferImpl<4096> rxBuffer(2);
MessageProcessor testMessageProcessor(testClock, rxBuffer, txBuffer,
    testRoutingTable, testInstrumentation, testConfig,
    10 * 1000, 2 * 1000);
MessageProcessor& systemMessageProcessor = testMessageProcessor;

void test_CommandProcessor() {
    
    systemRoutingTable.setRoute(3, 3);
    systemRoutingTable.setRoute(7, 3);

    // Increment boot count
    systemConfig.setBootCount(systemConfig.getBootCount() + 1);

    // PING
    {
        const char* a0 = "ping";
        const char* a1 = "7";
        const char *a_args[2] = { a0, a1 };

        sendPing(2, (char**)a_args);

        systemMessageProcessor.pump();

        // Make sure we see the outbound message
        assert(!txBuffer.isEmpty());
        txBuffer.popAndDiscard();
    }

    // INFO
    {
        const char* a0 = "info";
        const char *a_args[2] = { a0 };

        info(1, (char**)a_args);

        systemMessageProcessor.pump();

        // Make sure we dont see outbound message
        assert(txBuffer.isEmpty());
    }

    // SET ROUTE
    {
        const char* a0 = "setroute";
        const char* a1 = "8";
        const char* a2 = "3";
        const char *a_args[3] = { a0, a1, a2 };

        setRoute(3, (char**)a_args);

        systemMessageProcessor.pump();

        // Make sure we dont see the outbound message
        assert(txBuffer.isEmpty());

        // Check the routing table
        assert(systemRoutingTable.nextHop(8) == 3);
    }

    // SET ROUTE REMOTE
    {
        const char* a0 = "setrouteremote";
        const char* a1 = "7";
        const char* a2 = "1";
        const char* a3 = "4";
        const char *a_args[4] = { a0, a1, a2, a3 };

        sendSetRoute(4, (char**)a_args);

        systemMessageProcessor.pump();

        // Make sure we see the outbound message
        assert(!txBuffer.isEmpty());

        // Pull off the message and examine it
        Packet packet;
        unsigned int packetLen = sizeof(packet);
        txBuffer.pop(0, (void*)&packet, &packetLen);
        assert(packet.header.getType() == TYPE_SETROUTE);
        assert(packet.header.destAddr == 3);
        assert(packet.header.sourceAddr == 1);
        assert(packet.header.getOriginalSourceCall().isEqual("KC1FSZ"));

        // Look at payload
        SetRouteReqPayload payload;
        memcpy((void*)&payload, packet.payload, sizeof(payload));
        assert(payload.targetAddr == 1);
        assert(payload.nextHopAddr == 4);
    }

    // SEND TEXT
    {
        const char* a0 = "text";
        const char* a1 = "7";
        const char* a2 = "Hello World!";
        const char *a_args[3] = { a0, a1, a2 };

        sendText(3, (char**)a_args);

        systemMessageProcessor.pump();

        // Make sure we see the outbound message
        assert(!txBuffer.isEmpty());

        // Pull off the message and examine it
        Packet packet;
        unsigned int packetLen = sizeof(packet);
        txBuffer.pop(0, (void*)&packet, &packetLen);

        assert(packet.header.getType() == TYPE_TEXT);
        assert(packet.header.destAddr == 3);
        assert(packet.header.sourceAddr == 1);
        assert(packetLen == 12 + sizeof(Header));

        // Look at payload
        assert(memcmp(packet.payload, "Hello World!", 12) == 0);
    }
}

int main(int arg, const char** argv) {
    test_CommandProcessor();
}
