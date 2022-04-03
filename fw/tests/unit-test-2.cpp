#include <Arduino.h>

#include "../WARS-Birdhouse/CircularBuffer.h"
#include "../WARS-Birdhouse/packets.h"
#include "../WARS-Birdhouse/OutboundPacketManager.h"
#include "../WARS-Birdhouse/Clock.h"
#include "../WARS-Birdhouse/Instrumentation.h"
#include "../WARS-Birdhouse/RoutingTable.h"
#include "../WARS-Birdhouse/MessageProcessor.h"
#include "../WARS-Birdhouse/CommandProcessor.h"
#include "../WARS-Birdhouse/Configuration.h"

#include <iostream>
#include <assert.h>
#include <string.h>

using namespace std;

// ===== DUMMY COMPONENTS =============================================

class TestStream : public Stream {
public:

    void print(const char* m) { cout << m; }
    void print(uint16_t m) { cout << m; }
    void println() { cout << endl; }
    void println(const char* m) { cout << m << endl; }
};

class TestClock : public Clock {
public:

    uint32_t time() const {
        return _time;
    };

    void setTime(uint32_t t) {
        _time = t;
    }

    void advanceSeconds(uint32_t seconds) {
        _time += (seconds * 1000);
    }

private:

    uint32_t _time;
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
    uint16_t getBootCount() const { return 1; }
    uint16_t getSleepCount() const { return 1; }
    void restart() { cout << "RESTART" << endl; }
};

class TestRoutingTable : public RoutingTable {
public:
    
    TestRoutingTable() {
        for (unsigned int i = 0; i < 64; i++)
            _table[i] = RoutingTable::NO_ROUTE;
    }

    nodeaddr_t nextHop(nodeaddr_t finalDestAddr) {
        if (finalDestAddr == 0) {
            return 0;
        } else if (finalDestAddr >= 0xfff0) {
            return finalDestAddr;
        } else {
            return _table[finalDestAddr];
        }
    }

    void setRoute(nodeaddr_t target, nodeaddr_t nextHop) {
        _table[target] = nextHop;
    }

private:

    nodeaddr_t _table[64];
};

class TestConfiguration : public Configuration {
public:

    TestConfiguration(nodeaddr_t myAddr, const char* myCall) 
    : _myAddr(myAddr), 
      _myCall(myCall) {
    }

    nodeaddr_t getAddr() const {
        return _myAddr;
    }

    const char* getCall() const {
        return _myCall;
    }

private:

    nodeaddr_t _myAddr;
    const char* _myCall;
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

static TestStream testStream;
Stream& logger = testStream;

TestClock systemClock;

// Node #1
TestConfiguration testConfig(1, "KC1FSZ");
TestInstrumentation testInstrumentation;
static TestRoutingTable testRoutingTable;
CircularBufferImpl<4096> testTxBuffer(0);
CircularBufferImpl<4096> testRxBuffer(2);
static MessageProcessor testMessageProcessor(systemClock, testRxBuffer, testTxBuffer,
    testRoutingTable, testInstrumentation, testConfig,
    10 * 1000, 2 * 1000);

// Exposed base interfaces to the rest of the program
Configuration& config = testConfig;
RoutingTable& routingTable = testRoutingTable;
MessageProcessor& messageProcessor = testMessageProcessor;

void test_CommandProcessor() {
    
    routingTable.setRoute(3, 3);
    routingTable.setRoute(7, 3);

    const char* a0 = "ping";
    const char* a1 = "7";
    const char *args[2] = { a0, a1 };

    sendPing(2, args);

    messageProcessor.pump();

    // Make sure we see the outbound message
    assert(!testTxBuffer.isEmpty());
}

int main(int arg, const char** argv) {
    test_CommandProcessor();
}



