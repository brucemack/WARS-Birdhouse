#include <Arduino.h>

#include "../WARS-Birdhouse/CircularBuffer.h"
#include "../WARS-Birdhouse/packets.h"
#include "../WARS-Birdhouse/OutboundPacketManager.h"
#include "../WARS-Birdhouse/Clock.h"
#include "../WARS-Birdhouse/Instrumentation.h"
#include "../WARS-Birdhouse/RoutingTable.h"
#include "../WARS-Birdhouse/MessageProcessor.h"

#include <iostream>
#include <assert.h>
#include <string.h>

using namespace std;

// Dummy stream for unit test
class TestStream : public Stream {
public:

    void print(const char* m) { cout << m; }
    void print(uint16_t m) { cout << m; }
    void println() { cout << endl; }
};

static TestStream testStream;
Stream& logger = testStream;
static const uint8_t SW_VERSION = 1;

// Dummy clock for unit test.

class TestClock : public Clock {
public:

    uint32_t time() const {
        return _time;
    };

    void setTime(uint32_t t) {
        _time = t;
    }

private:

    uint32_t _time;
};

// Dummy Instrumentation
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

// Dummy routing table for node 1 (KC1FSZ)

class TestRoutingTable1 : public RoutingTable {
public:
    
    virtual nodeaddr_t nextHop(nodeaddr_t finalDestAddr) {
        if (finalDestAddr == 7) {
            return 3;
        } else {
            return RoutingTable::NO_ROUTE;
        }
    }

    virtual void setRoute(nodeaddr_t target, nodeaddr_t nextHop) {
        cout << "setRoute " << target << "->" << nextHop << endl;
    }
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

void test_MessageProcessor() {

    TestClock clock;

    // Node #1
    TestInstrumentation instrumentation1;
    TestRoutingTable1 routingTable1;
    routingTable1.setRoute(7, 3);
    CircularBufferImpl<4096> txBuffer1(0);
    CircularBufferImpl<4096> rxBuffer1(2);
    MessageProcessor mp1(clock, rxBuffer1, txBuffer1,
        routingTable1, instrumentation1, 1, "KC1FSZ");

    // Node #3 (intermediate)
    TestInstrumentation instrumentation3;
    TestRoutingTable1 routingTable3;
    routingTable3.setRoute(7, 7);
    CircularBufferImpl<4096> txBuffer3(0);
    CircularBufferImpl<4096> rxBuffer3(2);
    MessageProcessor mp3(clock, rxBuffer3, txBuffer3,
        routingTable3, instrumentation3, 3, "W1TKZ");

    // Node #7 (desktop)
    TestInstrumentation instrumentation7;
    TestRoutingTable1 routingTable7;
    CircularBufferImpl<4096> txBuffer7(0);
    CircularBufferImpl<4096> rxBuffer7(2);
    MessageProcessor mp7(clock, rxBuffer7, txBuffer7,
        routingTable7, instrumentation7, 7, "WA3ITR");

    clock.setTime(60 * 1000);

    // Send in a ping message directly to node 1
    {
        // Make a packet to send
        Packet packet1;
        packet1.header.setType(TYPE_PING_REQ);
        packet1.header.setId(2);
        packet1.header.setSourceAddr(3);
        packet1.header.setDestAddr(1);
        packet1.header.setOriginalSourceAddr(7);
        packet1.header.setFinalDestAddr(1);
        packet1.header.setSourceCall("W1TKZ");
        packet1.header.setOriginalSourceCall("WA3ITR");
        packet1.header.setFinalDestCall("KC1FSZ");
        unsigned int packet1Len = sizeof(Header);
        int16_t rssi = 100;
        rxBuffer1.push(&rssi, (const char*)&packet1, packet1Len);
    }

    // Make things happen on node 1
    assert(txBuffer1.isEmpty());
    mp1.pump();
    assert(!txBuffer1.isEmpty());

    // Transfer the TX.1->TX.3
    movePacket(txBuffer1, rxBuffer3);

    // Make things happen on node 3
    mp3.pump();
    assert(!txBuffer3.isEmpty());
}

void test_OutboundPacket() {
    
    TestClock clock;
    TestInstrumentation instrumentation;
    TestRoutingTable1 routingTable1;
    CircularBufferImpl<4096> txBuffer(0);
    OutboundPacketManager opm(clock, txBuffer);
    assert(opm.getFreeCount() == 8);

    clock.setTime(10 * 1000);

    // Make a packet to send
    Packet packet0;
    packet0.header.setType(TYPE_PING_REQ);
    packet0.header.setId(1);
    packet0.header.setSourceAddr(1);
    packet0.header.setDestAddr(3);
    packet0.header.setOriginalSourceAddr(1);
    packet0.header.setFinalDestAddr(7);
    packet0.header.setSourceCall("KC1FSZ");
    packet0.header.setOriginalSourceCall("KC1FSZ");
    packet0.header.setFinalDestCall("WA3ITR");
    packet0.payload[0] = 'A';
    unsigned int packet0Len = sizeof(Header) + 1;

    // Queue something 
    assert(opm.allocateIfPossible(packet0, packet0Len, 12 * 1000));
    assert(opm.getFreeCount() == 7);

    // Move things
    opm.pump();

    // See that the transmission went onto the queue
    assert(!txBuffer.isEmpty());
    txBuffer.popAndDiscard();
    assert(txBuffer.isEmpty());

    // Validate that we still are holding a slot (not ACKed)
    assert(opm.getFreeCount() == 7);

    // Create an ACK that we can use 
    Packet ackPacket0;
    ackPacket0.header.setupAckFor(packet0.header, "W1TKZ", 3);
    assert(ackPacket0.header.getId() == 1);
    assert(ackPacket0.header.getSourceAddr() == 3);
    assert(ackPacket0.header.getDestAddr() == 1);
    char call[9];
    ackPacket0.header.getSourceCall(call);
    // Make sure the ACK packet uses his own call
    assert(strcmp(call, "W1TKZ") == 0);
    // Check type
    assert(ackPacket0.header.getType() == TYPE_ACK);

    // Show the ACK to the OPM
    opm.processAck(ackPacket0);

    // Move things
    opm.pump();

    // Validate that the slot is freed now
    assert(opm.getFreeCount() == 8);

    // ==============================================================
    // Validate Re-Transmit And Timeout

    clock.setTime(20 * 1000);

    // Make a packet to send
    Packet packet1;
    packet1.header.setType(TYPE_PING_REQ);
    packet1.header.setId(2);
    packet1.header.setSourceAddr(1);
    packet1.header.setDestAddr(3);
    packet1.header.setOriginalSourceAddr(1);
    packet1.header.setFinalDestAddr(7);
    packet1.header.setSourceCall("KC1FSZ");
    packet1.header.setOriginalSourceCall("KC1FSZ");
    packet1.header.setFinalDestCall("WA3ITR");
    packet1.payload[0] = 'A';
    unsigned int packet1Len = sizeof(Header) + 1;

    // Nothing in the TX queue
    assert(txBuffer.isEmpty());

    // Queue something for delivery 
    assert(opm.allocateIfPossible(packet1, packet1Len, 30 * 1000));
    assert(opm.getFreeCount() == 7);

    // Move things
    opm.pump();

    // See the message on the queue, and pop it (should have been the only one)
    assert(!txBuffer.isEmpty());
    txBuffer.popAndDiscard();
    assert(txBuffer.isEmpty());

    // Validate that we still are holding a slot (not ACKed yet)
    assert(opm.getFreeCount() == 7);

    // Move time forward (three seconds) so that we can see the 
    // re-transmit.
    clock.setTime(23 * 1000);

    // Move things
    opm.pump();

    // Pull the message off the TX queue and have a look
    assert(!txBuffer.isEmpty());
    Packet packet2;
    unsigned int packet2Len = sizeof(Packet);
    txBuffer.pop(0, &packet2, &packet2Len);
    assert(packet2Len == sizeof(Header) + 1);
    assert(packet2.payload[0] == 'A');
    assert(packet2.header.getDestAddr() == 3);

    // Move time forward so that we can see the timeout
    clock.setTime(31 * 1000);

    // Move things
    opm.pump();

    // Validate that we still are holding a slot (not ACKed yet)
    assert(opm.getFreeCount() == 8);

    // ========================================================
    // Queue Two Messages That Don't Need ACK

    clock.setTime(40 * 1000);
    
    Packet packet3;
    packet3.header.setType(TYPE_ACK);
    packet3.header.setId(3);
    packet3.header.setSourceAddr(1);
    packet3.header.setDestAddr(3);
    packet3.header.setOriginalSourceAddr(1);
    packet3.header.setFinalDestAddr(7);
    packet3.header.setSourceCall("KC1FSZ");
    packet3.header.setOriginalSourceCall("KC1FSZ");
    packet3.header.setFinalDestCall("WA3ITR");
    packet3.payload[0] = '3';
    unsigned int packet3Len = sizeof(Header) + 1;
    // Queue something for delivery 
    assert(opm.allocateIfPossible(packet3, packet3Len, clock.time() + 10000));

    Packet packet4;
    packet4.header.setType(TYPE_ACK);
    packet4.header.setId(4);
    packet4.header.setSourceAddr(1);
    packet4.header.setDestAddr(3);
    packet4.header.setOriginalSourceAddr(1);
    packet4.header.setFinalDestAddr(7);
    packet4.header.setSourceCall("KC1FSZ");
    packet4.header.setOriginalSourceCall("KC1FSZ");
    packet4.header.setFinalDestCall("WA3ITR");
    packet4.payload[0] = '4';
    unsigned int packet4Len = sizeof(Header) + 1;
    // Queue something for delivery 
    assert(opm.allocateIfPossible(packet4, packet4Len, clock.time() + 10000));

    assert(txBuffer.isEmpty());

    // Move things
    opm.pump();

    // Validate that all slots are free (no ACKs needed)
    assert(opm.getFreeCount() == 8);

    // See that both messages went onto queue
    assert(!txBuffer.isEmpty());
    txBuffer.popAndDiscard();
    txBuffer.popAndDiscard();
    assert(txBuffer.isEmpty());
}

void test_buffer() {

    CircularBufferImpl<4096> buf(2);

    assert(buf.isEmpty());

    // Put some data on the queue
    {
        uint16_t oobInt = 18;
        uint8_t text[8] = { "TEST   " };
        buf.push((const uint8_t*)&oobInt, text, 8);
        assert(!buf.isEmpty());
    }

    // Pop it off
    {
        uint16_t oobInt = 0;
        uint8_t text[8];
        unsigned int textLen = 8;
        buf.pop((uint8_t*)&oobInt, text, &textLen);
        assert(buf.isEmpty());
        assert(textLen == 8);
        assert(oobInt == 18);
    }

    // Put some data on the queue more than once
    {
        uint16_t oobInt = 8;
        uint8_t text1[8] = { "1TEST  " };
        buf.push((const uint8_t*)&oobInt, text1, 8);
        oobInt = 9;
        uint8_t text2[8] = { "2TEST  " };
        buf.push((const uint8_t*)&oobInt, text2, 8);
        assert(!buf.isEmpty());
    }
    // Pop it off
    {
        uint16_t oobInt = 0;
        uint8_t text[8];
        unsigned int textLen = 8;
        buf.pop((uint8_t*)&oobInt, text, &textLen);
        assert(!buf.isEmpty());
        assert(textLen == 8);
        assert(oobInt == 8);
        assert(text[0] == '1');

        textLen = 8;
        buf.pop((uint8_t*)&oobInt, text, &textLen);
        assert(buf.isEmpty());
        assert(textLen == 8);
        assert(oobInt == 9);
        assert(text[0] == '2');
    }

    // Test the overflow situation
    CircularBufferImpl<16> buf2(2);
    {
        uint16_t oobInt = 1;
        uint8_t text1[11] = { "0123456789" };
        bool good = buf2.push((const uint8_t*)&oobInt, text1, 10);
        assert(good);
        // Buffer will be full
        good = buf2.push((const uint8_t*)&oobInt, text1, 10);
        assert(!good);
        // Pop off 
        oobInt = 0;
        unsigned int textLen = 10;
        buf2.pop((uint8_t*)&oobInt, text1, &textLen);
        assert(oobInt == 1);
        assert(buf2.isEmpty());
        assert(textLen == 10);
    }

    // Test the wrap situation
    CircularBufferImpl<20> buf3(2);
    {
        uint16_t oobInt = 1;
        uint8_t text1[5] = { "0123" };
        bool good = buf3.push((const uint8_t*)&oobInt, text1, 4);
        assert(good);
        good = buf3.push((const uint8_t*)&oobInt, text1, 4);
        assert(good);
        // Buffer will be full
        good = buf3.push((const uint8_t*)&oobInt, text1, 4);
        assert(!good);
        // Pop off 
        oobInt = 0;
        unsigned int textLen = 4;
        buf3.pop((uint8_t*)&oobInt, text1, &textLen);
        assert(oobInt == 1);
        assert(!buf3.isEmpty());
        assert(textLen == 4);
        // Push again (will wrap)
        oobInt = 6;
        uint8_t text2[5] = { "4567" };
        good = buf3.push((const uint8_t*)&oobInt, text2, 4);
        assert(good);
        // Discard
        buf3.popAndDiscard();
        // read the other one
        assert(!buf3.isEmpty());
        oobInt = 0;
        textLen = 4;
        buf3.pop((uint8_t*)&oobInt, text1, &textLen);
        assert(oobInt == 6);
        assert(buf3.isEmpty());
        assert(textLen == 4);
        assert(text1[0] == '4');
    }

    // ZERO LENGTH OOB
    CircularBufferImpl<128> buf4(0);
    assert(buf4.isEmpty());

    // Put some data on the queue
    {
        uint8_t text[8] = { "TEST   " };
        buf4.push((const uint8_t*)0, text, 8);
        assert(!buf4.isEmpty());
    }

    // Pop it off
    {
        uint8_t text[8];
        unsigned int textLen = 8;
        buf4.pop(0, text, &textLen);
        assert(buf4.isEmpty());
        assert(textLen == 8);
        assert(text[0] == 'T');
    }
}

static const char* MY_CALL = "KC1FSZ";

void test_header() {

    // What we are receiving
    Header header;
    header.type = 5;
    header.id = 2;
    header.setSourceAddr(2);
    header.setDestAddr(1);
    header.setOriginalSourceAddr(2);
    header.setFinalDestAddr(1);
    header.setSourceCall(MY_CALL);
    header.setFinalDestCall("XX1XXX");
    header.setOriginalSourceCall("YY1YYY");

    Header ack_header;
    ack_header.setupAckFor(header, MY_CALL, 1); 
    // Type
    assert(ack_header.type == 1);
    // Source/dest
    assert(ack_header.sourceAddr == 1);
    assert(ack_header.destAddr == 2);
    char callArea[9];
    ack_header.getSourceCall(callArea);
    assert(strcmp(callArea, MY_CALL) == 0);
    ack_header.getOriginalSourceCall(callArea);
    assert(strcmp(callArea, "YY1YYY") == 0);
}

int main(int argc, const char** argv) {
    test_buffer();
    test_header();
    test_OutboundPacket();
    test_MessageProcessor();
}

