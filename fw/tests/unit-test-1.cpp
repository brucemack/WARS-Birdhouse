#include <Arduino.h>

#include "../WARS-Birdhouse/CircularBuffer.h"
#include "../WARS-Birdhouse/packets.h"
#include "../WARS-Birdhouse/OutboundPacketManager.h"
#include "../WARS-Birdhouse/Instrumentation.h"
#include "../WARS-Birdhouse/RoutingTable.h"
#include "../WARS-Birdhouse/RoutingTableImpl.h"
#include "../WARS-Birdhouse/MessageProcessor.h"
#include "../WARS-Birdhouse/Configuration.h"
#include "TestClockImpl.h"

#include <iostream>
#include <assert.h>
#include <string.h>

using namespace std;

// Dummy stream for unit test
class TestStream : public Stream {
public:

    void print(const char* m) { cout << m; }
    void print(uint16_t m) { cout << m; }
    void print(uint32_t m) { cout << m; }
    void println() { cout << endl; }
    void println(const char* m) { cout << m << endl; }
};

static TestStream testStream;
Stream& logger = testStream;
static const uint8_t SW_VERSION = 1;

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
    void restart() { cout << "RESTART" << endl; }
    void restartRadio() { cout << "RESTART" << endl; }
    void sleep(uint32_t ms) { cout << "SLEEP " << ms << endl; }
};

// Dummy configuration
class TestConfiguration : public Configuration {
public:

    TestConfiguration(nodeaddr_t myAddr, const char* myCall) 
    : _myAddr(myAddr), 
      _myCall(myCall) {
    }

    nodeaddr_t getAddr() const {
        return _myAddr;
    }

    CallSign getCall() const {
        return _myCall;
    }    
    
    bool checkPasscode(uint32_t) const {
        return true;
    }
    
    void setPasscode(uint32_t) {
    }

    uint16_t getBatteryLimit() const {
        return 3400;
    }


    uint16_t getBootCount() const {
        return 1;
    }

    uint16_t getSleepCount() const {
        return 1;
    }

   void factoryReset() { }

private:

    nodeaddr_t _myAddr;
    const CallSign _myCall;
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
    Preferences nvram1;
    TestConfiguration config1(1, "KC1FSZ");
    TestInstrumentation instrumentation1;
    RoutingTableImpl routingTable1(nvram1);
    routingTable1.setRoute(3, 3);
    routingTable1.setRoute(7, 3);
    CircularBufferImpl<4096> txBuffer1(0);
    CircularBufferImpl<4096> rxBuffer1(2);
    MessageProcessor mp1(clock, rxBuffer1, txBuffer1,
        routingTable1, instrumentation1, config1,
        10 * 1000, 2 * 1000);

    // Node #3 (intermediate)
    Preferences nvram3;
    TestConfiguration config3(3, "W1TKZ");
    TestInstrumentation instrumentation3;
    RoutingTableImpl routingTable3(nvram3);
    routingTable3.setRoute(1, 1);
    routingTable3.setRoute(7, 7);
    CircularBufferImpl<4096> txBuffer3(0);
    CircularBufferImpl<4096> rxBuffer3(2);
    MessageProcessor mp3(clock, rxBuffer3, txBuffer3,
        routingTable3, instrumentation3, config3,
        10 * 1000, 2 * 1000);

    // Node #7 (desktop)
    Preferences nvram7;
    TestConfiguration config7(7, "WA3ITR");
    TestInstrumentation instrumentation7;
    //TestRoutingTable routingTable7;
    RoutingTableImpl routingTable7(nvram7);
    routingTable7.setRoute(1, 3);
    routingTable7.setRoute(3, 3);
    CircularBufferImpl<4096> txBuffer7(0);
    CircularBufferImpl<4096> rxBuffer7(2);
    MessageProcessor mp7(clock, rxBuffer7, txBuffer7,
        routingTable7, instrumentation7, config7,
        10 * 1000, 2 * 1000);

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
        // Inject into node 3
        mp3.transmitIfPossible(packet1, packet1Len);
    }

    // Make things happen on node 3
    assert(txBuffer3.isEmpty());
    mp3.pump();
    assert(!txBuffer3.isEmpty());
    // Move message onto the node 1 RX queue
    movePacket(txBuffer3, rxBuffer1);

    // Make things happen on node 1
    assert(txBuffer1.isEmpty());
    mp1.pump();
    assert(!txBuffer1.isEmpty());

    // Transfer the TX.1->RX.3
    // The first is the ACK, the second is the PING_RESP
    movePacket(txBuffer1, rxBuffer3);
    assert(!txBuffer1.isEmpty());
    movePacket(txBuffer1, rxBuffer3);
    assert(txBuffer1.isEmpty());

    // Make things happen on node 3.
    mp3.pump();

    // Transfer the TX.3->RX.7
    // There should be two messages (the type 1 back to 
    // node 1 and the type 4 being forwarded to 7)
    assert(!txBuffer3.isEmpty());
    movePacket(txBuffer3, rxBuffer7);
    movePacket(txBuffer3, rxBuffer7);
    assert(txBuffer3.isEmpty());

    // Make things happen on node 7.
    mp7.pump();

    // There should be one message (ACK on type 4)
    assert(!txBuffer7.isEmpty());
    // Throw that message away in order to test the retry
    txBuffer7.popAndDiscard();
    assert(txBuffer7.isEmpty());

    // Watch node 3 attempt to retry
    mp3.pump();
    assert(txBuffer3.isEmpty());

    // There should be a re-transmit of the type 4 from 3->7
    // since we got lost the ACK
    clock.advanceSeconds(3);
    mp3.pump();
    // See the re-transmit
    assert(!txBuffer3.isEmpty());
    // Discard it 
    txBuffer3.popAndDiscard();

    // Move forward past the timeout 
    clock.advanceSeconds(10);
    mp3.pump();
    // No more re-transmit
    assert(txBuffer3.isEmpty());
}

void test_OutboundPacket() {
    
    Preferences nvram1;
    TestConfiguration config3(3, "W1TKZ");
    TestClock clock;
    TestInstrumentation instrumentation;
    RoutingTableImpl routingTable1(nvram1);
    CircularBufferImpl<4096> txBuffer(0);
    OutboundPacketManager opm(clock, txBuffer, 10 * 1000, 2 * 1000);
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
    assert(opm.scheduleTransmitIfPossible(packet0, packet0Len));
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
    ackPacket0.header.setupAckFor(packet0.header, config3);
    assert(ackPacket0.header.getId() == 1);
    assert(ackPacket0.header.getSourceAddr() == 3);
    assert(ackPacket0.header.getDestAddr() == 1);
    CallSign call = ackPacket0.header.getSourceCall();
    // Make sure the ACK packet uses his own call
    assert(call.isEqual("W1TKZ"));
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
    assert(opm.scheduleTransmitIfPossible(packet1, packet1Len));
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
    assert(opm.scheduleTransmitIfPossible(packet3, packet3Len));

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
    assert(opm.scheduleTransmitIfPossible(packet4, packet4Len));

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

void test_header() {

    TestConfiguration config1(1, "KC1FSZ");

    // What we are receiving
    Header header;
    header.type = 5;
    header.id = 2;
    header.setSourceAddr(2);
    header.setDestAddr(1);
    header.setOriginalSourceAddr(2);
    header.setFinalDestAddr(1);
    header.setSourceCall(config1.getCall());
    header.setFinalDestCall("XX1XXX");
    header.setOriginalSourceCall("YY1YYY");

    Header ack_header;
    ack_header.setupAckFor(header, config1); 
    // Type
    assert(ack_header.type == 1);
    // Source/dest
    assert(ack_header.sourceAddr == 1);
    assert(ack_header.destAddr == 2);

    CallSign call = header.getSourceCall();
    assert(call.equals(config1.getCall()));
    call = ack_header.getOriginalSourceCall();
    assert(call.isEqual("YY1YYY"));
}

void test_Loopback() {

    TestClock clock;

    // Node #1
    Preferences nvram1;
    TestConfiguration config1(1, "KC1FSZ");
    TestInstrumentation instrumentation1;
    RoutingTableImpl routingTable1(nvram1);
    routingTable1.setRoute(3, 3);
    routingTable1.setRoute(7, 3);
    CircularBufferImpl<4096> txBuffer1(0);
    CircularBufferImpl<4096> rxBuffer1(2);
    MessageProcessor mp1(clock, rxBuffer1, txBuffer1,
        routingTable1, instrumentation1, config1,
        10 * 1000, 2 * 1000);

    // Message to myself
    Packet packet;
    packet.header.type = 5;
    packet.header.id = 2;
    packet.header.setSourceAddr(1);
    packet.header.setDestAddr(1);
    packet.header.setOriginalSourceAddr(1);
    packet.header.setFinalDestAddr(1);
    packet.header.setSourceCall(config1.getCall());
    packet.header.setFinalDestCall(config1.getCall());
    packet.header.setOriginalSourceCall(config1.getCall());

    assert(txBuffer1.isEmpty());
    assert(mp1.getBadRouteCounter() == 0);
    assert(mp1.transmitIfPossible(packet, sizeof(packet)));
    mp1.pump();
    assert(mp1.getBadRouteCounter() == 1);
    assert(txBuffer1.isEmpty());
    assert(rxBuffer1.isEmpty());
    assert(mp1.getPendingCount() == 0);

    // Fix the route
    mp1.resetCounters();
    routingTable1.setRoute(1, 1);
    assert(mp1.transmitIfPossible(packet, sizeof(packet)));
    // Nothing will be pending here - local!
    assert(mp1.getPendingCount() == 0);
    assert(mp1.getBadRouteCounter() == 0);
    assert(!rxBuffer1.isEmpty());
    // This will cause the received packet to be processed
    mp1.pump();

    // We should see this in the RX queue, not in the TX queue
    assert(txBuffer1.isEmpty());
    assert(rxBuffer1.isEmpty());
}

int main(int argc, const char** argv) {
    test_buffer();
    test_header();
    test_OutboundPacket();
    test_MessageProcessor();
    test_Loopback();
}
