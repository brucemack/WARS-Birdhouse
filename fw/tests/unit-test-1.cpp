#include <Arduino.h>

#include "../WARS-Birdhouse/CircularBuffer.h"
#include "../WARS-Birdhouse/packets.h"
#include "../WARS-Birdhouse/OutboundPacketManager.h"
#include "../WARS-Birdhouse/Clock.h"

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

void test_OutboundPacket() {
    
    TestClock clock;
    CircularBufferImpl<4096> txBuffer(0);
    OutboundPacketManager opm(clock, txBuffer);
    assert(opm.getFreeCount() == 8);

    clock.setTime(10 * 1000);

    // Make a packet to send
    Packet packet0;
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
    assert(opm.allocateIfPossible(packet0, packet0Len, true, 12 * 1000));
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
    ackPacket0.header.createAckFor(packet0.header, "W1TKZ", 3);
    assert(ackPacket0.header.getId() == 1);
    assert(ackPacket0.header.getSourceAddr() == 3);
    assert(ackPacket0.header.getDestAddr() == 1);
    char call[9];
    ackPacket0.header.getSourceCall(call);
    // Make sure the ACK packet uses his own call
    assert(strcmp(call, "W1TKZ") == 0);

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
    assert(opm.allocateIfPossible(packet1, packet1Len, true, 30 * 1000));
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
    assert(opm.allocateIfPossible(packet3, packet3Len, false, clock.time() + 10000));

    Packet packet4;
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
    assert(opm.allocateIfPossible(packet4, packet4Len, false, clock.time() + 10000));

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
    ack_header.createAckFor(header, MY_CALL, 1); 
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
}

