#include "../WARS-Birdhouse/CircularBuffer.h"
#include "../WARS-Birdhouse/packets.h"
#include "../WARS-Birdhouse/OutboundPacket.h"

#include <iostream>
#include <assert.h>
#include <string.h>

using namespace std;

void test_OutboundPacket() {
    OutboundPacket op;
    assert(sizeof(Header) == 36);
    assert(sizeof(op.packet) == 128);
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

