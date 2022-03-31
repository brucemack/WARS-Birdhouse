#include "../CircularBuffer.h"

#include <iostream>
#include <assert.h>

using namespace std;

int main(int argc, const char** argv) {

    CircularBuffer<4096> buf(2);

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




    return 0;
}