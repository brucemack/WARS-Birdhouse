#include "../CircularBuffer.h"

#include <iostream>
#include <assert.h>

using namespace std;

int main(int argc, const char** argv) {

    CircularBuffer<4096> buf(2);

    assert(buf.isEmpty());

    uint16_t oobInt;
    uint8_t text[8] = { "TEST   " };
    buf.push((const uint8_t*)&oobInt, text, 8);
    assert(!buf.isEmpty());


    return 0;
}