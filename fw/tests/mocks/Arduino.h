#ifndef _Arduino_h
#define _Arduino_h

#define F(x) (x)

#define HEX 1

class Stream {
public:

    void print(const char*) {
    }

    void print(uint8_t, int) {
    }

    void print(uint16_t, int) {
    }

    void print(uint16_t) {
    }

    void println(const char*) {
    }

    void println(unsigned int) {
    }

    void println() {
    }
};

#endif

