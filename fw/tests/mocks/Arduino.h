#ifndef _Arduino_h
#define _Arduino_h

#include <stdint.h>

#define F(x) (x)

#define HEX 1

class Stream {
public:

    virtual void print(char) {}
    virtual void print(const char* m) {}
    virtual void print(uint8_t, int) {}
    virtual void print(uint16_t, int) {}
    virtual void print(uint16_t) {};
    virtual void print(uint32_t) {};
    virtual void print(int) {};
    virtual void println(const char*) {};
    virtual void println(unsigned int) {};
    virtual void println() {};
};

#endif

