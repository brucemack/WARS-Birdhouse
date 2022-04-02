#ifndef _Clock_h
#define _Clock_h

#include <stdint.h>

class Clock {
public:

    virtual uint32_t time() const = 0;
};

#endif
