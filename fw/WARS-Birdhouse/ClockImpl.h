#ifndef _ClockImpl_h
#define _ClockImpl_h

#ifdef ARDUINO
#include <Arduino.h>
#endif


#include "Clock.h"

class ClockImpl : public Clock {
public:

    uint32_t time() const {
        return millis();
    };
};

#endif
