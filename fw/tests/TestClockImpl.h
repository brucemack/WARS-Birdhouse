#ifndef _TestClockImpl_h
#define _TestClockImpl_h

#include "../station/Clock.h"

class TestClock : public Clock {
public:

    TestClock() {
        _time = 10 * 1000;
    }

    uint32_t time() const {
        return _time;
    };

    void setTime(uint32_t t) {
        _time = t;
    }

    void advanceSeconds(uint32_t seconds) {
        _time += (seconds * 1000);
    }

private:

    uint32_t _time;
};

#endif
