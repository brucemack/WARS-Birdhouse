#ifndef _Configuration_h
#define _Configuration_h

#include "Utils.h"

class Configuration {
public:

    virtual const char* getCall() const = 0;
    virtual nodeaddr_t getAddr() const = 0;
};

#endif
