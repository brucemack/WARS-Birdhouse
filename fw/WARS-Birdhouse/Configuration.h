#ifndef _Configuration_h
#define _Configuration_h

#include "Utils.h"

class Configuration {
public:

    virtual const char* getCall() const = 0;
    virtual nodeaddr_t getAddr() const = 0;

    virtual void setAddr(nodeaddr_t a) { }
    virtual void setCall(const char* call) { }
};

#endif
