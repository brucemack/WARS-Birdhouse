#ifndef _Instrumentation_h
#define _Instrumentation_h

#include <stdint.h>

/**
 * @brief An interface for accessing device-level instrumentation data.
 * This is done through an abstract interface to make unit testing 
 * easier.
 */
class Instrumentation {
public:

    virtual uint16_t getSoftwareVersion() = 0;
    virtual uint16_t getDeviceClass() = 0;
    virtual uint16_t getDeviceRevision() = 0;
    virtual uint16_t getBatteryVoltage() = 0;
    virtual int16_t getTemperature() = 0;
    virtual int16_t getHumidity() = 0;
    virtual uint16_t getPanelVoltage() = 0;
    virtual uint16_t getBootCount() = 0;
    virtual uint16_t getSleepCount() = 0;
    virtual uint16_t getDeviceClass() = 0;
    virtual uint16_t getDeviceRevision() = 0;
};

#endif
