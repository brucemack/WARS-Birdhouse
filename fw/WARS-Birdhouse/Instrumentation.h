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

    virtual uint16_t getSoftwareVersion() const = 0;
    virtual uint16_t getDeviceClass() const = 0;
    virtual uint16_t getDeviceRevision() const = 0;
    virtual uint16_t getBatteryVoltage() const = 0;
    virtual uint16_t getPanelVoltage() const = 0;
    virtual int16_t getTemperature() const = 0;
    virtual int16_t getHumidity() const = 0;
    virtual uint16_t getBootCount() const = 0;
    virtual uint16_t getSleepCount() const = 0;
    
    virtual void restart() = 0;
};

#endif
