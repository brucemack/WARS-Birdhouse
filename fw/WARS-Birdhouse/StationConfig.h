#ifndef _StationConfig_h
#define _StationConfig_h

#include "Utils.h"

// Size is approximately 24 bytes
struct StationConfig {
    nodeaddr_t myAddr;
    uint8_t myCall[8];
    uint16_t batteryLimitMv;
    uint8_t debugMode;
    uint32_t securityToken;
};

#endif

