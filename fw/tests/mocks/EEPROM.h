#ifndef _EEPROM_h
#define _EEPROM_h

#include <stdint.h>

class EEPROMClass {
public:

    uint8_t read(int addr);
    void write(int addr, uint8_t data);
};

extern EEPROMClass EEPROM;

#endif
