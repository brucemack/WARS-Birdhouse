#include "Arduino.h"
#include "EEPROM.h"

static uint8_t DUMMY[256];

uint8_t EEPROMClass::read(int addr) {
    return DUMMY[addr];
}

void EEPROMClass::write(int addr, uint8_t data) {
    DUMMY[addr] = data;
}

EEPROMClass EEPROM;

