#include "Arduino.h"
#include "EEPROM.h"
#include "Preferences.h"

static uint8_t DUMMY[256];

uint8_t EEPROMClass::read(unsigned int addr) {
    return DUMMY[addr];
}

void EEPROMClass::write(unsigned int addr, uint8_t data) {
    DUMMY[addr] = data;
}

EEPROMClass EEPROM;

uint32_t millis() {
    return 1000;
}

// ----- Pref

void Preferences::begin(const char*) {
}

void Preferences::getBytes(const char*, void*, unsigned int len) {
}

void Preferences::putBytes(const char*, const void*, unsigned int len) {
}
