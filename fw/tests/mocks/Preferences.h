#ifndef _Preferences_h
#define _Preferences_h

#include <stdint.h>

class Preferences {
public:

    void begin(const char*);
    void remove(const char*);

    void getBytes(const char*, void*, unsigned int len);
    void putBytes(const char*, const void*, unsigned int len);
};

#endif
