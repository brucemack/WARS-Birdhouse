#include "Utils.h"
#include <cstdlib>
#include <iostream>

nodeaddr_t parseAddr(const char* textAddr) {
    return atoi(textAddr);
}

CallSign::CallSign() {
    _clear();
}

CallSign::CallSign(const char* call) {
    _clear();
    // Read 8, or to the null, whatever comes first
    for (unsigned int i = 0; i < 8 && call[i] != 0; i++)
        _call[i] = call[i];
}

CallSign::CallSign(const CallSign& other) {
    readFrom(other._call);
}

void CallSign::_clear() {
    for (unsigned int i = 0; i < 8; i++)
        _call[i] = 0;
}

void CallSign::writeTo(void* buffer8) const {
    for (unsigned int i = 0; i < 8; i++)
        ((char *)buffer8)[i] = _call[i];
}

void CallSign::readFrom(const void* buffer8) {
    for (unsigned int i = 0; i < 8; i++)
        _call[i] = ((const char *)buffer8)[i];
}

/**
 * @brief Prints the call into the stream.  No padding
 * is used.
 * 
 * @param stream 
 */
void CallSign::printTo(Stream& stream) const {
    for (unsigned int i = 0; i < 8 && _call[i] != 0; i++) {
        stream.print(_call[i]);
    }
}

bool CallSign::isEqual(const char* call) const {
    CallSign other(call);
    return equals(other);
}

bool CallSign::equals(const CallSign& other) const {
    return memcmp(_call, other._call, 8) == 0;
}
