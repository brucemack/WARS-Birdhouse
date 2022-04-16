/* 
 * LoRa Birdhouse Mesh Network Project
 * Wellesley Amateur Radio Society
 * 
 * Copyright (C) 2022 Bruce MacKinnon
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef _Utils_h
#define _Utils_h

#include <stdint.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif

typedef uint16_t nodeaddr_t;

nodeaddr_t parseAddr(const char* textAddr);

class CallSign {
public:

    CallSign();

    /**
     * @brief Construct a new Call Sign object from a null-terminated
     * string.
     */
    CallSign(const char* call);
    
    CallSign(const CallSign& other);

    bool isValid() const;

    /**
     * @brief Writes the call sign into an 8 byte buffer,
     * padding with spaces if necessary to full.  There
     * is no null-termination provided.
     */
    void writeTo(void* buffer8) const;

    /**
     * @brief Reads the call sign from an 8 byte buffer.
     * There is no null-termination. 
     */
    void readFrom(const void* buffer8);

    /**
     * @brief Prints the call into the stream.  No padding
     * is used.
     * 
     * @param stream 
     */
    void printTo(Stream& stream) const;

    /**
     * @brief Compares the callsign to a null-terminated
     * string.
     * 
     * @param call 
     * @return true 
     * @return false 
     */
    bool isEqual(const char* call) const;

    bool equals(const CallSign& other) const;

private:

    void _clear();

    char _call[8];
};

#endif
