#include "OutboundPacket.h"

#include <assert.h>
#include <iostream>

#define RETRY_INTERVAL_SECONDS 2

using namespace std;

OutboundPacket::OutboundPacket()
: _isUsed(false) {
}

void OutboundPacket::transmitIfReady(CircularBuffer& tx_buffer, int32_t time) {
    if (_isUsed) {
        // Check for timeouts.  If we hit a timeout then reset the packet
        if (time > _timeoutTime) {
            cout << "TIMEOUT " << packet.header.id << endl;
            _reset();
        } 
        // If there is no timeout:
        else {
            // Check to see if it is time to send/resend the packet
            if ((time - _lastTransmitTime) > RETRY_INTERVAL_SECONDS * 1000) {
                cout << "SENDING " << packet.header.id << endl;
                bool good = tx_buffer.push(0, &packet, totalLen);
                if (good) {
                    if (_isAckRequired) {
                        // If an acknowledgement is required then record the 
                        // necessary information to manage the retries.
                        _lastTransmitTime = time;
                    } 
                    else {
                        _reset();
                    }    
                }
            }
        }
    }
}

void OutboundPacket::_reset() {
    _isUsed = false;
    _lastTransmitTime = 0;
    totalLen = 0;
}