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
#ifndef _packets_h
#define _packets_h

#include <string.h>

#include "Utils.h"
#include "Configuration.h"

static const uint8_t PACKET_VERSION = 2;
static const nodeaddr_t BROADCAST_ADDR = 0xffff;

// The top bit indicates whether an ACK is needed
enum MessageType {
    TYPE_UNUSED        = 0,
    // This message is used when a delivery acknowledgment is needed
    TYPE_ACK           = 1,
    // Used to spontaneously 
    TYPE_STATION_ID    = 2,
    // The ping message will be acknowledged at every step along the way
    TYPE_PING_REQ      = 3,
    // Response is acknowledged as well
    TYPE_PING_RESP     = 4,
    // Station engineering data
    TYPE_GETSED_REQ    = 5,
    TYPE_GETSED_RESP   = 6,
    TYPE_SETROUTE      = 10,
    TYPE_GETROUTE_REQ  = 11,
    TYPE_GETROUTE_RESP = 12,
    TYPE_RESET         = 15,
    // Routine text traffic
    TYPE_TEXT          = 32,
    TYPE_ALERT         = 36
};

struct Header {

    uint8_t version;
    uint8_t type;
    uint16_t id;
    char sourceCall[8];
    char finalDestCall[8];
    char originalSourceCall[8];
    nodeaddr_t destAddr;
    nodeaddr_t sourceAddr;
    // Used for multi-hop communication.  This is the node that is 
    // the ultimate destination of a message.
    nodeaddr_t finalDestAddr;
    // Used for multi-hop communication.  This is the node that 
    // originated the message.
    nodeaddr_t originalSourceAddr;

    Header() 
    : version(PACKET_VERSION) 
    {
    }

    /**
     * @brief Constructor that initializes from a binary buffer
     * (most likely one that we just took from the radio).
     * 
     * @param rx_buf 
     */
    Header(const char* rx_buf) {
        memcpy(this, rx_buf, sizeof(Header));
    }

    /**
     * @brief Create a ACK message for a message that we received.
     * 
     * @param rx_packet 
     * @param myCall 
     * @param myAddr 
     */
    void setupAckFor(const Header& rx_packet, const Configuration& config) {
        version = PACKET_VERSION;
        type = TYPE_ACK;
        id = rx_packet.id;
        // Address stuff
        sourceAddr = config.getAddr();
        destAddr = rx_packet.sourceAddr;
        // Echo back the original stuff
        originalSourceAddr = rx_packet.originalSourceAddr;
        finalDestAddr = rx_packet.finalDestAddr;
        loadCall(sourceCall, config.getCall());
        // Echo back the original stuff
        memcpy(finalDestCall, rx_packet.finalDestCall, 8);
        memcpy(originalSourceCall, rx_packet.originalSourceCall, 8);
    }

    /**
     * @brief Fills in this header with the information needed to 
     * generate a response to the request header.  Deals with 
     * updating the callsigns and addresses.
     * 
     * @param req_header 
     * @param myCall 
     * @param myAddr 
     * @param packetType 
     */
    void setupResponseFor(const Header& reqHeader, const Configuration& config,
        uint8_t respType, uint16_t respId, nodeaddr_t respDestAddr) {
        
        version = PACKET_VERSION;
        type = respType;
        id = respId;
        // Address stuff
        sourceAddr = config.getAddr();
        destAddr = respDestAddr;
        originalSourceAddr = config.getAddr();
        // SWAP 
        finalDestAddr = reqHeader.originalSourceAddr;
        // Call stuff
        loadCall(sourceCall, config.getCall());
        loadCall(originalSourceCall, config.getCall());
        // SWAP
        memcpy(finalDestCall, reqHeader.originalSourceCall, 8);
    }

    bool isAck() const {
        return (type == TYPE_ACK);
    }

    bool isAckRequired() const {
        return !(type == TYPE_ACK || type == TYPE_STATION_ID) && 
          (destAddr != BROADCAST_ADDR);
    }

    bool isResponseRequired() const {
        return (type == TYPE_PING_REQ || type == TYPE_GETSED_REQ ||
          type == TYPE_GETROUTE_REQ);
    }

    uint8_t getType() const {
        return type;
    }

    void setType(uint8_t t) {
        type = t;
    }

    uint16_t getId() const {
        return id;
    }

    void setId(uint16_t i) {
        id = i;
    }

    nodeaddr_t getSourceAddr() const {
        return sourceAddr;
    }

    void setSourceAddr(nodeaddr_t addr) {
        sourceAddr = addr;
    }

    nodeaddr_t getDestAddr() const {
        return destAddr;
    }

    void setDestAddr(nodeaddr_t addr) {
        destAddr = addr;
    }

    nodeaddr_t getOriginalSourceAddr() const {
        return originalSourceAddr; 
    }

    void setOriginalSourceAddr(nodeaddr_t addr) {
        originalSourceAddr = addr;
    }

    nodeaddr_t getFinalDestAddr() const {
        return finalDestAddr;
    }

    void setFinalDestAddr(nodeaddr_t addr) {
        finalDestAddr = addr;
    }

    void setSourceCall(const char* call) {
        loadCall(sourceCall, call);
    }

    void getSourceCall(char* call) {
        unloadCall(call, sourceCall);
    }

    void setOriginalSourceCall(const char* call) {
        loadCall(originalSourceCall, call);
    }

    void getOriginalSourceCall(char* call) {
        unloadCall(call, originalSourceCall);
    }

    void setFinalDestCall(const char* call) {
        loadCall(finalDestCall, call);
    }

    bool isRelevant(nodeaddr_t myAddr) const {
        return destAddr == myAddr || destAddr == BROADCAST_ADDR;
    }

    /**
     * @brief Copies a null-terminated string into a non-null
     * terminated buffer which is space-padded.
     * 
     * @param dest 
     * @param sourceWithNull 
     */
    static void loadCall(char* dest, const char* sourceWithNull) {
        // Blank out target
        for (unsigned int i = 0; i < 8; i++)
            dest[i] = ' ';
        // Copy until we see a null
        for (unsigned int i = 0; i < 8 && sourceWithNull[i] != 0; i++)
            dest[i] = sourceWithNull[i];
    }

    static void unloadCall(char* destWithNull, const char* source) {
        // Null out target
        for (unsigned int i = 0; i < 9; i++)
            destWithNull[i] = 0;
        // Copy until we see a pad space
        for (unsigned int i = 0; i < 8 && source[i] != ' '; i++) 
            destWithNull[i] = source[i];
    }
};

static const unsigned int MAX_PAYLOAD_SIZE = 128 - sizeof(Header);

struct Packet {
    Header header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
};

struct SadRespPayload {
  uint16_t version;
  uint16_t batteryMv;
  uint16_t panelMv;
  uint32_t uptimeSeconds;
  uint32_t time;
  uint16_t bootCount;
  uint16_t sleepCount;
  uint16_t rxPacketCount;
  uint16_t routeErrorCount;
  int16_t temp;
  int16_t humidity;
  uint16_t deviceClass;
  uint16_t deviceRevision;
  uint16_t wrongNodeRxCount;
};

struct SetRouteReqPayload {
  uint32_t passcode;
  nodeaddr_t targetAddr;
  nodeaddr_t nextHopAddr;  
};

struct GetRouteReqPayload {
  nodeaddr_t targetAddr;
};

struct GetRouteRespPayload {
  nodeaddr_t targetAddr;
  nodeaddr_t nextHopAddr;  
  uint16_t rxPacketCount;  
  uint16_t txPacketCount;  
};

struct ResetReqPayload {
  uint32_t passcode;
};

#endif
