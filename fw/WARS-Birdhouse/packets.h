#ifndef _packets_h
#define _packets_h

#include <string.h>
#include "Utils.h"

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
    TYPE_SAD_REQ       = 5,
    TYPE_SAD_RESP      = 6,
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
    void createAckFor(const Header& rx_packet, 
        const char* myCall, uint16_t myAddr) {
        version = PACKET_VERSION;
        type = TYPE_ACK;
        id = rx_packet.id;
        loadCall(sourceCall, myCall);
        memcpy(finalDestCall, rx_packet.finalDestCall, 8);
        memcpy(originalSourceCall, rx_packet.originalSourceCall, 8);
        destAddr = rx_packet.sourceAddr;
        sourceAddr = myAddr;
        finalDestAddr = rx_packet.finalDestAddr;
        originalSourceAddr = rx_packet.originalSourceAddr;
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
    void createResponseFor(const Header& req_header,
        const char* myCall, nodeaddr_t myAddr,
        uint8_t packetType) {
    }

    bool isAckRequired() {
        return !(type == 1 || type == 2) && (destAddr != BROADCAST_ADDR);
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

    void setOriginalSourceAddr(nodeaddr_t addr) {
        originalSourceAddr = addr;
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

struct SadRespPacket {
  Header header;  
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

struct SetRoutePacket {
  Header header;
  nodeaddr_t targetAddr;
  nodeaddr_t nextHopAddr;  
};

struct GetRoutePacket {
  Header header;
  nodeaddr_t targetAddr;
};

struct GetRouteRespPacket {
  Header header;
  nodeaddr_t targetAddr;
  nodeaddr_t nextHopAddr;  
  uint16_t rxPacketCount;  
  uint16_t txPacketCount;  
};

#endif