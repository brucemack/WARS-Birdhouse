#ifndef _packets_h
#define _packets_h

static const uint8_t PACKET_VERSION = 2;
static const uint16_t BROADCAST_ADDR = 0xffff;

// The top bit indicates whether an ACK is needed
enum MessageType {
    TYPE_UNUSED        = 0
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
    uint8_t sourceCall[8];
    uint8_t finalDestCall[8];
    uint8_t originalSourceCall[8];
    uint16_t destAddr;
    uint16_t sourceAddr;
    // Used for multi-hop communication.  This is the node that is 
    // the ultimate destination of a message.
    uint16_t finalDestAddr;
    // Used for multi-hop communication.  This is the node that 
    // originated the message.
    uint16_t originalSourceAddr;

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
    Header(const uint8_t* rx_buf) {
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
        const uint8_t* myCall, uint16_t myAddr) {
        version = PACKET_VERSION;
        type = MessageType::TYPE_ACK;
        id = rx_packet.id;
        memcpy(sourceCall, myCall, 8);
        memcpy(finalDestCall, rx_packet.finalDestCall, 8);
        memcpy(originalSourceCall, rx_packet.originalSourceCall, 8);
        destAddr = rx_packet.sourceAddr;
        sourceAddr = myAddr;
        finalDestAddr = rx_packet.finalDestAddr;
        originalSourceAddr = rx_packet.originalSourceAddr;
    }
    
    bool isAckRequired() {
        return !(type == 1 || type == 2) && (destAddr != BROADCAST_ADDR);
    }

    void setSourceAddr(uint16_t addr) {
        sourceAddr = addr;
    }

    void setDestAddr(uint16_t addr) {
        destAddr = addr;
    }

    void setOriginalSourceAddr(uint16_t addr) {
        originalSourceAddr = addr;
    }

    void setFinalDestAddr(uint16_t addr) {
        finalDestAddr = addr;
    }

    void setSourceCall(const uint8_t* call) {
        memcpy(sourceCall, call);
    }

    void setOriginalSourceCall(const uint8_t* call) {
        memcpy(originalSourceCall, call);
    }

    bool isRelevant(uint16_t myAddr) const {
        return destAddr == myAddr || destAddr == BROADCAST_ADDR;
    }
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
  uint16_t targetAddr;
  uint16_t nextHopAddr;  
};

struct GetRoutePacket {
  Header header;
  uint16_t targetAddr;
};

struct GetRouteRespPacket {
  Header header;
  uint16_t targetAddr;
  uint16_t nextHopAddr;  
  uint16_t rxPacketCount;  
  uint16_t txPacketCount;  
};

#endif