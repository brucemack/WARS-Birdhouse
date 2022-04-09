#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>
#include "Utils.h"
#include "RoutingTable.h"
#include "MessageProcessor.h"
#include "CommandProcessor.h"
#include "Configuration.h"

#include <iostream>

static auto msg_arg_error = F("ERR: Argument error");
static auto msg_no_route = F("ERR: No route available");
static auto msg_bad_address = F("ERR: Bad address");
static auto msg_bad_message = F("ERR: Bad message");

extern Stream& logger;
extern Configuration& systemConfig;
extern Instrumentation& systemInstrumentation;
extern RoutingTable& systemRoutingTable;
extern MessageProcessor& systemMessageProcessor;

int sendPing(int argc, const char** argv) { 
 
    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDestAddr = parseAddr(argv[1]);
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDestAddr);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(F("ERR: No route"));
        return -1;
    }

    // Make a ping request
    Packet packet;
    packet.header.setType(TYPE_PING_REQ);
    packet.header.setId(systemMessageProcessor.getUniqueId());
    packet.header.setSourceAddr(systemConfig.getAddr());
    packet.header.setDestAddr(nextHop);
    packet.header.setOriginalSourceAddr(systemConfig.getAddr());
    packet.header.setFinalDestAddr(finalDestAddr);
    packet.header.setSourceCall(systemConfig.getCall());
    packet.header.setOriginalSourceCall(systemConfig.getCall());
    unsigned int packetLen = sizeof(Header);
    // Send it
    bool good = systemMessageProcessor.transmitIfPossible(packet, packetLen);
    if (!good) {
      logger.println("ERR: TX full");
      return -1;
    } else {
      return 0;
    }
}

int sendReset(int argc, char **argv) { 

    if (argc != 3) {
      logger.println(msg_arg_error);
      return -1;
    }

    nodeaddr_t finalDestAddr = parseAddr(argv[1]);
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDestAddr);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(F("ERR: No route"));
        return -1;
    }

    ResetReqPayload resetReqPayload;
    resetReqPayload.passcode = atoi(argv[2]);

    Packet packet;
    packet.header.setType(TYPE_RESET);
    packet.header.setId(systemMessageProcessor.getUniqueId());
    packet.header.setSourceAddr(systemConfig.getAddr());
    packet.header.setDestAddr(nextHop);
    packet.header.setOriginalSourceAddr(systemConfig.getAddr());
    packet.header.setFinalDestAddr(finalDestAddr);
    packet.header.setSourceCall(systemConfig.getCall());
    packet.header.setOriginalSourceCall(systemConfig.getCall());
    unsigned int packetLen = sizeof(Header) + sizeof(ResetReqPayload);
    
    // Send it
    bool good = systemMessageProcessor.transmitIfPossible(packet, packetLen);
    if (!good) {
      logger.println("ERR: TX full");
      return -1;
    } else {
      return 0;
    }
}

int boot(int argc, char **argv) { 
    logger.println("INF: Rebooting");
    systemInstrumentation.restart();
    return 0;
}

int bootRadio(int argc, char **argv) { 
    logger.println("INF: Rebooting radio");
    systemInstrumentation.restartRadio();
    return 0;
}

int info(int argc, char **argv) { 
    logger.print(F("{ \"node\": "));
    logger.print(systemConfig.getAddr());
    logger.print(F(", \"version\": "));
    logger.print(systemInstrumentation.getSoftwareVersion());
    logger.print(F("\", \"blimit\": "));
    logger.print(systemConfig.getBatteryLimit());
    logger.print(F(", \"batteryMv\": "));
    logger.print(systemInstrumentation.getBatteryVoltage());
    logger.print(F(", \"panelMv\": "));
    logger.print(systemInstrumentation.getPanelVoltage());
    logger.print(F(", \"bootCount\": "));
    logger.print(systemInstrumentation.getBootCount());
    logger.print(F(", \"sleepCount\": "));
    logger.print(systemInstrumentation.getSleepCount());
    logger.print(F(", \"routes\": ["));
    bool first = true;
    for (int i = 0; i < 256; i++) {
      if (systemRoutingTable.nextHop(i) != RoutingTable::NO_ROUTE) {
        if (!first) 
          logger.print(", ");
        first = false;
        logger.print("[");
        logger.print(i);
        logger.print(", ");
        logger.print(systemRoutingTable.nextHop(i));
        logger.print("]");
      }
    }
    logger.print("]");
    logger.println(F("}"));
    return 0;
}

// Used for testing the watch dog 
static int sleep(int argc, char **argv) { 

  if (argc != 2) {
    logger.println(msg_arg_error);
    return -1;
  }

  uint16_t seconds = atoi(argv[1]);
  logger.println("INF: Sleeping ...");
  systemInstrumentation.sleep(seconds * 1000);
  logger.println("INF: Done");
  return 0;
}

int setAddr(int argc, char **argv) { 

  if (argc != 2) {
    logger.println(msg_arg_error);
    return -1;
  }

  systemConfig.setAddr(atoi(argv[1]));
}

int setRoute(int argc, char **argv) { 

    if (argc != 3) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t t = parseAddr(argv[1]);
    if (t == 0) {
        logger.println(msg_bad_address);
        return -1;
    }
    nodeaddr_t r = parseAddr(argv[2]);
    if (r == 0) {
        logger.println(msg_bad_address);
        return -1;
    }

    systemRoutingTable.setRoute(t, r);
}

int clearRoutes(int argc, char **argv) { 
    systemRoutingTable.clearRoutes();
}

int setBatteryLimit(int argc, char **argv) { 

    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    systemConfig.setBatteryLimit(atoi(argv[1]));
}

int doPrint(int argc, char **argv) { 

    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    logger.print("[");
    logger.print(argv[1]);
    logger.println("]");
}

/**
 * Used to put a comment into the console log
 */
int doRem(int argc, char **argv) { 
    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }
}

/** This function handles a request to send a text message to another node
 *  in the network.  There is no guarantee that the message will actually
 *  get to the destination.  
 *  
 *  Two arguments:
 *  
 *  1: The destination node number
 *  2: The text of the message, limited to 80 characters.
 */
/*
int sendText(int argc, char **argv) { 
 
  if (argc != 3) {
    shell.println(msg_arg_error);
    return -1;
  }
  
  uint8_t target = atoi(argv[1]);
  if (target == 0 || target == 255) {
      shell.println(F("ERR: Bad address"));
      return -1;
  }

  // Since we might not be directly connected to the 
  // destination node we use the routing table to
  // figure out the "next hop."
  uint8_t nextHop = Routes[target];
  if (nextHop == 0) {
    shell.println(F("ERR: No route"));
    return -1;
  }

  uint16_t textLen = strlen(argv[2]);
  if (textLen > 80) {
    shell.println(F("ERR: Length error"));
    return -1;
  }

  // Assign a unique ID to the message
  counter++;
  uint8_t tx_buf[256];

  // Fill out the standard header
  Header header;
  header.destAddr = nextHop;
  header.sourceAddr = MY_ADDR;
  header.id = counter;
  header.type = MessageType::TYPE_TEXT;
  header.finalDestAddr = target;
  header.originalSourceAddr = MY_ADDR;

  // Stream the header and message text into a continguous buffer
  memcpy(tx_buf, &header, sizeof(Header));
  memcpy(tx_buf + sizeof(Header), argv[2], textLen);

  // Push the header/message onto the TX queue for background
  // processing.
  tx_buffer.push(tx_buf, sizeof(Header) + textLen);
      
  return 0;
}

int sendSetRoute(int argc, char **argv) { 
 
  if (argc != 4) {
    shell.println(msg_arg_error);
    return -1;
  }
  
  uint8_t target = atoi(argv[1]);
  if (target == 0 || target == 255) {
      shell.println(msg_bad_address);
      return -1;
  }

  uint8_t a1 = atoi(argv[2]);
  uint8_t a2 = atoi(argv[3]);

  // Since we might not be directly connected to the 
  // destination node we use the routing table to
  // figure out the "next hop."
  uint8_t nextHop = Routes[target];
  if (nextHop == 0) {
    shell.println(msg_no_route);
    return -1;
  }

  // Assign a unique ID to the message
  counter++;

  // Fill out the standard header
  SetRouteMessage msg;
  msg.header.destAddr = nextHop;
  msg.header.sourceAddr = MY_ADDR;
  msg.header.id = counter;
  msg.header.type = MessageType::TYPE_SETROUTE;
  msg.header.finalDestAddr = target;
  msg.header.originalSourceAddr = MY_ADDR;
  msg.targetAddr = a1;
  msg.nextHopAddr = a2;

  // Push the header/message onto the TX queue for background
  // processing.
  tx_buffer.push((const uint8_t*)&msg, sizeof(SetRouteMessage));
      
  return 0;
}

int sendGetRoute(int argc, char **argv) { 
 
  if (argc != 3) {
    shell.println(msg_arg_error);
    return -1;
  }
  
  uint8_t target = atoi(argv[1]);
  if (target == 0 || target == 255) {
      shell.println(msg_bad_address);
      return -1;
  }

  uint8_t a1 = atoi(argv[2]);

  // Since we might not be directly connected to the 
  // destination node we use the routing table to
  // figure out the "next hop."
  uint8_t nextHop = Routes[target];
  if (nextHop == 0) {
    shell.println(msg_no_route);
    return -1;
  }

  // Assign a unique ID to the message
  counter++;

  // Fill out the standard header
  GetRouteMessage msg;
  msg.header.destAddr = nextHop;
  msg.header.sourceAddr = MY_ADDR;
  msg.header.id = counter;
  msg.header.type = MessageType::TYPE_GETROUTE;
  msg.header.finalDestAddr = target;
  msg.header.originalSourceAddr = MY_ADDR;
  msg.targetAddr = a1;

  // Push the header/message onto the TX queue for background
  // processing.
  tx_buffer.push((const uint8_t*)&msg, sizeof(GetRouteMessage));
      
  return 0;
}
*/
/**
 * Clears out diagnostic counters
 */
/*
int doResetCounters(int argc, char **argv) { 
  preferences.putUShort("bootcount", 0);
  preferences.putUShort("sleepcount", 0);
}
*/
