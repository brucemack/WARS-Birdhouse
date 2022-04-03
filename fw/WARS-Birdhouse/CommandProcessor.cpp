#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>
#include "Utils.h"
#include "RoutingTable.h"
#include "MessageProcessor.h"
#include "CommandProcessor.h"
#include "Configuration.h"

static auto msg_arg_error = F("ERR: Argument error");
static auto msg_no_route = F("ERR: No route available");
static auto msg_bad_address = F("ERR: Bad address");
static auto msg_bad_message = F("ERR: Bad message");

extern Stream& logger;
extern Configuration& config;
extern RoutingTable& routingTable;
extern MessageProcessor& messageProcessor;

int sendPing(int argc, const char** argv) { 
 
    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDestAddr = parseAddr(argv[1]);
    nodeaddr_t nextHop = routingTable.nextHop(finalDestAddr);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(F("ERR: No route"));
        return -1;
    }

    logger.println("GO");

    // Make a ping request
    Packet packet;
    packet.header.setType(TYPE_PING_REQ);
    packet.header.setId(messageProcessor.getUniqueId());
    packet.header.setSourceAddr(config.getAddr());
    packet.header.setDestAddr(nextHop);
    packet.header.setOriginalSourceAddr(config.getAddr());
    packet.header.setFinalDestAddr(finalDestAddr);
    packet.header.setSourceCall(config.getCall());
    packet.header.setOriginalSourceCall(config.getCall());
    unsigned int packetLen = sizeof(Header);
    // Send it
    messageProcessor.transmitIfPossible(packet, packetLen);
    return 0;
}
/*
int sendReset(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  counter++;
  uint8_t target = atoi(argv[1]);

  if (target > 0) {

    uint8_t nextHop = Routes[target];
    if (nextHop != 0) {

      shell.print(F("Resetting node "));
      shell.println(target);
    
      ResetMessage msg;
      msg.header.destAddr = nextHop;
      msg.header.sourceAddr = MY_ADDR;
      msg.header.id = counter;
      msg.header.type = MessageType::TYPE_RESET;
      msg.header.finalDestAddr = target;
      msg.header.originalSourceAddr = MY_ADDR;

      tx_buffer.push((uint8_t*)&msg, sizeof(ResetMessage));
    
      return 0;
    } else {
      shell.println(F("No route"));
    }
  } else {
    return -1;
  }
}

int sendBlink(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  counter++;
  uint8_t target = atoi(argv[1]);

  if (target > 0 && target < 255) {

    uint8_t nextHop = Routes[target];
    if (nextHop != 0) {

      BlinkMessage msg;
      msg.header.destAddr = nextHop;
      msg.header.sourceAddr = MY_ADDR;
      msg.header.id = counter;
      msg.header.type = MessageType::TYPE_BLINK;
      msg.header.finalDestAddr = target;
      msg.header.originalSourceAddr = MY_ADDR;

      tx_buffer.push((uint8_t*)&msg, sizeof(BlinkMessage));
    
      return 0;
    } else {
      shell.println(F("ERR: No route"));
    }
  } else {
    return -1;
  }
}

int boot(int argc, char **argv) { 
    shell.println("Asked to reboot ...");
    ESP.restart();
    return 0;
}

int bootRadio(int argc, char **argv) { 

  // Reset the radio 
  reset_radio();
  delay(250);

  // Initialize the radio
  if (init_radio() != 0) {
    shell.println("Problem with initialization");
    return -1;
  }
  else {
    shell.println("Radio initialized");

    // Start listening for messages
    state = State::LISTENING;
    enable_interrupt_RxDone();
    set_mode_RXCONTINUOUS();

    return 0;
  }
}

int info(int argc, char **argv) { 
    shell.print(F("{ \"node\": "));
    shell.print(MY_ADDR);
    shell.print(F(", \"version\": "));
    shell.print(SW_VERSION);
    shell.print(F(", \"MAC\": \""));
    shell.print(chipId, HEX);
    shell.print(F("\", \"blimit\": "));
    shell.print(preferences.getUShort("blimit", 0));
    shell.print(F(", \"batteryMv\": "));
    shell.print(checkBattery());
    shell.print(F(", \"panelMv\": "));
    shell.print(checkPanel());
    shell.print(F(", \"bootCount\": "));
    shell.print(preferences.getUShort("bootcount", 0));
    shell.print(F(", \"sleepCount\": "));
    shell.print(preferences.getUShort("sleepcount", 0));
    shell.print(F(", \"routes\": ["));
    bool first = true;
    for (int i = 0; i < 256; i++) {
      if (Routes[i] != 0) {
        if (!first) 
          shell.print(", ");
        first = false;
        shell.print("[");
        shell.print(i);
        shell.print(", ");
        shell.print(Routes[i]);
        shell.print("]");
      }
    }
    shell.print("]");
    shell.println(F("}"));
    return 0;
}

// Used for testing the watch dog 
static int sleep(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  uint16_t seconds = atoi(argv[1]);
  shell.println("INF: Sleeping ...");
  delay(seconds * 1000);
  shell.println("INF: Done");
  return 0;
}

int skipAcks(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  skipAckCount = atoi(argv[1]);
}

int setAddr(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  // Save the address in the NVRAM
  preferences.putUChar("addr", atoi(argv[1]));

  // Get the address
  MY_ADDR = preferences.getUChar("addr", 1);  
}

int setRoute(int argc, char **argv) { 

  if (argc != 3) {
    shell.println(msg_arg_error);
    return -1;
  }

  uint8_t t = atoi(argv[1]);
  uint8_t r = atoi(argv[2]);
  Routes[t] = r;

  // Save the address in the NVRAM
  preferences.putBytes("routes", Routes, 256);
}

int clearRoutes(int argc, char **argv) { 
  for (int i = 0; i < 256; i++)
    Routes[i] = 0;
  // Save the address in the NVRAM
  preferences.putBytes("routes", Routes, 256);
}

int setBlimit(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  // Save the address in the NVRAM
  preferences.putUShort("blimit", atoi(argv[1]));
}

int doPrint(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  shell.print("[");
  shell.print(argv[1]);
  shell.println("]");
}
*/

/**
 * Used to put a comment into the console log
 */
/*
int doRem(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }
}
*/
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
