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
static auto msg_tx_busy = F("ERR: TX busy");

extern Stream& logger;
extern Configuration& systemConfig;
extern Instrumentation& systemInstrumentation;
extern RoutingTable& systemRoutingTable;
extern MessageProcessor& systemMessageProcessor;

int sendPing(int argc, char** argv) { 
 
    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDestAddr = parseAddr(argv[1]);
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDestAddr);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(msg_no_route);
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
      logger.println(msg_tx_busy);
      return -1;
    } 
    return 0;
}

int sendGetSed(int argc, char** argv) { 
 
    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDestAddr = parseAddr(argv[1]);
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDestAddr);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println( msg_no_route);
        return -1;
    }
    
    Packet packet;
    packet.header.setType(TYPE_GETSED_REQ);
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
      logger.println(msg_tx_busy);
      return -1;
    } 
    return 0;
}

static int sendReset(int argc, char **argv, uint8_t resetType) { 

    if (argc != 3) {
      logger.println(msg_arg_error);
      return -1;
    }

    nodeaddr_t finalDestAddr = parseAddr(argv[1]);
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDestAddr);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(msg_no_route);
        return -1;
    }

    ResetReqPayload resetReqPayload;
    resetReqPayload.passcode = atol(argv[2]);

    Packet packet;
    packet.header.setType(resetType);
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
        logger.println(msg_tx_busy);
        return -1;
    } else {
        return 0;
    }
}

int sendReset(int argc, char **argv) { 
    return sendReset(argc, argv, TYPE_RESET);
}

int sendResetCounters(int argc, char **argv) { 
    return sendReset(argc, argv, TYPE_RESET_COUNTERS);
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
int sendText(int argc, char **argv) { 
 
    if (argc != 3) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDest = parseAddr(argv[1]);
    if (finalDest == 0) {
        logger.println(msg_bad_address);
        return -1;
    }

    // Since we might not be directly connected to the 
    // destination node we use the routing table to
    // figure out the "next hop."
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDest);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(msg_no_route);
        return -1;
    }

    uint16_t textLen = strlen(argv[2]);
    if (textLen > 80) {
        logger.println(F("ERR: Length error"));
        return -1;
    }

    // Build the request packet
    Packet packet;
    packet.header.setType(TYPE_TEXT);
    packet.header.setId(systemMessageProcessor.getUniqueId());
    packet.header.setSourceAddr(systemConfig.getAddr());
    packet.header.setDestAddr(nextHop);
    packet.header.setOriginalSourceAddr(systemConfig.getAddr());
    packet.header.setFinalDestAddr(finalDest);
    packet.header.setSourceCall(systemConfig.getCall());
    packet.header.setOriginalSourceCall(systemConfig.getCall());
    // Put the text into the payload secion of the packet
    memcpy(packet.payload, argv[2], textLen);
    unsigned int packetLen = sizeof(Header) + textLen;
    // Send it
    bool good = systemMessageProcessor.transmitIfPossible(packet, packetLen);
    if (!good) {
        logger.println(msg_tx_busy);
        return -1;
    }
    return 0;
}

int sendSetRoute(int argc, char **argv) { 
 
    if (argc != 5) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDest = parseAddr(argv[1]);
    nodeaddr_t a1 = parseAddr(argv[2]);
    nodeaddr_t a2 = parseAddr(argv[3]);
    // NOTE: The next hop address is allowed to be zero (used to 
    // disable a route).
    if (finalDest == 0 || a1 == 0) {
        logger.println(msg_bad_address);
        return -1;
    }

    // Since we might not be directly connected to the 
    // destination node we use the routing table to
    // figure out the "next hop."
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDest);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(msg_no_route);
        return -1;
    }

    // Build the request packet
    Packet packet;
    packet.header.setType(TYPE_SETROUTE);
    packet.header.setId(systemMessageProcessor.getUniqueId());
    packet.header.setSourceAddr(systemConfig.getAddr());
    packet.header.setDestAddr(nextHop);
    packet.header.setOriginalSourceAddr(systemConfig.getAddr());
    packet.header.setFinalDestAddr(finalDest);
    packet.header.setSourceCall(systemConfig.getCall());
    packet.header.setOriginalSourceCall(systemConfig.getCall());
    // Fill in the payload
    SetRouteReqPayload payload;
    payload.passcode = atol(argv[4]);
    payload.targetAddr = a1;
    payload.nextHopAddr = a2; 
    // Put the text into the payload secion of the packet
    memcpy(packet.payload, (const void*)&payload, sizeof(payload));
    unsigned int packetLen = sizeof(Header) + sizeof(payload);
    // Send it
    bool good = systemMessageProcessor.transmitIfPossible(packet, packetLen);
    if (!good) {
        logger.println(msg_tx_busy);
        return -1;
    }
    return 0;
}

int sendGetRoute(int argc, char **argv) { 
 
    if (argc != 3) {
        logger.println(msg_arg_error);
        return -1;
    }

    nodeaddr_t finalDest = parseAddr(argv[1]);
    nodeaddr_t a1 = parseAddr(argv[2]);
    // NOTE: The next hop address is allowed to be zero (used to 
    // disable a route).
    if (finalDest == 0 || a1 == 0) {
        logger.println(msg_bad_address);
        return -1;
    }

    // Since we might not be directly connected to the 
    // destination node we use the routing table to
    // figure out the "next hop."
    nodeaddr_t nextHop = systemRoutingTable.nextHop(finalDest);
    if (nextHop == RoutingTable::NO_ROUTE) {
        logger.println(msg_no_route);
        return -1;
    }

    // Build the request packet
    Packet packet;
    packet.header.setType(TYPE_GETROUTE_REQ);
    packet.header.setId(systemMessageProcessor.getUniqueId());
    packet.header.setSourceAddr(systemConfig.getAddr());
    packet.header.setDestAddr(nextHop);
    packet.header.setOriginalSourceAddr(systemConfig.getAddr());
    packet.header.setFinalDestAddr(finalDest);
    packet.header.setSourceCall(systemConfig.getCall());
    packet.header.setOriginalSourceCall(systemConfig.getCall());
    // Fill in the payload
    GetRouteReqPayload payload;
    payload.targetAddr = a1;
    // Load the payload section of the packet
    memcpy(packet.payload, (const void*)&payload, sizeof(payload));
    unsigned int packetLen = sizeof(Header) + sizeof(payload);
    // Send it
    bool good = systemMessageProcessor.transmitIfPossible(packet, packetLen);
    if (!good) {
        logger.println(msg_tx_busy);
        return -1;
    }
    return 0;
}

// ===== LOCAL COMMANDS ==============================================

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
    logger.print(F("INFO: { \"node\": "));
    logger.print(systemConfig.getAddr());
    logger.print(F(", \"call\": \""));
    CallSign cs = systemConfig.getCall();
    cs.printTo(logger);
    logger.print(F("\", \"version\": "));
    logger.print(systemInstrumentation.getSoftwareVersion());
    logger.print(F(", \"blimit\": "));
    logger.print(systemConfig.getBatteryLimit());
    logger.print(F(", \"batteryMv\": "));
    logger.print(systemInstrumentation.getBatteryVoltage());
    logger.print(F(", \"panelMv\": "));
    logger.print(systemInstrumentation.getPanelVoltage());
    logger.print(F(", \"bootCount\": "));
    logger.print(systemConfig.getBootCount());
    logger.print(F(", \"sleepCount\": "));
    logger.print(systemConfig.getSleepCount());
    logger.print(F(", \"routes\": ["));

    // Display the routing table
    bool first = true;
    for (unsigned int i = 0; i < 256; i++) {
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
int sleep(int argc, char **argv) { 

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
    return 0;
}

int setCall(int argc, char **argv) { 

    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    systemConfig.setCall(CallSign(argv[1]));
    return 0;
}

int setPasscode(int argc, char **argv) { 

    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    systemConfig.setPasscode(atol(argv[1]));
    return 0;
}

int setBatteryLimit(int argc, char **argv) { 

    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    systemConfig.setBatteryLimit(atoi(argv[1]));
    return 0;
}

int print(int argc, char **argv) { 

    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }

    logger.print("PRINT: ");
    logger.print(argv[1]);
    logger.println();
    return 0;
}

/**
 * Used to put a comment into the console log
 */
int rem(int argc, char **argv) { 
    if (argc != 2) {
        logger.println(msg_arg_error);
        return -1;
    }
    return 0;
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
    // A zero address is allowed here so that we can clear the route
    nodeaddr_t r = parseAddr(argv[2]);

    systemRoutingTable.setRoute(t, r);
    return 0;
}

int clearRoutes(int argc, char **argv) { 
    systemRoutingTable.clearRoutes();
    return 0;
}

int resetCounters(int argc, char **argv) { 
    systemInstrumentation.resetCounters();
    systemMessageProcessor.resetCounters();
    return 0;
}

int factoryReset(int argc, char **argv) {
    systemConfig.factoryReset();
    systemRoutingTable.factoryReset();
}
