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
#include "MessageProcessor.h"
#include "CircularBuffer.h"
#include "packets.h"
#include "RoutingTable.h"
#include "Clock.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

extern Stream& logger;

static const char* msg_bad_message = "ERR: Bad message";
static const char* msg_no_route = "ERR: No route";

MessageProcessor::MessageProcessor(
    Clock& clock, 
    CircularBuffer& rxBuffer, 
    CircularBuffer& txBuffer,
    RoutingTable& routingTable,  
    Instrumentation& instrumentation,
    Configuration& config,
    uint32_t txTimeoutMs, 
    uint32_t txRetryMs) 
    : _clock(clock),
      _rxBuffer(rxBuffer),
      _txBuffer(txBuffer),
      _routingTable(routingTable),
      _instrumentation(instrumentation),
      _config(config),
      _opm(clock, txBuffer, txTimeoutMs, txRetryMs),
      _idCounter(1),
      _startTime(clock.time()),
      _rxPacketCounter(0),
      _badRxPacketCounter(0),
      _wrongNodeRxPacketCounter(0),
      _badRouteCounter(0) {
}

void MessageProcessor::pump() {
    // We keep processing until the receive buffer
    // has been drained.
    while (true) {
      int16_t rssi = 0;
      Packet packet;
      unsigned int packetLen = sizeof(Packet);
      bool available = _rxBuffer.popIfNotEmpty((void*)&rssi, 
        (void*)&packet, &packetLen);
      if (!available) {
        break;
      }
      _process(rssi, packet, packetLen);
    }
    // Move any resulting packets onto the TX queue
    _opm.pump();
}

unsigned int MessageProcessor::getUniqueId() {
  return _idCounter++;
}

bool MessageProcessor::transmitIfPossible(const Packet& packet, 
    unsigned int packetLen) {

    // We need to use a special case if the message is being 
    // sent to the local node (i.e. loopback).  In this case
    // we just put the message on the receive queue immediately.
    if (packet.header.getDestAddr() == _config.getAddr()) {
        int16_t fakeRssi = 0;
        return _rxBuffer.push((const void*)&fakeRssi, 
            (const void*)&packet, packetLen);
    }
    // If the message is targeted at another node then put 
    // it into the outbound packet manager for later delivery. 
    else {
        return _opm.scheduleTransmitIfPossible(packet, packetLen);
    }
}

static void log_packet(Stream& l, const Packet& packet, int16_t rssi) {
    l.print(F("INF: Got type: "));
    l.print(packet.header.type);
    l.print(", id: ");
    l.print(packet.header.id);
    l.print(", from: ");
    l.print(packet.header.sourceAddr);
    l.print(", fromCall: ");
    packet.header.getSourceCall().printTo(l);
    l.print(", to: ");
    l.print(packet.header.destAddr);
    l.print(", originalSource: ");
    l.print(packet.header.originalSourceAddr);
    l.print(", originalSourceCall: ");
    packet.header.getOriginalSourceCall().printTo(l);
    l.print(", finalDest: ");
    l.print(packet.header.finalDestAddr);
    l.print(", RSSI: ");
    l.print(rssi);
    l.println();  
}

void MessageProcessor::_process(int16_t rssi, 
    const Packet& packet, unsigned int packetLen) { 

    // Error checking on new packet
    if (packetLen < sizeof(Header)) {
        _badRxPacketCounter++;
        logger.println(msg_bad_message);
        return;
    }

    // Ignore messages that are for different versions of the protocol
    if (packet.header.getPacketVersion() != PACKET_VERSION) {
        _badRxPacketCounter++;
        logger.println(msg_bad_message);
        return;
    }

    // Ignore messages that aren't targeted at this node.
    // This can happen when nodes are close to each other 
    // and they are able to hear traffic targed at other
    // nodes.
    if (packet.header.getDestAddr() != BROADCAST_ADDR &&
        packet.header.getDestAddr() != _config.getAddr()) {
        _wrongNodeRxPacketCounter++;
        return;
    }

    _rxPacketCounter++;

    // TODO: LOG LEVEL SETTING
    //log_packet(logger, packet, rssi);
    
    // If we got an ACK then process it directly 
    if (packet.header.isAck()) {
        _opm.processAck(packet);
        return;
    }

  // If the message we just received requires and ACK then 
  // generate one before proceeding with the local processing.
  if (packet.header.isAckRequired()) {
      Packet ack;
      ack.header.setupAckFor(packet.header, _config);
      bool good = transmitIfPossible(ack, sizeof(Header));
      if (!good) {
          logger.println("ERR: Full, no ACK");
      }
  }

  // Look for messages that need to be forwarded on to another node
  if (packet.header.getFinalDestAddr() != _config.getAddr()) {
    // This is a forward route (i.e. twoards the final destination)
    nodeaddr_t nextHop = _routingTable.nextHop(
      packet.header.getFinalDestAddr());
    if (nextHop != RoutingTable::NO_ROUTE) {
      // Make a clean packet so that we can adjust it
      Packet outPacket(packet);
      // Tweak the header and overlay. 
      // All messages need a unique ID so that the ACK mechanism
      // will work properly.
      outPacket.header.setId(getUniqueId()); 
      outPacket.header.setDestAddr(nextHop);
      outPacket.header.setSourceAddr(_config.getAddr());
      // Arrange for sending.
      // NOTE: WE USE THE SAME LENGTH THAT WE GOT ON THE RX
      bool good = transmitIfPossible(outPacket, packetLen);
      if (!good) {
        logger.println("ERR: Full, no forward");
      } else {
        logger.print("INF: Forward to ");
        logger.print(nextHop);
        logger.println();
      }
    }
    else {
      _badRouteCounter++;
      logger.println(msg_no_route);
    }
  }

  // All other messages are being directed to this node.
  // We process them according to the type.
  else {

    // Get the first hop for the response message. This is 
    // routing back towards the origin of the packet.
    const nodeaddr_t firstHop = _routingTable.nextHop(
      packet.header.getOriginalSourceAddr());

    // Do an error check to make sure we don't have a response
    // routing problem.
    if (packet.header.isResponseRequired() && 
        firstHop == RoutingTable::NO_ROUTE) {
        _badRouteCounter++;  
        logger.print("ERR: No route to ");
        logger.print(packet.header.getOriginalSourceAddr());
        logger.println();
        return;
    }

    // Ping
    if (packet.header.getType() == TYPE_PING_REQ) {
      // Create a pong and send back to the originator of the ping
      Packet resp;
      resp.header.setupResponseFor(packet.header, _config, 
        TYPE_PING_RESP, getUniqueId(), firstHop);
      bool good = transmitIfPossible(resp, sizeof(Header));
      if (!good) {
        logger.println("ERR: Full, no resp");
      }
    }

    // Get Station Engineering Data
    else if (packet.header.getType() == TYPE_GETSED_REQ) {

      Packet resp;
      resp.header.setupResponseFor(packet.header, _config, 
        TYPE_GETSED_RESP, getUniqueId(), firstHop);

      // Populate the payload
      SadRespPayload respPayload;
      respPayload.version = _instrumentation.getSoftwareVersion();
      respPayload.batteryMv = _instrumentation.getBatteryVoltage();
      respPayload.panelMv = _instrumentation.getPanelVoltage();
      respPayload.uptimeSeconds = (_clock.time() - _startTime) / 1000;
      respPayload.time = _clock.time();
      respPayload.bootCount = _config.getBootCount();
      respPayload.sleepCount = _config.getSleepCount();

      respPayload.temp = _instrumentation.getTemperature();
      respPayload.humidity = _instrumentation.getHumidity();
      respPayload.deviceClass = _instrumentation.getDeviceClass();
      respPayload.deviceRevision = _instrumentation.getDeviceRevision();
      
      // Message diagnostic counter 
      respPayload.rxPacketCount = _rxPacketCounter;
      respPayload.badRxPacketCount = _badRxPacketCounter;
      respPayload.wrongNodeRxPacketCount = _wrongNodeRxPacketCounter;
      respPayload.badRouteCount = _badRouteCounter;

      memcpy(resp.payload,(void*)&respPayload,sizeof(SadRespPayload));

      bool good = transmitIfPossible(resp, sizeof(Header) + sizeof(SadRespPayload));
      if (!good) {
        logger.println("ERR: Full, no resp");
      }
    }

    // Reboot
    else if (packet.header.getType() == TYPE_RESET) {
      logger.println("INF: Reset");
      _instrumentation.restart();
    }    

    // Get Engineering Data Response (for display)
    else if (packet.header.getType() == TYPE_GETSED_RESP) {
      
      if (packetLen < sizeof(Header) + sizeof(SadRespPayload)) {
        logger.println(msg_bad_message);
        return;
      }

      SadRespPayload respPayload;
      memcpy((void*)&respPayload,packet.payload, sizeof(SadRespPayload));

      // Display
      logger.print("GETSED_RESP: { \"version\": ");
      logger.print(respPayload.version);
      logger.print(", \"batteryMv\": ");
      logger.print(respPayload.batteryMv);
      logger.print(", \"panelMv\": ");
      logger.print(respPayload.panelMv);
      logger.print(", \"uptimeSeconds\": ");
      logger.print(respPayload.uptimeSeconds);
      logger.print(", \"bootCount\": ");
      logger.print(respPayload.bootCount);
      logger.print(", \"sleepCount\": ");
      logger.print(respPayload.sleepCount);
      logger.print(", \"rxPacketCount\": ");
      logger.print(respPayload.rxPacketCount);
      logger.print(", \"badRxPacketCount\": ");
      logger.print(respPayload.badRxPacketCount);
      logger.print(", \"wrongNodeRxPacketCount\": ");
      logger.print(respPayload.wrongNodeRxPacketCount);
      logger.print(", \"badRouteCount\": ");
      logger.print(respPayload.badRouteCount);
      logger.println("}");
    }

    else if (packet.header.getType() == TYPE_PING_RESP) {
      // Display
      logger.print("INF: Good response from ");
      logger.print(packet.header.getOriginalSourceAddr());
      logger.println();
    }
    
    // Reset
    else if (packet.header.getType() == TYPE_PING_RESP) {
      logger.println(F("INF: Resetting ..."));
      _instrumentation.restart();
    }
    
    // Text (for display)
    else if (packet.header.getType() == TYPE_TEXT) {
      
      // There is no null-termination, so we must use the message length here
      unsigned int textLen = packetLen - sizeof(Header);
      char scratch[128];
      memcpy(scratch, packet.payload, textLen);
      scratch[textLen] = 0;
      
      logger.print("MSG: [");
      packet.header.getOriginalSourceCall().printTo(logger);
      logger.print("] ");
      logger.println(scratch);
    }

    // Set route
    else if (packet.header.getType() == TYPE_SETROUTE) {
      if (packetLen < sizeof(Header) + sizeof(SetRouteReqPayload)) {
        logger.println(msg_bad_message);
        return;
      }
      // Unpack the request
      SetRouteReqPayload payload;
      memcpy((void*)&payload, packet.payload, sizeof(SetRouteReqPayload));

      _routingTable.setRoute(payload.targetAddr, payload.nextHopAddr);

      logger.print("INF: Set route ");
      logger.print(payload.targetAddr);
      logger.print("->");
      logger.print(payload.nextHopAddr);
      logger.println();
    }

    // Get route
    else if (packet.header.getType() == TYPE_GETROUTE_REQ) {

      if (packetLen < sizeof(Header) + sizeof(GetRouteReqPayload)) {
        logger.println(msg_bad_message);
        return;
      }
      
      // Unpack the request
      GetRouteReqPayload payload;
      memcpy((void*)&payload, packet.payload, sizeof(GetRouteReqPayload));

      // Look up the route
      nodeaddr_t nextHop = _routingTable.nextHop(payload.targetAddr);

      // Build a response
      Packet resp;
      resp.header.setupResponseFor(packet.header, _config, 
        TYPE_GETROUTE_RESP, getUniqueId(), firstHop);

      // Populate the response payload    
      GetRouteRespPayload respPayload;
      respPayload.targetAddr = payload.targetAddr;
      respPayload.nextHopAddr = nextHop;
      // #### TODO
      respPayload.txPacketCount = 0;
      // #### TODO
      respPayload.rxPacketCount = 0;

      memcpy(resp.payload,(void*)&respPayload, sizeof(respPayload));

      bool good = transmitIfPossible(resp, sizeof(Header) + sizeof(respPayload));
      if (!good) {
        logger.println("ERR: Full, no resp");
      }
    }

    // Get route response (display)
    else if (packet.header.getType() == TYPE_GETROUTE_RESP) {

      if (packetLen < sizeof(Header) + sizeof(GetRouteRespPayload)) {
        logger.println(msg_bad_message);
        return;
      }
      
      // Unpack the request
      GetRouteRespPayload payload;
      memcpy((void*)&payload, packet.payload, sizeof(GetRouteRespPayload));

      // Log the activity
      logger.print(F("GETROUTE_RESP: { "));
      logger.print(F("\"origSourceAddr\":")); 
      logger.print(packet.header.getOriginalSourceAddr());
      logger.print(F(", \"targetAddr\":"));
      logger.print(payload.targetAddr);
      logger.print(F(", \"nextHopAddr\":")); 
      logger.print(payload.nextHopAddr);
      logger.println(" }");
    }
    else {
      logger.println(F("ERR: Unknown message"));
    }
  }
}

uint16_t MessageProcessor::getPendingCount() const {
    return _opm.getPendingCount();
}


uint16_t MessageProcessor::getBadRouteCounter() const {
    return _badRouteCounter;
}

uint16_t MessageProcessor::getBadRxPacketCounter() const {
    return _badRxPacketCounter;
}

void MessageProcessor::resetCounters() {
    _rxPacketCounter = 0;
    _badRxPacketCounter = 0;
    _wrongNodeRxPacketCounter = 0;
    _badRouteCounter = 0;
}
