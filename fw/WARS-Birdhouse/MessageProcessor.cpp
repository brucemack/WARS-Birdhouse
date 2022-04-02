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
static const int32_t SEND_TIMEOUT = 10 * 1000;

MessageProcessor::MessageProcessor(
    Clock& clock, 
    CircularBuffer& rxBuffer, 
    CircularBuffer& txBuffer,
    RoutingTable& routingTable,  
    Instrumentation& instrumentation,
    nodeaddr_t myAddr, 
    const char* myCall) 
    : _clock(clock),
      _rxBuffer(rxBuffer),
      _txBuffer(txBuffer),
      _routingTable(routingTable),
      _instrumentation(instrumentation),
      _myAddr(myAddr),
      _opm(clock, txBuffer),
      _idCounter(1),
      _startTime(clock.time()) {
        // TODO: REVIEW THIS CLOSELY
      strncpy(_myCall, myCall, 8);
}

void MessageProcessor::pump() {
    int16_t rssi = 0;
    Packet packet;
    unsigned int packetLen = sizeof(Packet);
    bool notEmpty = _rxBuffer.popIfNotEmpty((void*)&rssi, (void*)&packet, &packetLen);
    if (notEmpty) {
        _process(rssi, packet, packetLen);
    }
}

unsigned int MessageProcessor::_getUniqueId() {
  return _idCounter++;
}

void MessageProcessor::_process(int16_t rssi, 
  const Packet& packet, unsigned int packetLen) { 

  // Error checking on new packet
  if (packetLen < sizeof(Header)) {
    logger.println(msg_bad_message);
    return;
  }

  logger.print(F("INF: Got type: "));
  logger.print(packet.header.type);
  logger.print(", id: ");
  logger.print(packet.header.id);
  logger.print(", from: ");
  logger.print(packet.header.sourceAddr);
  logger.print(", originalSource: ");
  logger.print(packet.header.originalSourceAddr);
  logger.print(", finalDest: ");
  logger.print(packet.header.finalDestAddr);
  logger.print(", RSSI: ");
  logger.print(rssi);
  logger.println();

  // Look for messages that need to be forwarded on to another node
  if (packet.header.getFinalDestAddr() != _myAddr) {
    // This is a forward route (i.e. twoards the final destination)
    nodeaddr_t nextHop = _routingTable.nextHop(
      packet.header.getFinalDestAddr());
    if (nextHop != RoutingTable::NO_ROUTE) {
      // Make a clean packet so that we can adjust it
      Packet outPacket(packet);
      // Tweak the header and overlay. 
      // All messages need a unique ID so that the ACK mechanism
      // will work properly.
      outPacket.header.setId(_getUniqueId()); 
      outPacket.header.setDestAddr(nextHop);
      outPacket.header.setSourceAddr(_myAddr);
      // Arrange for sending.
      // NOTE: WE USE THE SAME LENGTH THAT WE GOT ON THE RX
      bool good = _opm.allocateIfPossible(outPacket, packetLen, 
        _clock.time() + SEND_TIMEOUT);
      if (!good) {
        logger.println("ERR: Full, no forward");
      }
    }
    else {
      // TODO: COUNTERS
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
        logger.print("ERR: No route to ");
        logger.print(packet.header.getOriginalSourceAddr());
        logger.println();
        return;
    }

    // Ping
    if (packet.header.getType() == TYPE_PING_REQ) {
      // Create a pong and send back to the originator of the ping
      Packet resp;
      resp.header.setupResponseFor(packet.header, _myCall, _myAddr, 
        TYPE_PING_RESP, _getUniqueId(), firstHop);
      bool good = _opm.allocateIfPossible(resp, sizeof(Header), 
        _clock.time() + SEND_TIMEOUT);
      if (!good) {
        logger.printf("ERR: Full, no resp");
      }
    }

    // Get Station Engineering Data
    else if (packet.header.getType() == TYPE_GETSED_REQ) {

      Packet resp;
      resp.header.setupResponseFor(packet.header, _myCall, _myAddr, 
        TYPE_GETSED_RESP, _getUniqueId(), firstHop);

      // Populate the payload
      SadRespPayload respPayload;
      respPayload.version = _instrumentation.getSoftwareVersion();
      respPayload.batteryMv = _instrumentation.getBatteryVoltage();
      respPayload.panelMv = _instrumentation.getPanelVoltage();
      respPayload.uptimeSeconds = (_clock.time() - _startTime) / 1000;
      respPayload.time = _clock.time();
      respPayload.bootCount = _instrumentation.getBootCount();
      respPayload.sleepCount = _instrumentation.getSleepCount();
      // #### TODO
      respPayload.rxPacketCount = 0;
      // #### TODO
      respPayload.routeErrorCount = 0;
      respPayload.temp = _instrumentation.getTemperature();
      respPayload.humidity = _instrumentation.getHumidity();
      respPayload.deviceClass = _instrumentation.getDeviceClass();
      respPayload.deviceRevision = _instrumentation.getDeviceRevision();
      // #### TODO
      respPayload.wrongNodeRxCount = 0;

      memcpy(resp.payload,(void*)&respPayload,sizeof(SadRespPayload));

      bool good = _opm.allocateIfPossible(resp, sizeof(Header) + sizeof(SadRespPayload), 
        _clock.time() + SEND_TIMEOUT);
      if (!good) {
        logger.printf("ERR: Full, no resp");
      }
    }
    
    // Get Engineering Data Repsonse (for display)
    else if (packet.header.getType() == TYPE_GETSED_RESP) {
      
      if (packetLen < sizeof(Header) + sizeof(SadRespPayload)) {
        logger.println(msg_bad_message);
        return;
      }

      SadRespPayload respPayload;
      memcpy((void*)&respPayload,packet.payload, sizeof(SadRespPayload));

      // Display
      logger.print("{ \"version\": ");
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
      logger.print("MSG: ");
      // There is no null-termination, so we must use the message length here
      for (int i = 0; i < packetLen - sizeof(Header); i++) {
        logger.print(packet.payload[i + sizeof(Header)]);
      }
      logger.println();
    }

    // Set route
    else if (packet.header.getType() == TYPE_SETROUTE) {
      if (packetLen < sizeof(Header) + sizeof(SetRouteReqPayload)) {
        logger.println(msg_bad_message);
        return;
      }

      SetRouteReqPayload payload;
      memcpy((void*)&payload, packet.payload, sizeof(SetRouteReqPayload));
      


    else if (header.type == MessageType::TYPE_SETROUTE) {
      if (len < sizeof(SetRouteMessage)) {
        shell.println(msg_bad_message);
        return;
      }
      // Unpack request
      SetRouteMessage msg;
      memcpy(&msg, buf, sizeof(SetRouteMessage));
      // Modify the routing table
      Routes[msg.targetAddr] = msg.nextHopAddr;
      // Save the updated table in the NVRAM
      preferences.putBytes("routes", Routes, 256);
      // Log the activity
      shell.print(F("INF: Set route "));
      shell.print(msg.targetAddr);
      shell.print("->");
      shell.println(msg.nextHopAddr);
    }

    // Get route
    else if (header.type == MessageType::TYPE_GETROUTE) {
      if (len < sizeof(GetRouteMessage)) {
        shell.println(msg_bad_message);
        return;
      }
      // Unpack request
      GetRouteMessage msg;
      memcpy(&msg, buf, sizeof(GetRouteMessage));
      // Make a response
      GetRouteRespMessage resp;
      resp.header.destAddr = header.sourceAddr;
      resp.header.sourceAddr = MY_ADDR;
      resp.header.hops = header.hops + 1;        
      resp.header.id = header.id;
      resp.header.type = MessageType::TYPE_GETROUTERESP;
      resp.header.finalDestAddr = header.originalSourceAddr;
      resp.header.originalSourceAddr = MY_ADDR;
      resp.targetAddr = msg.targetAddr;
      // Here we are looking into the routing table
      resp.nextHopAddr = Routes[msg.targetAddr]; 
      tx_buffer.push((const uint8_t*)&resp, sizeof(GetRouteRespMessage));
    }

    // Get route response (display)
    else if (header.type == MessageType::TYPE_GETROUTERESP) {
      if (len < sizeof(GetRouteRespMessage)) {
        shell.println(msg_bad_message);
        return;
      }
      // Unpack request
      GetRouteRespMessage msg;
      memcpy(&msg, buf, sizeof(GetRouteRespMessage));
      // Log the activity
      shell.print(F("GETROUTERESP: { "));
      shell.print(F("\"origSourceAddr\":")); 
      shell.print(msg.header.originalSourceAddr);
      shell.print(F(", \"targetAddr\":"));
      shell.print(msg.targetAddr);
      shell.print(F(", \"nextHopAddr\":")); 
      shell.print(msg.nextHopAddr);
      shell.println(" }");
    }
    else {
      shell.println(F("ERR: Unknown message"));
    }
  }
}

/**
 * @brief Look to see if any data is available on the RX queue.
 */
static void check_for_rx_msg(CircularBuffer& rx_buffer, CircularBuffer& tx_buffer,
  Stream& shell, RoutingTable& routingTable, nodeaddr_t myAddr) {
  int16_t rssi = 0;
  unsigned int msg_len = 256;
  uint8_t msg[msg_len];
  bool notEmpty = rx_buffer.popIfNotEmpty((uint8_t*)&rssi, msg, &msg_len);
  if (notEmpty) {
    process_rx_msg(rssi, msg, msg_len, tx_buffer, shell, 
      routingTable, myAddr);
  }
}

void process(CircularBuffer& rx_buffer, CircularBuffer& tx_buffer, 
  Stream& shell, RoutingTable& routingTable,
  nodeaddr_t myAddr) {
  check_for_rx_msg(rx_buffer, tx_buffer, shell, routingTable,
    myAddr);
}
