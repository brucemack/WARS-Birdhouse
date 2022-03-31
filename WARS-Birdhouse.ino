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
// Current measurements:
//
// 80mA idle, 160mA transmit with normal clock
// 30mA at 10 MHz clock rate

// Prerequsisites to Install 
// * philj404:SimpleSerialShell
// * contrem:arduino-timer

// Build instructions
// * Set clock frequency to 10MHz to save power (also disables WIFI and BT)

/**
Currently using the ESP32 D1 Mini
http://www.esp32learning.com/wp-content/uploads/2017/12/esp32minikit.jpg
*/

#include "WiFi.h"
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <SimpleSerialShell.h>
#include "CircularBuffer.h"
#include "spi_utils.h"
#include <arduino-timer.h>

#define SW_VERSION 29

// This is the pin that is available on the D1 Mini module:
#define RST_PIN   26
// This is the ESP32 pin that is connected to the DIO0 pin on the RFM95W module.
#define DIO0_PIN  4
// NOTE: GPIO36 is the same as RTC_GPIO0 
// Generally the on-board LED on the ESP32 module
#define LED_PIN   2
// Analog input pin for measuring battery, connected via 1:2 voltage divider
#define BATTERY_LEVEL_PIN 33
// Analog input pin for measuring pannel, connected via 1:6 voltage divider
#define PANEL_LEVEL_PIN 34

// Watchdog timeout in seconds (NOTE: I think this time might be off because
// we are changing the CPU clock frequency)
#define WDT_TIMEOUT 5

#define S_TO_US_FACTOR 1000000UL

// Deep sleep duration when low battery is detected
#define DEEP_SLEEP_SECONDS (60UL * 60UL)

// How frequently to check the batter condition
#define BATTERY_CHECK_INTERVAL_SECONDS 30

static const float STATION_FREQUENCY = 906.5;

static Preferences preferences;

// NODE SPECIFIC STUFF
static uint8_t MY_ADDR = 1;
static uint8_t MY_CALL[8] = "KC1FSZ  ";
// Routing table (node 1)
static uint8_t Routes[256];

static int32_t get_time_seconds() {
  return esp_timer_get_time() / 1000000L;
}

// Returns the battery level in mV
// The voltage is sampled via a 1:2 voltage divider 
uint16_t checkBattery() {
  const float scale = (3.3 / 4096.0) * 2.0;
  float level = (float)analogRead(BATTERY_LEVEL_PIN) * scale;
  return level * 1000.0;
}

// Returns the panel level in mV
// The voltage is sampled via a 1:6 voltage divider 
uint16_t checkPanel() {
  const float scale = (3.3 / 4096.0) * 6.0;
  float level = (float)analogRead(PANEL_LEVEL_PIN) * scale;
  return level * 1000.0;
}

// The states of the state machine
enum State { IDLE, LISTENING, TRANSMITTING, TRANSMITTING_ACK, LISTENING_FOR_ACK };

// The overall state
static State state = State::IDLE;
// The state we need to go into after a transmit is successful
static State stateAfterTransmit = State::LISTENING;
// This is the message ID that we are waiting to have acknowledged.
// NOTE: Only relevant in LISTENING_FOR_ACK state.
static uint16_t listenAckId = 0;
// This is the time we started waiting for the ACK. Used for timeout tracking.
// NOTE: Only relevant in LISTENING_FOR_ACK state.
static uint32_t listenAckStart = 0;
// The number of times we've retried the transmit
static uint16_t transmitRetry = 0;
// The number of ACKs to skip (used for testing)
static uint16_t skipAckCount = 0;

// The circular buffer used for outgoing data
static CircularBuffer<4096> tx_buffer;
// The circular buffer used for incoming data
static CircularBuffer<4096> rx_buffer;

// ----- Interrupt Service -------

static volatile bool isr_hit = false;

void IRAM_ATTR isr() {
  // NOTE: We've had so many problems with interrupt enable/disable on the ESP32
  // so now we're just going to set a flag and let everything happen in the loop()
  // context.
  isr_hit = true;
}

void event_TxDone() { 

  // Waiting for an message transmit to finish successfull
  if (state != State::TRANSMITTING && state != State::TRANSMITTING_ACK) {
    Serial.println(F("ERR: TxDone received in unexpected state"));
    return;
  }

  // Revert back to whatever we were listening for. 
  state = stateAfterTransmit;
  // Ask for interrupt when receiving
  enable_interrupt_RxDone();
  // Go into RXCONTINUOUS so we can hear the response
  set_mode_RXCONTINUOUS();  
} 

void event_RxDone() {

  // How much data is available?
  const uint8_t len = spi_read(0x13);
  // Reset the FIFO read pointer to the beginning of the packet we just got
  spi_write(0x0d, spi_read(0x10));
  // Stream in from the FIFO. 
  uint8_t rx_buf[256];
  spi_read_multi(0x00, rx_buf, len);

  // Grab the RSSI value from the radio
  int8_t lastSnr = (int8_t)spi_read(0x19) / 4;
  int16_t lastRssi = spi_read(0x1a);
  if (lastSnr < 0)
    lastRssi = lastRssi + lastSnr;
  else
    lastRssi = (int)lastRssi * 16 / 15;
  // We are using the high frequency port
  lastRssi -= 157;

  // Handle based on state.  If we're not listening for anything then 
  // ignore what was just read.
  if (state != State::LISTENING && state != State::LISTENING_FOR_ACK) {
    Serial.println(F("ERR: Message received when not listening"));
    return;
  }

  // Make sure the message is valid (right length, version, etc.)
  if (len < sizeof(Header)) {
    Serial.print(F("ERR: Message invalid length "));
    Serial.println(len);
    return;
  }

  // Pull in the header into a structure so we can look at it
  Header rx_header(rx_buf);

  // Ignore messsages targeted for other stations
  if (!rx_header.isRelevant(MY_ADDR)) {
    // TODO: COUNTER
    //Serial.println(F("INF: Ignored message for another node"));
    return;
  }

  // Check to see if this is an ack that we are waiting for
  if (rx_header.type == MessageType::TYPE_ACK) {
    // When we get an ACK, figure out if it is the one we are waiting for
    if (state == State::LISTENING_FOR_ACK && rx_header.id == listenAckId) {
      // Flush the message that we sent previously (it's been confirmed now)
      tx_buffer.popAndDiscard();
      // Get back into normal listening mode
      state = State::LISTENING;
      listenAckId = 0;
      listenAckStart = 0;
      transmitRetry = 0;
    } else {
      Serial.print(F("INF: Ignoring ACK for "));
      Serial.println(rx_header.id);
    }
  } 
  // Any other message will be forwarded to the application
  else {

    // Put the RSSI (OOB) and the entire packet (not just the header)
    // into the circular queue
    rx_buffer.push(&lastRssi, rx_buf, len);

    // Check to see if an ACK is required for this packet.  If so, 
    // create the ACK and transmit it.
    if (rx_header.isAckRequired()) {
      // A testing feature that will skip ACKs
      if (skipAckCount != 0) {
        skipAckCount--;
        Serial.println(F("INF: Skipping an ACK (test)"));
      }
      else {
        Header ack_header;
        ack_header.createAckFor(rx_header, MY_CALL, MY_ADDR);
        // Go into stand-by so we know that nothing else is coming in
        set_mode_STDBY();
        // Move the data into the radio FIFO
        write_message((uint8_t*)&ack_header, sizeof(Header));
        // Keep track of what we were listening for before starting this transmission
        stateAfterTransmit = state;
        // Go into transmit mode
        state = State::TRANSMITTING_ACK;
        enable_interrupt_TxDone();
        set_mode_TX();
      }
    }
  }
}

static void event_tick_LISTENING_FOR_ACK() {

  // If we are listening for an ACK then don't send/resend until the timeout 
  // has passed.
  if (get_time_seconds() - listenAckStart < 5) {
    return;
  }

  // If we have retried a few times then give up and discard the message.
  if (transmitRetry > 3) {
    Serial.println(F("WRN: Giving up on "));
    Serial.println(listenAckId);
    // Flush the message
    tx_buffer.popAndDiscard();
    // Give up and get back into the normal listen state
    state = State::LISTENING;
    stateAfterTransmit = State::LISTENING;
    listenAckId = 0;
    listenAckStart = 0;
    transmitRetry = 0;
    return;
  }

  // If a retry is still possible then generate it.
  // Go into stand-by so we know that nothing else is coming in
  set_mode_STDBY();

  // Peek the data off the TX queue into the transmit buffer.  We use peek so
  // that re-transmits can be supported if necessary.
  unsigned int tx_buf_len = 256;
  uint8_t tx_buf[tx_buf_len];
  tx_buffer.peek(tx_buf, &tx_buf_len);

  // Move the data into the radio FIFO
  write_message(tx_buf, tx_buf_len);
  // Go into transmit mode
  state = State::TRANSMITTING;
  enable_interrupt_TxDone();
  set_mode_TX();

  stateAfterTransmit = State::LISTENING_FOR_ACK;
  listenAckStart = get_time_seconds();
  transmitRetry++;
}

static void event_tick_LISTENING() {

  // Check for pending transmissions.  If nothing is pending then 
  // return without any state change.
  if (tx_buffer.isEmpty()) {
    return;
  }
    
  // At this point we have something pending.
  // Go into stand-by so we know that nothing else is coming in
  set_mode_STDBY();

  // Peek the data off the TX queue into the transmit buffer.  We use peek so
  // that re-transmits can be supported if necessary.
  unsigned int tx_buf_len = 256;
  uint8_t tx_buf[tx_buf_len];
  tx_buffer.peek(tx_buf, &tx_buf_len);

  // Move the data into the radio FIFO
  write_message(tx_buf, tx_buf_len);
  // Go into transmit mode
  state = State::TRANSMITTING;
  enable_interrupt_TxDone();
  set_mode_TX();

  // Take a look at the header so we can determine whether an ACK is 
  // going to be required for this message.
  // TODO: SEE IF WE CAN SOLVE THIS USING A CAST RATHER THAN A COPY!
  Header tx_header(tx_buf);
  if (tx_header.isAckRequired()) {
    // After transmission we will need to wait for the ACK
    stateAfterTransmit = State::LISTENING_FOR_ACK;
    listenAckId = tx_header.id;
    listenAckStart = get_time_seconds();
  } else {
    // Pop the message off now - no ACK involved
    tx_buffer.popAndDiscard();
    // Transition directly into the listen mode
    stateAfterTransmit = State::LISTENING;
    listenAckId = 0;
    listenAckStart = 0;
  }    
  transmitRetry = 0;
}

// Call periodically to look for timeouts or other pending activity.  This will happen
// on the regular application thread, so we disable interrupts to avoid conflicts.
void event_tick() {
  if (state == State::LISTENING) {
    event_tick_LISTENING();
  } else if (state == State::LISTENING_FOR_ACK) {
    event_tick_LISTENING_FOR_ACK();
  }
}

// --------------------------------------------------------------------------
// Radio Utilty Functions
// 
// This reference will be very important to you:
// https://www.hoperf.com/data/upload/portal/20190730/RFM95W-V2.0.pdf
// 
// The LoRa register map starts on page 103.

void set_mode_SLEEP() {
  spi_write(0x01, 0x00);  
}

void set_mode_STDBY() {
  spi_write(0x01, 0x01);  
}

void set_mode_TX() {
  spi_write(0x01, 0x03);
}

void set_mode_RXCONTINUOUS() {
  spi_write(0x01, 0x05);
}

// See table 17 - DIO0 is controlled by bits 7-6
void enable_interrupt_TxDone() {
  spi_write(0x40, 0x40);
}

// See table 17 - DIO0 is controlled by bits 7-6
void enable_interrupt_RxDone() {
  spi_write(0x40, 0x00);
}

/** Sets the radio frequency from a decimal value that is quoted
 *   in MHz.
 */
// See page 103
void set_frequency(float freq_mhz) {
  const float CRYSTAL_MHZ = 32000000.0;
  const float FREQ_STEP = (CRYSTAL_MHZ / 524288);
  const uint32_t f = (freq_mhz * 1000000.0) / FREQ_STEP;
  spi_write(0x06, (f >> 16) & 0xff);
  spi_write(0x07, (f >> 8) & 0xff);
  spi_write(0x08, f & 0xff);
}

void write_message(uint8_t* data, uint8_t len) {
  // Move pointer to the start of the FIFO
  spi_write(0x0d, 0);
  // The message
  spi_write_multi(0x00, data, len);
  // Update the length register
  spi_write(0x22, len);
}

int reset_radio() {
  
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  delay(5);
  digitalWrite(RST_PIN, LOW);
  delay(5);
  digitalWrite(RST_PIN, HIGH);
  // Float the reset pin
  pinMode(RST_PIN, INPUT);
  // Per datasheet, wait 5ms after reset
  delay(5);
  
  return 0;  
}

/**
 * @brief Sets the Over Current Protection register.
 * 
 * @param current_ma 
 */
static void set_ocp(uint8_t current_ma) {

  uint8_t trim = 27;

  if (current_ma <= 120) {
    trim = (current_ma - 45) / 5;
  } else if (current_ma <= 240) {
    trim = (current_ma + 30) / 10;
  }

  spi_write(0x0b, 0x20 | (0x1F & trim));
}

static void setLowDatarate() {

    // called after changing bandwidth and/or spreading factor
    //  Semtech modem design guide AN1200.13 says 
    // "To avoid issues surrounding  drift  of  the  crystal  reference  oscillator  due  to  either  temperature  change  
    // or  motion,the  low  data  rate optimization  bit  is  used. Specifically for 125  kHz  bandwidth  and  SF  =  11  and  12,  
    // this  adds  a  small  overhead  to increase robustness to reference frequency variations over the timescale of the LoRa packet."
 
    // read current value for BW and SF
    uint8_t bw = spi_read(0x1d) >> 4;	// bw is in bits 7..4
    uint8_t sf = spi_read(0x1e) >> 4;	// sf is in bits 7..4
   
    // calculate symbol time (see Semtech AN1200.22 section 4)
    float bw_tab[] = { 7800, 10400, 15600, 20800, 31250, 41700, 62500, 
      125000, 250000, 500000};
    float bandwidth = bw_tab[bw];
    float symbolTime = 1000.0 * pow(2, sf) / bandwidth;	// ms
   
    // the symbolTime for SF 11 BW 125 is 16.384ms. 
    // and, according to this :- 
    // https://www.thethingsnetwork.org/forum/t/a-point-to-note-lora-low-data-rate-optimisation-flag/12007
    // the LDR bit should be set if the Symbol Time is > 16ms
    // So the threshold used here is 16.0ms
 
    // the LDR is bit 3 of register 0x26
    uint8_t current = spi_read(0x26) & ~0x08; // mask off the LDR bit
    if (symbolTime > 16.0)
      spi_write(0x26, current | 0x08);
    else
      spi_write(0x26, current);   
}

/** 
 *  All of the one-time initialization of the radio
 */
int init_radio() {

  // Check the radio version to make sure things are connected
  uint8_t ver = spi_read(0x42);
  if (ver != 18) {
    return -1;
  }
  
  // Switch into Sleep mode, LoRa mode
  spi_write(0x01, 0x80);
  // Wait for sleep mode 
  delay(10); 

  // Make sure we are actually in sleep mode
  if (spi_read(0x01) != 0x80) {
    return -1; 
  }

  // Setup the FIFO pointers
  // TX base:
  spi_write(0x0e, 0);
  // RX base:
  spi_write(0x0f, 0);

  set_frequency(STATION_FREQUENCY);

  // Set LNA boost
  spi_write(0x0c, spi_read(0x0c) | 0x03);

  // AgcAutoOn=LNA gain set by AGC
  spi_write(0x26, 0x04);

  // DAC enable (adds 3dB)
  spi_write(0x4d, 0x87);
  
  // Turn on PA and set power to +20dB
  // PaSelect=1
  // OutputPower=17 (20 - 3dB from DAC)
  spi_write(0x09, 0x80 | ((20 - 3) - 2));

  // Set OCP to 140 (as per the Sandeep Mistry library)
  set_ocp(140);

  // Go into stand-by
  set_mode_STDBY();

  // Configure the radio
  uint8_t reg = 0;

  // 7-4: 0111  (125k BW)
  // 3-1: 001   (4/5 coding rate)
  // 0:   0     (Explicit header mode)
  reg = 0b01110010;
  spi_write(0x1d, reg);

  // 7-4:   9 (512 chips/symbol, spreading factor 9)
  // 3:     0 (RX continuous mode normal)
  // 2:     1 (CRC mode on)
  // 1-0:   0 (RX timeout MSB) 
  reg = 0b10010100;
  spi_write(0x1e, reg);

  // Preable Length=8 (default)
  // Preamble MSB and LSB
  //spi_write(0x20, 8 >> 8);
  //spi_write(0x21, 8 & 0xff);

  setLowDatarate();

  return 0;
}

// ===== Application 

static auto msg_arg_error = F("ERR: Argument error");
static auto msg_no_route = F("ERR: No route available");
static auto msg_bad_address = F("ERR: Bad address");
static auto msg_bad_message = F("ERR: Bad message");

static int counter = 0;
static uint32_t chipId = 0;

auto timer = timer_create_default();

int sendPing(int argc, char **argv) { 
 
  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  counter++;
  
  uint8_t target = atoi(argv[1]);
  if (target > 0 && target < 255) {
    
    uint8_t nextHop = Routes[target];
    if (nextHop != 0) {
      Header header;
      header.type = MessageType::TYPE_PING;
      header.id = counter;
      // Addressing
      header.setSourceAddr(MY_ADDR);
      header.setOriginalSourceAddr(MY_ADDR);
      header.setDestAddr(nextHop);
      header.setFinalDestAddr(target);
      header.setSourceCall(MY_CALL);
      header.setOriginalSourceCall(MY_CALL);

      tx_buffer.push((uint8_t*)&header, sizeof(Header));
      
      return 0;

    } else {
      shell.println(F("ERR: No route"));
    }
  }
}

int sendReset(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
    return -1;
  }

  counter++;
  uint8_t target = atoi(argv[1]);

  if (target > 0 && target < 255) {

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

/**
 * Used to put a comment into the console log
 */
int doRem(int argc, char **argv) { 

  if (argc != 2) {
    shell.println(msg_arg_error);
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

/**
 * Clears out diagnostic counters
 */
int doResetCounters(int argc, char **argv) { 
  preferences.putUShort("bootcount", 0);
  preferences.putUShort("sleepcount", 0);
}

static bool isDelim(char d, const char* delims) {
  const char* ptr = delims;
  while (*ptr != 0) {
    if (d == *ptr) {
      return true;
    }
    ptr++;
  }
  return false;
}

char* tokenizer(char* str, const char* delims, char** saveptr) {

  // Figure out where to start scanning
  char* ptr = 0;
  if (str == 0) {
    ptr = *saveptr;
  } else {
    ptr = str;
  }
  
  // Consume/ignore any leading delimiters
  while (*ptr != 0 && isDelim(*ptr, delims)) {
    ptr++;
  }

  // At this point we either have a null or a non-delimiter
  // character to deal with. If there is nothing left in the 
  // string then return 0
  if (*ptr == 0) {
    return 0;
  }

  char* result;

  // Check to see if this is a quoted token 
  if (*ptr == '\"') {
    // Skip the opening quote
    ptr++;
    // Result is the first eligible character
    result = ptr;
    while (*ptr != 0) {
      if (*ptr == '\"') {
        // Turn the trailing delimiter into a null-termination
        *ptr = 0;
        // Skip forward for next start
        ptr++;
        break;
      } 
      ptr++;
    }
  } 
  else {
    // Result is the first eligible character
    result = ptr;
    while (*ptr != 0) {
      if (isDelim(*ptr, delims)) {
        // Turn the trailing delimiter into a null-termination
        *ptr = 0;
        // Skip forward for next start
        ptr++;
        break;
      } 
      ptr++;
    }
  }

  // We will start on the next character when we return
  *saveptr = ptr;
  return result;
}

/**
 * @brief Checks to see if the battery voltage is below the 
 * configured limited.  If so, put the radio in SLEEP mode
 * and tell the ESP32 to go into deep sleep.
 */
static bool check_low_battery(void*) {
  uint16_t lowBatteryLimitMv = preferences.getUShort("blimit", 0);
  // Check the battery
  uint16_t battery = checkBattery();
  // If the battery is low then deep sleep
  if (lowBatteryLimitMv != 0 && battery < lowBatteryLimitMv) {
    shell.println(F("INF: Low battery, entering deep sleep"));
    // Keep track of how many times this has happened
    uint16_t sleepCount = preferences.getUShort("sleepcount", 0);  
    preferences.putUShort("sleepcount", sleepCount + 1);   
    // Put the radio into SLEEP mode to minimize power consumpion.  Per the 
    // datasheet the sleep current is 1uA.
    set_mode_SLEEP();
    // Put the ESP32 into a deep sleep that will be awakened using the timer.
    // Wakeup will look like reboot.
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_SECONDS * S_TO_US_FACTOR);
    esp_deep_sleep_start();
  }
  // Keep repeating
  return true;
}

void setup() {

  // Changed to speed up boot
  delay(100);
  Serial.begin(115200);
  // Removed this to speed up boot
  //delay(1000);
  
  Serial.println();
  Serial.println(F("====================="));
  Serial.println(F("KC1FSZ LoRa Mesh Node"));
  Serial.println(F("====================="));
  Serial.println();

  Serial.print(F("Version: "));
  Serial.println(SW_VERSION);

  Serial.print(F("Build: "));
  Serial.println(__DATE__);

  // Extract the MAC address of the chip
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.print(F("MAC: "));
  Serial.println(chipId, HEX);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }

  preferences.begin("my-app", false); 

  // Radio interrupt pin
  pinMode(DIO0_PIN, INPUT);

  // LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Shell setup
  shell.attach(Serial); 
  shell.setTokenizer(tokenizer);
  shell.addCommand(F("ping <addr>"), sendPing);
  shell.addCommand(F("reset <addr>"), sendReset);
  shell.addCommand(F("blink <addr>"), sendBlink);
  shell.addCommand(F("text <addr> <text>"), sendText);
  shell.addCommand(F("boot"), boot);
  shell.addCommand(F("bootradio"), bootRadio);
  shell.addCommand(F("info"), info);
  shell.addCommand(F("sleep <seconds>"), sleep);
  shell.addCommand(F("skipacks"), skipAcks);
  shell.addCommand(F("setaddr <addr>"), setAddr);
  shell.addCommand(F("setroute <target addr> <next hop addr>"), setRoute);
  shell.addCommand(F("clearroutes"), clearRoutes);
  shell.addCommand(F("setblimit <limit_mv>"), setBlimit);
  shell.addCommand(F("print <text>"), doPrint);
  shell.addCommand(F("rem <text>"), doRem);
  shell.addCommand(F("setrouteremote <addr> <target addr> <next hop addr>"), sendSetRoute);
  shell.addCommand(F("getrouteremote <addr> <target addr>"), sendGetRoute);
  shell.addCommand(F("resetcounters"), doResetCounters);

  // Increment the boot count
  uint16_t bootCount = preferences.getUShort("bootcount", 0);  
  shell.print(F("Boot Count: "));
  shell.println(bootCount);
  preferences.putUShort("bootcount", bootCount + 1);

  uint16_t sleepCount = preferences.getUShort("sleepcount", 0);  
  shell.print(F("Sleep Count: "));
  shell.println(sleepCount);

  // Get the address loaded from NVRAM
  MY_ADDR = preferences.getUChar("addr", 1);  
  shell.print(F("Node Address: "));
  shell.println(MY_ADDR);

  uint16_t lowBatteryLimitMv = preferences.getUShort("blimit", 0);
  shell.print(F("Battery Limit: "));
  shell.println(lowBatteryLimitMv);

  // Get the initial routing table loaded from NVRAM
  for (int i = 0; i < 256; i++) {
    Routes[i] = 0;
  }
  preferences.getBytes("routes", Routes, 256);

  // Interrupt setup from radio
  // Allocating an external interrupt will always allocate it on the core that does the allocation.
  attachInterrupt(DIO0_PIN, isr, RISING);

  // Initialize SPI and configure
  delay(100);
  spi_setup();

  // Starting here, the process depends on why we are rebooting.
  // Check to see if we are rebooting because of a radio interrupt.
  // There is an assumption that we were in the State::LISTENING 
  // state when the processor was put to sleep.
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    
    // One LED flash
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    // Given that we just came up from an interrupt trigger, setup 
    // the state as if we were waiting for a message, and then 
    // set the flag that will cause the ISR code to run.
    state = State::LISTENING;
    enable_interrupt_RxDone();
    isr_hit = true;
  }
  // Any other reason for a reboot causes full radio reset/initialization
  else {
    // Reset the radio 
    reset_radio();
    delay(250);
 
    // Initialize the radio
    if (init_radio() != 0) {
      Serial.println(F("ERR: Problem with radio initialization"));
    } else {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
  
      // Start listening for messages
      state = State::LISTENING;
      enable_interrupt_RxDone();
      // Put the radio on the receive mode
      set_mode_RXCONTINUOUS();  
    }
  }
  
  // Enable the battery check timer
  timer.every(BATTERY_CHECK_INTERVAL_SECONDS * 1000, check_low_battery);

  // Enable the watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  esp_task_wdt_reset();
}

static void process_rx_msg(const uint8_t* buf, const unsigned int len);
static void check_for_rx_msg();
static void check_for_interrupts();

void loop() {

  // Interrupt stuff
  check_for_interrupts();
 
  // Service the shell
  shell.executeIfInput();
  
  // Service the timer
  timer.tick();

  // Check for radio activity
  event_tick();

  // Look for any received data that should be processed by the application
  check_for_rx_msg();

  // Keep the watchdog alive
  esp_task_wdt_reset();
}

static void check_for_interrupts() {
  // Interrupt stuff
  if (isr_hit) {

    isr_hit = false;

    // Read and reset the IRQ register at the same time:
    uint8_t irq_flags = spi_write(0x12, 0xff);    
    
    // RX timeout - ignored
    if (irq_flags & 0x80) {
    } 
    // RxDone without a CRC error
    if ((irq_flags & 0x40) && !(irq_flags & 0x20)) {
      event_RxDone();
    }
    // TxDone
    if (irq_flags & 0x08) {
      event_TxDone();
    }
  }
}

/**
 * @brief Look to see if any data is available on the RX queue.
 */
static void check_for_rx_msg() {
  unsigned int msg_len = 256;
  uint8_t msg[msg_len];
  boolean notEmpty = rx_buffer.popIfNotEmpty(msg, &msg_len);
  if (notEmpty) {
    process_rx_msg(msg, msg_len);
  }
}

/**
 * @brief Main processing for any message received by the node.
 * 
 * @param buf Bytes received 
 * @param len Length (in bytes)
 */
static void process_rx_msg(const uint8_t* buf, const unsigned int len) {

  if (len < sizeof(Header)) {
    shell.println(msg_bad_message);
    shell.println(len);
    return;
  }

  // Pull the header out
  Header header;
  memcpy(&header, buf, sizeof(Header));

  shell.print(F("INF: Got type: "));
  shell.print(header.type, HEX);
  shell.print(", id: ");
  shell.print(header.id);
  shell.print(", from: ");
  shell.print(header.sourceAddr);
  shell.print(", originalSource: ");
  shell.print(header.originalSourceAddr);
  shell.print(", finalDest: ");
  shell.print(header.finalDestAddr);
  shell.print(", hops: ");
  shell.print(header.hops);
  shell.print(", RSSI: ");
  shell.print(header.receiveRssi);
  shell.println();

  // Look for messages that need to be forwarded on to another node
  if (header.finalDestAddr != MY_ADDR) {
    if (header.finalDestAddr > 0 && header.finalDestAddr < 255) {
      // Look up the route
      uint8_t nextHop = Routes[header.finalDestAddr];
      if (nextHop == 0) {
        shell.println(msg_no_route);
      } else {
        // Copy the original message 
        uint8_t tx_buf[256];
        memcpy(tx_buf, buf, len);
        // Fix the header
        header.destAddr = nextHop;
        header.sourceAddr = MY_ADDR;
        header.hops = header.hops + 1;        
        memcpy(tx_buf, &header, sizeof(Header));
        // Send the entire message
        // Notice that we are using the original length
        tx_buffer.push(tx_buf, len);
      }
    } else {
      shell.println(msg_bad_address);
    }
  }

  // All other messages are being directed to this node
  else {

    // Ping
    if (header.type == MessageType::TYPE_PING) {

      // Create a pong and send back to the originator of the ping
      PongMessage msg;
      // Notice that the first hop is always the node that sent the PING.
      msg.header.destAddr = header.sourceAddr;
      msg.header.sourceAddr = MY_ADDR;
      msg.header.hops = header.hops + 1;        
      msg.header.id = header.id;
      msg.header.type = MessageType::TYPE_PONG;
      msg.header.finalDestAddr = header.originalSourceAddr;
      msg.header.originalSourceAddr = MY_ADDR;

      msg.version = SW_VERSION;
      msg.counter = counter;
      msg.rssi = header.receiveRssi;
      msg.batteryMv = checkBattery();
      msg.panelMv = checkPanel();
      msg.uptimeSeconds = get_time_seconds();
      msg.bootCount = preferences.getUShort("bootcount", 0);  
      msg.sleepCount = preferences.getUShort("sleepcount", 0);  
      
      tx_buffer.push((uint8_t*)&msg, sizeof(PongMessage));
    }
    
    // Pong (for display)
    else if (header.type == MessageType::TYPE_PONG) {
      
      if (len < sizeof(PongMessage)) {
        Serial.println(msg_bad_message);
        return;
      }

      // Re-read the message into the PongMessage format.  
      PongMessage pong;
      memcpy(&pong, buf, sizeof(PongMessage));
      // Display
      shell.print("{ \"counter\": ");
      shell.print(pong.counter, DEC);
      shell.print(", \"origSourceAddr\": ");
      shell.print(pong.header.originalSourceAddr, DEC);
      shell.print(", \"hops\": ");
      shell.print(pong.header.hops, DEC);
      shell.print(", \"version\": ");
      shell.print(pong.version, DEC);
      shell.print(", \"rssi\": ");
      shell.print(pong.rssi, DEC);
      shell.print(", \"batteryMv\": ");
      shell.print(pong.batteryMv, DEC);
      shell.print(", \"panelMv\": ");
      shell.print(pong.panelMv, DEC);
      shell.print(", \"uptimeSeconds\": ");
      shell.print(pong.uptimeSeconds, DEC);
      shell.print(", \"bootCount\": ");
      shell.print(pong.bootCount, DEC);
      shell.print(", \"sleepCount\": ");
      shell.print(pong.sleepCount, DEC);
      shell.println("}");
    }
    
    // Blink the on-board LED
    else if (header.type == MessageType::TYPE_BLINK) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
    }
    
    // Reset
    else if (header.type == MessageType::TYPE_RESET) {
      shell.println(F("INF: Resetting ..."));
      ESP.restart();
    }
    
    // Text (for display)
    else if (header.type == MessageType::TYPE_TEXT) {
      shell.print("MSG: ");
      // There is no null-termination, so we must use the message length here
      for (int i = 0; i < len - sizeof(Header); i++) {
        shell.write(buf[i + sizeof(Header)]);
      }
      shell.println();
    }

    // Set route
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
