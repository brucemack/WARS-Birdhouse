// LoRa Birdhouse Mesh Network Project
// Wellesley Amateur Radio Society
//
// Current measurements:
//
// 80mA idle, 160mA transmit with normal clock
// 30mA at 10 MHz clock rate

// Prerequsisites to Install 
// * SimpleSerialShell

// Build instructions
// * Set clock frequency to 10MHz to save power

#include "WiFi.h"
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <SimpleSerialShell.h>
#include "CircularBuffer.h"
#include "spi_utils.h"

#define SW_VERSION 21

//#define RST_PIN   14
// This is the pin that is available on the D1 Mini module:
#define RST_PIN   26
#define DIO0_PIN  4
#define LED_PIN   2
#define BATTERY_LEVEL_PIN 33

// Watchdog timeout in seconds (NOTE: I think this time might be off because
// we are changing the CPU clock frequency)
#define WDT_TIMEOUT 5

#define US_TO_S_FACTOR 1000000

// Deep sleep duration when low battery is detected
#define DEEP_SLEEP_SECONDS 60 * 30

static Preferences preferences;

// NODE SPECIFIC STUFF
static uint8_t MY_ADDR = 1;
// Routing table (node 1)
static uint8_t Routes[256];

static int32_t get_time_seconds() {
  return esp_timer_get_time() / 1000000L;
}

// Returns the battery level in mV
uint16_t checkBattery() {
  const float batteryScale = (3.3 / 4096.0) * 2.0;
  float batteryLevel = (float)analogRead(BATTERY_LEVEL_PIN) * batteryScale;
  return batteryLevel * 1000.0;
}

// The states of the state machine
enum State { IDLE, LISTENING, TRANSMITTING, TRANSMITTING_ACK, LISTENING_FOR_ACK };

static State state = State::IDLE;

// The state we need to go into after a transmit is successful
static State stateAfterTransmit = State::LISTENING;
// This is the message ID that we are waiting to have acknowledged.
static uint16_t listenAckId = 0;
// This is the time we started waiting for the ACK. Used for timeout tracking.
static uint32_t listenAckStart = 0;
// The number of times we've retried the transmit
static uint16_t transmitRetry = 0;
// The number of ACKs to skip (used for testing)
static uint16_t skipAckCount = 0;

// The circular buffer used for outgoing data
static CircularBuffer<4096> tx_buffer;
// The circular buffer used for incoming data
static CircularBuffer<4096> rx_buffer;

enum MessageType {
  // This message is used when a delivery acknowlegement is needed
  TYPE_ACK   = 0b00000001,
  // The ping message will be acknowledged at every step along the way
  TYPE_PING  = 0b10000010,
  // Response is acknowledged as well
  TYPE_PONG  = 0b10000011,
  TYPE_RESET = 0b00000100,
  TYPE_TEXT  = 0b10000101,
  TYPE_BLINK = 0b00000110
};

// Every message starts of with this header
struct Header {

  Header() 
  : id(0),
    hops(0),
    receiveRssi(0) {
  }
  
  uint8_t destAddr;
  uint8_t sourceAddr;
  // The high bit controls whether an acknowledgement is required
  uint8_t type;
  uint16_t id;
  // Used for multi-hop communication.  This is the node that is 
  // the ultimate destination of a message.
  uint8_t finalDestAddr;
  // Used for multi-hop communication.  This is the node that 
  // originated the message.
  uint8_t originalSourceAddr;
  // Used for multi-hop communication.  This is the number of hops
  // that the packet has traversed.
  uint8_t hops;
  // When possible this will be filled in on the *local* side of the 
  // receive.
  int16_t receiveRssi;

  bool isAckRequired() {
    return (type & 0b10000000) != 0;
  }
};

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
  uint8_t len = spi_read(0x13);
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

  // Ignore messsages targeted for other stations
  if (rx_buf[0] != MY_ADDR && rx_buf[0] != 255) {
    //Serial.println(F("INF: Ignored message for another node"));
    return;
  }

  // Pull in the header 
  Header rx_header;
  memcpy(&rx_header, rx_buf, sizeof(Header));
  // Tweak the RSSI attribute in the header 
  rx_header.receiveRssi = lastRssi;
  // Write the fixed header back into the receive buffer
  memcpy(rx_buf, &rx_header, sizeof(Header));

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
      Serial.print(F("INF: Ignorning ACK for "));
      Serial.println(rx_header.id);
    }
  } 
  // Any other message will be forwarded to the application
  else {
    // Put the data into the circular queue
    rx_buffer.push(rx_buf, len);

    // Check to see if an ACK was requested by the sender.  If so, create the ACK
    // and transmit it.
    if (rx_header.isAckRequired()) {

      if (skipAckCount != 0) {
        skipAckCount--;
        Serial.println(F("INF: Skipping an ACK (test)"));
      }
      else {
        Header ack_header;
        // Respond to the node that sent us the message
        ack_header.destAddr = rx_header.sourceAddr;
        ack_header.sourceAddr = MY_ADDR;
        ack_header.type = MessageType::TYPE_ACK;
        // Maintain the same ID
        ack_header.id = rx_header.id;
        ack_header.finalDestAddr = rx_header.sourceAddr;
        ack_header.originalSourceAddr = MY_ADDR;
        ack_header.hops = rx_header.hops + 1;
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
  Header tx_header;
  ::memcpy((uint8_t*)&tx_header, tx_buf, sizeof(Header));

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
const float CRYSTAL_MHZ = 32000000.0;
const float FREQ_STEP = (CRYSTAL_MHZ / 524288);

// See page 103
void set_frequency(float freq_mhz) {
  uint32_t f = (freq_mhz * 1000000.0) / FREQ_STEP;
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
 *  All of the one-time initializaiton of the radio
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

  // Go into stand-by
  set_mode_STDBY();

  // Configure the radio
  uint8_t reg = 0;

  // Bw=31.25kHz, CodingRate=4/5, ImplicitHeaderModeOn=Explicit header)
  // reg = 0b01000010;
  // Bw=125, CodingRate=4/5, ImplicitHeaderModeOn=Explicit header)
  reg = 0x72;
  spi_write(0x1d, reg);

  // SpreadingFactor=9, RxContinuousMode=Normal, RxPayloadCrcOn=Enable)
  reg = 0b10010100;
  spi_write(0x1e, reg);

  // AgcAutoOn=LNA gain set by AGC
  reg = 0b00000100;
  spi_write(0x26, reg);

  // Preable Length=8 (default)
  // Preamble MSB and LSB
  spi_write(0x20, 8 >> 8);
  spi_write(0x21, 8 & 0xff);

  setLowDatarate();

  // DAC enable (adds 3dB)
  spi_write(0x4d, 0x07);
  
  // Turn on PA and set power to +20dB
  // PaSelect=1
  // OutputPower=17 (20 - 3dB from DAC)
  //   NOTE: Actual Power=(17-(15-OutputPower)) = 2 + OutputPower = 20
  spi_write(0x09, 0x80 | ((20 - 3) - 2));

  set_frequency(916.0);

  // Adjust over-current protection
  spi_write(0x0b, 0x31);

  return 0;
}

// ===== Application 

static auto msg_arg_error = F("ERR: Argument error");
static int counter = 0;
static uint32_t lastLowBatteryCheckSeconds = 0;
static uint32_t chipId = 0;

struct PingMessage {
  Header header;  
};

struct PongMessage {
  Header header;  
  uint16_t version;
  uint16_t counter;
  int16_t rssi;
  uint16_t batteryMv;
  uint16_t panelMv;
  uint32_t uptimeSeconds;
  uint16_t bootCount;
  uint16_t sleepCount;
  //uint32_t macAddress;
};

struct ResetMessage {
  Header header;  
};

struct BlinkMessage {
  Header header;  
};

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

      PingMessage msg;
      msg.header.destAddr = nextHop;
      msg.header.sourceAddr = MY_ADDR;
      msg.header.id = counter;
      msg.header.type = MessageType::TYPE_PING;
      msg.header.finalDestAddr = target;
      msg.header.originalSourceAddr = MY_ADDR;

      tx_buffer.push((uint8_t*)&msg, sizeof(PingMessage));
      
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

  counter++;
  uint8_t tx_buf[256];

  Header header;
  header.destAddr = nextHop;
  header.sourceAddr = MY_ADDR;
  header.id = counter;
  header.type = MessageType::TYPE_TEXT;
  header.finalDestAddr = target;
  header.originalSourceAddr = MY_ADDR;

  memcpy(tx_buf, &header, sizeof(Header));
  memcpy(tx_buf + sizeof(Header), argv[2], textLen);

  tx_buffer.push(tx_buf, sizeof(Header) + textLen);
      
  return 0;
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

void setup() {

  delay(1000);
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println(F("====================="));
  Serial.println(F("KC1FSZ LoRa Mesh Node"));
  Serial.println(F("====================="));
  Serial.println();

  Serial.print("Version: ");
  Serial.println(SW_VERSION);

  Serial.print("Build: ");
  Serial.println(__DATE__);

  // Extract the MAC address of the chip
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.print("MAC: ");
  Serial.println(chipId, HEX);

  preferences.begin("my-app", false); 

  // Radio interrupt pin
  pinMode(DIO0_PIN, INPUT);

  // LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Shell setup
  shell.attach(Serial); 
  shell.setTokenizer(tokenizer);
  //shell.setPrompt("-> ");
  shell.addCommand(F("ping"), F("<addr>"), sendPing);
  shell.addCommand(F("reset"), sendReset);
  shell.addCommand(F("blink"), sendBlink);
  shell.addCommand(F("text"), sendText);
  shell.addCommand(F("boot"), boot);
  shell.addCommand(F("bootradio"), bootRadio);
  shell.addCommand(F("info"), info);
  shell.addCommand(F("sleep"), F("<seconds>"), sleep);
  shell.addCommand(F("skipacks"), skipAcks);
  shell.addCommand(F("setaddr"), F("<addr>"), setAddr);
  shell.addCommand(F("setroute"), F("<target addr> <next hop addr>"), setRoute);
  shell.addCommand(F("clearroutes"), clearRoutes);
  shell.addCommand(F("setblimit"), setBlimit);
  shell.addCommand(F("print"), doPrint);

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

  // Get the initial routing table loaded from NVRAM
  for (int i = 0; i < 256; i++)
    Routes[i] = 0;
  preferences.getBytes("routes", Routes, 256);

  // Interrupt setup from radio
  // Allocating an external interrupt will always allocate it on the core that does the allocation.
  attachInterrupt(DIO0_PIN, isr, RISING);

  // Initialize SPI and configure
  delay(100);
  spi_setup();
  
  // Reset the radio 
  reset_radio();
  delay(250);

  // Initialize the radio
  if (init_radio() != 0) {
    Serial.println("ERR: Problem with radio initialization");
  }
  else {
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
    set_mode_RXCONTINUOUS();  
  }

  // Enable the watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  esp_task_wdt_reset();

  // Make sure we don't check the battery immediately
  lastLowBatteryCheckSeconds = get_time_seconds();
}

static void process_rx_msg(const uint8_t* buf, const unsigned int len);
static void check_for_rx_msg();
static void check_low_battery();

void loop() {

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

  // Service the shell
  shell.executeIfInput();
  
  // Check for radio activity
  event_tick();

  // Look for any received data that should be processed by the application
  check_for_rx_msg();

  // Check for low battery condition.  But we don't check this on every loop.
  if (get_time_seconds() - lastLowBatteryCheckSeconds > 30) {
    check_low_battery();
    lastLowBatteryCheckSeconds = get_time_seconds();
  }

  // Keep the watchdog alive
  esp_task_wdt_reset();
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

static void process_rx_msg(const uint8_t* buf, const unsigned int len) {

  if (len >= sizeof(Header)) {

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

    // Look for messages that need to be forwarded on
    if (header.finalDestAddr != MY_ADDR) {
      if (header.finalDestAddr > 0 && header.finalDestAddr < 255) {
        // Look up the route
        uint8_t nextHop = Routes[header.finalDestAddr];
        if (nextHop == 0) {
          shell.println(F("ERR: No route available"));
        }
        else {
          shell.print(F("INF: Routing via "));
          shell.println(nextHop);
          // Copy the original message (minus the RSSI information)
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
        shell.println(F("ERR: Invalid address"));
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
        msg.panelMv = 0;
        msg.uptimeSeconds = get_time_seconds();
        msg.bootCount = preferences.getUShort("bootcount", 0);  
        msg.sleepCount = preferences.getUShort("sleepcount", 0);  
        //msg.macAddress = chipId;
        
        tx_buffer.push((uint8_t*)&msg, sizeof(PongMessage));
      }
      // Pong
      else if (header.type == MessageType::TYPE_PONG) {
        
        if (len >= sizeof(PongMessage)) {

          // Re-read the message into the PongMessage format.  
          PongMessage pong;
          memcpy(&pong, buf, sizeof(PongMessage));
  
          Serial.print("{ \"counter\": ");
          Serial.print(pong.counter, DEC);
          Serial.print(", \"origSourceAddr\": ");
          Serial.print(pong.header.originalSourceAddr, DEC);
          Serial.print(", \"hops\": ");
          Serial.print(pong.header.hops, DEC);
          Serial.print(", \"version\": ");
          Serial.print(pong.version, DEC);
          Serial.print(", \"rssi\": ");
          Serial.print(pong.rssi, DEC);
          Serial.print(", \"batteryMv\": ");
          Serial.print(pong.batteryMv, DEC);
          Serial.print(", \"panelMv\": ");
          Serial.print(pong.panelMv, DEC);
          Serial.print(", \"uptimeSeconds\": ");
          Serial.print(pong.uptimeSeconds, DEC);
          Serial.print(", \"bootCount\": ");
          Serial.print(pong.bootCount, DEC);
          Serial.print(", \"sleepCount\": ");
          Serial.print(pong.sleepCount, DEC);
          Serial.println("}");
          //Serial.print(", \"mac\": \"");
          //Serial.print(pong.macAddress, HEX);
          //Serial.println("\" }");
        }
        else {
          Serial.println(F("ERR: Pong too short"));
        }
      }
      // Blink the on-board LED
      else if (header.type == MessageType::TYPE_BLINK) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
      }
      // Reset
      else if (header.type == MessageType::TYPE_RESET) {
        shell.println(F("Resetting ..."));
        ESP.restart();
      }
      // Text
      else if (header.type == MessageType::TYPE_TEXT) {
        shell.print("MSG: ");
        // There is no null-termination, so we must use the message length here
        for (int i = 0; i < len - sizeof(Header); i++) {
          shell.write(buf[i + sizeof(Header)]);
        }
        shell.println();
      }
      else {
        shell.println(F("ERR: Unknown message"));
      }
    }
  }
  else {
    shell.println(F("ERR: Invalid message received"));
    shell.println(len);
  }
}

/**
 * @brief Checks to see if the battery voltage is below the 
 * configured limited.  If so, put the radio in SLEEP mode
 * and tell the ESP32 to go into deep sleep.
 */
static void check_low_battery() {
  uint16_t lowBatteryLimitMv = preferences.getUShort("blimit", 0);
  // Check the battery
  uint16_t battery = checkBattery();
  // If the battery is low then deep sleep
  if (lowBatteryLimitMv != 0 && battery < lowBatteryLimitMv) {
    shell.println(F("INF: Low battery detected"));
    // Keep track of how many times this has happened
    uint16_t sleepCount = preferences.getUShort("sleepcount", 0);  
    preferences.putUShort("sleepcount", sleepCount + 1);   
    // Put the radio into SLEEP mode
    set_mode_SLEEP();
    // Put the ESP32 into a deep sleep that will be awakened using the timer.
    // Wakeup will look like reboot.
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_SECONDS * US_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}
