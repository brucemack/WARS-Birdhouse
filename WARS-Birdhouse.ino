// LoRa Birdhouse Mesh Network Project
// Wellesley Amateur Radio Society
//
// Current measurements:
//
// 80mA idle, 160mA transmit with normal clock
// 30mA at 10 MHz clock rate

// Prerequsisites to Install 
// * SimpleSerialShell

#include <SPI.h>
#include "WiFi.h"
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <SimpleSerialShell.h>

#define SW_VERSION 6
#define MY_ADDR 1

// ===== TO BE MOVED =======

// A circular queue for byte buffers of arbitray length. 
template<unsigned int S> class CircularBuffer {
public:

  CircularBuffer() 
  : _front(0),
    _back(0)
  {
  }

  bool isEmpty() {
    return _front == _back;
  }

  // Returns true if the push was successful, otherwise false.  For example, if the buffer
  // is completely full.
  // NOTE: A two-byte length header is put into the buffer to manage size.
  bool push(const uint8_t* buf, unsigned int len) {
    // Keep the original pointer in case a failure/rollback is necessary.
    unsigned int original_back = _back;
    // Write a two-byte length
    uint8_t temp[2];
    temp[0] = (len >> 8) & 0xff;
    temp[1] = len & 0xff;
    if (_pushRaw(temp, 2) && _pushRaw(buf, len)) {
      return true;
    }
    // Reset like nothing happened
    _back = original_back;
    return false;
  }

  // The *len argument starts off with the maximum space available in buf and ends
  // with the actual number of bytes taken from the queue.
  void pop(uint8_t* buf, unsigned int* len) {
    unsigned int available_space = *len;
    unsigned int ptr = _front;
    // Get out the length
    unsigned int entry_size = 0;
    entry_size = _buf[ptr] << 8;
    ptr = (ptr + 1) % _bufSize;
    entry_size |= _buf[ptr];
    ptr = (ptr + 1) % _bufSize;
    
    *len = 0;
    
    for (int i = 0; i < entry_size; i++) {
      if (i < available_space) {
        buf[i] = _buf[ptr];
        (*len)++;
      }
      ptr = (ptr + 1) % _bufSize;
    }
    
    _front = ptr;
  }
  
private:

  // Where we pop (read) from
  unsigned int _front;
  // Where we push (write) to
  unsigned int _back;
  // The actual space
  const unsigned int _bufSize = S;
  uint8_t _buf[S];

  bool _pushRaw(const uint8_t* buf, unsigned int len) {
    for (unsigned int i = 0; i < len; i++) {
      // Store into the buffer
      _buf[_back] = buf[i];
      // Advance and wrap
      _back = (_back + 1) % _bufSize;
      // Check for overflow
      if (_back == _front) {
        return false;
      }
    }
    return true;
  }
};
  
#define SS_PIN    5
#define RST_PIN   14
#define DIO0_PIN  4
#define LED_PIN   2
#define BATTERY_LEVEL_PIN 33

// ----- SPI Stuff ---------------------------------------------------

SPISettings spi_settings(1000000, MSBFIRST, SPI_MODE0);

uint8_t spi_read(uint8_t reg) {
  // Make sure we don't interrupt in a transaction
  noInterrupts();
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask off
  SPI.transfer(reg & ~0x80); 
  // The written value is ignored, reg value is read
  uint8_t val = SPI.transfer(0); 
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  // Re-enable interrupts
  interrupts();
  return val;
}

void spi_read_multi(uint8_t reg, uint8_t* buf, uint8_t len) {
  // Make sure we don't interrupt in a transaction
  noInterrupts();
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask off
  SPI.transfer(reg & ~0x80); 
  while (len--) {
    // The written value is ignored, reg value is read
    *buf = SPI.transfer(0); 
    buf++;
  }
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  // Re-enable interrupts
  interrupts();
}

// Writes one byte to SPI.  Returns whatever comes back during the transfer.
//
// NOTE: From RFM95W datasheet (pg 76):
//
// During the write access, the byte transferred from the slave to the master on the MISO line 
// is the value of the written register before the write operation.
//
uint8_t spi_write(uint8_t reg, uint8_t val) {
  // Make sure we don't interrupt in a transaction
  noInterrupts();
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask on. 
  SPI.transfer(reg | 0x80);
  // Send the data, capturing the original value 
  uint8_t orig_val = SPI.transfer(val); 
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  // Re-enable interrupts
  interrupts();
  return orig_val;
}

uint8_t spi_write_multi(uint8_t reg, uint8_t* buf, uint8_t len) {
  // Make sure we don't interrupt in a transaction
  noInterrupts();
  SPI.beginTransaction(spi_settings);
  // Slave Select
  digitalWrite(SS_PIN, LOW);
  // Send the address with the write mask on
  uint8_t stat = SPI.transfer(reg | 0x80); 
  // Transfer each byte individually
  while (len--) {
    SPI.transfer(*buf);
    buf++;
  }
  // Slave deselect
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  // Re-enable interrupts
  interrupts();
  return stat;
}

void event_txdone();
void event_rxdone();

unsigned int isr_count = 0;
unsigned int last_irq = 0;
uint8_t rx_buf[256];
unsigned int rx_buf_len = 0;
uint8_t tx_buf[256];
unsigned int tx_buf_len = 0;

// The states of the state machine
enum State { IDLE, LISTENING, TRANSMITTING, LISTENING_FOR_ACK };

static State state = State::IDLE;

CircularBuffer<512> tx_buffer;

void (*rx_callback)(uint8_t*, unsigned int) = 0;

// ----- Interrupt Service -------

void IRAM_ATTR isr() {

  // Read and reset the IRQ register at the same time:
  uint8_t irq_flags = spi_write(0x12, 0xff);
    
  isr_count += 1;
  last_irq = irq_flags;
  
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

// Call periodically to look for timeouts or other pending activity
void event_tick() {
  // If we are in the normal listening state then we can start a transmit
  if (state == State::LISTENING) {
    // Check for pending writes
    if (!tx_buffer.isEmpty()) {
      // Go into stand-by so we know that nothing else is coming in
      set_mode_STDBY();
      // Pull the data off the queue into the transmit buffer.  We do this so
      // that re-transmits can be supported if necessary.
      tx_buf_len = 256;
      tx_buffer.pop(tx_buf, &tx_buf_len);
      // Move the data into the radio FIFO
      write_message(tx_buf, tx_buf_len);
      // Go into transmit mode
      state = State::TRANSMITTING;
      enable_interrupt_TxDone();
      set_mode_TX();
    }
  }
  else {
    //Serial.println("Not listening");
  }
}

void event_TxDone() { 
  Serial.println("event_TxDone");
  // Waiting for an message transmit to finish successfull
  if (state == State::TRANSMITTING) {
    state = State::LISTENING;
    // Ask for interrupt when receiving
    enable_interrupt_RxDone();
    // Go into RXCONTINUOUS so we can hear the response
    set_mode_RXCONTINUOUS();
    
  } 
} 

void event_RxDone() {
  
  Serial.println("event_RxDone");
  
  // How much data is available?
  uint8_t len = spi_read(0x13);
  // Reset the FIFO read pointer to the beginning of the packet we just got
  spi_write(0x0d, spi_read(0x10));
  // Stream in from the FIFO
  spi_read_multi(0x00, rx_buf, len);
  rx_buf_len = len;

  // Look at the message and determine if it belongs to us
  if (rx_buf_len > 0 && (rx_buf[0] == MY_ADDR || rx_buf[0] == 255)) {

    Serial.println("mine");
      
    // Handle based on state
    if (state == State::LISTENING) {
      // FIRE CALLBACK
      if (rx_callback != 0) {
        rx_callback(rx_buf, rx_buf_len);
      }
      else {
        Serial.println("No callback");
      }
    }
    else {
      Serial.println("Not listening");
    }
  } else {
    Serial.println("Ignored message");
  }
}

// --------------------------------------------------------------------------
// Radio Utilty Functions
// 
// This reference will be very important to you:
// https://www.hoperf.com/data/upload/portal/20190730/RFM95W-V2.0.pdf
// 
// The LoRa register map starts on page 103.

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

  // Preable Length=8 (default)

  // Bw=31.25kHz, CodingRate=4/5, ImplicitHeaderModeOn=Explicit header)
  //reg = 0b01000010;
  // Bw=125, CodingRate=4/5, ImplicitHeaderModeOn=Explicit header)
  reg = 0x72;
  spi_write(0x1d, reg);

  // SpreadingFactor=9, RxContinuousMode=Normal, RxPayloadCrcOn=Enable)
  reg = 0b10010100;
  spi_write(0x1e, reg);

  // AgcAutoOn=LNA gain set by AGC
  reg = 0b00000100;
  spi_write(0x26, reg);
 
  // Set freq
  set_frequency(916);
  
  // Adjust over-current protection
  spi_write(0x0b, 0x31);

  // DAC enable (adds 3dB)
  spi_write(0x4d, 0x87);
  
  // Turn on PA and set power to +20dB
  // PaSelect=1
  // OutputPower=18
  //   NOTE: Actual Power=(17-(15-OutputPower)) = 2 + OutputPower = 20
  spi_write(0x09, 0x80 | 18);

  set_frequency(916.0);
  
  return 0;
}

// ===== Application 

static auto msg_arg_error = F("Argument error");

int sendReset(int argc, char **argv) { 

  if (argc == 2) {

    uint8_t targetNodeAddr = atoi(argv[1]);
    
    uint8_t data[32];
    // VERSION
    data[0] = SW_VERSION;
    // COMMAND
    data[1] = 4;

    shell.print("Resetting node ");
    shell.println(targetNodeAddr);

    
  } else {
    shell.println(msg_arg_error);
  }
}

int sendPing(int argc, char **argv) { 

  static int counter = 0;
  
  if (argc == 2) {

    uint8_t targetNodeAddr;
    
    if (argv[1][0] == '*') 
      targetNodeAddr = 255;
    else 
      targetNodeAddr = atoi(argv[1]);

    shell.print("Pinging node ");
    shell.println(targetNodeAddr);

    uint8_t msg[5];
    msg[0] = targetNodeAddr;
    msg[1] = MY_ADDR;
    msg[2] = 0;
    msg[3] = 0;
    msg[4] = 1;
    tx_buffer.push(msg, 5);
    
  } else {
    shell.println(msg_arg_error);
  }
}

int boot(int argc, char **argv) { 
    shell.println("Asked to reboot ...");
    ESP.restart();
}

static void show_rx_msg(uint8_t* buf, unsigned int len) {

  if (len >= 5) {
    
    uint8_t from_addr = buf[1];
    unsigned int id = (buf[2] << 8);
    id |= buf[3];
    uint8_t type = buf[4];

    Serial.print("Got type: ");
    Serial.print(type);
    Serial.print(", id: ");
    Serial.print(id);
    Serial.print(", from: ");
    Serial.println(from_addr);
    
    // Ping
    if (type == 1) {
      // Create a pong and send back
      uint8_t msg[5];
      msg[0] = from_addr;
      msg[1] = MY_ADDR;
      msg[2] = (id >> 8) & 0xff;
      msg[3] = (id & 0xff);
      msg[4] = 2;
      tx_buffer.push(msg, 5);
    }
    // Pong
    else if (type == 2) {
    }

    else {
      Serial.println("Unknown message");
    }
  }
  else {
    Serial.println("Invalid message");
  }
}

void setup() {

  delay(1000);
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("KC1FSZ LoRa Mesh System"));
  Serial.print(F("Node "));
  Serial.println(MY_ADDR);

  // Slow down ESP32 to 10 MHz in order to reduce battery consumption
  setCpuFrequencyMhz(10);
  // Turn off WIFI and BlueTooth to reduce power 
  WiFi.mode(WIFI_OFF);
  btStop();

  // SPI slave select
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  // Radio interrupt pin
  pinMode(DIO0_PIN, INPUT);

  // LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Shell setup
  shell.attach(Serial); 
  shell.addCommand(F("ping"), sendPing);
  shell.addCommand(F("reset"), sendReset);
  shell.addCommand(F("boot"), boot);

  // Interrupt setup from radio
  // Allocating an external interrupt will always allocate it on the core that does the allocation.
  attachInterrupt(DIO0_PIN, isr, RISING);

  // Initialize SPI and configure
  delay(100);
  SPI.begin();
  
  // Reset the radio 
  reset_radio();

  // Initialize the radio
  if (init_radio() != 0) {
    Serial.println("Problem with initialization");
  }
  else {
    Serial.println("Radio initialized");
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }

  // Start listening for messages
  rx_callback = show_rx_msg;
  state = State::LISTENING;
  enable_interrupt_RxDone();
  set_mode_RXCONTINUOUS();
}

void loop() {

  event_tick();

  // Service the shell
  shell.executeIfInput();

  //Serial.print(isr_count);
  //Serial.print(" ");
  //Serial.print(last_irq);
  //Serial.print(" ");
  //Serial.println(state);
}
