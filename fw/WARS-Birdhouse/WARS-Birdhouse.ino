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

#include <esp_task_wdt.h>
#include <SimpleSerialShell.h>
#include <arduino-timer.h>
#include <Preferences.h>

#include "spi_utils.h"
#include "CircularBuffer.h"
#include "ClockImpl.h"
#include "ConfigurationImpl.h"
#include "packets.h"
#include "OutboundPacketManager.h"
#include "Instrumentation.h"
#include "RoutingTableImpl.h"
#include "MessageProcessor.h"
#include "CommandProcessor.h"

#define SW_VERSION 33

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
//#define PANEL_LEVEL_PIN 35 // NODE 5 ONLY!

// Watchdog timeout in seconds (NOTE: I think this time might be off because
// we are changing the CPU clock frequency)
#define WDT_TIMEOUT 5

// The time we will wait for a TxDone interrupt before giving up and resetting
// the transmit process.
#define TX_TIMEOUT_MS 10 * 1000

#define S_TO_US_FACTOR 1000000UL

// Deep sleep duration when low battery is detected
#define DEEP_SLEEP_SECONDS (60UL * 60UL)

// How frequently to check the batter condition
#define BATTERY_CHECK_INTERVAL_SECONDS 30

static const float STATION_FREQUENCY = 906.5;

int reset_radio();

// Used for scheduling events
auto timer = timer_create_default();

// Connect the logger stream to the SimpleSerialShell.  The shell
// global is defined (and externed) in the SimpleSerialShell module.
Stream& logger = shell;

// ===== Interface Classes ===========================================

class InstrumentationImpl : public Instrumentation {
public:

    uint16_t getSoftwareVersion() const { return SW_VERSION; }
    uint16_t getDeviceClass() const { return 2; }
    uint16_t getDeviceRevision() const { return 1; }

    uint16_t getBatteryVoltage() const { 
        // Returns the battery level in mV
        // The voltage is sampled via a 1:2 voltage divider 
        const float scale = (3.3 / 4096.0) * 2.0;
        float level = (float)analogRead(BATTERY_LEVEL_PIN) * scale;
        return level * 1000.0;
    }

    uint16_t getPanelVoltage() const { 
        // The voltage is sampled via a 1:6 voltage divider 
        const float scale = (3.3 / 4096.0) * 6.0;
        float level = (float)analogRead(PANEL_LEVEL_PIN) * scale;
        return level * 1000.0;
    }

    // Option is not implemented yet
    int16_t getTemperature() const { return 0; }
    int16_t getHumidity() const { return  0; }

    void restart() { 
        // This is scheduled in the background to make sure that any final communication
        // has time to happen.
        timer.in(2 * 1000, _backgroundRestart);
        logger.println("INF: Restart scheduled");
    }

    void restartRadio() {
        reset_radio();
    }

    void sleep(uint32_t ms) {
        sleep(ms);
    }

private:

    static bool _backgroundRestart(void*) {
        ESP.restart();
        return false;
    }    
};

// Used for persistent storage
static Preferences nvram;

static ClockImpl mainClock;
Clock& systemClock = mainClock;

static ConfigurationImpl mainConfig(nvram);
Configuration& systemConfig = mainConfig;

static InstrumentationImpl instrumentation;
Instrumentation& systemInstrumentation = instrumentation;

static RoutingTableImpl routingTable(nvram);
RoutingTable& systemRoutingTable = routingTable;

// We keep a pretty small TX buffer because the main area where we keep 
// outbound packets is in the MessageProcessor.
static CircularBufferImpl<256> txBuffer(0);
// There is a two-byte OOB allocation here for the RSSI data on receive
static CircularBufferImpl<2048> rxBuffer(2);

static MessageProcessor messageProcessor(mainClock, 
  rxBuffer, txBuffer, routingTable, instrumentation, mainConfig, 10 * 1000, 2 * 1000);
MessageProcessor& systemMessageProcessor = messageProcessor;

// The states of the state machine
enum State { IDLE, LISTENING, TRANSMITTING };

// The overall state
static State state = State::IDLE;
// This is volatile because it is set inside of the ISR context
static volatile bool isr_hit = false;
// The time when we started the last transmission.  This is needed 
// to create a timeout on transmissions so we don't accidentally get 
// stuck in a transmission.
static uint32_t startTxTime = 0;

/**
 * @brief This is the actual ISR that is called by the Arduino run-time.
 */
void IRAM_ATTR isr() {
  // NOTE: We've had so many problems with interrupt enable/disable on the ESP32
  // so now we're just going to set a flag and let everything happen in the loop()
  // context. This eliminates a lot of risk around race-conditions, etc.
  isr_hit = true;
}

/**
 * @brief This is called when the radio reports the end of a 
 * transmission sequence.  The radio is put back into 
 * continuous receive mode.
 * 
 * This gets called from the main processing loop.
 */
static void event_TxDone() { 

    // Sanity check on state transition
    if (state != State::TRANSMITTING) {
        logger.println(F("ERR: TxDone received in unexpected state"));
        return;
    }
  
    // Revert back to whatever we were listening for. 
    state = State::LISTENING;
    // Ask for interrupt when receiving
    enable_interrupt_RxDone();
    // Go into RXCONTINUOUS so we can hear the response
    set_mode_RXCONTINUOUS();  
} 

/**
 * @brief This is called when a complete message is received.
 * 
 * This gets called from the main processing loop.
 */
static void event_RxDone() {

    // How much data is available?
    const uint8_t len = spi_read(0x13);
    // Reset the FIFO read pointer to the beginning of the packet we just got
    spi_write(0x0d, spi_read(0x10));
    // Stream received data in from the FIFO. 
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
    if (state != State::LISTENING) {
        logger.println(F("ERR: Message received when not listening"));
        return;
    }

    // Put the RSSI (OOB) and the entire packet (not just the header)
    // into the circular queue for later processing.
    rxBuffer.push((const uint8_t*)&lastRssi, rx_buf, len);
}

/**
 * @brief This function gets called from inside of the main processing loop.  It looks
 * to see if any interrupt activitry has been detected and, if so, figiured out what 
 * kind of interrupt was reported and calls the correct handler.
 */ 
static void check_for_interrupts(bool force) {

    // Look at the flag that gets set by the ISR itself
    if (!force && !isr_hit) {
        return;
    }

    // *******************************************************************************
    // Critical Section:
    // Here we make sure that the cleaing of the isr_hit flag and the unloading 
    // of the pending interrupts in the radio's IRQ register happen atomically.
    // We are avoding the case where 
    noInterrupts();

    isr_hit = false;

    // Read and reset the IRQ register at the same time:
    uint8_t irq_flags = spi_write(0x12, 0xff);    

    interrupts();
    // *******************************************************************************
    
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

static void event_tick_LISTENING() {

  // Check for pending transmissions.  If nothing is pending then 
  // return without any state change.
  if (txBuffer.isEmpty()) {
    return;
  }
    
  // At this point we have something pending to be sent.
  // Go into stand-by so we know that nothing else is coming in
  set_mode_STDBY();

  // Pop the data off the TX queue into the transmit buffer.  
  unsigned int tx_buf_len = 256;
  uint8_t tx_buf[tx_buf_len];
  txBuffer.pop(0, tx_buf, &tx_buf_len);

  // Move the data into the radio FIFO
  write_message(tx_buf, tx_buf_len);
  
  // Go into transmit mode
  state = State::TRANSMITTING;
  startTxTime = mainClock.time();
  enable_interrupt_TxDone();
  set_mode_TX();
}

static void event_tick_TRANSMITTING() {
    // Check for the case where a transmission times out
    if (mainClock.time() - startTxTime > TX_TIMEOUT_MS) {
          logger.println("ERR: TX radio time out");
          // Reset the TX FIFO
          spi_write(0x0e, 0);
          // Force a TX done
          event_TxDone();
    }
}

// Call periodically to look for timeouts or other pending activity.  This will happen
// on the regular application thread, so we disable interrupts to avoid conflicts.
void event_tick() {
    if (state == State::LISTENING) {
        event_tick_LISTENING();
    } else if (state == State::TRANSMITTING) {
        event_tick_TRANSMITTING();
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
    // Not sure if this is really needed:
    delay(250);

    // Initialize the radio
    if (init_radio() != 0) {
      logger.println(F("ERR: Problem with radio initialization"));
      return -1;
    }

    logger.println(F("INF: Radio initialized"));

    // Flash the LED as a diagnostic indicator 
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
  uint16_t lowBatteryLimitMv = systemConfig.getBatteryLimit();
  // Check the battery
  uint16_t battery = systemInstrumentation.getBatteryVoltage();
  // If the battery is low then deep sleep
  if (lowBatteryLimitMv != 0 && battery < lowBatteryLimitMv) {
    shell.println(F("INF: Low battery, entering deep sleep"));
    // Keep track of how many times this has happened
    systemConfig.setSleepCount(systemConfig.getSleepCount() + 1);
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

static bool check_stranded_irq(void*) {
    check_for_interrupts(true);
    return true;
}

void setup() {

    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println(F("WARS Birdhouse Mesh Network (c) 2022 Bruce MacKinnon KC1FSZ"));
    Serial.print(F("V: "));
    Serial.print(SW_VERSION);
    Serial.print(", ");
    Serial.println(__DATE__);
    Serial.println();

    // Figure out why we restarted
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason){
        case 0:
            break;
        case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup EXT0"); break;
        case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup EXT1"); break;
        case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup Timer"); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup Touch"); break;
        case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup ULP"); break;
        default: Serial.printf("Wakeup %d\n",wakeup_reason); break;
    }
  
    // Do all of the one-time initialization
    nvram.begin("my-app", false);
    mainConfig.begin();
    routingTable.begin();

    // Radio interrupt pin
    pinMode(DIO0_PIN, INPUT);
  
    // LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
  
    // Shell setup
    shell.attach(Serial); 
    shell.setTokenizer(tokenizer);
    shell.addCommand(F("ping <addr>"), sendPing);
    shell.addCommand(F("sed <addr>"), sendGetSed);
    shell.addCommand(F("reset <addr> <passcode>"), sendReset);
    shell.addCommand(F("text <addr> <text>"), sendText);
    shell.addCommand(F("boot"), boot);
    shell.addCommand(F("bootradio"), bootRadio);
    shell.addCommand(F("info"), info);
    shell.addCommand(F("sleep <seconds>"), sleep);
    shell.addCommand(F("setaddr <addr>"), setAddr);
    shell.addCommand(F("setcall <call_sign>"), setCall);
    shell.addCommand(F("setroute <target addr> <next hop addr> <passcode>"), setRoute);
    shell.addCommand(F("clearroutes"), clearRoutes);
    shell.addCommand(F("setblimit <limit_mv>"), setBatteryLimit);
    shell.addCommand(F("print <text>"), doPrint);
    shell.addCommand(F("rem <text>"), doRem);
    shell.addCommand(F("setrouteremote <addr> <target addr> <next hop addr>"), sendSetRoute);
    shell.addCommand(F("getrouteremote <addr> <target addr>"), sendGetRoute);
    shell.addCommand(F("resetcounters"), doResetCounters);

    // Increment the boot count
    systemConfig.setBootCount(systemConfig.getBootCount() + 1);
  
    // Interrupt setup from radio
    // Allocating an external interrupt will always allocate it on the core that does the allocation.
    attachInterrupt(DIO0_PIN, isr, RISING);
  
    // Initialize SPI and configure
    delay(100);
    spi_setup();

    // Make sure the address and callsign are valid before allowing the radio to turn on
    if (!systemConfig.getCall().isValid() || 
        systemConfig.getAddr() == 0) {
        logger.println("ERR: Station not configured.");      
    }
    else {  
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
        }
    }
        
    // Enable the battery check timer
    timer.every(BATTERY_CHECK_INTERVAL_SECONDS * 1000, check_low_battery);

    // Enable a periodic interrupt check (to make sure that we don't 
    // accidentally leave an interrupt stranded in the IRQ
    //timer.every(30 * 1000, check_stranded_irq);
   
    // Enable the watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    esp_task_wdt_reset();
}

void loop() {

  // Interrupt stuff, no forcing
  check_for_interrupts(false);
 
  // Check for shell activity
  shell.executeIfInput();
  
  // Service the timers
  timer.tick();

  // Check for radio activity
  event_tick();

  // Perform any message processing that is pending
  systemMessageProcessor.pump();

  // Keep the watchdog alive
  esp_task_wdt_reset();
}
