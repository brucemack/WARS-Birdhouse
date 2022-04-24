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

// Prerequisites to Install 
// * philj404:SimpleSerialShell
// * contrem:arduino-timer

// Build instructions
// * Set clock frequency to 10MHz to save power (also disables WIFI and BT)

// NSS 5
// SCK 18
// MOSI 23
// MISO 19

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

#define SW_VERSION 46

// This is the pin that is available on the D1 Mini module:
#define RST_PIN   26
// This is the ESP32 pin that is connected to the DIO0 pin on the RFM95W module.
#define DIO0_PIN  4
// NOTE: GPIO36 is the same as RTC_GPIO0 
// Generally the on-board LED on the ESP32 module
#define LED_PIN   2
// Analog input pin for measuring battery, connected via 1:2 voltage divider
#define BATTERY_LEVEL_PIN 33
// Analog input pin for measuring panel, connected via 1:6 voltage divider
#define PANEL_LEVEL_PIN 34
// NODE 5 ONLY!!!
//#define PANEL_LEVEL_PIN 35

// Watchdog timeout in seconds (NOTE: I think this time might be off because
// we are changing the CPU clock frequency)
#define WDT_TIMEOUT 5

// The time we will wait for a TxDone interrupt before giving up.  This should
// be an unusual case.
#define TX_TIMEOUT_MS (30UL * 1000UL)
// The time we will wait for a CadDone interrupt before giving up. 
#define CAD_TIMEOUT_MS 50

#define S_TO_US_FACTOR 1000000UL

// Deep sleep duration when low battery is detected
#define DEEP_SLEEP_SECONDS (60UL * 60UL)

// How frequently to check the battery condition
#define BATTERY_CHECK_INTERVAL_SECONDS 30

// How frequently to check for idle potential
#define IDLE_CHECK_INTERVAL_SECONDS 15

// The definition of "idle" 
#define IDLE_INTERVAL_SECONDS 60

#define CONTROL_NODE 1
// How often the station sends its ID information to the control station
#define STATION_ID_INTERVAL_SECONDS (60UL * 60UL)

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
  rxBuffer, txBuffer, routingTable, instrumentation, mainConfig, 20 * 1000, 2 * 1000);
MessageProcessor& systemMessageProcessor = messageProcessor;

// The states of the state machine
enum State { IDLE, RX_STATE, TX_STATE, CAD };

// The overall state
static volatile State state = State::IDLE;
// This is volatile because it is set inside of the ISR context
static volatile bool isrHit = false;
// The time when we started the last transmission.  This is needed 
// to create a timeout on transmissions so we don't accidentally get 
// stuck in a transmission.
static volatile uint32_t startTxTime = 0;
// The time when we should give up on the CAD (channel activity detect).
static volatile uint32_t endCadTime = 0;

/**
 * @brief This is the actual ISR that is called by the Arduino run-time.
 */
void IRAM_ATTR isr() {
    // NOTE: We've had so many problems with interrupt enable/disable on the ESP32
    // so now we're just going to set a flag and let everything happen in the loop()
    // context. This eliminates a lot of risk around concurrency, etc.
    isrHit = true;
}

static void start_Tx() {

    //logger.println("start_Tx");

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
    state = State::TX_STATE;
    startTxTime = mainClock.time();
    enable_interrupt_TxDone();
    set_mode_TX();
}

/**
 * @brief Put the radio RXCONTINUOUS mode and enable the RxDone interrupt.
 */
static void start_Rx() {

    //logger.println("start_Rx");

    // Revert back to listening mode. 
    state = State::RX_STATE;
    // Ask for interrupt when receiving
    enable_interrupt_RxDone();
    // Go into RXCONTINUOUS so we can hear the response
    set_mode_RXCONTINUOUS();  
}

/**
 * @brief Put the radio into CAD (channel activity detect) mode
 * and enable the CadDone interrupt.
 * 
 * NOTE: It appears that we don't get the CadDone interrupt under
 * normal (inactive) circumstances.  So a timeout is used to stop
 * the CAD process later.
 */
static void start_Cad() {      

    //logger.println("start_Cad");

    state = State::CAD;
    // We use a random number here to try to limit transmissions stepping on each other
    endCadTime = mainClock.time() + (CAD_TIMEOUT_MS * random(1, 5));
    enable_interrupt_CadDone();
    set_mode_CAD();  
}

/**
 * @brief This is called when the radio reports the end of a 
 * transmission sequence.  The radio is put back into 
 * continuous receive mode.
 * 
 * This gets called from the main processing loop.
 */
static void event_TxDone() {   

    //logger.println("TxDone");

    // Check for pending transmissions.  If nothing is pending then 
    // put the radio back into receive mode.
    if (txBuffer.isEmpty()) {
        start_Rx();
    }
    // If we have pending data then send it out immediately (the assumption
    // is that the channel is still open). 
    else {
        start_Tx();
    }
} 

/**
 * @brief This is called when a complete message is received.
 * 
 * This gets called from the main processing loop.
 */
static void event_RxDone() {

    //logger.println("RxDone");

    // How much data is available?
    const uint8_t len = spi_read(0x13);
    // Reset the FIFO read pointer to the beginning of the packet we just got
    spi_write(0x0d, spi_read(0x10));

    // We do nothing for zero-length messages
    if (len == 0) {
        return;
    }

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

    // Put the RSSI (OOB) and the entire packet into the circular queue for 
    // later processing.
    rxBuffer.push((const uint8_t*)&lastRssi, rx_buf, len);
}

/**
 * @brief This function is called when the radio completes a CAD cycle
 * and it is determined that these is channel activity. 
 */
static void event_CadDone_Detection() {

    logger.println("INF: CadDone Detection");

    // Channel is busy so revert back to listening mode so that we're
    // ready to receive the data. 
    start_Rx();
}

static void event_CadDone_NoDetection() {

    //logger.println("CadDone");

    // Check for pending transmissions.  If nothing is pending then 
    // put the radio back into receive mode.
    if (txBuffer.isEmpty()) {
        start_Rx();
    }
    // If something is pending then transmit it (since we've been told
    // that the channel is innactive).
    else {     
        start_Tx();
    }
}

/**
 * @brief This function gets called from inside of the main processing loop.  It looks
 * to see if any interrupt activity has been detected and, if so, figure 
 * out what kind of interrupt was reported and calls the correct handler.
 */ 
static void check_for_interrupts() {

    // Look at the flag that gets set by the ISR itself
    if (!isrHit) {
        return;
    } else {
        if (systemConfig.getLogLevel() > 0) {
            logger.println("INF: Int");
        }
    }

    // *******************************************************************************
    // Critical Section:
    // Here we make sure that the clearing of the isr_hit flag and the unloading 
    // of the pending interrupts in the radio's IRQ register happen atomically.
    // We are avoding the case where 
    noInterrupts();

    isrHit = false;

    // Read and reset the IRQ register at the same time:
    uint8_t irq_flags = spi_write(0x12, 0xff);    
    
    // We saw a comment in another implementation that said that there are problems
    // clearing the ISR sometimes.  Notice we do a logical OR so we don't loose anything.
    irq_flags |= spi_write(0x12, 0xff);    

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
    // CadDone
    if (irq_flags & 0x04) {
        if (irq_flags & 0x01) {
            event_CadDone_Detection();
        } else {
            event_CadDone_NoDetection();
        }
    }
}

static void event_tick_Tx() {
    // Check for the case where a transmission times out
    if (mainClock.time() - startTxTime > TX_TIMEOUT_MS) {
        logger.println("ERR: TX time out");
        start_Rx();
    }
}

static void event_tick_Rx() {
    // Check for pending transmissions.  If nothing is pending then 
    // return without any state change.
    if (txBuffer.isEmpty()) {
        // No state change needed here
        return;
    }
    else {
        // At this point we know there is something pending.  We first 
        // go into CAD mode to make sure the channel is innactive.
        // A successful CAD check (with no detection) will trigger 
        // the transmission.
        start_Cad();
    }
}

static void event_tick_Cad() {
    // Check for the case where a CAD check times out
    if (mainClock.time() > endCadTime) {
        event_CadDone_NoDetection();
    }
}

// Call periodically to look for timeouts or other pending activity.  This will happen
// on the regular application thread.
void event_tick() {
    if (state == State::RX_STATE) {
        event_tick_Rx();
    } else if (state == State::TX_STATE) {
        event_tick_Tx();
    } else if (state == State::CAD) {
        event_tick_Cad();
    }
}

// --------------------------------------------------------------------------
// Radio Utilty Functions
// 
// This reference will be very important to you:
// https://www.hoperf.com/data/upload/portal/20190730/RFM95W-V2.0.pdf
// 
// The LoRa register map starts on page 103.

static void set_mode_SLEEP() {
    spi_write(0x01, 0x00);  
}

static void set_mode_STDBY() {
    spi_write(0x01, 0x01);  
}

static void set_mode_TX() {
    spi_write(0x01, 0x03);
}

static void set_mode_RXCONTINUOUS() {
    spi_write(0x01, 0x05);
}

static void set_mode_CAD() {
    spi_write(0x01, 0x07);
}

// See table 17 - DIO0 is controlled by bits 7-6
static void enable_interrupt_TxDone() {
    spi_write(0x40, 0x40);
}

// See table 17 - DIO0 is controlled by bits 7-6
static void enable_interrupt_RxDone() {
    spi_write(0x40, 0x00);
}

// See table 17 - DIO0 is controlled by bits 7-6
static void enable_interrupt_CadDone() {
    spi_write(0x40, 0x80);
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
    start_Rx();

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

/**
 * @brief (Advanced feature) This is used in a time to see if we 
 * can put the processor to sleep between messages.  
 * 
 * @return true 
 * @return false 
 */
static bool check_idle(void*) {
  
    if (state == State::RX_STATE &&
        systemMessageProcessor.getPendingCount() == 0 &&
        systemMessageProcessor.getSecondsSinceLastRx() > IDLE_INTERVAL_SECONDS) {
        logger.println("INF: Sleeping due to inactivity");
        // Put the ESP32 into a deep sleep that will be awakened using 
        // an external interrupt.  Wakeup will look like reboot.
        //
        // There are a limited number of GPIOs that can be used 
        // for this purpose.
        //
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1);
        esp_deep_sleep_start();        
    }

    return true;
}

/**
 * @brief This function iis called periodically to send engineering/diagnostic data to the master control node
 * on the network
 */
static bool send_station_id(void*) {
    systemMessageProcessor.sendEngineeringData(CONTROL_NODE);
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

    shell.addCommand(F("p <addr>"), sendPing);
    shell.addCommand(F("d <addr>"), sendGetSed);
    shell.addCommand(F("sendreset <addr> <passcode>"), sendReset);
    shell.addCommand(F("sendresetcounters <addr> <passcode>"), sendResetCounters);
    shell.addCommand(F("t <addr> <text>"), sendText);
    shell.addCommand(F("sendsetroute <addr> <target addr> <next hop addr> <passcode>"), sendSetRoute);
    shell.addCommand(F("sendgetroute <addr> <target addr>"), sendGetRoute);
    shell.addCommand(F("factoryreset"), factoryReset);

    shell.addCommand(F("setaddr <addr>"), setAddr);
    shell.addCommand(F("setcall <call_sign>"), setCall);
    shell.addCommand(F("setroute <target addr> <next hop addr> <passcode>"), setRoute);
    shell.addCommand(F("clearroutes"), clearRoutes);
    shell.addCommand(F("setblimit <limit_mv>"), setBatteryLimit);
    shell.addCommand(F("setpasscode <passcode>"), setPasscode);
    shell.addCommand(F("setlog <level>"), setLog);
    shell.addCommand(F("setmode <mode>"), setMode);

    shell.addCommand(F("reset"), boot);
    shell.addCommand(F("resetradio"), bootRadio);
    shell.addCommand(F("info"), info);
    shell.addCommand(F("sleep <seconds>"), sleep);
    shell.addCommand(F("print <text>"), print);
    shell.addCommand(F("rem <text>"), rem);
    shell.addCommand(F("resetcounters"), resetCounters);

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
          state = State::RX_STATE;
          isrHit = true;
        }
        // Any other reason for a reboot causes full radio reset/initialization
        else {
            // Reset the radio 
            reset_radio();
        }
    }
        
    // Enable the battery check timer
    timer.every(BATTERY_CHECK_INTERVAL_SECONDS * 1000, check_low_battery);
    // Enable the idle check
    // TODO: NOT WORKING YET - NOT SURE WHY
    //timer.every(IDLE_CHECK_INTERVAL_SECONDS * 1000, check_idle);

    // Enable the station ID timer
    timer.every(STATION_ID_INTERVAL_SECONDS * 1000, send_station_id);
    // Force an initial update one minute after startup
    timer.in(60L * 1000L, send_station_id);
   
    // Enable the watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    esp_task_wdt_reset();
}

void loop() {

  // Check to see if any interrupts were fired
  check_for_interrupts();
 
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
