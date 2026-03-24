/**
 * nRF52840 & LoRa ANEMOMETER
 * 
 * Based on popular 3 cup anemometer available from multiple suppliers such as the Renke Polycarbonate Wind Sensor.
 * As is, the anemometer draws too much current for battery operation. The optoelectronic rotation sensor and encoding
 * board are replaced with a low current tunnel magnetoristance sensor, XIAO nrf2840 board and SX1262 radio. This gives a mean
 * current consumption of 17 microamps, giving years of life on 3 AA batteries and several hundred metres range.
 * Peak power consumption is 45 milliamps for 4.75 millseconds every 2 minutes
 * 
 * Every 2 minutes, the anemometer broadcasts mean wind speed and maximum 3 second gust of the previous 10 minutes.
 * 
 * Uses a semaphore to sleep the controller, enabling fast wake up via interrupts.
 * 
 * Copyright (C) 2026 Russell Holmes
 * 
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License along with this program. 
 * If not, see <https://www.gnu.org/licenses/>. 
 */

#include <Arduino.h>
#include <nrf_timer.h>
#include <Adafruit_SPIFlash.h> 
#include <RadioLib.h>
#include <InternalFileSystem.h>
#include <Adafruit_LittleFS.h>
#include <hw_rng.h>
#include <ArduinoLog.h>

const unsigned long kSerialSpeed = 115200;

// Transceiver pin connections
const uint32_t kPinNSS = D4;
const uint32_t kPinDI01 = D1;
const uint32_t kPinRST = D2;
const uint32_t kPinMOSI = D10;
const uint32_t kPinMISO = D9;
const uint32_t kPinSCK = D8;
const uint32_t kPinBUSY = D3;
const uint32_t kPinRF_SW = D5;

const u_int8_t kInterruptPin = D6;      // pin to receive pulses from anemometer - not much choice on Xiao nRF52840!
const u_int32_t kPinBatEnable = 14;     // Send LOW to enable voltage divider for battery monitor
const u_int32_t kPinBatADC = PIN_VBAT;  // ADC pin connected to XIAO on-board battery voltage divider

// LoRa modem settings
const float kTransFreq = 868.3;
const float kLoRaBandwidth = 500.0;     // maximum bandwidth gives maximum data rate, reducing on-air time and power consumptio
const uint8_t kSpreadingFactor = 5;     // minimum spreading factor gives maximum data rate
const uint8_t kCodingRate = 8;          // maximum forward error correction to maximise noise immunity
const uint8_t kLoRaSyncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;  // standard sync word
const int8_t kOutputPower = 14;        // maximum legal power in EU & UK 25 mW
const size_t kPreambleSensor = 12;      // default preamble, receiver constantly listening so no need to lengthen
const size_t kPreambleHub = 32;         // hub ack preamble - longer than default to give sensor more chance to detect
const uint8_t kLoRaCRC = 2;             // enable 2 byte CRC
const bool kRxBoostedGainMode = true;   // slightly improves receiver gain at the expense of extra 0.6 mA current consumption
const float kTCXOVoltage = 1.6;          // control voltage for TCXO, 0 to disable

// various payload and transmission interval settings
// payload consists of 3 byte system ID + 1 byte sensor type + 1 byte sensor ID
// + 1 byte transmission counter + 2 byte voltage + 2 byte mean wind speed + 2 byte max gust 
const uint8_t kSensorPayloadLen = 12;             // data payload length
const uint8_t kHubPayloadLen = 7;                 // ack payload length
const RadioLibTime_t kAckTimeout = 10000;         // how long to wait for ack in microseconds before timeout - ack transmission time + extra
const uint8_t kSystemID[] = {0xba, 0xbb, 0xee};   // 3 byte system ID - reduces chance of processing ghost or foreign packets
const uint8_t kSystemIDLen = sizeof(kSystemID);   // number of bytes in payload ID
const uint8_t kSensorType = 0x04;                 // anemometer sensor type ID: 4 = anemometer
const uint8_t kAckStatusOK = 0;                   // nominal ack status byte
const uint8_t kTransRetries = 2;                  // how many transission retries if ack not received
const u_int32_t kSleepInterval = 3;               // interval between clock interrupts in seconds - gust measurement duration (3 s standard)
const uint16_t kBroadcastInterval = 2;            // interval between LoRa data broadcasts in minutes
const uint16_t kOneMinute = 60 / kSleepInterval;  // number of sleep intervals in 1 minute - used in loop for minute timer
const uint16_t kMeanSpeedInterval = 4;           // number of minutes to average wind over - the standard is 10

// Calibration factor converts rotational frequency to wind speed in m/s.
// Physical model measurements suggest 1.681.
// Manufacturer suggests 1.75.
// Anemometer generates several pulses per whole rotation.
const uint16_t kPulsesPerTurn = 2;                  // anemometer modification uses 2 magnets
const float kAnemometerCal = 1.75 / kPulsesPerTurn; // converts pulse frequency to wind speed in m/s
const float kVoltageCal = 3.055;                    // measured voltage divider calibration factor

// watchdog timeout in seconds
// NB must be greater than watchdog reset interval (now 1 minute)
const uint16_t kWatchdogTimeout = (kOneMinute * kSleepInterval) + 30;

const char kFilePath[] = "/sensorid.txt";       // LittleFS path to store sensor id
using namespace Adafruit_LittleFS_Namespace;

struct Payload {
  uint8_t payload[kSensorPayloadLen] = {};
  uint8_t payloadlen = {kSensorPayloadLen};
};

/**
 * Simple class to store count and max gust count in crude ring buffer once per minute.
 * Once buffer is full, returns 10 minute pulse count and max 3 pulse count.
 * Divide by duration in seconds to get frequency for wind speed calculation.
 */ 
class WindBuffer {
    public:
        WindBuffer(uint16_t s) :elem_{new WindData[s]}, sz_{s} {
          for (int i = 0; i < sz_; i++) {
            elem_[i].max_gust_count = 0;
            elem_[i].total_count = 0;
          }
        }
        ~WindBuffer() {
          delete[] elem_;
        }
        bool IsFull() {
          return buffer_full_;
        }
        void Add(uint16_t total_count, uint16_t max_gust_count) {
            elem_[buffer_pos_].total_count = total_count;
            elem_[buffer_pos_].max_gust_count = max_gust_count;
            buffer_pos_++;
            if (buffer_pos_ == sz_) {
              buffer_pos_ = 0;
              buffer_full_ = true;
            }
        }
        uint16_t TotalCount() {
          uint16_t total = 0;
          for (int i = 0; i < sz_; i++) {
            total += elem_[i].total_count;
          }
          return total;
        }
        uint16_t MaxGust() {
          uint16_t max_gust = 0;
          for (int i = 0; i < sz_; i++) {
            if (elem_[i].max_gust_count > max_gust) {
              max_gust = elem_[i].max_gust_count;
            }
          }
          return max_gust;
        }
    private:
        struct WindData {
            uint16_t total_count;
            uint16_t max_gust_count;
        };
        WindData* elem_;
        uint16_t sz_;
        uint16_t buffer_pos_{0};
        bool buffer_full_{false};
};

SX1262 radio = new Module(kPinNSS, kPinDI01, kPinRST, kPinBUSY);

// Flash Class instance - used to put flash to sleep and save power
Adafruit_FlashTransport_QSPI flashTransport;

// semaphore used to make processor wait and sleep
// - gives fast wake response following interrupt
SemaphoreHandle_t xSemaphore;

// Radio interrupt flag set after packet sent or packet received.
volatile bool flag_radio_interrupt = false;

/**
 * Interrupt service routine for radio module
 */
void RadioIRQHandler(void) {
  // disable interrupts for critical code, then renable
  UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  flag_radio_interrupt = true;
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);


  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

// Clock interrupt flag set after every kSleepInterval seconds.
volatile bool flag_clock_interrupt = false;  

/**
 * Interrupt service routine for RTC timer.
 * NB extern to avoid C++ name mangling
 */
extern "C" void RTC2_IRQHandler(void)
{
  if ((NRF_RTC2->EVENTS_COMPARE[0] != 0) && ((NRF_RTC2->INTENSET & 0x10000) != 0)) {
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    NRF_RTC2->EVENTS_COMPARE[0] = 0;  // clear compare register 0 event
    NRF_RTC2->TASKS_CLEAR = 1;        // clear counter

    flag_clock_interrupt = true;
  
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // immediate context switch (don't wait for tick)
  }
}

// Anemometer interrupt flag set after every low pulse from TMR sensor.
volatile bool flag_anemometer_interrupt = false;

/**
 * Interrupt service routine for anemometer pulse.
 */
void AnemometerIRQHandler() {
  // disable interrupts for critical code, then renable
  UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  flag_anemometer_interrupt = true;
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // immediate context switch (don't wait for tick)
}

/**
 * Retrieve sensor ID byte from flash memory at startup.
 * - if not present, generate and store random value between 0 and 255
 * @returns stored value or negative error code
 */
int16_t GetSensorID(void) {
  uint8_t sensor_id = 0;

  if (!InternalFS.begin()) {
    // unable to initialise InternalFS
    return -1;
  }
  Adafruit_LittleFS_Namespace::File file(InternalFS);
  file.open(kFilePath, FILE_O_READ);
  if (file) { // file exists
    file.read((uint8_t*)&sensor_id, sizeof(sensor_id));
  }
  else {      // files does not exist: create 
    SimpleHacks::HW_RNG::begin(); // initialise rng
    sensor_id = SimpleHacks::HW_RNG::get_uint8();
    file.open(kFilePath, FILE_O_WRITE);
    if (file) {
      file.write((uint8_t*)&sensor_id, sizeof(sensor_id));
    }
  }

  file.close();
  InternalFS.end();

  return sensor_id;
}

/**
 * Generate and store new sensor ID to flash - random value between 0 and 255.
 * @returns 0 on success, -1 if failed to intialise file system, -2 if could not open file to save
 */
int16_t ResetSensorID(void) {
  uint8_t sensor_id = 0;

  if (!InternalFS.begin()) {
    // unable to initialise InternalFS
    return -1;
  }
  Adafruit_LittleFS_Namespace::File file(InternalFS);
  InternalFS.remove(kFilePath);
  SimpleHacks::HW_RNG::begin(); // initialise rng
  sensor_id = SimpleHacks::HW_RNG::get_uint8();
  file.open(kFilePath, FILE_O_WRITE);

  if (file) {
    file.write((uint8_t*)&sensor_id, sizeof(sensor_id));
  }
  else {
    // could not open file
    return -2;
  }

  file.close();
  InternalFS.end();

  return sensor_id;
}

/**
 * Initialise low frequency RTC and configure interrupt
 * @param[in] RTCcount time interval in seconds * 1024
 */
void InitialiseLFRTC(const uint32_t& RTCcount) 
{
  NRF_CLOCK->TASKS_LFCLKSTOP = 1;
  NRF_CLOCK->LFCLKSRC = 1;       // select LFXO
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
  while(NRF_CLOCK->LFCLKSTAT != 0x10001)
  
  NRF_RTC2->TASKS_STOP = 1;      // stop counter 
  NRF_RTC2->TASKS_CLEAR = 1;     // clear counter
  NRF_RTC2->PRESCALER = 31;      // set counter prescaler, fout=32768/(1+PRESCALER)　1024Hz
  NRF_RTC2->CC[0] = RTCcount;    // set value for TRC compare register 0
  NRF_RTC2->INTENSET = 0x10000;  // enable interrupt CC[0] compare match event
  NRF_RTC2->EVTENCLR = 0x10000;  // clear counter when CC[0] event
  NVIC_SetPriority(RTC2_IRQn,3); // Higher priority than SoftDeviceAPI
  NVIC_EnableIRQ(RTC2_IRQn);     // enable interrupt  
  NRF_RTC2->TASKS_START = 1;     // start Timer
}

/**
 * Power down the flash memory.
 * (Saves approximately 10 µA.)
 */
void PowerDownFlash(void) {
  flashTransport.begin();
  flashTransport.runCommand(0xB9); 
  delayMicroseconds(5);
  flashTransport.end();  
}

/**
 * Activate battery voltage measurement by connecting one end of the voltage divider to ground.
 * NB although this increases total current consumption by several µA, this is necessary to
 * avoid over-voltage of the ADC by the connected battery.
 */
void EnableVoltageDivider(void) {
  pinMode(kPinBatEnable, OUTPUT);
  digitalWrite(kPinBatEnable, LOW);
}

/**
 * Measure voltage of battery.
 * Onboard voltage divider and ADC used to measure voltage
 * fresh lithium AA cell - 1.7V; nearly depleted at 1.4V
 * fresh alkaline AA cell - 1.6V; nearly depleted at 1.1V
 * 3 cells in series gives practical battery voltage range from 3.3V to 5.1V.
 * At 10 bits, 1.8 V reference, resolution is 5 millivolts
 * @returns battery voltage in volts
 */
float GetBatteryVoltage(void) {
  pinMode(kPinBatADC, INPUT);
  // voltage divider is about 1/3: 3 fresh lithium AA ccells can produce 5.1V, so voltage reference must be higher than 1.7V
  analogReference(AR_INTERNAL_1_8);  
  analogReadResolution(10);

  // let ADC settle
  delay(1);

  // 10 bits gives better than 2 dp precision
  float battery_voltage = (float) analogRead(PIN_VBAT) / 1023.0 * 1.8 * kVoltageCal;

  // turn off adc
  pinMode(kPinBatADC, OUTPUT);

  return battery_voltage;
}

/**
 * Flashes on-board LED.
 * @param led_colour pin address of LED to flash
 * @param duration duration of LED flash in milliseconds
 * @param num_flashes how many flashes to perform
 */
void FlashLED(const uint32_t& led_colour, const uint32_t& duration, const uint16_t& num_flashes) {
  pinMode(led_colour, OUTPUT);
  for (int i = 0; i < num_flashes; i++) {
    digitalWrite(led_colour, LOW);
    delay(duration);       
    digitalWrite(led_colour, HIGH);
    delay(duration);
  }
}

/**
 * Initialise and configure the SX1262 transceiver
 * @returns 1st Radiolib error code, otherwise RADIOLIB_ERR_NONE = 0
 */
int16_t ConfigureTransceiver(void) {
  int16_t state;

  // NB PE4259 antenna switch has 2 control modes: single-pin and complementary-pin
  // When pin 6 (RF Switch) connected to Vcc (HIGH): single-pin controlled by DIO2
  // When pin 6 (RF Switch) set LOW: complementary pin logic
  pinMode(kPinRF_SW, OUTPUT);
  digitalWrite(kPinRF_SW, HIGH);

  // reset transceiver in case it is in undefined state after reset
  if ((state = radio.reset(true)) != RADIOLIB_ERR_NONE) {
    return state;
  }
  if ((state = radio.begin(kTransFreq, kLoRaBandwidth, kSpreadingFactor, kCodingRate, kLoRaSyncWord, kOutputPower, kPreambleSensor, kTCXOVoltage)) != RADIOLIB_ERR_NONE) {
    return state;
  } 
  if ((state = radio.setCRC(kLoRaCRC)) != RADIOLIB_ERR_NONE) {
    return state;
  }
  if ((state = radio.setDio2AsRfSwitch(true)) != RADIOLIB_ERR_NONE) {
    return state;
  }
  if ((state = radio.setRxBoostedGainMode(kRxBoostedGainMode, true)) != RADIOLIB_ERR_NONE) {
    return state;
  }
  // optimising PA settings reduces transmit current from 90 mA to 45 mA at 14 dB
  if ((state = radio.setOutputPower(kOutputPower, true)) != RADIOLIB_ERR_NONE) {
    return state;
  }
  // ISR to call when transceiver action completed 
  radio.setDio1Action(RadioIRQHandler);

  return RADIOLIB_ERR_NONE;
}

/**
 * Configure and start the watchdog timer.
 * @param[in] duration countdown duration in seconds
 */
void StartWatchdog(uint16_t duration) {
  NRF_WDT->CONFIG         = 0x01;     // Keep WDT running while the CPU is sleeping
  NRF_WDT->CRV            = (32768 * duration) + 1;    // Counter reload value in number of cycles of the 32768 Hz clock
  NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
  NRF_WDT->TASKS_START    = 1;        // Start WDT     
}

/**
 * Calculates wind speed from pulse count and duration.
 * @param[in] count number of pulses received from anemometer
 * @param[in] duration period over which pulses counted
 * @returns wind speed in metres per second
 */
float CalculateWindSpeed(const int16_t& count, const int16_t& duration) {
  // rotational frequency = counts divided by duration in seconds
  float frequency = (float) count / (float) duration;
  // convert rotational frequency to wind speed
  float speed = kAnemometerCal * frequency;

  return speed;
}

/**
 * Configure radio to listen for ack.
 * Set preamble length to match hub and put in non-blocking receive mode.
 * Triggers radio interrupt when packet received.
 * @returns 1st Radiolib error code, otherwise RADIOLIB_ERR_NONE = 0
 */
int16_t ListenForAck(void) {
  int16_t state;

  if ((state = radio.setPreambleLength(kPreambleHub)) != RADIOLIB_ERR_NONE) {
    return state;
  }
  if ((state = radio.startReceive()) != RADIOLIB_ERR_NONE) {
    return state;
  }

  return RADIOLIB_ERR_NONE;
}

/**
 * Process payload to see whether is an ack.
 * If valid ack, also performs action according to control byte:
 * 0 - do nothing, 1 - generate new random sensor ID and reset, 2 - reset and go into Bluetooth OTA update mode
 * @param[in] payload - the transmitted payload to compare initial bytes with ack payload
 * @returns 0 if valid ack, -1 if wrong length, -2 if does not match, -3 if non-zero control byte,
 * otherwise any error code returned by the readData() method.
 */
int16_t ProcessAck(const Payload& payload) {
  int16_t state;
  uint8_t ack_payload[kHubPayloadLen] = {};

  size_t ack_payload_length = radio.getPacketLength();

  if (ack_payload_length == kHubPayloadLen) {
    state = radio.readData(ack_payload, ack_payload_length);
  } else {
    // wrong length ack_payload
    return -1;
  }

  // ack does not match sent payload
  if (memcmp(ack_payload, payload.payload, kHubPayloadLen - 1) != 0) {
    return -2;
  }
  
  // check control byte at end of ack payload
  if (ack_payload[kSystemIDLen + 3] != kAckStatusOK) {
    if (ack_payload[kSystemIDLen + 3] == 1) {
      // if control byte == 1, reset ID and reboot
      ResetSensorID();
      NVIC_SystemReset();
    }
    else if (ack_payload[kSystemIDLen + 3] == 2) {
      // if control byte == 2, reboot into DFU mode
      NRF_POWER->GPREGRET = 0xA8; 
	    NVIC_SystemReset();
    }
    return -3;
  }

  return state;
}

/**
 * Set transceiver wake/sleep mode.
 * @param[in] is_awake - true = wake up, false = go to sleep
 * @returns 1st Radiolib error status code, otherwise RADIOLIB_ERR_NONE = 0 on success
 */
int16_t TransceiverWake(const bool is_awake) {
  int16_t state;

  if (is_awake) {
    SPI.begin();
    // set RF_SW high to allow single pin control by DIO2
    digitalWrite(kPinRF_SW, HIGH);
    // force immediate wake to standby mode, EDIT: don't use external crystal oscillator
    // takes 340 µs
    if ((state = radio.standby(RADIOLIB_SX126X_STANDBY_XOSC, true)) != RADIOLIB_ERR_NONE) {
      return state;
    }
    // data sheet specifies that warm start does not restore boosted gain mode, so set it again
    // NB: probably not necessary as looks like Radiolib library does this already
    if ((state = radio.setRxBoostedGainMode(kRxBoostedGainMode, true)) != RADIOLIB_ERR_NONE) {
      return state;
    }
  } else {
    // data sheet specifies that transceiver should be in STDBY_RC mode 1st to put to sleep
    // NB: probably not necessary as looks like Radiolib library does this already
    if ((state = radio.standby(RADIOLIB_SX126X_STANDBY_RC)) != RADIOLIB_ERR_NONE) {
      return state;
    }
    // put to sleep with warm start - i.e. retain modem configuration
    // warm start increases current consumption from 600 nA to 1.2 µA
    if ((state = radio.sleep(true)) != RADIOLIB_ERR_NONE) {
      return state;
    }
    // leaving high increases current consumption by 55 µA!
    digitalWrite(kPinRF_SW, LOW);

    // stop SPI to prevent transceiver wakes
    SPI.end();
  }
  
  return RADIOLIB_ERR_NONE;
}

/** 
 * Populate transmission payload with sensor data.
 * Converts pulse count to wind speed using conversion factor.
 * Also reads battery voltage
 * @param[out] payload payload structure populate for broadcast
 * @param[in] wind_counter total pulse count over averaging period - 10 minutes is standard
 * @param[in] max_gust_counter maximum 3 second gust count
 * @param[in] transmission_count counter that is incrememted for every data transmission
 */
void CreatePayload(Payload& payload, const uint16_t& wind_counter, const uint16_t& max_gust_counter, const uint8_t& transmission_count) {
  float battery_voltage = GetBatteryVoltage();
  float wind_speed_mean = CalculateWindSpeed(wind_counter, kSleepInterval * kOneMinute * kMeanSpeedInterval);
  float wind_speed_gust = CalculateWindSpeed(max_gust_counter, kSleepInterval);

  payload.payload[5] = transmission_count;
  
  payload.payload[6] = ((uint16_t) (battery_voltage * 100)) >> 8;
  payload.payload[7] = ((uint16_t) (battery_voltage * 100));

  payload.payload[8] = ((uint16_t) (wind_speed_mean * 100)) >> 8;
  payload.payload[9] = (uint16_t) (wind_speed_mean * 100);

  payload.payload[10] = ((uint16_t) (wind_speed_gust * 100)) >> 8;
  payload.payload[11] = (uint16_t) (wind_speed_gust * 100);
}

/**
 * Transmit payload using non-blocking method.
 * Triggers radio interrupt on completion.
 * @param[in] payload structure containing payload array and length to transmit
 * @returns RADIOLIB_ERR_NONE = 0 on success, otherwise 1st Radiolib error
 */
int16_t SendPayload(const Payload& payload) {
  int16_t state;

  if ((state = radio.setPreambleLength(kPreambleSensor)) != RADIOLIB_ERR_NONE) {
    return state;
  }

  if ((state = radio.startTransmit(payload.payload, payload.payloadlen)) != RADIOLIB_ERR_NONE) {
    return state;
  }

  return RADIOLIB_ERR_NONE; 
}

void setup() {
  // necessary to prevent over-voltage of ADC pin
  // - increases quiescent current by Vbat / 1.5E6
  // - for 4.5 V supply : 3 microamps
  EnableVoltageDivider();

  FlashLED(LED_GREEN, 50, 5);

  /*
  Serial.begin(kSerialSpeed);
  while (!Serial);
  Serial.println("Starting up.");
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.noticeln("Starting up"); 
  Serial.flush();
  */

  // retrieve (or create) sensor ID prior to shutting flash down
  uint8_t sensor_id = GetSensorID();

  // enable interrupt on anemometer pulses
  // normally HIGH, goes LOW once per revolution
  // TMR sensor, so no debounce required
  pinMode(kInterruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kInterruptPin), AnemometerIRQHandler, FALLING);

  // Use DC-DC converter to save power - best improvement is on high duty cycle
  NRF_POWER->DCDCEN = 1;   

  // saves a few microamps
  PowerDownFlash();
 
  // RTC interval timer interrupts
  InitialiseLFRTC(1024 * kSleepInterval);

  int16_t state;
  if ((state = ConfigureTransceiver()) != RADIOLIB_ERR_NONE) {
    FlashLED(LED_RED, 20, 10);
  }

  // create payload array and populate static values
  Payload payload;
  memcpy(payload.payload, kSystemID, sizeof(kSystemID));
  payload.payload[kSystemIDLen] = kSensorType;
  payload.payload[kSystemIDLen + 1] = sensor_id;

  // crude ring buffer used to store anemometer count and max_gust once every minute
  // returns previous 10 minutes' anemometer count and max 3 second gust count
  WindBuffer wind_buffer(kMeanSpeedInterval);

  unsigned long ack_timer = 0;
  uint16_t trans_retries = 0;
  bool flag_ack_fail = false;
  int8_t transmission_count = 0; // incrememented after every transmission
  uint16_t anemometer_count = 0;
  uint16_t gust_count = 0;
  uint16_t last_anemometer_count = 0;
  uint16_t max_gust_count = 0;
  uint16_t clock_count = 0;
  uint16_t minute_count = 0;

  xSemaphore = xSemaphoreCreateBinary();

  // define wake or sleep state for loop
  bool flag_stay_awake = false;

  // fdefine transceiver state: true = receiving, false = transmitting
  // NB: normally in transmit state unless waiting for ack
  bool flag_receive_state = false;

  TransceiverWake(false);

  // watchdog timer reset every minute
  // countdown interval must be greater than this
  StartWatchdog(kWatchdogTimeout);

  FlashLED(LED_BLUE, 50, 5);

  // The big loop, and a big headache to debug.
  // The controller is either asleep waiting for an interrupt (radio, timer or anemometer),
  // or awake and looping while waiting for a task to complete.
  // The tasks are:
  // 1. Waiting for payload transmission to complete
  // 2. Waiting for an ack to be received
  // If an ack is not received it will retransmit several times

  while(true) {
    // if in sleep state, wait here and go to sleep until woken by interrupt

    if (!flag_stay_awake) {
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
    }

    //FlashLED(LED_BLUE, 20, 1);
    /*
    Log.noticeln("wake."); 
    if (flag_anemometer_interrupt) {
      Log.noticeln("anemometer"); 
    }
    if (flag_clock_interrupt) {
      Log.noticeln("clock"); 
    }
    if (flag_radio_interrupt) {
      Log.noticeln("radio"); 
    }
    Serial.flush();
    */

    // to get here, either has been in sleep state and woken by anemometer/timer interrupt, or in wake state and looping
    // while waiting for transceiver packet sent/received interrupt, or running around loop again after failed to get ack.

    if (flag_anemometer_interrupt) {
      // Increment anemometer pulse counter and go back to sleep.
      flag_anemometer_interrupt = false;
      anemometer_count++;
    } else if (flag_clock_interrupt) { 
      // Increment 3 second interval counter.
      flag_clock_interrupt = false;
      clock_count++;
      
      // get number of anemometer pulses since last clock interrupt (3 seconds)
      gust_count = anemometer_count - last_anemometer_count;
      last_anemometer_count = anemometer_count;
      // store if max gust
      if (gust_count > max_gust_count) {
        max_gust_count = gust_count;
      }

      // store total pulse count and max gust count for each minute
      if (clock_count == kOneMinute) {
        clock_count = 0;
        minute_count++;
        wind_buffer.Add(anemometer_count, max_gust_count);

        // zero counters ready for next minute
        anemometer_count = 0;
        gust_count = 0;
        max_gust_count = 0;
        last_anemometer_count = 0;

        // reset watchdog timer
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;

        if (minute_count == kBroadcastInterval) {
          minute_count = 0;
          // check that wind buffer is full
          if (wind_buffer.IsFull()) {
            transmission_count++;
            CreatePayload(payload, wind_buffer.TotalCount(), wind_buffer.MaxGust(), transmission_count);
            TransceiverWake(true);
            SendPayload(payload);
            // Now wait for "packet sent" interrupt while in sleep state
            flag_receive_state = false;
            flag_stay_awake = false;
          }
        }
      }
    } else if (flag_radio_interrupt) {
      // packet has been sent or received
      flag_radio_interrupt = false;

      if (!flag_receive_state) {   
        //packet has been sent - now stay awake and listen for ack packet   
        flag_receive_state = true;
        flag_stay_awake = true;   // stay awake so can monitor for timeout
        radio.finishTransmit();   // tidy up after transmission before entering receive mode
        radio.setPreambleLength(kPreambleHub);  // set preamble length to match hub broadcast
        radio.startReceive();
        ack_timer = micros();     // record time so can check for timeout
      } else {
        // packet has been received - check whether valid ack
        if (ProcessAck(payload) == 0) {
          // valid ack: go to sleep
          flag_receive_state = false;
          flag_stay_awake = false;
          flag_ack_fail = false;
          radio.finishReceive();
          TransceiverWake(false);
        } else {
          // not a valid ack: set flag to retransmit data
          flag_ack_fail = true;
          flag_stay_awake = true;
          flag_receive_state = false;
          radio.finishReceive();
        }
      }

    } else {
      // Not an interrupt, so either:
      // 1. Waiting for ack - in which case, check for timeout
      // 2. No ack received, resend data
      
      if (flag_receive_state) {
        // waiting for ack
        if ((micros() - ack_timer) > kAckTimeout) {
          // timed out: stop waiting for packet - set flag to retransmit data
          flag_ack_fail = true;
          flag_stay_awake = true;
          flag_receive_state = false;
          radio.finishReceive();
        }
      }

      if (flag_ack_fail) {
        flag_ack_fail = false;
        trans_retries++;
        if (trans_retries <= kTransRetries) {
          // give hub a few milliseconds to start listening
          delay(5);
          SendPayload(payload);   // Function sets preamble length for transmission, so no need here.
          // put into transmit state and wait for "packet sent" interrupt
          flag_receive_state = false;
          flag_stay_awake = false;
        } else {    
          // no successful ack after retries - give up and go to sleep
          trans_retries = 0;
          flag_receive_state = false;
          flag_stay_awake = false;
          TransceiverWake(false);
        }
      }

    }
  }
}

void loop() {
}