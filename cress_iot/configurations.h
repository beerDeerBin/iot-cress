/****************************************************************
* Authors: Stefan Wild, Finn Storr, Simon Zeidler
* Created: 12.11.2024

* Brief: This .h file contains all structs and defines for the 
         Internet of Things Project.

* Last Update: 13.11.2024
****************************************************************/

#ifndef _CONFIGURATIONS_H 
#define _CONFIGURATIONS_H

/****************************************************************
* Includes
****************************************************************/
#include <EEPROM.h>
#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Seeed_SHT35.h"
#include "ESP8266TimerInterrupt.h"
#include "ESP8266_ISR_Timer.hpp"
#include "c_types.h"
#include <string>
#include "spiffs/spiffs_config.h"
#include <BH1750.h>

/****************************************************************
* Module: Genral
****************************************************************/

#define MODE_STARTUP 0                          // system mode: start up
#define MODE_NORMAL 1                           // system mode: normal
#define MODE_CALIBRATION 2                      // system mode: calibration

#define CALIBRATION_STEP1 0                     // calivartion step 1
#define CALIBRATION_STEP2 1                     // calivartion step 2
#define CALIBRATION_STEP3 2                     // calivartion step 3

#define ENVIROMENT_MSG_FREQUNCY 600             // enviroment message frequncy [s] i.e every ENVIROMENT_MSG_FREQUNCY a new enviroment message is sent
#define NUM_OF_SAMPLES_PER_MSG 30               // number of samples mesaured per enviroment message i.e ENVIROMENT_MSG_FREQUNCY/value = sample rate [s]

#define ISR_1KHZ_TIME 1                         // 1kHz interrupt length [ms]
#define BUTTON_WATERING_PIN D7                  // pin of the watering button
#define BUTTON_SYSTEM_PIN D4                    // pin of the system button
#define BUTTON_SHORT_PRESS_IDLE_TIME 50         // short button pressed idle time [ms]
#define BUTTON_LONG_PRESS_IDLE_TIME 5000        // long button pressed idle time [ms]

// Working varibales for the System
typedef struct WorkVarSystem_S {
  uint32_t    currentSystemMode;                // current system mode
  uint32_t    currentCalibrationStep;           // current calibration step if mode is calibration
  uint64_t    lastMeasuredTime;                 // time when the last measurment cycle was performed
  uint32_t    measurementTime;                  // time between measurments
  uint32_t    measurementSlot;                  // current measurment slot 0 <= measurementSlot <= NUM_OF_SAMPLES_PER_MSG
  uint32_t    wateringBtnCnt;                   // watering button counter stored in the system
  uint32_t    systemBtnCnt;                     // system button counter stored in the system
  uint64_t    maxRuntime;                       // maximum runtime of the background loop [ms]
} WorkVarSystem_T;

// Working varibales for a Button
typedef struct WorkVarBtn_S {
  uint32_t    pin;                              // pin of the button
  uint32_t    isPullup;                         // indicator if the button is connected via a pullup
  uint32_t    btnCnt;                           // button counter
  uint32_t    isButtonPressed;                  // indicator if the button is currently pressed
  uint32_t    buttonPressedCnt;                 // counter which counts in 1ms steps the length of the current button press
  uint32_t    buttonPressedIdleTime;            // time which the button needs to keep a stable signal to register as a press [ms]
} WorkVarBtn_T;

// Working varibales for 1kHz ISR
typedef struct WorkVar1kHzISR_S {
  WorkVarBtn_T  workVarWateringBtn;             // working variables of the watering button
  WorkVarBtn_T  workVarSystemBtn;               // working variables of the system button
  uint64_t      msCounter;                      // millisecond counter
  uint64_t      maxRuntime;                     // maximum runtime of the background loop [us]
} WorkVar1kHzISR_T;

/****************************************************************
* Module: Serial
****************************************************************/

#define UART_BAUD_RATE 115200                   // the baud rate of the UART

/****************************************************************
* Module: System LED
****************************************************************/

#define SYSTEM_LED_PIN D0                       // pin number of the System LED
#define SYSTEM_LED_OFF 1                        // the System LED is off
#define SYSTEM_LED_ON 0                         // the System LED is on
#define SYSTEM_LED_DELAY_STARTUP 100            // start up mode System led blink delay [ms]
#define SYSTEM_LED_DELAY_NORMAL 1000            // normal mode System led blink delay [ms]
#define SYSTEM_LED_DELAY_CALIBRATION 250        // calibration mode System led blink delay [ms]

// Working varibales for the System LED module
typedef struct WorkVarSystemLed_S {
  uint8_t     pin;                              // the pin of the System LED
  uint8_t     currentValue;                     // the current value of the System LED i.e off or on
  bool        isEnabled;                        // indicator if the system LED is currently enabled
  uint8_t     buffer;                           // buffer to keep the data 32-Bit alligned
  uint32_t    delay;                            // current blink delay [ms]
  uint32_t    callCnt;                          // call counter
} WorkVarSystemLed_T;

/****************************************************************
* Module: SOIL LED
****************************************************************/

#define SOIL_LED_RED_PIN D8                     // pin number of the Soil LED RED
#define SOIL_LED_BLUE_PIN D5                    // pin number of the Soil LED BLUE
#define SOIL_LED_GREEN_PIN D6                   // pin number of the Soil LED GREEN
#define SOIL_LED_STATE_DRY 0                    // Soil LED dry state
#define SOIL_LED_STATE_WET 1                    // Soil LED wet state
#define SOIL_LED_STATE_OK 2                     // Soil LED ok state
#define SOIL_LED_STATE_UNKNOWN 3                // Soil LED unknown

// Working varibales for the System LED module
typedef struct WorkVarSoilLed_S {
  uint8_t     pinRed;                           // the pin of the Soil LED RED
  uint8_t     pinBlue;                          // the pin of the Soil LED BLUE
  uint8_t     pinGreen;                         // the pin of the Soil LED GREEN
  bool        isEnabled;                        // indicator if the Soil LED is currently enabled
  uint32_t    soilState;                        // current state of the Soil LED
} WorkVarSoilLed_T;

/****************************************************************
* Module: EEPROM
****************************************************************/

#define EEPROM_SIZE 12                          // size of the EEPROM memeory in byte
#define EEPROM_SENSOR_IS_CALIBRATED 0xABABABAB  // calibration is done value

// EEPROM data mirror
typedef struct EepromMirror_S {
  uint32_t    isCalibrated;                     // indicator if the calibration is done
  uint32_t    minValueCalib;                    // minimum analog value of the moisture sensor i.e the sensor is dry
  uint32_t    maxValueCalib;                    // maximum analog value of the moisture sensor i.e the sensor is submerged in water
} EepromMirror_T;

/****************************************************************
* Module: OLED Display
****************************************************************/

#define DISPLAY_NUM_OF_CHAR 21                  // the maximum number of characters the OLED Display can hold in one line

// Display Text Constants
const char* DISPLAY_STARTUP_MODE_TITLE = "    START-UP MODE";
const char* DISPLAY_STARTUP_LINE1 = "The system is ";
const char* DISPLAY_STARTUP_LINE2 = "initializing please";
const char* DISPLAY_STARTUP_LINE3 = "wait";

const char* DISPLAY_NORMAL_LINE1 = "Soil moist";
const char* DISPLAY_NORMAL_LINE2 = "Soil temp";
const char* DISPLAY_NORMAL_LINE3 = "Air moist";
const char* DISPLAY_NORMAL_LINE4 = "Air temp";
const char* DISPLAY_NORMAL_LINE5 = "Light";

const char* DISPLAY_CALIBRATION_MODE_TITLE = "  CALIBRATION MODE";
const char* DISPLAY_CALIBRATION_STEP1_LINE1 = "Step 1:";
const char* DISPLAY_CALIBRATION_STEP1_LINE2 = "Dry & clean sensor,";
const char* DISPLAY_CALIBRATION_STEP1_LINE3 = "Press button to";
const char* DISPLAY_CALIBRATION_STEP1_LINE4 = "continue";

const char* DISPLAY_CALIBRATION_STEP2_LINE1 = "Step 2:";
const char* DISPLAY_CALIBRATION_STEP2_LINE2 = "Immerse in water,";
const char* DISPLAY_CALIBRATION_STEP2_LINE3 = "Press button to";
const char* DISPLAY_CALIBRATION_STEP2_LINE4 = "continue";

const char* DISPLAY_CALIBRATION_STEP3_LINE1 = "Step 3:";
const char* DISPLAY_CALIBRATION_STEP3_LINE2 = "Put sensor back,";
const char* DISPLAY_CALIBRATION_STEP3_LINE3 = "Press button to";
const char* DISPLAY_CALIBRATION_STEP3_LINE4 = "continue";

typedef struct WorkVarOled_S {
  bool        isEnabled;                        // indicator if the OLED-Display is currently enabled
  uint8_t     buffer[3];                        // buffer to keep the data 32-Bit alligned
  char        lineBuffer[DISPLAY_NUM_OF_CHAR];  // buffer for one line on the OLED-Display
  uint8_t     buffer2[3];                       // buffer to keep the data 32-Bit alligned
} WorkVarOled_T;

/****************************************************************
* Module: WiFi
****************************************************************/

// WiFi Credentials
const char* WIFI_SSID = TODO;
const char* WIFI_PSK = TODO;

/****************************************************************
* Module: MQTT
****************************************************************/

// MQTT Broker for internal use
//const char* MQTT_SERVER = "iot-emqx.iot.kub3.fh-joanneum.at";
//const char* MQTT_CLIENT = "sensor6";
//const char* MQTT_USER = "sensor6";
//const char* MQTT_PASSWORD = "sensor6";
//const int   MQTT_PORT = 8883;

// MQTT Broker for external use
const char* MQTT_SERVER = "mqtt.medien-transparenz.at";
const char* MQTT_CLIENT = "sensor6";
const char* MQTT_USER = "sensor6";
const char* MQTT_PASSWORD = "sensor6";
const int MQTT_PORT = 8883;

// MQTT topics
const char* MQTT_TOPIC_SUBSCRIBED_FEEDBACK_SLEEP = "sensor/6/feedback/sleepMode";
const char* MQTT_TOPIC_SUBSCRIBED_FEEDBACK_SOIL = "sensor/6/feedback/soilState";
const char* MQTT_TOPIC_PUBLISH_ENVIROMENT = "sensor/6/environment";
const char* MQTT_TOPIC_PUBLISH_WATERING = "sensor/6/urgent";

#define MQTT_MSG_BUFFER_SIZE 512                // size of the MQTT message buffer in byte
#define MQTT_MSG_ENV_VERSION 1                  // current version of the MQTT enviroment message

// JSON KEYS PUBLISHING
const char* MQTT_JSON_KEY_VERSION = "version";
const char* MQTT_JSON_KEY_TIME = "milliseconds";
const char* MQTT_JSON_KEY_SOIL_MOISTURE = "soilMoisture";
const char* MQTT_JSON_KEY_SOIL_TEMPERATURE = "soilTemperature";
const char* MQTT_JSON_KEY_AIR_MOISTURE = "airMoisture";
const char* MQTT_JSON_KEY_AIR_TEMPERATURE = "airTemperature";
const char* MQTT_JSON_KEY_LIGHT_INTENSITY = "lightIntensity";
const char* MQTT_JSON_KEY_WATERING = "watering";

// JSON KEYS SUBSCRIPTIONS
const char* MQTT_JSON_KEY_SLEEP_MODE = "sleepMode";
const char* MQTT_JSON_KEY_SOIL_STATE = "soilState";

// JSON VALUES SUBSCRIPTIONS
const char* MQTT_JSON_VALUE_SLEEP_MODE_OFF = "off";
const char* MQTT_JSON_VALUE_SLEEP_MODE_ON = "on";
const char* MQTT_JSON_VALUE_SOIL_STATE_DRY = "dry";
const char* MQTT_JSON_VALUE_SOIL_STATE_WET = "wet";
const char* MQTT_JSON_VALUE_SOIL_STATE_OK = "ok";

// Working varibales for the MQTT module
typedef struct WorkVarMqtt_S {
  bool        publishWateringMsg;               // indicator if the watering message should be published
  bool        publishEnviromentMsg;             // indicator if the enviroment message should be published
  uint8_t     buffer[2];                        // buffer to keep the data 32-Bit alligned
  char        msgBuffer[MQTT_MSG_BUFFER_SIZE];  // MQTT message buffer
} WorkVarMqtt_T;

/****************************************************************
* Module: Soil Temperature Sensor
****************************************************************/

#define STS_ONE_WIRE_BUS_PIN D3                 // pin of the Soil Temperature Sensor one wire bus1

// Working varibales for the Soil Temperature Sensor module
typedef struct WorkVarSTS_S {
  float       lastValue;                        // last recorded value from the Soil Temperature Sensor
  float       values[NUM_OF_SAMPLES_PER_MSG];   // buffer for the Soil Temperature Sensor values for one message
} WorkVarSTS_T;

/****************************************************************
* Module: Soil Moisture Sensor
****************************************************************/

#define SMS_ANALOG_PIN A0                       // pin of the Soil Moisture Sensor analog input

// Working varibales for the Soil Moisture Sensor module
typedef struct WorkVarSMS_S {
  uint32_t    minValueCalib;                    // Minium calibration value
  uint32_t    maxValueCalib;                    // Maximum calibration value
  uint32_t    lastValueRaw;                     // last recorded raw value from the Soil Moisture Sensor
  float       lastValue;                        // last recorded value from the Soil Moisture Sensor
  float       values[NUM_OF_SAMPLES_PER_MSG];   // buffer for the Soil Moisture Sensor values for one message
} WorkVarSMS_T;

/****************************************************************
* Module: Air Temperature/Humidity Sensor
****************************************************************/

#define ATS_SDA_BUS_PIN  D2                     // SDA pin of the Air Temperature/Humidity Sensor I2C bus
#define ATS_SCL_BUS_PIN  D1                     // SCL pin of the Air Temperature/Humidity Sensor I2C bus

// Working varibales for the Air Temperature/Humidity Sensor module
typedef struct WorkVarATS_S {
  float       lastTempValue;                    // last recorded temperature value from the Air Temperature/Humidity Sensor
  float       lastHumidValue;                   // last recorded humdity value from the Air Temperature/Humidity Sensor
  float       tempValues[NUM_OF_SAMPLES_PER_MSG];   // buffer for the Air Temperature/Humidity Sensor temperature values for one message
  float       humidValues[NUM_OF_SAMPLES_PER_MSG];   // buffer for the Air Temperature/Humidity Sensor temperature values for one message
} WorkVarATS_T;

/****************************************************************
* Module: Light Sensor
****************************************************************/

#define LS_SDA_BUS_PIN  D2                      // SDA pin of the Light Sensor I2C bus
#define LS_SCL_BUS_PIN  D1                      // SCL pin of the Light Sensor I2C bus

// Working varibales for the Light Sensor module
typedef struct WorkVarLS_S {
  float       lastValue;                        // last recorded Light value from the Light Sensor Sensor
  float       values[NUM_OF_SAMPLES_PER_MSG];   // buffer for the Light Sensor values for one message
} WorkVarLS_T;

#endif // _CONFIGURATIONS_H
















