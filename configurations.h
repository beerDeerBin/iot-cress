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

#define BUTTON_WATERING_PIN D7                  // pin of the watering button
#define BUTTON_SYSTEM_PIN D4                    // pin of the system button
#define BUTTON_SHORT_PRESS_IDLE_TIME 100000     // short button pressed idle time [us]
#define BUTTON_LONG_PRESS_IDLE_TIME 5000000     // long button pressed idle time [us]

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

// Working varibales for the Watering Button
typedef struct WorkVarWateringBtn_S {
  uint32_t    wateringBtnCnt;                   // watering button counter
  uint32_t    isButtonPressed;                  // indicator if the button is currently pressed
  uint64_t    idleTime;                         // idle time which the watering must be pressed to be proccessed
  uint64_t    lastRisingEdge;                   // time when the watering button pressed [us] i.e rising edge
  uint64_t    maxRuntime;                       // maximum runtime of the watering button ISR [us]
} WorkVarWateringBtn_T;

// Working varibales for the System Button
typedef struct WorkVarSystemBtn_S {
  uint32_t    systemBtnCnt;                     // system button counter
  uint32_t    isButtonPressed;                  // indicator if the button is currently pressed
  uint64_t    idleTime;                         // idle time which the watering must be pressed to be proccessed
  uint64_t    lastRisingEdge;                   // time when the system button pressed [us] i.e rising edge
  uint64_t    maxRuntime;                       // maximum runtime of the watering button ISR [us]
} WorkVarSystemBtn_T;

/****************************************************************
* Module: Serial
****************************************************************/

#define UART_BAUD_RATE 115200                   // the baud rate of the UART

/****************************************************************
* Module: System LED
****************************************************************/

#define SYSTEM_LED_TIMER_ISR 10                 // system LED timer interrupt length [ms]
#define SYSTEM_LED_PIN D0                       // pin number of the System LED
#define SYSTEM_LED_OFF 1                        // the System LED is off
#define SYSTEM_LED_ON 0                         // the System LED is on
#define SYSTEM_LED_DELAY_STARTUP 10             // start up mode System led blink delay in 10ms steps
#define SYSTEM_LED_DELAY_NORMAL 100             // normal mode System led blink delay in 10ms steps
#define SYSTEM_LED_DELAY_CALIBRATION 25         // calibration mode System led blink delay in 10ms steps

// Working varibales for the System LED module
typedef struct WorkVarSystemLed_S {
  uint8_t     pin;                              // the pin of the System LED
  uint8_t     currentValue;                     // the current value of the System LED i.e off or on
  uint8_t     buffer[2];                        // buffer to keep the data 32-Bit alligned
  uint32_t    delay;                            // current blink delay in 10ms steps
  uint32_t    callCnt;                          // counter which counts the calls to the timer ISR
  uint64_t    maxRuntime;                       // maximum runtime of the timer ISR [us]
} WorkVarSystemLed_T;

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

const char* DISPLAY_NORMAL_MODE_TITLE = "    NORMAL MODE";
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

/****************************************************************
* Module: WiFi
****************************************************************/

// WiFi Credentials
const char* WIFI_SSID = TODO
const char* WIFI_PSK = TODO

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
const char* MQTT_TOPIC_SUBSCRIBED_FEEDBACK = "sensor/6/sandbox/feedback";
const char* MQTT_TOPIC_PUBLISH_ENVIROMENT = "sensor/6/sandbox/environment";
const char* MQTT_TOPIC_PUBLISH_WATERING = "sensor/6/sandbox/urgent";

#define MQTT_MSG_BUFFER_SIZE 512                // size of the MQTT message buffer in byte
#define MQTT_MSG_ENV_VERSION 1                  // current version of the MQTT enviroment message

// JSON KEYS
const char* MQTT_JSON_KEY_VERSION = "Version";
const char* MQTT_JSON_KEY_TIME = "Milliseconds";
const char* MQTT_JSON_KEY_SOIL_MOISTURE = "SoilMoisture";
const char* MQTT_JSON_KEY_SOIL_TEMPERATURE = "SoilTemperature";
const char* MQTT_JSON_KEY_AIR_MOISTURE = "AirMoisture";
const char* MQTT_JSON_KEY_AIR_TEMPERATURE = "AirTemperature";
const char* MQTT_JSON_KEY_LIGHT_INTENSITY = "LightIntensity";
const char* MQTT_JSON_KEY_WATERING = "Watering";

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
















