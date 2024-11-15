/****************************************************************
* Authors: Stefan Wild, Finn Storr, Simon Zeidler
* Created: 12.11.2024

* Brief: This .ino sketch contains our Internet of Things
         Project to monitor a cress.

* Required Board: esp8266
* Required External Libaries:
*   - U8g2 [2.35.30]
*   - PubSubClient [2.8.0]
*   - ArduinoJson [7.2.0]
*   - ESP8266TimerInterrupt [1.6.0]
*   - OneWire [2.3.8]
*   - DallasTemperature [3.9.0]
*   - Seeed_SHT35 [1.0.2]
*   - BH1750 [1.3.0]
*
* Last Update: 13.11.2024
****************************************************************/

/****************************************************************
 * Includes
 ****************************************************************/
#include "configurations.h"

/****************************************************************
 * Global Variables
 ****************************************************************/

// General
WorkVarSystem_T                     workVarSystem;
WorkVarSystem_T*                    pWorkVarSystem;
volatile WorkVarWateringBtn_T       workVarWateringBtn;
volatile WorkVarWateringBtn_T*      pWorkVarWateringBtn;
volatile WorkVarSystemBtn_T         workVarSystemBtn;
volatile WorkVarSystemBtn_T*        pWorkVarSystemBtn;

// System LED
WorkVarSystemLed_T                  workVarSystemLed;
WorkVarSystemLed_T*                 pWorkVarSystemLed;
ESP8266Timer                        ITimer;

// EEPROM
EepromMirror_T                      eepromMirror;
EepromMirror_T*                     pEepromMirror;

// OLED
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
char                                lineBuffer[DISPLAY_NUM_OF_CHAR];

// MQTT
WiFiClientSecure                    espClient;
PubSubClient                        client(espClient);
WorkVarMqtt_T                       workVarMqtt;
WorkVarMqtt_T*                      pWorkVarMqtt;
DynamicJsonDocument                 doc(MQTT_MSG_BUFFER_SIZE);

// Soil Temperature Sensor
OneWire                             oneWire(STS_ONE_WIRE_BUS_PIN);
DallasTemperature                   soilTempSensor(&oneWire);
WorkVarSTS_T                        workVarSTS;
WorkVarSTS_T*                       pWorkVarSTS;

// Soil Moisture Sensor
WorkVarSMS_T                        workVarSMS;
WorkVarSMS_T*                       pWorkVarSMS;

// Air Temperature/Humidity Sensor
SHT35                               airSensor(ATS_SCL_BUS_PIN);
WorkVarATS_T                        workVarATS;
WorkVarATS_T*                       pWorkVarATS;

// Light Sensor
BH1750                              lightMeter;
WorkVarLS_T                         workVarLS;
WorkVarLS_T*                        pWorkVarLS;

/****************************************************************
 * Interrupt Service Routines
 ****************************************************************/

/**
 * @brief Blinks the System LED.
 */
void ICACHE_RAM_ATTR blinkSystemLedISR()
{
  uint64_t startTime = micros();
  uint64_t runTime;

  // Increment the call counter
  pWorkVarSystemLed->callCnt++;

  // Check if the System LED needs to be toggled
  if (pWorkVarSystemLed->callCnt == pWorkVarSystemLed->delay)
  {

    // Toggle the System LED value
    pWorkVarSystemLed->currentValue = !pWorkVarSystemLed->currentValue;

    // Toggle the System LED
    digitalWrite(pWorkVarSystemLed->pin, pWorkVarSystemLed->currentValue);

    // Reset the call counter
    pWorkVarSystemLed->callCnt = 0;
  }

  // Calculate the ISR runtime
  runTime = micros() - startTime;

  // Check if the current runtime is greater than the max runtime
  if (runTime > pWorkVarSystemLed->maxRuntime)
  {

    // Update the max runtime
    pWorkVarSystemLed->maxRuntime = runTime;
  }
}

/**
 * @brief Handles the watering button.
 */
void ICACHE_RAM_ATTR handleWateringBtnISR()
{
  uint32_t signal = (uint32_t)digitalRead(BUTTON_WATERING_PIN);
  uint64_t startTime = micros();
  uint64_t runTime, timeDiff;

  // Check if the button was previosly released and the signal is high
  if (pWorkVarWateringBtn->isButtonPressed == (uint32_t)false && signal == (uint32_t)HIGH)
  {
    // Update the button state
    pWorkVarWateringBtn->isButtonPressed = (uint32_t)true;

    // Update the last risisng edge time
    pWorkVarWateringBtn->lastRisingEdge = startTime;
  }
  
  // Check if the button was previosly prssed and the signal is low
  if (pWorkVarWateringBtn->isButtonPressed == (uint32_t)true && signal == (uint32_t)LOW)
  {
    // Update the button state
    pWorkVarWateringBtn->isButtonPressed = (uint32_t)false;

    // Calculate the time difference, accounting for overflow
    timeDiff = (startTime - pWorkVarWateringBtn->lastRisingEdge);

    // Check if the button press should be registered as a press
    if (timeDiff > pWorkVarWateringBtn->idleTime)
    {
      // Increment the Watering button counter
      pWorkVarWateringBtn->wateringBtnCnt++;

      // Debug output
      Serial.println("----WATERING BUTTON DEBUG-----");
      Serial.print("Button released, time difference: ");
      Serial.println(timeDiff);
      Serial.print("Button released, start time: ");
      Serial.println(startTime);
      Serial.print("Button released, lastRisingEdge time: ");
      Serial.println(pWorkVarWateringBtn->lastRisingEdge);
    }
  }

  // Calculate the ISR runtime
  runTime = micros() - startTime;

  // Check if the current runtime is greater than the max runtime
  if (runTime > pWorkVarWateringBtn->maxRuntime)
  {
    // Update the max runtime
    pWorkVarWateringBtn->maxRuntime = runTime;
  }
}

/**
 * @brief Handles the system button.
 */
void ICACHE_RAM_ATTR handleSystemBtnISR()
{
  uint32_t signal = (uint32_t)digitalRead(BUTTON_SYSTEM_PIN);
  uint64_t startTime = micros();
  uint64_t runTime, timeDiff;

  // Check if the button was previosly released and the signal is high
  if (pWorkVarSystemBtn->isButtonPressed == (uint32_t)false && signal == (uint32_t)HIGH)
  {
    // Update the button state
    pWorkVarSystemBtn->isButtonPressed = (uint32_t)true;

    // Update the last risisng edge time
    pWorkVarSystemBtn->lastRisingEdge = startTime;
  }

  // Check if the button was previosly pressed and the signal is low
  if (pWorkVarSystemBtn->isButtonPressed == (uint32_t)true && signal == (uint32_t)LOW) 
  {

    // Update the button state
    pWorkVarSystemBtn->isButtonPressed = (uint32_t)false;

    // Calculate the time difference, accounting for overflow
    timeDiff = (startTime - pWorkVarSystemBtn->lastRisingEdge);

    // Check if the button press should be registered as a press
    if (timeDiff > pWorkVarSystemBtn->idleTime)
    {

      // Increment the System button counter
      pWorkVarSystemBtn->systemBtnCnt++;

      // Debug output
      Serial.println("----SYSTEM BUTTON DEBUG-----");
      Serial.print("Button released, time difference: ");
      Serial.println(timeDiff);
      Serial.print("Button released, start time: ");
      Serial.println(startTime);
      Serial.print("Button released, lastRisingEdge time: ");
      Serial.println(pWorkVarSystemBtn->lastRisingEdge);
    }
  }

  // Calculate the ISR runtime
  runTime = micros() - startTime;

  // Check if the current runtime is greater than the max runtime
  if (runTime > pWorkVarSystemBtn->maxRuntime)
  {

    // Update the max runtime
    pWorkVarSystemBtn->maxRuntime = runTime;
  }
}

/****************************************************************
 * Main Programm
 ****************************************************************/

/**
 * @brief Setup function of the System.
 */
void setup()
{

  // Initialize the System
  initSystem();

  // Initialize the OLED Display Module
  initOledDisplay();

  // Initialize the System LED Module
  initSystemLed();

  // Pre-Initialization finsihed, set the system mode to start up
  updateMode(MODE_STARTUP, 0);

  // Initialize Soil Temperature Sensor Module
  initSTS();

  // Initialize Air Temperature/Humidity Sensor Module
  initATS();

  // Initialize the Light Sensor Module
  initLS();

  // Initialize the WiFi Module
  initWiFi();

  // Initialize the MQTT Module
  initMqtt();

  // Initialize the EEPROM Module
  initEeprom();

  // Check if the Soil Humidity Sensor is calibrated
  if (pEepromMirror->isCalibrated != EEPROM_SENSOR_IS_CALIBRATED)
  {

    // Sensor not calibarted, start calibration mode
    pWorkVarSystem->currentSystemMode = MODE_CALIBRATION;

    // Set the calibration mode step to 1
    pWorkVarSystem->currentCalibrationStep = CALIBRATION_STEP1;

    // Initialize the Soil Moisture Sensor Module with invalid values
    initSMS(0, 0);
  }
  else
  {

    // Sensor calibarted, start normal mode
    pWorkVarSystem->currentSystemMode = MODE_NORMAL;

    // Initialize the Soil Moisture Sensor Module with the read calibartion values
    initSMS(pEepromMirror->minValueCalib, pEepromMirror->maxValueCalib);
  }

  // Initialization finshed, set the correct mode
  updateMode(pWorkVarSystem->currentSystemMode, pWorkVarSystem->currentCalibrationStep);
}

/**
 * @brief Background Loop of the System.
 */
void loop()
{

  bool sendEnviromentMsg = false;
  bool sendWateringMsg = false;

  // Get the current number of milliseconds passed
  uint64_t startTime = millis();
  uint64_t runTime;

  // Check if the system is in normal mode
  if (pWorkVarSystem->currentSystemMode == MODE_NORMAL)
  {

    // Check if a new measurment cycle must be performed
    if ((startTime - pWorkVarSystem->lastMeasuredTime) > pWorkVarSystem->measurementTime)
    {

      // Measure the Soil Moisture
      measureSMS(pWorkVarSystem->measurementSlot);

      // Measure the Soil Temperature
      measureSTS(pWorkVarSystem->measurementSlot);

      // Measure the Air Temperature & Humidity
      measureATS(pWorkVarSystem->measurementSlot);

      // Measure the Light
      measureLS(pWorkVarSystem->measurementSlot);

      // Increment the measurment slot
      pWorkVarSystem->measurementSlot = (pWorkVarSystem->measurementSlot + 1) % NUM_OF_SAMPLES_PER_MSG;

      // Check if the enviroment message must be sent
      if (pWorkVarSystem->measurementSlot == 0)
      {

        // Remeber to send the enviroment message
        sendEnviromentMsg = true;
      }

      // update the display
      updateDisplay(pWorkVarSystem->currentSystemMode, pWorkVarSystem->currentCalibrationStep);

      // Update the last measurment time
      pWorkVarSystem->lastMeasuredTime = startTime;
    }

    // Check if the watering button has been pressed
    if (pWorkVarSystem->wateringBtnCnt != pWorkVarWateringBtn->wateringBtnCnt)
    {

      // Remeber to send the watering message
      sendWateringMsg = true;

      // Increment the watering button counter i.e the button press has been proccessed
      pWorkVarSystem->wateringBtnCnt++;
    }

    // Check if the System button has been pressed
    if (pWorkVarSystem->systemBtnCnt != pWorkVarSystemBtn->systemBtnCnt)
    {
      // Increment the system button counter i.e the button press has been proccessed
      pWorkVarSystem->systemBtnCnt++;

      // Invalidate the calibration data
      pEepromMirror->isCalibrated = 0;

      // Update the EEPROM data
      updateEeprom();

      // Restart the system into calibration mode
      ESP.restart();
    }
  }
  else
  {

    // Call calibrate sensor service
    calibrateSensorService();
  }

  // Call MQTT service
  mqttService(sendWateringMsg, sendEnviromentMsg, startTime);

  // Calculate the ISR runtime
  runTime = millis() - startTime;

  // Check if the current runtime is greater than the max runtime
  if (runTime > pWorkVarSystem->maxRuntime)
  {

    // Update the max runtime
    pWorkVarSystem->maxRuntime = runTime;
  }
}

/****************************************************************
 * Module: General
 ****************************************************************/

/**
 * @brief Initializes the System.
 */
void initSystem()
{

  // Initialize the working varibles pointer
  pWorkVarSystem = &workVarSystem;

  // Initialize the working varibles pointer
  pWorkVarWateringBtn = &workVarWateringBtn;

  // Initialize the working varibles pointer
  pWorkVarSystemBtn = &workVarSystemBtn;

  // Clear the working varibles
  memset((void *)pWorkVarSystem, 0, sizeof(WorkVarSystem_S));

  // Clear the working varibles
  memset((void *)pWorkVarWateringBtn, 0, sizeof(WorkVarWateringBtn_S));

  // Clear the working varibles
  memset((void *)pWorkVarSystemBtn, 0, sizeof(WorkVarSystemBtn_S));

  // Initialize the System working varibles
  pWorkVarSystem->measurementTime = (uint32_t)((ENVIROMENT_MSG_FREQUNCY / NUM_OF_SAMPLES_PER_MSG) * 1000);

  // Initialize the watering button idle time
  pWorkVarWateringBtn->idleTime = BUTTON_SHORT_PRESS_IDLE_TIME;

  // Initialize the system button idle time
  pWorkVarSystemBtn->idleTime = BUTTON_LONG_PRESS_IDLE_TIME;

  // Initialize the UART
  Serial.begin(UART_BAUD_RATE);

  // Initialize the Watering Button pin as input
  pinMode(BUTTON_WATERING_PIN, INPUT);

  // Initialize the System Button pin as input
  pinMode(BUTTON_SYSTEM_PIN, INPUT);

  // Initialize the Watering Button ISR
  attachInterrupt(digitalPinToInterrupt(BUTTON_WATERING_PIN), handleWateringBtnISR, CHANGE);

  // Initialize the System Button ISR
  attachInterrupt(digitalPinToInterrupt(BUTTON_SYSTEM_PIN), handleSystemBtnISR, CHANGE);
}

/**
 * @brief Updates the mode of the system.
 *
 * @param uint32_t mode: The mode to set.
 * @param uint32_t calibStep: The current calibration step, if the
                             system is in calibration mode.
 */
void updateMode(uint32_t mode, uint32_t calibStep)
{

  // Update the system mode
  pWorkVarSystem->currentSystemMode = mode;

  // Update the calibration step mode
  pWorkVarSystem->currentCalibrationStep = calibStep;

  // Switch depending on the system mode
  switch (pWorkVarSystem->currentSystemMode)
  {
  case MODE_STARTUP:
    // change the system to startup mode
    pWorkVarSystemLed->delay = SYSTEM_LED_DELAY_STARTUP;
    break;

  default:
  case MODE_NORMAL:
    // change the system LED to normal mode
    pWorkVarSystemLed->delay = SYSTEM_LED_DELAY_NORMAL;

    // Change the system button idle time to long
    pWorkVarSystemBtn->idleTime = BUTTON_LONG_PRESS_IDLE_TIME;
    break;

  case MODE_CALIBRATION:
    // Change the system LED to calibration mode
    pWorkVarSystemLed->delay = SYSTEM_LED_DELAY_CALIBRATION;

    // Change the system button idle time to short
    pWorkVarSystemBtn->idleTime = BUTTON_SHORT_PRESS_IDLE_TIME;
    break;
  }

  // update the display
  updateDisplay(pWorkVarSystem->currentSystemMode, pWorkVarSystem->currentCalibrationStep);
}

/**
 * @brief Sensor Calibration service.
 */
void calibrateSensorService()
{

  // Check if the System button has been pressed
  if (pWorkVarSystem->systemBtnCnt != pWorkVarSystemBtn->systemBtnCnt)
  {

    // Increment the system button counter i.e the button press has been proccessed
    pWorkVarSystem->systemBtnCnt++;

    // Switch depending on current calibration step
    switch (pWorkVarSystem->currentCalibrationStep)
    {
    default:
    case CALIBRATION_STEP1:
      // Measure the Soil Moisture
      measureSMS(pWorkVarSystem->measurementSlot);

      // Save the maximum calibartion value
      pEepromMirror->maxValueCalib = getLastRawSMS();

      // Update the calibartion step
      pWorkVarSystem->currentCalibrationStep = CALIBRATION_STEP2;
      break;

    case CALIBRATION_STEP2:
      // Measure the Soil Moisture
      measureSMS(pWorkVarSystem->measurementSlot);

      // Save the minium calibartion value
      pEepromMirror->minValueCalib = getLastRawSMS();

      // Update the calibartion step
      pWorkVarSystem->currentCalibrationStep = CALIBRATION_STEP3;
      break;

    case CALIBRATION_STEP3:
      // Update the calibartion step
      pWorkVarSystem->currentCalibrationStep = CALIBRATION_STEP3;

      // Set the calibration signature
      pEepromMirror->isCalibrated = EEPROM_SENSOR_IS_CALIBRATED;

      // Update the EEPROM data
      updateEeprom();

      // Restart the system into calibration mode
      ESP.restart();
      break;
    }

    // Update system mode
    updateMode(pWorkVarSystem->currentSystemMode, pWorkVarSystem->currentCalibrationStep);
  }
}

/****************************************************************
 * Module: System LED
 ****************************************************************/

/**
 * @brief Initializes the System LED module.
 */
void initSystemLed()
{

  // Initialize the working varibles pointer
  pWorkVarSystemLed = &workVarSystemLed;

  // Clear the working varibles
  memset((void *)pWorkVarSystemLed, 0, sizeof(WorkVarSystemLed_S));

  // Initialize the working varibales
  pWorkVarSystemLed->pin = SYSTEM_LED_PIN;
  pWorkVarSystemLed->currentValue = SYSTEM_LED_OFF;
  pWorkVarSystemLed->delay = SYSTEM_LED_DELAY_NORMAL;

  // Initialize the System LED pin as output
  pinMode(pWorkVarSystemLed->pin, OUTPUT);

  // Turn off the System LED as the initla state
  digitalWrite(pWorkVarSystemLed->pin, pWorkVarSystemLed->currentValue);

  // Define the System LED timer interrupt in [us]
  ITimer.attachInterruptInterval(SYSTEM_LED_TIMER_ISR * 1000, blinkSystemLedISR);
}

/****************************************************************
 * Module: EEPROM
 ****************************************************************/

/**
 * @brief Initializes the EEPROM module.
 */
void initEeprom()
{

  // Initialize the EEPROM mirror pointer
  pEepromMirror = &eepromMirror;

  // Initialize the EEPROM
  EEPROM.begin(sizeof(EepromMirror_S));

  // Read out the whole EEPROM and store it in the EEPROM mirror
  EEPROM.get(0, eepromMirror);
}

/**
 * @brief Udpate EEPROM mirror into the EEPROM.
 */
void updateEeprom()
{

  // Write the whole EEPROM mirror into the EEPROM
  EEPROM.put(0, eepromMirror);

  // Commit the written data
  EEPROM.commit();
}

/****************************************************************
 * Module: OLED Display
 ****************************************************************/

/**
 * @brief Initializes the OLED Display module.
 *
 * @note Must be initialized before System LED module becauuse of interrupt problems.
 */
void initOledDisplay()
{

  // Initialize the line buffer
  memset((void *)&lineBuffer[0], 0, sizeof(char) * DISPLAY_NUM_OF_CHAR);

  // Connect to the display
  u8g2.begin();

  // Clear the write buffer
  u8g2.clearBuffer();

  // Define the front for all text
  u8g2.setFont(u8g2_font_profont12_mf);
}

/**
 * @brief Updates the OLED display.
 *
 * @param uint32_t mode: The current mode of the system.
 * @param uint32_t calibStep: The current calibration step, if the
                              system is in calibration mode.
 */
void updateDisplay(uint32_t mode, uint32_t calibStep)
{

  // Clear the write buffer
  u8g2.clearBuffer();

  // check the mode
  if (mode == MODE_STARTUP)
  {

    // Draw the startup mode screen
    u8g2.drawStr(0, 10, DISPLAY_STARTUP_MODE_TITLE);
    u8g2.drawStr(0, 20, DISPLAY_STARTUP_LINE1);
    u8g2.drawStr(0, 30, DISPLAY_STARTUP_LINE2);
    u8g2.drawStr(0, 40, DISPLAY_STARTUP_LINE3);
  }
  else if (mode == MODE_NORMAL)
  {

    // Draw the normal mode screen
    u8g2.drawStr(0, 10, DISPLAY_NORMAL_MODE_TITLE);
    sprintf(lineBuffer, "%s.: %2.4f", DISPLAY_NORMAL_LINE1, getLastSMS());
    u8g2.drawStr(0, 20, lineBuffer);
    sprintf(lineBuffer, "%s. : %2.4f", DISPLAY_NORMAL_LINE2, getLastSTS());
    u8g2.drawStr(0, 30, lineBuffer);
    sprintf(lineBuffer, "%s. : %2.4f", DISPLAY_NORMAL_LINE3, getLastHumidATS());
    u8g2.drawStr(0, 40, lineBuffer);
    sprintf(lineBuffer, "%s.  : %2.4f", DISPLAY_NORMAL_LINE4, getLastTempATS());
    u8g2.drawStr(0, 50, lineBuffer);
    sprintf(lineBuffer, "%s      : %2.4f", DISPLAY_NORMAL_LINE5, getLastLS());
    u8g2.drawStr(0, 60, lineBuffer);
  }
  else
  {

    // Draw the calibration mode screen depeding on the calibration step
    u8g2.drawStr(0, 10, DISPLAY_CALIBRATION_MODE_TITLE);

    switch (calibStep)
    {
    case CALIBRATION_STEP1:
      u8g2.drawStr(0, 20, DISPLAY_CALIBRATION_STEP1_LINE1);
      u8g2.drawStr(0, 30, DISPLAY_CALIBRATION_STEP1_LINE2);
      u8g2.drawStr(0, 40, DISPLAY_CALIBRATION_STEP1_LINE3);
      u8g2.drawStr(0, 50, DISPLAY_CALIBRATION_STEP1_LINE4);
      break;

    case CALIBRATION_STEP2:
      u8g2.drawStr(0, 20, DISPLAY_CALIBRATION_STEP2_LINE1);
      u8g2.drawStr(0, 30, DISPLAY_CALIBRATION_STEP2_LINE2);
      u8g2.drawStr(0, 40, DISPLAY_CALIBRATION_STEP2_LINE3);
      u8g2.drawStr(0, 50, DISPLAY_CALIBRATION_STEP2_LINE4);
      break;

    default:
    case CALIBRATION_STEP3:
      u8g2.drawStr(0, 20, DISPLAY_CALIBRATION_STEP3_LINE1);
      u8g2.drawStr(0, 30, DISPLAY_CALIBRATION_STEP3_LINE2);
      u8g2.drawStr(0, 40, DISPLAY_CALIBRATION_STEP3_LINE3);
      u8g2.drawStr(0, 50, DISPLAY_CALIBRATION_STEP3_LINE4);
      break;
    }
  }

  // Update the Display by sending the buffer
  u8g2.sendBuffer();
}

/****************************************************************
 * Module: WiFi
 ****************************************************************/

/**
 * @brief Initializes the WiFi module.
 */
void initWiFi()
{
  // Set WiFI mode
  WiFi.mode(WIFI_STA);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  // Wait until a connection to WiFi is established
  while (WiFi.status() != WL_CONNECTED)
  {
    // wait for 500ms before checking the WiFi status again
    delay(500);
  };
}

/****************************************************************
 * Module: MQTT
 ****************************************************************/

/**
 * @brief Initializes the MQTT module.
 *
 * @note  Must be initialized AFTER a WiFi connection is established.
 */
void initMqtt()
{

  // Initialize the working varibles pointer
  pWorkVarMqtt = &workVarMqtt;

  // Clear the working varibles
  memset((void *)pWorkVarMqtt, 0, sizeof(WorkVarMqtt_S));

  // Set the MQTT client connection to insecure
  espClient.setInsecure();

  // Define the MQTT server
  client.setServer(MQTT_SERVER, MQTT_PORT);

  // Initialize the callback function for received MQTT messages
  client.setCallback(mqttCallback);
}

/**
 * @brief Performs the MQTT service, must be called from background continuously.
 *
 * @param bool sendWateringMsg: Indicator if the watering message should be published.
 * @param bool sendEnviromentMsg: Indicator if the enviroment message should be published.
 * @param uint64_t millis: Number of milliseconds passed since the system started.
 */
void mqttService(bool sendWateringMsg, bool sendEnviromentMsg, uint64_t millis)
{

  // Connect/Reconnect to the MQTT server
  mqttConnect();

  // Maintain connection and check for incomming messsages
  client.loop();

  // check if the watering message needs to be published
  if (sendWateringMsg == true)
  {

    // Prepare the JSON message
    doc[MQTT_JSON_KEY_WATERING] = true;
    doc[MQTT_JSON_KEY_TIME] = millis;

    // Serialize the JSON document and load it into the msg buffer
    serializeJson(doc, pWorkVarMqtt->msgBuffer);

    // Publish the message
    client.publish(MQTT_TOPIC_PUBLISH_WATERING, &pWorkVarMqtt->msgBuffer[0], true);

    // Clear the json memory pool
    doc.clear();
  }

  // check if the enviroment message needs to be published
  if (sendEnviromentMsg == true)
  {

    // Prepare the JSON message
    doc[MQTT_JSON_KEY_VERSION] = MQTT_MSG_ENV_VERSION;
    doc[MQTT_JSON_KEY_TIME] = millis;
    doc[MQTT_JSON_KEY_SOIL_MOISTURE] = getAvgSMS();
    doc[MQTT_JSON_KEY_SOIL_TEMPERATURE] = getAvgSTS();
    doc[MQTT_JSON_KEY_AIR_MOISTURE] = getAvgHumidATS();
    doc[MQTT_JSON_KEY_AIR_TEMPERATURE] = getAvgTempATS();
    doc[MQTT_JSON_KEY_LIGHT_INTENSITY] = getAvgLS();

    // Serialize the JSON document and load it into the msg buffer
    serializeJson(doc, pWorkVarMqtt->msgBuffer);

    // Publish the message
    client.publish(MQTT_TOPIC_PUBLISH_ENVIROMENT, &pWorkVarMqtt->msgBuffer[0], true);

    // Clear the json memory pool
    doc.clear();
  }
}

/**
 * @brief Cconnects/Reconnects to the MQTT server.
 */
void mqttConnect()
{

  // Loop until the connection to the MQTT server is established
  while (client.connected() == false)
  {

    // Try to connect to the MQTT client
    if (client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASSWORD))
    {

      // Subscribe to the feedback topic
      client.subscribe(MQTT_TOPIC_SUBSCRIBED_FEEDBACK);
    }
    else
    {

      // Delay for 500ms before tyring again
      delay(500);
    }
  }
}

/**
 * @brief Callback function for messages received by the MQTT server.
 *
 * @param char* topic: Topic of the received message.
 * @param byte* pPayload: Pointer that points to the start of the message payload.
 * @param uint32_t length: Length of the received payload.
 */
void mqttCallback(char *topic, byte *pPayload, uint32_t length)
{
  (void)topic;
  (void)pPayload;
  (void)length;
}

/****************************************************************
 * Module: Soil Temperature Sensor
 ****************************************************************/

/**
 * @brief Initializes the Soil Temperature Sensor module.
 */
void initSTS()
{

  // Initialize the working varibles pointer
  pWorkVarSTS = &workVarSTS;

  // Clear the working varibles
  memset((void *)pWorkVarSTS, 0, sizeof(WorkVarSTS_S));

  // Initialize the connection to the Soil Temperature Sensor
  soilTempSensor.begin();
}

/**
 * @brief Measures and saves the current Soil Temperature value.
 *
 * @param uint32_t slot: Current slot of the value i.e 0 <= slot <= NUM_OF_SAMPLES_PER_MSG
 */
void measureSTS(uint32_t slot)
{

  // Request new sensort values from the Soil Temperature sensor
  soilTempSensor.requestTemperatures();

  // Get the current Soil Temperature value
  pWorkVarSTS->lastValue = soilTempSensor.getTempCByIndex(0);

  // Save the value
  pWorkVarSTS->values[slot] = pWorkVarSTS->lastValue;

  // Check if a valid value was received
  if (pWorkVarSTS->lastValue == DEVICE_DISCONNECTED_C)
  {

    // Error Message
    Serial.println("Error: Could not read temperature data");
  }
}

/**
 * @brief Returns the avage Soil Temperature from the last NUM_OF_SAMPLES_PER_MSG.
 *
 * @param float: Avarage Soil Temperature value.
 */
float getAvgSTS()
{

  float sum = 0;

  // Calculate and return the Soil Temperature avarge value
  for (int i = 0; i < NUM_OF_SAMPLES_PER_MSG; i++)
  {
    sum += pWorkVarSTS->values[i];
  }

  return (sum / (float)NUM_OF_SAMPLES_PER_MSG);
}

/**
 * @brief Returns the last measured Soil Temperature value.
 *
 * @param float: Last measured Soil Temperature value.
 */
float getLastSTS()
{
  return pWorkVarSTS->lastValue;
}

/****************************************************************
 * Module: Soil Moisture Sensor
 ****************************************************************/

/**
 * @brief Initializes the Soil Moisture Sensor module.
 *
 * @param uint32_t minValueCalib: Calibrated minium value read from the EEPROM.
 * @param uint32_t maxValueCalib: Calibrated maximum value read from the EEPROM.
 */
void initSMS(uint32_t minValueCalib, uint32_t maxValueCalib)
{

  // Initialize the working varibles pointer
  pWorkVarSMS = &workVarSMS;

  // Clear the working varibles
  memset((void *)pWorkVarSMS, 0, sizeof(WorkVarSMS_S));

  // Initialize the calibrated minium value
  pWorkVarSMS->minValueCalib = minValueCalib;

  // Initialize the calibrated maximum value
  pWorkVarSMS->maxValueCalib = maxValueCalib;

  // Initialize the Soil Moisture Sensor pin as input
  pinMode(SMS_ANALOG_PIN, INPUT);
}

/**
 * @brief Measures and saves the current Soil Moisture value.
 *
 * @param uint32_t slot: Current slot of the value i.e 0 <= slot <= NUM_OF_SAMPLES_PER_MSG
 */
void measureSMS(uint32_t slot)
{

  // Measure the current Soil Moisture value
  pWorkVarSMS->lastValueRaw = analogRead(SMS_ANALOG_PIN);

  // Check if the raw sensor data can be normalized
  if ((pWorkVarSMS->maxValueCalib - pWorkVarSMS->minValueCalib) != 0)
  {

    // Normalize the Soil Moisture value
    pWorkVarSMS->lastValue = float(pWorkVarSMS->maxValueCalib - pWorkVarSMS->lastValueRaw) / float(pWorkVarSMS->maxValueCalib - pWorkVarSMS->minValueCalib);
  }

  // Save the value
  pWorkVarSMS->values[slot] = pWorkVarSMS->lastValue;
}

/**
 * @brief Returns the avage Soil Moisture from the last NUM_OF_SAMPLES_PER_MSG.
 *
 * @param float: Avarage Soil Moisture value.
 */
float getAvgSMS()
{

  float sum = 0;

  // Calculate and return the Soil Moisture avarge value
  for (int i = 0; i < NUM_OF_SAMPLES_PER_MSG; i++)
  {
    sum += pWorkVarSMS->values[i];
  }

  return (sum / (float)NUM_OF_SAMPLES_PER_MSG);
}

/**
 * @brief Returns the last measured Soil Moisture value.
 *
 * @param float: Last measured Soil Moisture value.
 */
float getLastSMS()
{
  return pWorkVarSMS->lastValue;
}

/**
 * @brief Returns the last raw measured Soil Moisture value.
 *
 * @param float: Last raw measured Soil Moisture value.
 */
float getLastRawSMS()
{
  return pWorkVarSMS->lastValueRaw;
}

/****************************************************************
 * Module: Air Temperature/Humidity Sensor
 ****************************************************************/

/**
 * @brief Initializes the Air Temperature/Humidity Sensor module.
 */
void initATS()
{

  // Initialize the working varibles pointer
  pWorkVarATS = &workVarATS;

  // Clear the working varibles
  memset((void *)pWorkVarATS, 0, sizeof(WorkVarATS_S));

  // Initialize the Air Temperature/Humidity Sensor
  airSensor.init();
}

/**
 * @brief Measures and saves the current Air Temperature/Humidity value.
 *
 * @param uint32_t slot: Current slot of the value i.e 0 <= slot <= NUM_OF_SAMPLES_PER_MSG
 */
void measureATS(uint32_t slot)
{

  // Measure and save the current Air Temperature/Humidity values
  airSensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &pWorkVarATS->lastTempValue, &pWorkVarATS->lastHumidValue);

  // Save the temperature value
  pWorkVarATS->tempValues[slot] = pWorkVarATS->lastTempValue;

  // Save the humidity value
  pWorkVarATS->humidValues[slot] = pWorkVarATS->lastHumidValue;
}

/**
 * @brief Returns the avarage Air Temperature from the last NUM_OF_SAMPLES_PER_MSG.
 *
 * @param float: Avarage Air Temperature value.
 */
float getAvgTempATS()
{

  float sum = 0;

  // Calculate and return the Air Temperature avarge value
  for (int i = 0; i < NUM_OF_SAMPLES_PER_MSG; i++)
  {
    sum += pWorkVarATS->tempValues[i];
  }

  return (sum / (float)NUM_OF_SAMPLES_PER_MSG);
}

/**
 * @brief Returns the avarage Air Humidity from the last NUM_OF_SAMPLES_PER_MSG.
 *
 * @param float: Avarage Air Humidity value.
 */
float getAvgHumidATS()
{

  float sum = 0;

  // Calculate and return the Air Temperature avarge value
  for (int i = 0; i < NUM_OF_SAMPLES_PER_MSG; i++)
  {
    sum += pWorkVarATS->humidValues[i];
  }

  return (sum / (float)NUM_OF_SAMPLES_PER_MSG);
}

/**
 * @brief Returns the last measured Air Temperature value.
 *
 * @param float: Last measured Air Temperature value.
 */
float getLastTempATS()
{
  return pWorkVarATS->lastTempValue;
}

/**
 * @brief Returns the last measured Air Humidity value.
 *
 * @param float: Last measured Air Humidity value.
 */
float getLastHumidATS()
{
  return pWorkVarATS->lastHumidValue;
}

/****************************************************************
 * Module: Light Sensor
 ****************************************************************/

/**
 * @brief Initializes the Light Sensor module.
 */
void initLS()
{

  // Initialize the working varibles pointer
  pWorkVarLS = &workVarLS;

  // Clear the working varibles
  memset((void *)pWorkVarLS, 0, sizeof(WorkVarLS_S));

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin(LS_SDA_BUS_PIN, LS_SCL_BUS_PIN);

  // Establish a connection to the Light sensor
  lightMeter.begin();
}

/**
 * @brief Measures and saves the current Light value.
 *
 * @param uint32_t slot: Current slot of the value i.e 0 <= slot <= NUM_OF_SAMPLES_PER_MSG
 */
void measureLS(uint32_t slot)
{

  // Get the current Light value
  pWorkVarLS->lastValue = lightMeter.readLightLevel();

  // Save the value
  pWorkVarLS->values[slot] = pWorkVarLS->lastValue;
}

/**
 * @brief Returns the avarage Light value from the last NUM_OF_SAMPLES_PER_MSG.
 *
 * @param float: Avarage Light value.
 */
float getAvgLS()
{

  float sum = 0;

  // Calculate and return the Light avarge value
  for (int i = 0; i < NUM_OF_SAMPLES_PER_MSG; i++)
  {
    sum += pWorkVarLS->values[i];
  }

  return (sum / (float)NUM_OF_SAMPLES_PER_MSG);
}

/**
 * @brief Returns the last measured Light value.
 *
 * @param float: Last measured Light value.
 */
float getLastLS()
{
  return pWorkVarLS->lastValue;
}
