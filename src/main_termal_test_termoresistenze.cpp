/*#include <Arduino.h>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <LIS3DHTRSensor.hpp>
#include <MS561101BA03.hpp>
#include <GPS.hpp>
#include <Termoresistenze.hpp>
#include <config.h>
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "utils/logger/data/SensorData.hpp"
#include "utils/logger/LogData.hpp"

BNO055Sensor bno;
//MPRLSSensor mprls;
//LIS3DHTRSensor lis3dh;
//MS561101BA03 ms56_1(0x77);
//MS561101BA03 ms56_2(0x76);
//GPS gps;
Termoresistenze termoresistenze(THERMISTOR_PIN, SERIES_RESISTOR, NOMINAL_RESISTANCE, NOMINAL_TEMPERATURE, B_COEFFICIENT);

// Logger object
RocketLogger logger;

// ADC pin for voltage measurement
#define ADC_PIN A0

// Timing variables for actuators and buzzer
#define MAIN_ACTUATORS_PIN D0
#define DROGUE_ACTUATORS_PIN D1
unsigned long actuatorsTimer = 0;
unsigned long actuatorsDuration = 1000;
unsigned long actuatorsDelay = 3000;
bool toggleStateMainAct = false;
bool toggleStateDrogueAct = false;

// State machine for actuator sequencing
enum ActuatorState {
    MAIN_ON,
    MAIN_PAUSE,
    DROGUE_ON,
    DROGUE_PAUSE
};
ActuatorState currentActuatorState = MAIN_ON;

#define BUZZER_PIN D2
unsigned long buzzerTimer = 0;
unsigned long buzzerDuration = 3000;
unsigned long buzzerInterval = 10000;
bool buzzerPlaying = false;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setRxBufferSize(2048);
    
    //bno.init();
    //mprls.init();
    //lis3dh.init();
    //ms56_1.init();
    //ms56_2.init();
    //gps.init();
    termoresistenze.init();

    // Testing the buzzer or actuators (just change the pins)
    pinMode(MAIN_ACTUATORS_PIN, OUTPUT);
    pinMode(DROGUE_ACTUATORS_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(MAIN_ACTUATORS_PIN, LOW);
    digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);}

void loop() {
    // Retrieve data from all sensors
    //auto mprlsDataOpt = mprls.getData();
    //auto bnoDataOpt = bno.getData();
    //auto lis3dhDataOpt = lis3dh.getData();
    //auto ms56DataOpt_1 = ms56_1.getData();
    //auto ms56DataOpt_2 = ms56_2.getData();

    //auto gpsDataOpt = gps.getData();

    auto termoresistenzeDataOpt = termoresistenze.getData();

    // Create SensorData objects for each sensor
    //if (mprlsDataOpt.has_value()) {
    //    logger.logSensorData(mprlsDataOpt.value());
    //}

    //if (bnoDataOpt.has_value()) {
    //    logger.logSensorData(bnoDataOpt.value());
    //} else {
    //    logger.logError("BNO055 data not available");
    //}

    //if (lis3dhDataOpt.has_value()) {
    //    logger.logSensorData(lis3dhDataOpt.value());
    //} else {
    //    logger.logError("LIS3DHTR data not available");
    //}

    //if (ms56DataOpt_1.has_value()) {
    //    logger.logSensorData("BAR1", ms56DataOpt_1.value());
    //} else {
    //    logger.logError("BAR1 data not available");
    //}

    //if (ms56DataOpt_2.has_value()) {
    //    logger.logSensorData("BAR2", ms56DataOpt_2.value());
    //} else {
    //    logger.logError("BAR2 data not available");
    //}

    if (termoresistenzeDataOpt.has_value()) {
        logger.logSensorData("Termoresistenze", *termoresistenzeDataOpt);
    } else {
        logger.logError("Termoresistenze data not available");
    }

    //if (gpsDataOpt.has_value()) {
    //    logger.logSensorData("GPS", gpsDataOpt.value());
    //} else {
    //    logger.logError("GPS data not available");
    //}

    // Buzzer control logic
    //unsigned long currentTime = millis();

    //switch (currentActuatorState) {
    //    case MAIN_ON:
    //        if (!toggleStateMainAct) {
    //            digitalWrite(MAIN_ACTUATORS_PIN, HIGH);
    //            digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
    //            toggleStateMainAct = true;
    //            toggleStateDrogueAct = false;
    //            actuatorsTimer = currentTime;
    //        } else if (currentTime - actuatorsTimer >= actuatorsDuration) {
    //            digitalWrite(MAIN_ACTUATORS_PIN, LOW);
    //            toggleStateMainAct = false;
    //            currentActuatorState = MAIN_PAUSE;
    //            actuatorsTimer = currentTime;
    //        }
    //        break;
    //        
    //    case MAIN_PAUSE:
    //        if (currentTime - actuatorsTimer >= actuatorsDelay) {
    //            currentActuatorState = DROGUE_ON;
    //            actuatorsTimer = currentTime;
    //        }
    //        break;
    //        
    //    case DROGUE_ON:
    //        if (!toggleStateDrogueAct) {
    //            digitalWrite(DROGUE_ACTUATORS_PIN, HIGH);
    //            digitalWrite(MAIN_ACTUATORS_PIN, LOW);
    //            toggleStateDrogueAct = true;
    //            toggleStateMainAct = false;
    //            actuatorsTimer = currentTime;
    //        } else if (currentTime - actuatorsTimer >= actuatorsDuration) {
    //            digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
    //            toggleStateDrogueAct = false;
    //            currentActuatorState = DROGUE_PAUSE;
    //            actuatorsTimer = currentTime;
    //        }
    //        break;
    //        
    //    case DROGUE_PAUSE:
    //        if (currentTime - actuatorsTimer >= actuatorsDelay) {
    //            currentActuatorState = MAIN_ON;
    //            actuatorsTimer = currentTime;
    //        }
    //        break;
    //}

    //auto mainActuatorsData = SensorData("MainActuators");
    //mainActuatorsData.setData("State", toggleStateMainAct ? "ON" : "OFF");
    //logger.logSensorData(mainActuatorsData);

    //auto drogueActuatorsData = SensorData("DrogueActuators");
    //drogueActuatorsData.setData("State", toggleStateDrogueAct ? "ON" : "OFF");
    //logger.logSensorData(drogueActuatorsData);

    //if (buzzerPlaying) {
    //    if (currentTime - buzzerTimer >= buzzerDuration) {
    //        digitalWrite(BUZZER_PIN, LOW);
    //        buzzerPlaying = false;
    //        buzzerTimer = currentTime;
    //    }
    //} else {
    //    if (currentTime - buzzerTimer >= buzzerInterval) {
    //        digitalWrite(BUZZER_PIN, HIGH);
    //        buzzerPlaying = true;
    //        buzzerTimer = currentTime;
    //    }
    //}

    //auto buzzerData = SensorData("Buzzer");
    //buzzerData.setData("State", buzzerPlaying ? "ON" : "OFF");
    //logger.logSensorData(buzzerData);

    // Voltage
    int adcValue = analogRead(ADC_PIN);
    float voltage = (adcValue / 4095.0) * 3.3;
    auto voltageData = SensorData("Voltage");
    voltageData.setData("ADC_Value", adcValue);
    voltageData.setData("Voltage", voltage);
    logger.logSensorData(voltageData);

    // Log all data
    Serial.println(logger.getJSONAll().dump().c_str());
    Serial.flush();

    logger.clearData();

    delay(100);
}
*/