#include "Nemesis.hpp"

// ADC pin for voltage measurement
#define ADC_PIN A0

// Actuator and Buzzer pins
#define MAIN_ACTUATORS_PIN D0
#define DROGUE_ACTUATORS_PIN D1
#define BUZZER_PIN D2

Nemesis::Nemesis() : 
    ms56_1(0x77), 
    ms56_2(0x76),
    //termoresistenze(THERMISTOR_PIN, SERIES_RESISTOR, NOMINAL_RESISTANCE, NOMINAL_TEMPERATURE, B_COEFFICIENT),
    actuatorsTimer(0),
    actuatorsDuration(1000),
    actuatorsDelay(3000),
    toggleStateMainAct(false),
    toggleStateDrogueAct(false),
    currentActuatorState(ActuatorState::MAIN_ON),
    buzzerTimer(0),
    buzzerDuration(3000),
    buzzerInterval(10000),
    buzzerPlaying(false)
{}

void Nemesis::init() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setRxBufferSize(2048);
    
    bno.init();
    //mprls.init();
    lis3dh.init();
    ms56_1.init();
    ms56_2.init();
    gps.init();
    //termoresistenze.init();

    pinMode(MAIN_ACTUATORS_PIN, OUTPUT);
    pinMode(DROGUE_ACTUATORS_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(MAIN_ACTUATORS_PIN, LOW);
    digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

void Nemesis::run() {
    readSensors();
    controlActuators();
    // controlBuzzer();
    readBattery();
    logData();
}

void Nemesis::readSensors() {
    //auto mprlsDataOpt = mprls.getData();
    auto bnoDataOpt = bno.getData();
    auto lis3dhDataOpt = lis3dh.getData();
    auto ms56DataOpt_1 = ms56_1.getData();
    auto ms56DataOpt_2 = ms56_2.getData();
    auto gpsDataOpt = gps.getData();
    //auto termoresistenzeDataOpt = termoresistenze.getData();

    //if (mprlsDataOpt.has_value()) {
    //    rocketLogger.logSensorData(mprlsDataOpt.value());
    //}

    if (bnoDataOpt.has_value()) {
        rocketLogger.logSensorData(bnoDataOpt.value());
    } else {
        rocketLogger.logError("BNO055 data not available");
    }

    if (lis3dhDataOpt.has_value()) {
        rocketLogger.logSensorData(lis3dhDataOpt.value());
    } else {
        rocketLogger.logError("LIS3DHTR data not available");
    }

    if (ms56DataOpt_1.has_value()) {
        rocketLogger.logSensorData("BAR1", ms56DataOpt_1.value());
    } else {
        rocketLogger.logError("BAR1 data not available");
    }

    if (ms56DataOpt_2.has_value()) {
        rocketLogger.logSensorData("BAR2", ms56DataOpt_2.value());
    } else {
        rocketLogger.logError("BAR2 data not available");
    }
    
    //if (termoresistenzeDataOpt.has_value()) {
    //    rocketLogger.logSensorData("Termoresistenze", *termoresistenzeDataOpt);
    //} else {
    //    rocketLogger.logError("Termoresistenze data not available");
    //}

    if (gpsDataOpt.has_value()) {
        rocketLogger.logSensorData("GPS", gpsDataOpt.value());
    } else {
        rocketLogger.logError("GPS data not available");
    }
}

void Nemesis::controlActuators() {
    unsigned long currentTime = millis();

    switch (currentActuatorState) {
        case ActuatorState::MAIN_ON:
            if (!toggleStateMainAct) {
                digitalWrite(MAIN_ACTUATORS_PIN, HIGH);
                digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
                toggleStateMainAct = true;
                toggleStateDrogueAct = false;
                actuatorsTimer = currentTime;
            } else if (currentTime - actuatorsTimer >= actuatorsDuration) {
                digitalWrite(MAIN_ACTUATORS_PIN, LOW);
                toggleStateMainAct = false;
                currentActuatorState = ActuatorState::MAIN_PAUSE;
                actuatorsTimer = currentTime;
            }
            break;
            
        case ActuatorState::MAIN_PAUSE:
            if (currentTime - actuatorsTimer >= actuatorsDelay) {
                currentActuatorState = ActuatorState::DROGUE_ON;
                actuatorsTimer = currentTime;
            }
            break;
            
        case ActuatorState::DROGUE_ON:
            if (!toggleStateDrogueAct) {
                digitalWrite(DROGUE_ACTUATORS_PIN, HIGH);
                digitalWrite(MAIN_ACTUATORS_PIN, LOW);
                toggleStateDrogueAct = true;
                toggleStateMainAct = false;
                actuatorsTimer = currentTime;
            } else if (currentTime - actuatorsTimer >= actuatorsDuration) {
                digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
                toggleStateDrogueAct = false;
                currentActuatorState = ActuatorState::DROGUE_PAUSE;
                actuatorsTimer = currentTime;
            }
            break;
            
        case ActuatorState::DROGUE_PAUSE:
            if (currentTime - actuatorsTimer >= actuatorsDelay) {
                currentActuatorState = ActuatorState::MAIN_ON;
                actuatorsTimer = currentTime;
            }
            break;
    }

    auto mainActuatorsData = SensorData("MainActuators");
    mainActuatorsData.setData("State", digitalRead(MAIN_ACTUATORS_PIN) == HIGH ? "ON" : "OFF");
    rocketLogger.logSensorData(mainActuatorsData);

    auto drogueActuatorsData = SensorData("DrogueActuators");
    drogueActuatorsData.setData("State", digitalRead(DROGUE_ACTUATORS_PIN) == HIGH ? "ON" : "OFF");
    rocketLogger.logSensorData(drogueActuatorsData);
}

void Nemesis::controlBuzzer() {
    unsigned long currentTime = millis();
    if (buzzerPlaying) {
        if (currentTime - buzzerTimer >= buzzerDuration) {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerPlaying = false;
            buzzerTimer = currentTime;
        }
    } else {
        if (currentTime - buzzerTimer >= buzzerInterval) {
            digitalWrite(BUZZER_PIN, HIGH);
            buzzerPlaying = true;
            buzzerTimer = currentTime;
        }
    }

    auto buzzerData = SensorData("Buzzer");
    buzzerData.setData("State", buzzerPlaying ? "ON" : "OFF");
    rocketLogger.logSensorData(buzzerData);
}

void Nemesis::readBattery() {
    int adcValue = analogRead(ADC_PIN);
    float voltage = ((adcValue / 4095.0) * 3.3) * 2;
    float percent = (voltage - 5.6f) / (7.2f - 5.6f) * 100.0f;
    std::string voltageStr = (String(percent, 1) + "%").c_str();
    
    auto voltageData = SensorData("Voltage");
    voltageData.setData("ADC_Value", adcValue);
    voltageData.setData("Voltage", voltage);
    voltageData.setData("Percentage", voltageStr);
    rocketLogger.logSensorData(voltageData);
}

void Nemesis::logData() {
    Serial.println(rocketLogger.getJSONAll().dump().c_str());
    rocketLogger.clearData();
}
