#include <Arduino.h>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <LIS3DHTRSensor.hpp>
#include <MS561101BA03.hpp>
#include <GPS.hpp>
#include <Termoresistenze.hpp>
#include <config.h>
#include <RocketLogger.hpp>
#include <CSVLogger.hpp> // Aggiungi il CSV Logger

// Sensor objects
BNO055Sensor bno;
// MPRLSSensor mprls;
LIS3DHTRSensor lis3dh;
MS561101BA03 ms56_1(0x77);
MS561101BA03 ms56_2(0x76);
GPS gps;
// Termoresistenze termoresistenze(THERMISTOR_PIN, SERIES_RESISTOR, NOMINAL_RESISTANCE, NOMINAL_TEMPERATURE, B_COEFFICIENT);

// Logger objects
RocketLogger rocketLogger;
CSVLogger csvLogger("flight_data.csv"); // Crea il CSV logger

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
enum ActuatorState
{
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

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setRxBufferSize(2048);

    bno.init();
    // mprls.init();
    lis3dh.init();
    ms56_1.init();
    ms56_2.init();
    gps.init();
    // termoresistenze.init();

    // Inizializza il CSV Logger
    if (!csvLogger.init())
    {
        Serial.println("Errore inizializzazione CSV Logger - continuando senza CSV");
    }
    else
    {
        Serial.println("CSV Logger inizializzato correttamente");
    }

    // Testing the buzzer or actuators (just change the pins)
    pinMode(MAIN_ACTUATORS_PIN, OUTPUT);
    pinMode(DROGUE_ACTUATORS_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(MAIN_ACTUATORS_PIN, LOW);
    digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

void loop()
{
    // Retrieve data from all sensors
    // auto mprlsDataOpt = mprls.getData();
    auto bnoDataOpt = bno.getData();
    auto lis3dhDataOpt = lis3dh.getData();
    auto ms56DataOpt_1 = ms56_1.getData();
    auto ms56DataOpt_2 = ms56_2.getData();

    auto gpsDataOpt = gps.getData();

    // auto termoresistenzeDataOpt = termoresistenze.getData();

    // Create SensorData objects for each sensor
    // if (mprlsDataOpt.has_value()) {
    //    rocketLogger.logSensorData(mprlsDataOpt.value());
    //}

    if (bnoDataOpt.has_value())
    {
        rocketLogger.logSensorData(bnoDataOpt.value());
    }
    else
    {
        rocketLogger.logError("BNO055 data not available");
    }

    if (lis3dhDataOpt.has_value())
    {
        rocketLogger.logSensorData(lis3dhDataOpt.value());
    }
    else
    {
        rocketLogger.logError("LIS3DHTR data not available");
    }

    if (ms56DataOpt_1.has_value())
    {
        rocketLogger.logSensorData("BAR1", ms56DataOpt_1.value());
    }
    else
    {
        rocketLogger.logError("BAR1 data not available");
    }

    if (ms56DataOpt_2.has_value())
    {
        rocketLogger.logSensorData("BAR2", ms56DataOpt_2.value());
    }
    else
    {
        rocketLogger.logError("BAR2 data not available");
    }

    // if (termoresistenzeDataOpt.has_value()) {
    //     rocketLogger.logSensorData("Termoresistenze", *termoresistenzeDataOpt);
    // } else {
    //     rocketLogger.logError("Termoresistenze data not available");
    // }

    if (gpsDataOpt.has_value())
    {
        rocketLogger.logSensorData("GPS", gpsDataOpt.value());
    }
    else
    {
        rocketLogger.logError("GPS data not available");
    }

    // Buzzer control logic
    unsigned long currentTime = millis();

    switch (currentActuatorState)
    {
    case MAIN_ON:
        if (!toggleStateMainAct)
        {
            digitalWrite(MAIN_ACTUATORS_PIN, HIGH);
            digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
            toggleStateMainAct = true;
            toggleStateDrogueAct = false;
            actuatorsTimer = currentTime;
        }
        else if (currentTime - actuatorsTimer >= actuatorsDuration)
        {
            digitalWrite(MAIN_ACTUATORS_PIN, LOW);
            toggleStateMainAct = false;
            currentActuatorState = MAIN_PAUSE;
            actuatorsTimer = currentTime;
        }
        break;

    case MAIN_PAUSE:
        if (currentTime - actuatorsTimer >= actuatorsDelay)
        {
            currentActuatorState = DROGUE_ON;
            actuatorsTimer = currentTime;
        }
        break;

    case DROGUE_ON:
        if (!toggleStateDrogueAct)
        {
            digitalWrite(DROGUE_ACTUATORS_PIN, HIGH);
            digitalWrite(MAIN_ACTUATORS_PIN, LOW);
            toggleStateDrogueAct = true;
            toggleStateMainAct = false;
            actuatorsTimer = currentTime;
        }
        else if (currentTime - actuatorsTimer >= actuatorsDuration)
        {
            digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
            toggleStateDrogueAct = false;
            currentActuatorState = DROGUE_PAUSE;
            actuatorsTimer = currentTime;
        }
        break;

    case DROGUE_PAUSE:
        if (currentTime - actuatorsTimer >= actuatorsDelay)
        {
            currentActuatorState = MAIN_ON;
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

    // if (buzzerPlaying) {
    //     if (currentTime - buzzerTimer >= buzzerDuration) {
    //         digitalWrite(BUZZER_PIN, LOW);
    //         buzzerPlaying = false;
    //         buzzerTimer = currentTime;
    //     }
    // } else {
    //     if (currentTime - buzzerTimer >= buzzerInterval) {
    //         digitalWrite(BUZZER_PIN, HIGH);
    //         buzzerPlaying = true;
    //         buzzerTimer = currentTime;
    //     }
    // }

    auto buzzerData = SensorData("Buzzer");
    buzzerData.setData("State", buzzerPlaying ? "ON" : "OFF");
    rocketLogger.logSensorData(buzzerData);

    // Voltage from batteries
    int adcValue = analogRead(ADC_PIN);
    float voltage = ((adcValue / 4095.0) * 3.3) * 2;
    // Normalize as percentage in range [5.6, 7.2]
    float percent = (voltage - 5.6f) / (7.2f - 5.6f) * 100.0f;
    std::string voltageStr = (String(percent, 1) + "%").c_str();
    auto voltageData = SensorData("Voltage");
    voltageData.setData("ADC_Value", adcValue);
    voltageData.setData("Voltage", voltage);
    voltageData.setData("Percentage", voltageStr); // Current 62.5
    rocketLogger.logSensorData(voltageData);

    // Get all JSON data from RocketLogger
    json allData = rocketLogger.getJSONAll();

    // Log to CSV with current timestamp
    csvLogger.logSensorData(allData, currentTime);

    // Continue with your existing serial output
    std::vector<uint8_t> cborData = nlohmann::json::to_cbor(allData);

    // Option 1: just send raw CBOR (not framed)
    // Serial.write(cborData.data(), cborData.size());

    // Option 2: send framed (length first) for Python parsing
    // uint32_t size = cborData.size();
    // Serial.write(reinterpret_cast<uint8_t*>(&size), sizeof(size));
    // Serial.write(cborData.data(), cborData.size());

    Serial.println(allData.dump(4).c_str());
    Serial.flush();

    rocketLogger.clearData();

    delay(100);
}