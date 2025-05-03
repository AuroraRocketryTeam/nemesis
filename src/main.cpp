#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <optional>
#include <vector>
#include <string>
#include "global/config.h"
#include "utils/utilities/functions.h"
#include "utils/logger/ILogger.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "sensors/ISensor.hpp"
#include "sensors/BME680/BME680Sensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include "sensors/MPRLS/MPRLSSensor.hpp"
#include "telemetry/LoRa/E220LoRaTransmitter.hpp"

ILogger *rocketLogger;
ISensor *bno055;
ISensor *mprls1;
ISensor *mprls2;
ITransmitter *loraTransmitter;
HardwareSerial loraSerial(LORA_SERIAL);

void logTransmitterStatus(ResponseStatusContainer &transmitterStatus);
void logTransmissionResponse(ResponseStatusContainer &response);
void tcaSelect(uint8_t bus);
void setup()
{
    rocketLogger = new RocketLogger();
    rocketLogger->logInfo("Setup started.");

    loraSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();

    tcaSelect(I2C_MULTIPLEXER_MPRLS1);
    mprls1 = new MPRLSSensor();
    mprls1->init();
    tcaSelect(I2C_MULTIPLEXER_MPRLS2);
    mprls2 = new MPRLSSensor();
    mprls2->init();

    bno055 = new BNO055Sensor();
    bno055->init();

    loraTransmitter = new E220LoRaTransmitter(loraSerial, LORA_AUX, LORA_M0, LORA_M1);
    auto transmitterStatus = loraTransmitter->init();
    logTransmitterStatus(transmitterStatus);
    rocketLogger->logInfo(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)).c_str());
    rocketLogger->logInfo("Setup complete.");
    auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
    logTransmissionResponse(response);

    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
}

void loop()
{
    {
        auto bno055_data = bno055->getData();
        if (bno055_data.has_value())
        {
            rocketLogger->logSensorData(bno055_data.value());
        }

        tcaSelect(I2C_MULTIPLEXER_MPRLS1);
        auto mprls1_data = mprls1->getData();
        if (mprls1_data.has_value())
        {
            rocketLogger->logSensorData(mprls1_data.value());
        }

        tcaSelect(I2C_MULTIPLEXER_MPRLS2);
        auto mprls2_data = mprls2->getData();
        if (mprls2_data.has_value())
        {
            rocketLogger->logSensorData(mprls2_data.value());
        }
    }
    auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
    logTransmissionResponse(response);
    Serial.write((rocketLogger->getJSONAll().dump(4) + "\n").c_str());
    Serial.println("######################################");
    rocketLogger->clearData();
}

// Log transmitter initialization status
void logTransmitterStatus(ResponseStatusContainer &transmitterStatus)
{
    if (transmitterStatus.getCode() == RESPONSE_STATUS::E220_SUCCESS)
    {
        rocketLogger->logInfo(
            ("LoRa transmitter initialized with configuration: " +
             static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                .c_str());
    }
    else
    {
        rocketLogger->logError(
            ("Failed to initialize LoRa transmitter with error: " +
             transmitterStatus.getDescription() +
             " (" + String(transmitterStatus.getCode()) + ")")
                .c_str());
        rocketLogger->logInfo(("Current configuration: " +
                               static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                                  .c_str());
    }
}

// Log data transmission response
void logTransmissionResponse(ResponseStatusContainer &response)
{
    response.getCode() != RESPONSE_STATUS::E220_SUCCESS
        ? rocketLogger->logError(("Failed to transmit data with error: " + response.getDescription() + " (" + String(response.getCode()) + ")").c_str())
        : rocketLogger->logInfo("Data transmitted successfully.");
}

// Function to select the TCA9548A multiplexer bus
void tcaSelect(uint8_t bus)
{
    Wire.beginTransmission(0x70); // TCA9548A address
    Wire.write(1 << bus);         // send byte to select bus
    Wire.endTransmission();
}
