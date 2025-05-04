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
#include "utils/logger/SD/SD-master.hpp"

ILogger *rocketLogger;
SD *sdModule;

ISensor *bno055;
ISensor *mprls1;
ISensor *mprls2;
ITransmitter *loraTransmitter;
HardwareSerial loraSerial(LORA_SERIAL);

std::string temperature_log_file = "board_temperature_test.csv";

void logTransmitterStatus(ResponseStatusContainer &transmitterStatus);
void logTransmissionResponse(ResponseStatusContainer &response);
void tcaSelect(uint8_t bus);
void setup()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    rocketLogger = new RocketLogger();
    rocketLogger->logInfo("Setup started.");

    sdModule = new SD();

    sdModule->init() ? rocketLogger->logInfo("SD card initialized.") : rocketLogger->logError("Failed to initialize SD card.");

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

    std::string jsonOutput = rocketLogger->getJSONAll().dump(4) + "\n";
    Serial.write(jsonOutput.c_str());

    if (sdModule->openFile("log.json"))
    {
        sdModule->writeFile("log.json", jsonOutput);
        sdModule->closeFile();
    }
    // sdModule->writeFile("log.json", rocketLogger->getJSONAll().dump(4));
    rocketLogger->clearData();
    std::string csv_header = "TIME_ELAPSED_SEC,BOARD_TEMPERATURE_CEL_DEG\n";
    if (sdModule->openFile("board_temperature_test.csv"))
    {
        sdModule->writeFile("board_temperature_test.csv", csv_header);
        sdModule->closeFile();
    }
}

void loop()
{

    {
        auto bno055_data = bno055->getData();
        if (bno055_data.has_value())
        {
            rocketLogger->logSensorData(bno055_data.value());
            // Calculate elapsed time in milliseconds
            static unsigned long lastTime = 0; // Store the last execution time
            unsigned long currentTime = millis();

            // Execute temperature logging every 5000 milliseconds
            if (currentTime - lastTime >= 5000)
            {
                lastTime = currentTime; // Update the last execution time
                auto data_variant = bno055_data.value().getData("board_temperature");
                int board_temperature = 0;
                if (data_variant.has_value() && std::holds_alternative<int>(data_variant.value()))
                {
                    board_temperature = std::get<int>(data_variant.value());
                }
                else
                {
                    digitalWrite(LED_BLUE, LOW);
                }
                auto board_temperature_string = std::to_string(board_temperature);
                // Track time since first execution
                static unsigned long firstExecutionTime = 0;
                if (firstExecutionTime == 0)
                {
                    firstExecutionTime = millis();
                }

                // Calculate total elapsed time in seconds since program started
                unsigned long totalElapsedSeconds = (millis() - firstExecutionTime) / 1000;

                auto timeString = std::to_string(totalElapsedSeconds);

                String serial_output = "TIME ELAPSED: " + String(timeString.c_str()) + "s | BOARD TEMPERATURE: " + String(board_temperature_string.c_str());
                auto csv_output_string = timeString + "," + board_temperature_string + "\n";
                Serial.println(serial_output);
                if (sdModule->openFile(temperature_log_file))
                {
                    if (sdModule->writeFile(temperature_log_file, csv_output_string))
                    {
                        digitalWrite(LED_GREEN, LOW); // Turn on the green LED
                        delay(250);
                        digitalWrite(LED_GREEN, HIGH); // Turn off the green LED
                    }
                    sdModule->closeFile();
                }
                else
                {
                    Serial.println("Failed to open " + String(temperature_log_file.c_str()));
                    // Turn on the red LED to indicate error
                    digitalWrite(LED_RED, LOW); // Turn on the red LED
                    // Log the error
                    rocketLogger->logError("Failed to open temperature log file");
                }
            }
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
    // logTransmissionResponse(response);
    // Serial.write((rocketLogger->getJSONAll().dump(4) + "\n").c_str());
    // Serial.println("######################################");
    if (sdModule->openFile("log.json"))
    {
        sdModule->writeFile("log.json", rocketLogger->getJSONAll().dump(4) + "\n");
        sdModule->closeFile();
    }
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
