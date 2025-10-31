/*#include <Wire.h>
#include "sensors/ISensor.hpp"
#include "utils/logger/ILogger.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include <ISensor.hpp>
#include <BME680Sensor.hpp>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include "telemetry/LoRa/E220LoRaTransmitter.hpp"
#include "utils/logger/SD/SD-master.hpp"

using TransmitDataType = std::variant<char*, String, std::string, nlohmann::json>;

ILogger *logger;
SD *sdModule;

ISensor *bno055;
ISensor *mprls1;
ISensor *mprls2;
ITransmitter<TransmitDataType> *loraTransmitter;
HardwareSerial loraSerial(LORA_SERIAL);

std::string log_file = "log.json";

void logTransmitterStatus(ResponseStatusContainer &transmitterStatus);
void logTransmissionResponse(ResponseStatusContainer &response);
void tcaSelect(uint8_t bus);
void logToSDCard(const std::string &filename, const std::string &data);

void setup()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    logger = new RocketLogger();
    logger->logInfo("Setup started.");

    sdModule = new SD();

    sdModule->init() ? logger->logInfo("SD card initialized.") : logger->logError("Failed to initialize SD card.");

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
    logger->logInfo("Setup complete.");
    logToSDCard(log_file, logger->getJSONAll().dump(4) + "\n");
    auto response = loraTransmitter->transmit(logger->getJSONAll());
    logTransmissionResponse(response);
}

void loop()
{

    {
        auto bno055_data = bno055->getData();
        if (bno055_data.has_value())
        {
            logger->logSensorData(bno055_data.value());
        }

        tcaSelect(I2C_MULTIPLEXER_MPRLS1);
        auto mprls1_data = mprls1->getData();
        if (mprls1_data.has_value())
        {
            logger->logSensorData(mprls1_data.value());
        }

        tcaSelect(I2C_MULTIPLEXER_MPRLS2);
        auto mprls2_data = mprls2->getData();
        if (mprls2_data.has_value())
        {
            logger->logSensorData(mprls2_data.value());
        }
    }
    logToSDCard(log_file, logger->getJSONAll().dump(4) + "\n");
    auto response = loraTransmitter->transmit(logger->getJSONAll());
    logTransmissionResponse(response);
    logger->clearData();
}

// Log transmitter initialization status
void logTransmitterStatus(ResponseStatusContainer &transmitterStatus)
{
    if (transmitterStatus.getCode() == RESPONSE_STATUS::E220_SUCCESS)
    {
        logger->logInfo(
            ("LoRa transmitter initialized with configuration: " +
             static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                .c_str());
    }
    else
    {
        logger->logError(
            ("Failed to initialize LoRa transmitter with error: " +
             transmitterStatus.getDescription() +
             " (" + String(transmitterStatus.getCode()) + ")")
                .c_str());
        logger->logInfo(("Current configuration: " +
                               static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                                  .c_str());
    }

    auto bno055Value = bno055->getData();
    if (bno055Value.has_value())
    {
        logger->logSensorData(bno055Value.value());
    }

    Serial.write(logger->getJSONAll().dump(4).c_str());
    logger->clearData();
    delay(1000);
}
*/