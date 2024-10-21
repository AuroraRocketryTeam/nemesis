#include <Wire.h>
#include "sensors/ISensor.hpp"
#include "utils/logger/ILogger.hpp"
#include "sensors/BME680/BME680Sensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"

ILogger *rocketLogger;
ISensor *bme680;
ISensor *bme680_2;
ISensor *bno055;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    rocketLogger = new RocketLogger();
    bme680 = new BME680Sensor(BME680_I2C_ADDR_1);
    bme680_2 = new BME680Sensor(BME680_I2C_ADDR_2);
    bno055 = new BNO055Sensor();

    // Define a struct to store sensor initialization information
    struct SensorInitInfo
    {
        ISensor *sensor;
        std::string name;
        std::optional<int> address;
    };

    // Utility vector to initialize all sensors in a loop
    std::vector<SensorInitInfo> sensors = {
        {bme680, "BME680", BME680_I2C_ADDR_1},
        {bme680_2, "BME680", BME680_I2C_ADDR_2},
        {bno055, "BNO055", BNO055_I2C_ADDR}};

    // Lambda function to log sensor initialization result based on initialization success
    auto logInitializationResult = [&](const SensorInitInfo &sensorInfo, bool success)
    {
        if (success)
        {
            rocketLogger->logInfo(sensorInfo.name + " sensor initialized" +
                                  (sensorInfo.address.has_value() ? " on address " + std::to_string(sensorInfo.address.value())
                                                                  : ""));
        }
        else
        {
            rocketLogger->logError("Failed to initialize " + sensorInfo.name + " sensor" +
                                   (sensorInfo.address.has_value() ? " on address " + std::to_string(sensorInfo.address.value()) : ""));
        }
    };

    // Initialize all sensors
    for (const auto &sensorInfo : sensors)
    {
        bool initSuccess = sensorInfo.sensor->init();

        logInitializationResult(sensorInfo, initSuccess);
    }

    rocketLogger->logInfo("Setup complete.");
    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
    rocketLogger->clearData();
}

void loop()
{
    auto bme680Value = bme680->getData();
    if (bme680Value.has_value())
    {
        rocketLogger->logSensorData(bme680Value.value());
    }

    auto bme680_2Value = bme680_2->getData();
    if (bme680_2Value.has_value())
    {
        rocketLogger->logSensorData(bme680_2Value.value());
    }

    auto bno055Value = bno055->getData();
    if (bno055Value.has_value())
    {
        rocketLogger->logSensorData(bno055Value.value());
    }

    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
    rocketLogger->clearData();
    delay(1000);
}
