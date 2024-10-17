#include <Wire.h>
#include "sensors/ISensor.hpp"
#include "utils/logger/ILogger.hpp"
#include "sensors/BME680/BME680Sensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"

ILogger* rocketLogger;
ISensor* bme680;
ISensor* bme680_2;
ISensor* bno055;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    rocketLogger = new RocketLogger();
    bme680 = new BME680Sensor();
    bme680_2 = new BME680Sensor();
    bno055 = new BNO055Sensor();
    
    Serial.println("BME680 & BNO055 test");
    //bme680->init();
    //bme680_2->init();
    //bno055->init();    
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

    Serial.write(rocketLogger->getJSONAll().dump().c_str());
    rocketLogger->clearData();
    delay(1000);
}

