#include "BME680Sensor.hpp"
#include <Wire.h>
#include <Adafruit_Sensor.h>

bool initialized = false;

BME680Sensor::BME680Sensor()
{
    bme = Adafruit_BME680(&Wire);
}

bool BME680Sensor::init(uint8_t addr)
{
    int attempts = 0;
    while (!bme.begin(addr) && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        delay(1000);
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return initialized;
    }
    // Set up oversampling and filter initialization (NOTE: need to study the optimal values)

    Serial.println("BME680 Sensor initialized successfully");
    initialized = true;
    return initialized;
}

bool BME680Sensor::init()
{
    return this->init(0x77);
}

std::optional<SensorData> BME680Sensor::getData()
{
    if (!bme.performReading())
    {
        Serial.println("BME680 Failed to perform reading :(");
        return std::nullopt;
    }

    SensorData data("BME680");
    data.setData("temperature", bme.temperature);
    data.setData("pressure", bme.pressure);
    data.setData("humidity", bme.humidity);
    data.setData("gas_resistance", bme.gas_resistance);

    return data;
}