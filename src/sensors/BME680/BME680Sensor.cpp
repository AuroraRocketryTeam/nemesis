#include "BME680Sensor.hpp"
#include <Wire.h>
#include <Adafruit_Sensor.h>

BME680Sensor::BME680Sensor(uint8_t addr)
{
    bme = Adafruit_BME680(&Wire);
    this->addr = addr;
}

bool BME680Sensor::init()
{
    int attempts = 0;
    uint start = millis();
    while (!bme.begin(addr) && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        uint end = millis();
        if (end - start > SENSOR_LOOKUP_TIMEOUT)
        {
            start = millis();
        }
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return this->isInitialized();
    }
    // Set up oversampling (NOTE: need to study the optimal values)
    bme.setGasHeater(0,0);                           // Turn off gas heater
    bme.setTemperatureOversampling(BME680_OS_8X);   // Set temperature oversampling to 8x
    bme.setHumidityOversampling(BME680_OS_1X);      // Set humidity oversampling to 1x
    bme.setPressureOversampling(BME680_OS_16X);     // Set pressure oversampling to 16x (max)
    
    this->setInitialized(true);
    return this->isInitialized();
}

std::optional<SensorData> BME680Sensor::getData()
{
    if (!bme.performReading() || !this->isInitialized())
    {
        return std::nullopt;
    }

    SensorData data("BME680");
    data.setData("temperature", bme.temperature);
    data.setData("pressure", bme.pressure);
    data.setData("humidity", bme.humidity);
    data.setData("gas_resistance", bme.gas_resistance);

    return data;
}