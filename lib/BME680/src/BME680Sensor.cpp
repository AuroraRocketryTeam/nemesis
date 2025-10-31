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

bool BME680Sensor::updateData()
{
    if (!bme.performReading() || !this->isInitialized())
    {
        return false;
    }

    _data = std::make_shared<BME680Data>();
    _data->temperature = bme.temperature;
    _data->humidity = bme.humidity;
    _data->pressure = bme.pressure;
    _data->gasResistance = bme.gas_resistance;
    _data->timestamp = millis();

    return true;
}

std::shared_ptr<BME680Data> BME680Sensor::getData()
{
    return _data;
}
