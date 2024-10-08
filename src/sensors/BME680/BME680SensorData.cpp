#include "sensors/BME680/BME680SensorData.h"

float BME680SensorData::getTemperature() const
{
    return this->temperature;
}

uint32_t BME680SensorData::getPressure() const
{
    return this->pressure;
}

float BME680SensorData::getHumidity() const
{
    return this->humidity;
}

uint32_t BME680SensorData::getGasResistance() const
{
    return this->gas_resistance;
}

String BME680SensorData::toString()
{
    return "Temperature: " + String(this->temperature) + "°C\n" +
           "Pressure: " + String(this->pressure) + "Pa\n" +
           "Humidity: " + String(this->humidity) + "%\n" +
           "Gas Resistance: " + String(this->gas_resistance) + "Ohms\n";
}