#include "sensors/BME680/BME680SensorData.h"

float BME680SensorData::getTemperature() const
{
    return temperature;
}

uint32_t BME680SensorData::getPressure() const
{
    return pressure;
}

float BME680SensorData::getHumidity() const
{
    return humidity;
}

uint32_t BME680SensorData::getGasResistance() const
{
    return gas_resistance;
}

String BME680SensorData::toString()
{
    return "Temperature: " + String(temperature) + "°C\n" +
           "Pressure: " + String(pressure) + "hPa\n" +
           "Humidity: " + String(humidity) + "%\n" +
           "Gas Resistance: " + String(gas_resistance) + "Ohms\n";
}