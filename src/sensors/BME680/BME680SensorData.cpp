#include "sensors/BME680/BME680SensorData.h"

String BME680SensorData::toString()
{
    return "Temperature: " + String(this->getTemperature()) + " °C\n" +
           "Pressure: " + String(this->getPressure()) + " Pa\n" +
           "Humidity: " + String(this->getHumidity()) + " %";
           // "Gas Resistance: " + String(this->getGasResistance()) + " Ohms\n";    // Not used.
}