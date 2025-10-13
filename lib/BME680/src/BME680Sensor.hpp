#pragma once
/**
 * @class BME680Sensor
 * @brief A sensor class for the BME680 sensor, inheriting from the Sensor class template.
 *
 * This class provides methods to initialize the sensor, read data synchronously and asynchronously,
 * and retrieve the sensor data.
 *
 * @note The BME680 sensor is used for measuring temperature, humidity, pressure, and gas.
 *
 * @file BME680Sensor.h
 *
 * @see Sensor
 * @see BME680SensorData
 *
 * @author alessandr.monticell4@studio.unibo.it
 * @author luca.pulga@studio.unibo.it
 */
#include <Adafruit_BME680.h>
#include <ISensor.hpp>
#include <pins.h>
#include <config.h>

class BME680Sensor : public ISensor
{
public:
    BME680Sensor(uint8_t addr);
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    Adafruit_BME680 bme;
    uint8_t addr;
};
