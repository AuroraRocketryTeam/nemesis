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

class BME680Data : public SensorData
{
public:
    BME680Data() : SensorData("BME680") {}

    // Temperature (°C)
    float temperature;
    // Humidity (%)
    float humidity;
    // Pressure (Pa)
    float pressure;
    // Gas resistance (Ohm)
    float gasResistance;

    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["temperature"] = temperature;
        sensorDataJson["humidity"] = humidity;
        sensorDataJson["pressure"] = pressure;
        sensorDataJson["gasResistance"] = gasResistance;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};

class BME680Sensor : public ISensor
{
public:
    BME680Sensor(uint8_t addr);
    bool init() override;
    bool updateData() override;
    
    // Public getter for each sensor value
    std::shared_ptr<BME680Data> getData();

private:
    Adafruit_BME680 bme;
    uint8_t addr;

    std::shared_ptr<BME680Data> _data;
};
