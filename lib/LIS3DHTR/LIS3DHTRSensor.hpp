#pragma once
#include <ISensor.hpp>
#include <LIS3DHTR.h>
#include <Wire.h>
#include <config.h>

class LIS3DHTRData : public SensorData
{
public:
    LIS3DHTRData() : SensorData("LIS3DHTR") {}

    // Acceleration (m/s^2)
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    
    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["acceleration_x"] = acceleration_x;
        sensorDataJson["acceleration_y"] = acceleration_y;
        sensorDataJson["acceleration_z"] = acceleration_z;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};

class LIS3DHTRSensor : public ISensor
{
public:
    LIS3DHTRSensor();
    bool init() override;
    bool updateData() override;

    // Public getter for each sensor value
    std::shared_ptr<LIS3DHTRData> getData();

private:
    LIS3DHTR<TwoWire> _lis;

    std::shared_ptr<LIS3DHTRData> _data;
};