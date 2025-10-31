#pragma once
#include <ISensor.hpp>
#include <Adafruit_MPRLS.h>

class MPRLSData : public SensorData
{
public:
    MPRLSData() : SensorData("MPRLS") {}

    // Pressure (hPa)
    float pressure;

    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["pressure"] = pressure;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};

class MPRLSSensor : public ISensor
{
public:
    MPRLSSensor();
    bool init() override;
    bool updateData() override;

    // Public getter for each sensor value
    std::shared_ptr<MPRLSData> getData();

private:
    Adafruit_MPRLS _mprls;
    std::shared_ptr<MPRLSData> _data;
};
