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
    std::optional<SensorData> getData() override;

private:
    Adafruit_MPRLS mprls;
    float pressure;
};
