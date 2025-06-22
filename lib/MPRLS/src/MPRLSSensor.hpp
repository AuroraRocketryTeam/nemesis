#pragma once
#include <ISensor.hpp>
#include <Adafruit_MPRLS.h>

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
