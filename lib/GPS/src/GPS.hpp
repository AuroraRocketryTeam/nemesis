#pragma once

#include <ISensor.hpp>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <config.h>

class GPS : public ISensor
{
public:
    GPS();
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    SFE_UBLOX_GNSS myGNSS;
};