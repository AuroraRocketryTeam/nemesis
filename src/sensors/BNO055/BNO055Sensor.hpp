#pragma once

#include <Adafruit_BNO055.h>
#include "sensors/ISensor.hpp"

class BNO055Sensor : public ISensor
{
public:
    BNO055Sensor();
    bool init() override;    
    bool calibrate();
    std::optional<SensorData> getData() override;

private:
    Adafruit_BNO055 bno055;
};

