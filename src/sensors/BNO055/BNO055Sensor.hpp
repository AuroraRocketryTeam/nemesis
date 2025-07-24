#ifndef BNO055_SENSOR_HPP
#define BNO055_SENSOR_HPP

#include "Arduino.h"
#include "sensors/BNO055/BNO055SensorInterface.hpp"
#include "sensors/ISensor.hpp"

class BNO055Sensor : public ISensor
{
public:
    BNO055Sensor();
    bool init() override;
    bool calibrate();
    std::optional<SensorData> getData() override;

private:
    BNO055SensorInterface bno_interface;
};
#endif
