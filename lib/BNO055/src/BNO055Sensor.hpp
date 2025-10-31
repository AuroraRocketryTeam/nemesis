#pragma once

#include <BNO055SensorInterface.hpp>
#include <SensorData.hpp>
#include <ISensor.hpp>

class BNO055Sensor : public ISensor
{
public:
    BNO055Sensor();
    bool init() override;
    bool calibrate();
    bool hardwareTest();
    std::optional<SensorData> getData() override;
    
    struct CalibrationStatus {
        uint8_t sys;
        uint8_t gyro;
        uint8_t accel;
        uint8_t mag;
    };
    CalibrationStatus getCalibration();

private:
    BNO055SensorInterface bno_interface;
};

