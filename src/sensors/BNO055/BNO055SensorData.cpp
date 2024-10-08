#include "BNO055SensorData.h"

sensors_vec_t BNO055SensorData::getOrientation() const
{
    return this->orientation;
}

sensors_vec_t BNO055SensorData::getAngularVelocity() const
{
    return this->angularVelocity;
}

sensors_vec_t BNO055SensorData::getLinearAcceleration() const
{
    return this->linearAcceleration;
}

sensors_vec_t BNO055SensorData::getMagnetometer() const
{
    return this->magnetometer;
}

sensors_vec_t BNO055SensorData::getAccelerometer() const
{
    return this->accelerometer;
}

sensors_vec_t BNO055SensorData::getGravity() const
{
    return this->gravity;
}

int8_t BNO055SensorData::getBoardTemperature() const
{
    return this->board_temperature;
}

uint8_t BNO055SensorData::getSystemCalibration() const
{
    return this->system_calibration;
}

uint8_t BNO055SensorData::getGyroCalibration() const
{
    return this->gyro_calibration;
}

uint8_t BNO055SensorData::getAccelCalibration() const
{
    return this->accel_calibration;
}

uint8_t BNO055SensorData::getMagCalibration() const
{
    return this->mag_calibration;
}

String BNO055SensorData::toString()
{
    sensors_vec_t orient = this->getOrientation(), angVel = this->getAngularVelocity(),
                  linAcc = this->getLinearAcceleration(), mag = this->getMagnetometer(),
                  accel = this->getAccelerometer(), grav = this->getGravity();

    String result = "Orientation: [" + String(orient.x) + ", " + String(orient.y) + ", " + String(orient.z) + "]\n";
    result += "Angular Velocity: [" + String(angVel.x) + ", " + String(angVel.y) + ", " + String(angVel.z) + "]\n";
    result += "Linear Acceleration: [" + String(linAcc.x) + ", " + String(linAcc.y) + ", " + String(linAcc.z) + "]\n";
    result += "Magnetometer: [" + String(mag.x) + ", " + String(mag.y) + ", " + String(mag.z) + "]\n";
    result += "Accelerometer: [" + String(accel.x) + ", " + String(accel.y) + ", " + String(accel.z) + "]\n";
    result += "Gravity: [" + String(grav.x) + ", " + String(grav.y) + ", " + String(grav.z) + "]\n";
    result += "Board Temperature: " + String(this->getBoardTemperature()) + " °C\n";
    
    result += String("Calibration Levels:") +
              " Sys=" + String(this->getSystemCalibration()) +
              " Gyro=" + String(this->getGyroCalibration()) +
              " Accel=" + String(this->getAccelCalibration()) +
              " Mag=" + String(this->getMagCalibration());
    return result;
}
