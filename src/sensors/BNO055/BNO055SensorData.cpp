#include "BNO055SensorData.h"

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
