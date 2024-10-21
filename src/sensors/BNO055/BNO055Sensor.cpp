#include "BNO055Sensor.hpp"
#include "const/config.h"

static bool initialized = false;
static sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
static imu::Quaternion quaternionData;
static uint8_t systemCal, gyroCal, accelCal, magCal = 0;

static bool isCalibrated()
{
    return systemCal > 0 && gyroCal > 0 && accelCal > 0 && magCal > 0;
}

BNO055Sensor::BNO055Sensor()
{
    bno055 = Adafruit_BNO055(55, 0x28, &Wire);
}

bool BNO055Sensor::init()
{
    int attempts = 0;
    uint start = millis();
    while (!bno055.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {   
        uint end = millis();
        while (end - start < SENSOR_LOOKUP_TIMEOUT)
        {
            end = millis();
        }
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return initialized;
    }
    this->calibrate();
    initialized = true;
    return initialized;
}

std::optional<SensorData> BNO055Sensor::getData()
{
    bno055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    bno055.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
    quaternionData = bno055.getQuat();

    SensorData data("BNO055");
    /** 
     * TODO: Implementare la conversione da sensors_vec_t ad uno dei dati supportati (float, int o string)
    data.setData("orientation", orientationData.orientation);
    data.setData("angular_velocity", angVelocityData.gyro);
    data.setData("linear_acceleration", linearAccelData.acceleration);
    data.setData("magnetometer", magnetometerData.magnetic);
    data.setData("accelerometer", accelerometerData.acceleration);
    data.setData("gravity", gravityData.acceleration);
    data.setData("temperature", bno055.getTemp());
    */
    data.setData("system_calibration", systemCal);
    data.setData("gyro_calibration", gyroCal);
    data.setData("accel_calibration", accelCal);
    data.setData("mag_calibration", magCal);

    data.setData("orientation", std::vector<float>{orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z});
    data.setData("angular_velocity", std::vector<float>{angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z});
    data.setData("linear_acceleration", std::vector<float>{linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z});
    data.setData("magnetometer", std::vector<float>{magnetometerData.magnetic.x, magnetometerData.magnetic.y, magnetometerData.magnetic.z});
    data.setData("accelerometer", std::vector<float>{accelerometerData.acceleration.x, accelerometerData.acceleration.y, accelerometerData.acceleration.z});
    data.setData("gravity", std::vector<float>{gravityData.acceleration.x, gravityData.acceleration.y, gravityData.acceleration.z});
    data.setData("board_temperature", bno055.getTemp());
    data.setData("quaternion", std::vector<double>{quaternionData.w(), quaternionData.x(), quaternionData.y(), quaternionData.z()});

    return data;
}

bool BNO055Sensor::calibrate()
{
    if (!initialized)
    {
        return false;
    }
    uint start = millis();
    int attempts = 0;
    while (!isCalibrated() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        bno055.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
        uint end = millis();
        while (end - start < SENSOR_LOOKUP_TIMEOUT)
        {
            end = millis();
        }
    }
    return true;
}