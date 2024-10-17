#include "BNO055Sensor.hpp"
#include "const/config.h"

static bool initialized = false;
static sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
static uint8_t systemCal, gyroCal, accelCal, magCal = 0;

static bool isCalibrated()
{
    return systemCal != 0 && gyroCal != 0 && accelCal != 0 && magCal != 0;
}

BNO055Sensor::BNO055Sensor()
{
    bno055 = Adafruit_BNO055(55, 0x28, &Wire);
}

bool BNO055Sensor::init()
{
    int attempts = 0;
    while (!bno055.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        Serial.println("Could not find a valid BNO055 sensor, check wiring!");
        delay(1000);
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return initialized;
    }
    this->calibrate();
    Serial.println("BNO055 Sensor initialized successfully");
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

    SensorData data("BNO055");
    /* TODO: Implementare la conversione da sensors_vec_t ad uno dei dati supportati (float, int o string)
    data.setData("orientation", orientationData.orientation);
    data.setData("angular_velocity", angVelocityData.gyro);
    data.setData("linear_acceleration", linearAccelData.acceleration);
    data.setData("magnetometer", magnetometerData.magnetic);
    data.setData("accelerometer", accelerometerData.acceleration);
    data.setData("gravity", gravityData.acceleration);
    data.setData("temperature", bno055.getTemp());
    */

    return data;
}

bool BNO055Sensor::calibrate()
{
    if (!initialized)
    {
        Serial.println("BNO055 Sensor not initialized");
        return false;
    }
    while (!isCalibrated())
    {
        Serial.println("BNO055 Sensor not fully calibrated. Proceed to calibration...");
        bno055.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
        delay(1000);
    }
    Serial.println("BNO055 Sensor fully calibrated");
    return true;
}