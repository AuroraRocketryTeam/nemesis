#include "sensors/BNO055/BNO055SensorData.h"
#include "BNO055Sensor.h"
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

bool BNO055Sensor::readData()
{
    if (!initialized)
    {
        Serial.println("BNO055 Sensor not initialized");
        return false;
    }    
    bno055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    return true;
}

BNO055SensorData BNO055Sensor::getData()
{
    bno055.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
    return BNO055SensorData(
        orientationData.orientation,
        angVelocityData.gyro,
        linearAccelData.acceleration,
        magnetometerData.magnetic,
        accelerometerData.acceleration,
        gravityData.acceleration,
        bno055.getTemp(),
        systemCal, gyroCal, accelCal, magCal);
}

bool BNO055Sensor::calibrate()
{
    if(!initialized)
    {
        Serial.println("BNO055 Sensor not initialized");
        return false;
    }
    while(!isCalibrated())
    {
        Serial.println("BNO055 Sensor not fully calibrated. Proceed to calibration...");
        bno055.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
        delay(1000);
    }
    Serial.println("BNO055 Sensor fully calibrated");
    return true;
}