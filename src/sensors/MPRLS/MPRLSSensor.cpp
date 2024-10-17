#include "MPRLSSensor.hpp"
#include "const/config.h"

static bool initialized = false;

MPRLSSensor::MPRLSSensor()
{
    mprls = Adafruit_MPRLS();
}

bool MPRLSSensor::init()
{
    int attempts = 0;
    while (!mprls.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        Serial.println("Could not find a valid MPRLS sensor, check wiring!");
        delay(1000);
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return initialized;
    }
    Serial.println("MPRLS Sensor initialized successfully");
    initialized = true;
    return initialized;
}

std::optional<SensorData> MPRLSSensor::getData()
{
    this->pressure = mprls.readPressure();

    SensorData data("MPRLS");
    data.setData("pressure", this->pressure);

    return data;
}