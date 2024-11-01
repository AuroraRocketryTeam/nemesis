#include "MPRLSSensor.hpp"
#include "const/config.h"

MPRLSSensor::MPRLSSensor()
{
    mprls = Adafruit_MPRLS();
}

bool MPRLSSensor::init()
{
    int attempts = 0;
    uint start = millis();
    while (!mprls.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        uint end = millis();
        if (end - start > SENSOR_LOOKUP_TIMEOUT)
        {
            start = millis();
        }
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return this->isInitialized();
    }
    this->setInitialized(true);
    return this->isInitialized();
}

std::optional<SensorData> MPRLSSensor::getData()
{
    this->pressure = mprls.readPressure();

    SensorData data("MPRLS");
    data.setData("pressure", this->pressure);

    return data;
}