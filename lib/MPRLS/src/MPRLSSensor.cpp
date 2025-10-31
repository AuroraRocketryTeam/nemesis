#include "MPRLSSensor.hpp"
#include <config.h>

MPRLSSensor::MPRLSSensor()
{
    _mprls = Adafruit_MPRLS();
}

bool MPRLSSensor::init()
{
    int attempts = 0;
    uint start = millis();
    while (!_mprls.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
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

bool MPRLSSensor::updateData()
{
    _data = std::make_shared<MPRLSData>();

    _data->pressure = _mprls.readPressure();

    _data->timestamp = millis();

    if (isnan(_data->pressure)) {
        return false;
    }

    return true;
}

std::shared_ptr<MPRLSData> MPRLSSensor::getData() {
    return _data;
}