#ifndef MPRLS_SENSOR_HPP
#define MPRLS_SENSOR_HPP
#include "sensors/ISensor.hpp"
#include <Adafruit_MPRLS.h>

class MPRLSSensor : public ISensor
{
public:
    MPRLSSensor();
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    Adafruit_MPRLS mprls;
    float pressure;
};
#endif // MPRLS_SENSOR_HPP