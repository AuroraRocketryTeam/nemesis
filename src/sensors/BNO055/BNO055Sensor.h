#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

#include "sensors/Sensor.h"
#include "sensors/BNO055/BNO055SensorData.h"

class BNO055Sensor : public Sensor<BNO055SensorData>
{
public:
    BNO055Sensor();
    bool init() override;
    bool readData() override;
    bool calibrate();
    BNO055SensorData getData() override;
private:
    Adafruit_BNO055 bno055;
};
#endif // BNO055_SENSOR_H
