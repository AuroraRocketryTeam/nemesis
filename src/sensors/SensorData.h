#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <Arduino.h>
/**
 * @brief An abstract base class for sensor data.
 */
class SensorData
{
public:
    virtual ~SensorData() = default;
    virtual String toString() = 0;

protected:
    // Costruttore protetto per impedire l'istanza diretta
    SensorData() {}
};

#endif // SENSORDATA_H
