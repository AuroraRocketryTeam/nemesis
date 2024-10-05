#ifndef SENSOR_H
#define SENSOR_H

#include "SensorData.h"
#include <Arduino.h>

template <typename T> 
class Sensor {
public:
    virtual ~Sensor() = default;

    virtual bool init() = 0;
    virtual T* readData() = 0;
private:
    T data;
};

#endif // SENSOR_H