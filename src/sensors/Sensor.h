#ifndef SENSOR_H
#define SENSOR_H

#include "SensorData.h"
#include <Arduino.h>

template <typename T>
class Sensor
{
public:
    virtual ~Sensor() = default;

    /**
     * @brief Initialize the sensor.
     *
     * @return true if the sensor was initialized successfully
     * @return false if the sensor failed to initialize
     */
    virtual bool init() = 0;
    /**
     * @brief Read data from the sensor.
     *
     * @return true if the data was read successfully
     * @return false if the data failed to read
     */
    virtual bool readData() = 0;
    /**
     * @brief Get the sensor data.
     *
     * @return T the sensor data
     */
    virtual T getData() = 0;

protected:
    // Costruttore protetto
    Sensor() {}
};

#endif // SENSOR_H