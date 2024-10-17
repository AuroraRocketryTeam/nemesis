#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include "sensors/SensorData.hpp"
#include <optional>

class ISensor
{
public:
    /**
     * @brief Initialize the sensor.
     *
     * @return true if the sensor was initialized successfully
     * @return false if the sensor failed to initialize
     */
    virtual bool init() = 0;

    /**
     * @brief Get the sensor data.
     *
     * @return T the sensor data
     */
    virtual std::optional<SensorData> getData() = 0;
};

#endif // ISENSOR_HPP