#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include "sensors/SensorData.hpp"
#include <optional>

/**
 * @brief Interface for sensors.
 * 
 */
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
     * @brief Read and then get the sensor data.
     *
     * @return Just read data.
     */
    virtual std::optional<SensorData> getData() = 0;

protected:
    bool initialized = false;
};

#endif // ISENSOR_HPP