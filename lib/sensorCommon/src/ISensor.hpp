#pragma once

#include <SensorData.hpp>
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

    /**
     * @brief Check if the sensor is initialized.
     *
     * @return true if the sensor is initialized
     * @return false if the sensor is not initialized
     */
    bool isInitialized() const
    {
        return initialized;
    }
protected:
    void setInitialized(bool initialized)
    {
        this->initialized = initialized;
    }

private:
    bool initialized = false;
};

