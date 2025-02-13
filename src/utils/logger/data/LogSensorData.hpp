#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "sensors/SensorData.hpp"
#include "utils/logger/data/ILoggable.hpp"

/**
 * @brief A class to represent a log of sensor data.
 *
 */
class LogSensorData : public ILoggable
{
private:
    // The sensor data to be logged
    SensorData sensorData;

public:
    /**
     * @brief Construct a new Log Sensor Data object
     *
     * @param source The origin of the log.
     * @param sensorData The sensor data to be logged.
     */
    LogSensorData(std::string source, SensorData sensorData)
        : sensorData(sensorData)
    {
        this->source = source;
    }

    /**
     * @brief Get the sensor data.
     *
     * @return The sensor data.
     */
    SensorData getSensorData() const
    {
        return sensorData;
    }

    /**
     * @brief Get the JSON representation of the object.
     *
     * @return A json object.
     */
    json toJSON() const override
    {
        json j;

        // Add source to JSON
        j["source"] = source;

        // Add sensor data to JSON
        json sensorDataJson;
        auto dataMap = sensorData.getDataMap();

        for (const auto &[key, value] : dataMap)
        {
            // Use std::visit to handle each variant type
            std::visit([&sensorDataJson, &key](const auto &arg)
                       { sensorDataJson[key] = arg; }, value);
        }
        j["sensorData"] = sensorDataJson;

        return j;
    }
};

