// LoggerInterface.h
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "utils/logger/LogData.hpp"
#include <SensorData.hpp>

using json = nlohmann::json;

/**
 * @brief Interface for loggers.
 * 
 */
class ILogger {
protected:
    // Vector to store logged sensor data
    std::vector<LogData> logDataList;
public:
    /**
     * @brief Log an informational message.
     * 
     * @param message The informational message to log.
     */
    virtual void logInfo(const std::string& message) = 0;

    /**
     * @brief Log a warning message.
     * 
     * @param message The warning message to log.
     */
    virtual void logWarning(const std::string& message) = 0;

    /**
     * @brief Log an error message.
     * 
     * @param message The error message to log.
     */
    virtual void logError(const std::string& message) = 0;

    /**
     * @brief Log sensor data.
     * 
     * @param sensorData The sensor data to log.
     */
    virtual void logSensorData(const SensorData sensorData) = 0;

    /**
     * @brief Get all logged sensor data as a JSON list.
     * 
     * @return A json object.
     */
    virtual json getJSONAll() const = 0;

    /**
     * @brief Clear all logged sensor data.
     * 
     */
    virtual void clearData() {
        logDataList.clear();
    };
};


