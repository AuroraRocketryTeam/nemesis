// LoggerInterface.h
#ifndef ILOGGER_HPP
#define ILOGGER_HPP

#include <string>
#include <nlohmann/json.hpp>
#include "utils/logger/LogData.hpp"
#include "sensors/SensorData.hpp"

using json = nlohmann::json;

class ILogger {
protected:
    // Vector to store logged sensor data
    std::vector<LogData> logDataList;
public:
    // Virtual function to log informational messages
    virtual void logInfo(const std::string& message) = 0;

    // Virtual function to log warning messages
    virtual void logWarning(const std::string& message) = 0;

    // Virtual function to log error messages
    virtual void logError(const std::string& message) = 0;

    // Virtual function to log sensor data
    virtual void logSensorData(const SensorData sensorData) = 0;

    // Virtual function to get all logged sensor data as a JSON list
    virtual json getJSONAll() const = 0;

    // Virtual function to clear all stored data
    virtual void clearData() {
        logDataList.clear();
    };
};

#endif
