#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "utils/logger/ILogger.hpp"
#include "utils/logger/data/LogMessage.hpp"
#include "utils/logger/data/LogSensorData.hpp"

using json = nlohmann::json;

/**
 * @brief Class to log messages and sensor data.
 * 
 */
class RocketLogger : public ILogger {
public:
    /**
     * @brief Log an informational message.
     * 
     * @param message The informational message to log.
     */
    void logInfo(const std::string& message) override;

    /**
     * @brief Log a warning message.
     * 
     * @param message The warning message to log.
     */
    void logWarning(const std::string& message) override;

    /**
     * @brief Log an error message.
     * 
     * @param message The error message to log.
     */
    void logError(const std::string& message) override;

    /**
     * @brief Log sensor data.
     * 
     * @param sensorData The sensor data to log.
     */
    void logSensorData(const SensorData sensorData) override;

    /**
     * @brief Get all logged sensor data as a JSON list.
     * 
     * @return A json object.
     */
    json getJSONAll() const override;

    /**
     * @brief Clear all logged sensor data.
     * 
     */
    void clearData();
};
