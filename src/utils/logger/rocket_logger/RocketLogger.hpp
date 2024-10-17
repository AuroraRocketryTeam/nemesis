#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "utils/logger/ILogger.hpp"
#include "utils/logger/data/LogMessage.hpp"
#include "utils/logger/data/LogSensorData.hpp"

using json = nlohmann::json;

class RocketLogger : public ILogger {
public:
    void logInfo(const std::string& message) override;
    void logWarning(const std::string& message) override;
    void logError(const std::string& message) override;
    void logSensorData(const SensorData sensorData) override;
    json getJSONAll() const override;
    void clearData();
};
