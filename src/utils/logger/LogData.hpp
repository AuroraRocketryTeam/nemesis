#ifndef LOG_DATA_H
#define LOG_DATA_H

#include <string>
#include <nlohmann/json.hpp>
#include "sensors/SensorData.hpp"
#include "utils/logger/data/ILoggable.hpp"

using json = nlohmann::json;

/**
 * @brief Class to store log data.
 * 
 */
class LogData {
private:
    // Origin of the log
    std::string source;

    // Data to be logged
    const ILoggable* data;
    
public:
    /**
     * @brief Construct a new Log Data object
     * 
     * @param source Origin of the log.
     * @param data Data to be logged.
     */
    LogData(const std::string& source, const ILoggable* data) : source(source), data(data) {}

    /**
     * @brief Get the Source of the log.
     * 
     * @return A string representing the name of the source.
     */
    std::string getSource() const { return this->source; }

    /**
     * @brief Return a JSON representation of the log data.
     * 
     * @return A json object.
     */
    json toJSON() const { return this->data->toJSON(); }
};

#endif
