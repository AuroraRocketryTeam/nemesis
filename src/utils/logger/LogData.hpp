#ifndef LOG_DATA_H
#define LOG_DATA_H

#include <string>
#include <nlohmann/json.hpp>
#include "sensors/SensorData.hpp"
#include "utils/logger/data/ILoggable.hpp"

using json = nlohmann::json;

class LogData {
private:
    std::string source;
    const ILoggable* data;
    
public:
    LogData(const std::string& source, const ILoggable* data) : source(source), data(data) {}

    // Getter for source
    std::string getSource() const { return this->source; }

    // Getter for JSON data
    json toJSON() const { return this->data->toJSON(); }
};

#endif
