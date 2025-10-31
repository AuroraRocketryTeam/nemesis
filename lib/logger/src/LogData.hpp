#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <SensorData.hpp>
#include <ILoggable.hpp>

/**
 * @brief Class to store log data.
 * 
 */
class LogData {
private:
    // Origin of the log
    std::string source;

    // Data to be logged
    std::shared_ptr<ILoggable> data;
    
public:
    /**
     * @brief Construct a new Log Data object
     * 
     * @param source Origin of the log (e.g. "RocketLogger", "SensorData").
     * @param data Data to be logged.
     */
    LogData(const std::string& source, std::shared_ptr<ILoggable> data) : source(source), data(data) {}

    /**
     * @brief Get the Source of the log.
     * 
     * @return A string representing the name of the source.
     */
    std::string getSource() const { return this->source; }

    /**
     * @brief Get the Data pointer.
     * 
     * @return A pointer to the ILoggable data.
     */
    const std::shared_ptr<ILoggable> getData() const { return this->data; }

    /**
     * @brief Return a JSON representation of the log data.
     * 
     * @return A json object.
     */
    json toJSON() const { 
        json j;
        j["type"] = this->getSource();
        j["content"] = this->data->toJSON();
        return j;
    }
};


