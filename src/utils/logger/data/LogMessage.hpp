#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <SensorData.hpp>
#include "utils/logger/data/ILoggable.hpp"

using json = nlohmann::json;

/**
 * @brief A class to represent a log message.
 * 
 */
class LogMessage : public ILoggable {
private:
    // Message to be logged
    std::string message;

public:
    /**
     * @brief Construct a new Log Message object.
     * 
     * @param source The origin of the log.
     * @param message The message to be logged.
     */
    LogMessage(std::string source, std::string message) {
        this->source = source;
        this->message = message;
    }

    /**
     * @brief Get the message string.
     * 
     * @return A string representing the message.
     */
    std::string getMessage() const {
        return message;
    }

    /**
     * @brief Get the JSON representation of the object.
     * 
     * @return A json object.
     */
    json toJSON() const override {
        json j;
        j["source"] = source;
        j["message"] = message;
        return j;
    }
};


