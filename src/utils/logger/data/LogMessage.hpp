#ifndef LOG_MESSAGE_H
#define LOG_MESSAGE_H

#include <string>
#include <nlohmann/json.hpp>
#include "sensors/SensorData.hpp"
#include "utils/logger/data/ILoggable.hpp"

using json = nlohmann::json;

class LogMessage : public ILoggable {
private:
    std::string source;
    std::string message;

public:
    LogMessage(std::string source, std::string message) 
        : source(source), message(message) {}

    std::string getSource() const {
        return source;
    }

    std::string getMessage() const {
        return message;
    }

    json toJSON() const override {
        json j;
        j["source"] = source;
        j["message"] = message;
        return j;
    }
};

#endif
