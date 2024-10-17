// LoggerInterface.h
#ifndef LOG_SENSOR_DATA_H
#define LOG_SENSOR_DATA_H

#include <string>
#include <nlohmann/json.hpp>
#include "sensors/SensorData.hpp"
#include "utils/logger/data/ILoggable.hpp"

class LogSensorData : public ILoggable {
private:
    std::string source;
    SensorData sensorData;
public:
    LogSensorData(std::string source, SensorData sensorData) : source(source), sensorData(sensorData) {}
    ~LogSensorData() = default;

    // Virtual function to get the log source
    std::string getSource() const {
        return source;
    }

    // Virtual function to get the sensor data
    SensorData getSensorData() const {
        return sensorData;
    }

    // Function to convert the object to JSON
    json toJSON() const override {
        json j;

        // Add source to JSON
        j["source"] = source;

        // Add sensor data to JSON
        json sensorDataJson;
        auto dataMap = sensorData.getDataMap();

        for (const auto& [key, value] : dataMap) {
            // Use std::visit to handle each variant type
            std::visit([&sensorDataJson, &key](const auto& arg) {
                sensorDataJson[key] = arg;
            }, value);
        }
        j["sensorData"] = sensorDataJson;

        return j;
    }
};

#endif