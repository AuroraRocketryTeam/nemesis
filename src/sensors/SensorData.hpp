#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <string>
#include <nlohmann/json.hpp>
#include <map>
#include <variant>
#include <optional>

using json = nlohmann::json;

class SensorData {
protected:
    const std::string sensorName;
    // Map to store key-value pairs where values can be of different types
    std::map<std::string, std::variant<int, unsigned int, double, std::string>> dataMap;

public:
    // Constructor
    SensorData(const std::string& sensorName) : sensorName(sensorName) {}  // Use this->sensorName to avoid shadowing
        
    // Function to set data with different types
    void setData(const std::string& key, const std::variant<int, unsigned int, double, std::string>& value) {
        dataMap[key] = value;
    }

    // Function to retrieve single data, now using std::optional to handle missing keys
    std::optional<std::variant<int, unsigned int, double, std::string>> getData(const std::string& key) const {
        auto it = dataMap.find(key);
        if (it != dataMap.end()) {
            return it->second;  // Return value if key is found
        }

        // Return std::nullopt if key is not found
        return std::nullopt;
    }

    // Function to get all data
    std::map<std::string, std::variant<int, unsigned int, double, std::string>> getDataMap() const {
        return dataMap;
    }

    // Function that return a string representation of the sensor
    std::string getSensorName() const {
        return sensorName;
    }
};

#endif
