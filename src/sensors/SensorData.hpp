#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <string>
#include <nlohmann/json.hpp>
#include <map>
#include <variant>
#include <optional>

using json = nlohmann::json;

/**
 * @brief Class to store sensor data.
 * 
 */
class SensorData {
protected:
    // Name of the sensor
    const std::string sensorName;
    // Map to store key-value pairs where values can be of different types
    std::map<std::string, std::variant<int, unsigned int, double, std::string, std::vector<float>, std::vector<double>>> dataMap;
    

public:
    /**
     * @brief Construct a new Sensor Data object.
     * 
     * @param sensorName Name of the sensor.
     */
    SensorData(const std::string& sensorName) : sensorName(sensorName) {}
        
    /**
     * @brief Set the Data object
     * 
     * @param key The key to store the data.
     * @param value The value to store.
     */
    void setData(const std::string& key, const std::variant<int, unsigned int, double, std::string, std::vector<float>, std::vector<double>>& value) {
        dataMap[key] = value;
    }

    /**
     * @brief Function to retrieve single data, now using std::optional to handle missing keys.
     * 
     * @param key The key to retrieve the data.
     * @return The data if the key is found, otherwise std::nullopt.
     */
    std::optional<std::variant<int, unsigned int, double, std::string, std::vector<float>, std::vector<double>>> getData(const std::string& key) const {
        auto it = dataMap.find(key);
        if (it != dataMap.end()) {
            return it->second;
        }

        // Return std::nullopt if key is not found
        return std::nullopt;
    }

    /**
     * @brief Get the Data Map object.
     * 
     * @return The data map.
     */
    std::map<std::string, std::variant<int, unsigned int, double, std::string, std::vector<float>, std::vector<double>>> getDataMap() const {
        return dataMap;
    }

    /**
     * @brief Get the Sensor Name object
     * 
     * @return A string representing the name of the sensor.
     */
    std::string getSensorName() const {
        return sensorName;
    }
};

#endif // SENSOR_DATA_HPP
