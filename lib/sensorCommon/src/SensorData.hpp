#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <map>
#include <variant>
#include <optional>

using json = nlohmann::json;
using SensorDataVariant = std::variant<
    uint8_t,
    int,
    unsigned int,
    float,
    double,
    std::string,
    std::vector<float>,
    std::vector<double>,
    std::map<std::string, double>,
    std::map<std::string, float>   
>;

/**
 * @brief Class to store sensor data.
 *
 */
class SensorData
{
protected:
    // Name of the sensor
    const std::string sensorName;
    // Map to store key-value pairs where values can be of different types
    std::map<std::string, SensorDataVariant> dataMap;

public:
    /**
     * @brief Construct a new Sensor Data object.
     *
     * @param sensorName Name of the sensor.
     */
    SensorData(const std::string &sensorName) : sensorName(sensorName) {}

    /**
     * @brief Copy assignment operator
     *
     * @param other The SensorData object to copy from.
     * @return Reference to this object.
     */
    SensorData& operator=(const SensorData& other)
    {
        if (this != &other)
        {
            dataMap = other.dataMap;
        }
        return *this;
    }
    /**
     * @brief Set the Data object
     *
     * @param key The key to store the data.
     * @param value The value to store.
     */
    void setData(const std::string &key, const SensorDataVariant &value)
    {
        dataMap[key] = value;
    }

    /**
     * @brief Function to retrieve single data, now using std::optional to handle missing keys.
     *
     * @param key The key to retrieve the data.
     * @return The data if the key is found, otherwise std::nullopt.
     */
    std::optional<SensorDataVariant> getData(const std::string &key) const
    {
        auto it = dataMap.find(key);
        if (it != dataMap.end())
        {
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
    std::map<std::string, SensorDataVariant> getDataMap() const
    {
        return dataMap;
    }

    /**
     * @brief Get the Sensor Name object
     *
     * @return A string representing the name of the sensor.
     */
    std::string getSensorName() const
    {
        return sensorName;
    }
};


