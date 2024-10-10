#ifndef CBOR_LOGGER_H
#define CBOR_LOGGER_H

#include <Arduino.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "Logger.h" // Ensure this path is correct and Logger.h is in the include path

using json = nlohmann::json;
class CborLogger : public Logger<nlohmann::json>
{
public:
    /**
     * @brief Log the data.
     *
     */
    void log() override
    {
        Serial.println(data.dump().c_str());
    };

    /**
     * @brief Adds an integer to the logger.
     *
     * @param key
     * @param value
     */
    void addEntry(const std::string &key, int value)
    {
        data[key] = value;
    }

    /**
     * @brief Adds a float to the logger.
     *
     * @param key
     * @param value
     */
    void addEntry(const std::string &key, float value)
    {
        data[key] = value;
    }

    /**
     * @brief Adds a double to the logger.
     *
     * @param key
     * @param value
     */
    void addEntry(const std::string &key, double value)
    {
        data[key] = value;
    }

    /**
     * @brief Adds a string to the logger.
     *
     * @param key
     * @param value
     */
    void addEntry(const std::string &key, const std::string &value)
    {
        data[key] = value;
    }

    /**
     * @brief Adds a vector of integers to the logger.
     *
     * @param key
     * @param values
     */
    void addEntry(const std::string &key, const std::vector<int> &values)
    {
        data[key] = values;
    }

    /**
     * @brief Adds a vector of floats to the logger.
     *
     * @param key
     * @param values
     */
    void addEntry(const std::string &key, const std::vector<float> &values)
    {
        data[key] = values;
    }

    /**
     * @brief Adds a vector of doubles to the logger.
     *
     * @param key
     * @param values
     */
    void addEntry(const std::string &key, const std::vector<double> &values)
    {
        data[key] = values;
    }

    /**
     * @brief Adds a vector of strings to the logger.
     *
     * @param key
     * @param values
     */
    void addEntry(const std::string &key, const std::vector<std::string> &values)
    {
        data[key] = values;
    }

    /**
     * @brief Adds a JSON object to the logger.
     *
     * @param key
     * @param value
     */
    void addEntry(const std::string &key, const json &value)
    {
        data[key] = value;
    }

    /**
     * @brief Convert the JSON object to CBOR format.
     *
     * @return std::vector<uint8_t>
     */
    std::vector<uint8_t> to_cbor() const
    {
        // Converte l'oggetto JSON in un vettore di byte (CBOR)
        return json::to_cbor(data);
    }

    /**
     * @brief Prints the CBOR data in hexadecimal format.
     *
     */
    void print_cbor_hex() const
    {
        std::vector<uint8_t> cbor_data = to_cbor();
        Serial.print("CBOR Hex: ");
        for (const auto &b : cbor_data)
        {
            Serial.printf("%02X ", b);
        }
        Serial.println();
    }

    /* Pulisce tutti i dati accumulati */
    void clear()
    {
        data.clear();
    }
};

#endif // CBOR_LOGGER_H
