#pragma once

#include <nlohmann/json.hpp>
#include <LoRa_E220.h>
#include "utils/logger/ILogger.hpp"
#include <FS.h> // Include the appropriate header for the File class

/**
 * @brief Class to deserialize LoRa configuration from JSON object or file
 *
 */
class LoRaConfigurationDeserializer
{
public:
    LoRaConfigurationDeserializer(Configuration configuration, ILogger *logger) : configuration(configuration), logger(logger) {};
    LoRaConfigurationDeserializer(ILogger *logger) : logger(logger)
    {
        // Set default configuration
        configuration.ADDL = 0x02;
        configuration.ADDH = 0x00;

        configuration.CHAN = 23;

        configuration.SPED.uartBaudRate = UART_BPS_9600;
        configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
        configuration.SPED.uartParity = MODE_00_8N1;

        configuration.OPTION.subPacketSetting = SPS_200_00;
        configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
        configuration.OPTION.transmissionPower = POWER_22;

        configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
        configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
        configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
        configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
    };

    /**
     * @brief Check if JSON configuration is invalid. 
     *        Required parameters are ADDH, ADDL, CHAN.
     *
     * @param json A JSON object
     * @return true if JSON object is valid, false if JSON object is invalid
     */
    bool isJsonValid(const nlohmann::json &json) const
    {
        return !(json.empty() || json.is_null() || !json.is_object() || json.size() == 0 || 
                json["ADDH"].is_null() || json["ADDL"].is_null() || json["CHAN"].is_null() ||);
    }

    /**
     * @brief Deserialize configuration from JSON object
     *
     * @param json A JSON object containing configuration for Ebyte E220-900T22D LoRa module.
     * @return true if configuration is deserialized successfully, false otherwise
     */
    bool deserializeConfiguration(const nlohmann::json &json)
    {
        if (!isJsonValid(json))
        {
            logger->logError("[E220 Module] Failed to deserialize configuration from JSON object. JSON object is empty or invalid.");
            return false;
        }

        configuration.COMMAND = json["COMMAND"];
        configuration.STARTING_ADDRESS = json["STARTING_ADDRESS"];
        configuration.LENGHT = json["LENGHT"];
        configuration.ADDH = json["ADDH"];
        configuration.ADDL = json["ADDL"];
        configuration.CHAN = json["CHAN"];

        configuration.SPED.airDataRate = json["SPED"]["airDataRate"];
        configuration.SPED.uartParity = json["SPED"]["uartParity"];
        configuration.SPED.uartBaudRate = json["SPED"]["uartBaudRate"];

        configuration.OPTION.transmissionPower = json["OPTION"]["transmissionPower"];
        configuration.OPTION.reserved = json["OPTION"]["reserved"];
        configuration.OPTION.RSSIAmbientNoise = json["OPTION"]["RSSIAmbientNoise"];
        configuration.OPTION.subPacketSetting = json["OPTION"]["subPacketSetting"];

        configuration.TRANSMISSION_MODE.WORPeriod = json["TRANSMISSION_MODE"]["WORPeriod"];
        configuration.TRANSMISSION_MODE.reserved2 = json["TRANSMISSION_MODE"]["reserved2"];
        configuration.TRANSMISSION_MODE.enableLBT = json["TRANSMISSION_MODE"]["enableLBT"];
        configuration.TRANSMISSION_MODE.reserved = json["TRANSMISSION_MODE"]["reserved"];
        configuration.TRANSMISSION_MODE.fixedTransmission = json["TRANSMISSION_MODE"]["fixedTransmission"];
        configuration.TRANSMISSION_MODE.enableRSSI = json["TRANSMISSION_MODE"]["enableRSSI"];

        configuration.CRYPT.CRYPT_H = json["CRYPT"]["CRYPT_H"];
        configuration.CRYPT.CRYPT_L = json["CRYPT"]["CRYPT_L"];

        return true;
    }
    /**
     * @brief Deserialize configuration from JSON file
     *
     * @param file A file containing JSON configuration for Ebyte E220-900T22D LoRa module.
     * @return true if configuration is deserialized successfully, false otherwise
     */
    bool deserializeConfiguration(File &file)
    {
        if (!file)
        {
            return false;
        }

        size_t size = file.size();
        if (size == 0)
        {
            String error = "[E220 Module] JSON file is empty: " + String(file.name());
            logger->logError(error.c_str());
            return false;
        }

        // Read the file content into a string
        std::ostringstream contentStream;
        contentStream << file.readString().c_str();
        std::string content = contentStream.str();

        nlohmann::json json;
        try
        {
            json = nlohmann::json::parse(content);
        }
        catch (nlohmann::json::parse_error &e)
        {
            String error = "[E220 Module] Failed to parse JSON file: " + String(file.name()) + ". Error: " + e.what();
            logger->logError(error.c_str());
            return false;
        }
        file.close();
        return deserializeConfiguration(json);
    }

    /**
     * @brief Get the Configuration object
     *
     * @return Configuration
     */
    Configuration getConfiguration() const
    {
        return configuration;
    }

private:
    Configuration configuration;
    ILogger *logger;
};

