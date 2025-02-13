#pragma once
#include "telemetry/ITransmitter.hpp"
#include "global/config.h"
#include <LoRa_E220.h>
#include <HardwareSerial.h>

/**
 * @brief Class for the Ebyte E220-900T22D LoRa module transmitter
 *
 */
class E220LoRaTransmitter : public ITransmitter
{
public:
    /**
     * @brief Construct a new E220LoRaTransmitter object
     *
     * @param serial The serial port for the LoRa module
     * @param auxPin AUX pin
     * @param m0Pin  M0 pin
     * @param m1Pin  M1 pin
     */
    E220LoRaTransmitter(HardwareSerial &serial, byte auxPin, byte m0Pin, byte m1Pin) : transmitter(&serial, auxPin, m0Pin, m1Pin, getBpsRate()) {};

    /**
     * @brief Construct a new E220LoRaTransmitter object
     *
     * @param serial The serial port for the LoRa module
     * @param auxPin AUX pin
     */
    E220LoRaTransmitter(HardwareSerial &serial, byte auxPin) : transmitter(&serial, auxPin, -1, -1, getBpsRate()) {};

    /**
     * @brief Construct a new E220LoRaTransmitter object
     *
     * @param serial The serial port for the LoRa module
     */
    E220LoRaTransmitter(HardwareSerial &serial) : transmitter(&serial, -1, -1, -1, getBpsRate()) {};

    /**
     * @brief Initialize the LoRa module with default configuration.
     *
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer init() override;

    /**
     * @brief Init the LoRa module with a given configuration
     *
     * @param config The configuration to use (see LoRa_E220.h)
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer init(Configuration config);

    /**
     * @brief Transmit data over LoRa
     *
     * @param data The data to transmit
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer transmit(std::variant<char *, String, std::string, nlohmann::json> data) override;

    /**
     * @brief Set a new configuration for the LoRa module
     *
     * @param configuration The configuration to use
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer configure(Configuration configuration);

    /**
     * @brief Get the configuration object
     *
     * @return ResponseStructContainer
     */
    ResponseStructContainer getConfiguration();

    /**
     * @brief Get the configuration string (an utility function)
     *
     * @param configuration The configuration to convert to a string
     * @return String
     */
    String getConfigurationString(Configuration configuration) const;

private:
    LoRa_E220 transmitter;
    //* NOTE: These methods might be moved to a utility class if they are needed elsewhere. 

    /**
     * @brief Maps the SERIAL_BAUD_RATE (defined in global/config.h) to a UART_BPS_RATE enum element from the E220_LoRa library.
     * 
     * @return UART_BPS_RATE 
     */
    UART_BPS_RATE getBpsRate() const;
    /**
     * @brief Maps the SERIAL_BAUD_RATE (defined in global/config.h) to a UART_BPS_TYPE enum element from the E220_LoRa library.
     * 
     * @return UART_BPS_TYPE
     */
    UART_BPS_TYPE getBpsType() const;

    /**
     * @brief Get the Bps Value object
     * 
     * @tparam T The type of the baud rate map (UART_BPS_RATE or UART_BPS_TYPE)
     * @param baudRateMap The baud rate map
     * @param defaultValue The default value
     * @return T 
     */
    template <typename T>
    T getBpsValue(const std::map<int, T>& baudRateMap, T defaultValue) const;
};