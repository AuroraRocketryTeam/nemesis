#pragma once
#include "telemetry/ITransmitter.hpp"
#include "utils/logger/ILogger.hpp"
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
     * @param logger The logger
     * @param serial The serial port for the LoRa module
     * @param auxPin AUX pin
     * @param m0Pin  M0 pin
     * @param m1Pin  M1 pin
     */
    E220LoRaTransmitter(ILogger *logger, HardwareSerial &serial, byte auxPin, byte m0Pin, byte m1Pin) : 
                        logger(logger), serial(serial), auxPin(auxPin), m0Pin(m0Pin), m1Pin(m1Pin), transmitter(&serial, auxPin, m0Pin, m1Pin) {};

    E220LoRaTransmitter(ILogger *logger, HardwareSerial &serial) : 
                        logger(logger), serial(serial), auxPin(-1), m0Pin(-1), m1Pin(-1), transmitter(&serial, -1, -1, -1) {};

    bool init() override;
    bool init(unsigned long serialBaudRate, Configuration config);
    void transmit(std::variant<char *, String, std::string, nlohmann::json> data) override;
    bool configure(Configuration configuration);
    ResponseStructContainer getConfiguration();
    String getConfigurationString(Configuration configuration) const;
private:
    ILogger *logger;
    HardwareSerial serial;
    byte auxPin;
    byte m0Pin;
    byte m1Pin;
    LoRa_E220 transmitter;
};