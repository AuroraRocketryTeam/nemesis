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
                        logger(logger), serial(serial), auxPin(auxPin), m0Pin(m0Pin), m1Pin(m1Pin) {};

    E220LoRaTransmitter(ILogger *logger, HardwareSerial &serial) {
        E220LoRaTransmitter(logger, serial, -1, -1, -1);
    };

    bool init() override;
    bool init(unsigned long serialBaudRate);
    void transmit(std::variant<char *, String, std::string, nlohmann::json> data) override;
    void configure(Configuration configuration);

private:
    ILogger *logger;
    byte auxPin;
    byte m0Pin;
    byte m1Pin;
    LoRa_E220 transmitter;
    HardwareSerial serial;
}