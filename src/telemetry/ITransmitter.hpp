#pragma once

#include <nlohmann/json.hpp>
#include <variant>
#include <string>
#include <Arduino.h>
#include "ResponseStatusContainer.hpp"

/**
 * @brief Interface for a transmitter
 * 
 */

class ITransmitter
{
    public:
        virtual ResponseStatusContainer init() = 0;
        virtual ResponseStatusContainer transmit(std::variant<char *, String, std::string, nlohmann::json> data) = 0;
};

