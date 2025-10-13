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
template <typename T>
class ITransmitter
{
    public:
        virtual ResponseStatusContainer init() = 0;
        virtual ResponseStatusContainer transmit(T data) = 0;
};

