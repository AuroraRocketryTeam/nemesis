#pragma once

#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Interface for objects that can be logged.
 * 
 */
class ILoggable {
protected:
    // Origin of the log
    std::string source;
public:
    /**
     * @brief Get the Source of the log.
     * 
     * @return A string representing the name of the source.
     */
    virtual std::string getSource() const { return this->source; };

    /**
     * @brief Get the JSON representation of the object.
     * 
     * @return A json object.
     */
    virtual json toJSON() const = 0;
};


