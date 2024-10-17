#ifndef ILOGGABLE_HPP
#define ILOGGABLE_HPP

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class ILoggable {
public:
    // Virtual function for JSON serialization
    virtual json toJSON() const = 0;
};

#endif
