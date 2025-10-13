#ifndef PRINT_UTILS_HPP
#define PRINT_UTILS_HPP

#include <Arduino.h>

#define DECIMAL_PLACES 4
/**
 * @class PrintUtils
 * @brief A utility class for formatting printed output to the Serial monitor.
 * 
 * @author eric.aquilotti@aurorarocketry.eu
 */
class PrintUtils {
public:
    /**
     * @brief Prints a formatted header with hyphen lines above and below the
     * title.
     * @param title The title to be printed as the header.
     */
    static void printHeader(const char* title);

private:
    /**
     * @brief Prints a line of hyphens for separation.
     */
    static  void hyphenLine();
};

#endif
