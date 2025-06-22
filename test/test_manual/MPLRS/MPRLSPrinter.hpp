#ifndef MPRLS_SENSOR_PRINTER_HPP
#define MPRLS_SENSOR_PRINTER_HPP

#include <Arduino.h>
#include "sensors/ISensor.hpp"

/**
 * @class MPRLSPrinter
 * @brief Utility class to print formatted sensor data from an Adafruit MPRLS IMU.
 *
 * This class provides methods for printing various sensor readings and metadata
 * from a MPRLS inertial measurement unit using the Adafruit_MPRLS library.
 *
 * The class does not own the sensor instance; it keeps a reference to an
 * existing MPRLSSensor object, allowing the caller to manage sensor
 * initialization and configuration.
 * 
 * @author davide.rossi@aurorarocketry.eu
 */
class MPRLSPrinter {
    public:
        MPRLSPrinter(ISensor* sensor);
        void displayPressure();
    private:
        ISensor* mprls;
};

#endif