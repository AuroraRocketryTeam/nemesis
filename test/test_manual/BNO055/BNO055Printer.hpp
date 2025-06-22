#ifndef BNO055_SENSOR_PRINTER_HPP
#define BNO055_SENSOR_PRINTER_HPP

#include <Arduino.h>
#include <ISensor.hpp>

/**
 * @class BNO055Printer
 * @brief Utility class to print formatted sensor data from an Adafruit BNO055 IMU.
 *
 * This class provides methods for printing various sensor readings and metadata
 * from a BNO055 inertial measurement unit using the Adafruit_BNO055 library.
 *
 * The class does not own the sensor instance; it keeps a reference to an
 * existing BNO055Sensor object, allowing the caller to manage sensor
 * initialization and configuration.
 * 
 * @author eric.aquilotti@aurorarocketry.eu
 */
class BNO055Printer {
    public:
        BNO055Printer(ISensor* sensor);
        void displayCalibrationStatus();
        void displayOrientation();
        void displayAccelleration();
        void displayGyroscope();
        void displayMagnetometer();
        void displayLinearAccelleration();
        void displayGravity();

    private:
        ISensor* bno055;
        void printXYZMap(const char* key);
};

#endif
