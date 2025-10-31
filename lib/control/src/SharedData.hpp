#pragma once

#include <SensorData.hpp>
#include <freertos/FreeRTOS.h>

/**
 * @brief Structure to hold shared sensor data between tasks.
 * 
 */
struct SharedSensorData
{
    class SensorData imuData;
    class SensorData baroData1;
    class SensorData baroData2;
    class SensorData gpsData;
    uint32_t timestamp;
    bool dataValid;

    SharedSensorData() : imuData("bno055"), baroData1("baro1"), baroData2("baro2"), gpsData("gps"), timestamp(0), dataValid(false) {}
};

/**
 * @brief Structure to hold filtered sensor data between tasks.
 * 
 */
struct SharedFilteredData
{
    float altitude;
    float verticalVelocity;
    float orientation[4]; // Quaternion [w, x, y, z]
    uint32_t timestamp;
    bool dataValid;
};