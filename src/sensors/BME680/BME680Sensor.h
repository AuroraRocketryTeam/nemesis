/**
 * @class BME680Sensor
 * @brief A sensor class for the BME680 sensor, inheriting from the Sensor class template.
 *
 * This class provides methods to initialize the sensor, read data synchronously and asynchronously,
 * and retrieve the sensor data.
 *
 * @note The BME680 sensor is used for measuring temperature, humidity, pressure, and gas.
 *
 * @file BME680Sensor.h
 *
 * @see Sensor
 * @see BME680SensorData
 *
 * @author alessandr.monticell4@studio.unibo.it
 * @author luca.pulga@studio.unibo.it
 */
#include "sensors/Sensor.h"
#include "sensors/BME680/BME680SensorData.h"
#include "const/pins.h"

class BME680Sensor : public Sensor<BME680SensorData>
{
public:
    BME680Sensor();
    ~BME680Sensor();

    bool init() override;
    bool readData() override;
    BME680SensorData getData() override;

    /**
     * @brief Start an asynchronous reading of the sensor data.
     *
     * @return true if the reading started successfully
     * @return false if the reading failed to start
     */
    bool readData_async_start();

    /**
     * @brief End an asynchronous reading of the sensor data.
     *        If no reading is in progress, the method will
     *        perform a blocking read.
     * @return true if the reading ended successfully
     * @return false if the reading failed to end
     */
    bool readData_async_end();

private:
    BME680SensorData data;  // sensor data object
    Adafruit_BME680 bme680; // BME680 sensor object
};