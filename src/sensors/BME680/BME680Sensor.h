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
    bool readData_async();
private:
    BME680SensorData data;
};