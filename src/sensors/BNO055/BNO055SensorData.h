#ifndef BNO055_SENSOR_DATA_H
#define BNO055_SENSOR_DATA_H
#include "sensors/SensorData.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class BNO055SensorData : public SensorData
{
public:
    BNO055SensorData() : board_temperature(-39), system_calibration(0), gyro_calibration(0), accel_calibration(0), mag_calibration(0) {}

    /**
     * @brief Construct a new BNO055SensorData object
     *
     * @param _orientation The orientation on x,y,z axes in degrees
     * @param _angularVelocity The angular velocity on x,y,z axes in rad/s
     * @param _linearAcceleration The linear acceleration on x,y,z axes in m/s^2
     * @param _magnetometer The magnetometer on x,y,z axes in uT
     * @param _accelerometer The accelerometer on x,y,z axes in m/s^2
     * @param _gravity The gravity on x,y,z axes in m/s^2
     * @param _board_temperature The board temperature in °C
     * @param _system_calibration The system calibration status
     * @param _gyro_calibration The gyroscope calibration status
     * @param _accel_calibration The accelerometer calibration status
     * @param _mag_calibration The magnetometer calibration status
     */
    BNO055SensorData(sensors_vec_t _orientation, sensors_vec_t _angularVelocity, sensors_vec_t _linearAcceleration,
                     sensors_vec_t _magnetometer, sensors_vec_t _accelerometer, sensors_vec_t _gravity,
                     int8_t _board_temperature,
                     uint8_t _system_calibration, uint8_t _gyro_calibration, uint8_t _accel_calibration, uint8_t _mag_calibration) : orientation(_orientation), angularVelocity(_angularVelocity), linearAcceleration(_linearAcceleration),
                                                                                                                                     magnetometer(_magnetometer), accelerometer(_accelerometer), gravity(_gravity), board_temperature(_board_temperature),
                                                                                                                                     system_calibration(_system_calibration), gyro_calibration(_gyro_calibration), accel_calibration(_accel_calibration),
                                                                                                                                     mag_calibration(_mag_calibration) {}

    /**
     * @brief Get the Orientation on x,y,z axes in degrees
     *
     * @return sensors_vec_t orientation
     */
    sensors_vec_t getOrientation() const { return this->orientation; }
    /**
     * @brief Get the Angular Velocity on x,y,z axes in rad/s
     *
     * @return sensors_vec_t angularVelocity
     */
    sensors_vec_t getAngularVelocity() const { return this->angularVelocity; }
    /**
     * @brief Get the Linear Acceleration on x,y,z axes in m/s^2
     *
     * @return sensors_vec_t linearAcceleration
     */
    sensors_vec_t getLinearAcceleration() const { return this->linearAcceleration; }
    /**
     * @brief Get the Magnetometer on x,y,z axes in uT
     *
     * @return sensors_vec_t magnetometer
     */
    sensors_vec_t getMagnetometer() const { return this->magnetometer; }
    /**
     * @brief Get the Accelerometer on x,y,z axes in m/s^2
     *
     * @return sensors_vec_t accelerometer
     */
    sensors_vec_t getAccelerometer() const { return this->accelerometer; }
    /**
     * @brief Get the Gravity on x,y,z axes in m/s^2
     *
     * @return sensors_vec_t gravity
     */
    sensors_vec_t getGravity() const { return this->gravity; }

    /**
     * @brief Get the Board Temperature
     * @note The temperature is in °C
     * @note The working temperature of the sensor is between -40° and 85°C
     * @return int8_t
     */
    int8_t getBoardTemperature() const { return this->board_temperature; }

    /**
     * @brief Get the System Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t
     */
    uint8_t getSystemCalibration() const { return this->system_calibration; }
    /**
     * @brief Get the Gyroscope Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t
     */
    uint8_t getGyroCalibration() const { return this->gyro_calibration; }
    /**
     * @brief Get the Accelerometer Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t
     */
    uint8_t getAccelCalibration() const { return this->accel_calibration; }
    /**
     * @brief Get the Magnetometer Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t
     */
    uint8_t getMagCalibration() const { return this->mag_calibration; }

    String toString() override;

private:
    /* Sensor data */
    sensors_vec_t orientation;
    sensors_vec_t angularVelocity;
    sensors_vec_t linearAcceleration;
    sensors_vec_t magnetometer;
    sensors_vec_t accelerometer;
    sensors_vec_t gravity;

    /* Board info */
    int8_t board_temperature;
    uint8_t system_calibration;
    uint8_t gyro_calibration;
    uint8_t accel_calibration;
    uint8_t mag_calibration;
};
#endif // BNO055_SENSOR_DATA_H