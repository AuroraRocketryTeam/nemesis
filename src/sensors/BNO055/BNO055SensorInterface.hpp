#ifndef BNO055_SENSOR_INTERFACE_HPP
#define BNO055_SENSOR_INTERFACE_HPP

extern "C" {
  #include "bno055.h"
}
#include "sensors/ISensor.hpp"

// !!! Errors given by the BNO APIs should be managed, maybe with a Publisher-Subscriber pattern?

/**
 * @brief Interface for BNO055 sensor operations, using low-level APIs from the BNO055_SensorAPI official library, instead of Adafruit_BNO055.
 */
class BNO055SensorInterface
{
private:
    bno055_t bno;

public:    
    /**
     * @brief Default constructor for BNO055SensorInterface, the actual sensor structure will be initialized in init()
     */
    BNO055SensorInterface();
    
    /**
     * @brief Initialize the BNO055 sensor
     * @return true if initialization successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Check if all sensor components are calibrated
     * @return the minimum calibration value among all sensors (accel, mag, gyro, system), from 0 (not calibrated) to 3 (fully calibrated)
     */
    uint8_t check_calibration();
    
    // ========== INDIVIDUAL CALIBRATION FUNCTIONS ==========
    /**
     * @brief Check accelerometer calibration status
     * @return the calibration value of the accelerometer, from 0 (not calibrated) to 3 (fully calibrated)
     */
    uint8_t check_calibration_accel();
    
    /**
     * @brief Check magnetometer calibration status
     * @return the calibration value of the magnetometer, from 0 (not calibrated) to 3 (fully calibrated)
     * @note Magnetometer calibration typically requires figure-8 movements !!! Look for how to start the actual calibration
     */
    uint8_t check_calibration_mag();
    
    /**
     * @brief Check gyroscope calibration status
     * @return the calibration value of the gyroscope, from 0 (not calibrated) to 3 (fully calibrated)
     * @note Gyroscope calibration requires the sensor to remain stationary
     */
    uint8_t check_calibration_gyro();
    
    /**
     * @brief Check system-wide calibration status
     * @return the calibration value of the system, from 0 (not calibrated) to 3 (fully calibrated)
     */
    uint8_t check_calibration_sys();
    
    // ========== SELF-TEST FUNCTIONS ==========
    /**
     * @brief Perform accelerometer hardware self-test
     * @return true if accelerometer hardware test passes (result = 0x01), false if fails (result = 0x00)
     */
    bool selftest_accel();
    
    /**
     * @brief Perform magnetometer hardware self-test
     * @return true if magnetometer hardware test passes (result = 0x01), false if fails (result = 0x00)
     */
    bool selftest_mag();
    
    /**
     * @brief Perform gyroscope hardware self-test
     * @return true if gyroscope hardware test passes (result = 0x01), false if fails (result = 0x00)
     */
    bool selftest_gyro();
    
    /**
     * @brief Perform microcontroller unit hardware self-test
     * @return true if MCU hardware test passes (result = 0x01), false if fails (result = 0x00)
     */
    /**
     * @brief Perform microcontroller unit hardware self-test
     * @return true if MCU hardware test passes (result = 0x01), false if fails (result = 0x00)
     */
    bool selftest_mcu();
    
    // ========== SYSTEM STATUS AND ERROR CHECKING ==========
    /**
     * @brief Check if the system is running properly
     * @return true if system is in operational state (status 5 or 6), false otherwise
     * @details System status codes:
     *          - 0: System idle, no operation being performed
     *          - 1: System error
     *          - 2: Initializing peripherals
     *          - 3: System initialization
     *          - 4: Executing self-test
     *          - 5: Sensor fusion algorithm running (GOOD)
     *          - 6: System running without fusion (GOOD)
     */
    bool check_system_status();
    
    /**
     * @brief Check if there are any system errors
     * @return true if no system errors detected (error code = 0), false if errors present
     */
    bool check_system_error();
    
    /**
     * @brief Check if the main clock is running properly
     * @return true if main clock is operational (status = 1), false otherwise
     */
    bool check_clock_status();
    
    /**
     * @brief Get the specific system error code for diagnosis
     * @return System error code (0 = no error, >0 = specific error condition)
     */
    uint8_t get_system_error_code();
    
    /**
     * @brief Get the specific system status code for diagnosis
     * @return System status code (see check_system_status() for code meanings)
     */
    /**
     * @brief Get the specific system status code for diagnosis
     * @return System status code (see check_system_status() for code meanings)
     */
    uint8_t get_system_status_code();
    
    // ========== OPERATION AND POWER MODE MANAGEMENT ==========
    /**
     * @brief Set the sensor operation mode
     * @param mode Operation mode to set
     * @return true if mode set successfully, false otherwise
     * @details Available operation modes:
     *          - BNO055_OPERATION_MODE_CONFIG (0x00): Configuration mode
     *          - BNO055_OPERATION_MODE_ACCONLY (0x01): Accelerometer only
     *          - BNO055_OPERATION_MODE_MAGONLY (0x02): Magnetometer only
     *          - BNO055_OPERATION_MODE_GYRONLY (0x03): Gyroscope only
     *          - BNO055_OPERATION_MODE_ACCMAG (0x04): Accelerometer + Magnetometer
     *          - BNO055_OPERATION_MODE_ACCGYRO (0x05): Accelerometer + Gyroscope
     *          - BNO055_OPERATION_MODE_MAGGYRO (0x06): Magnetometer + Gyroscope
     *          - BNO055_OPERATION_MODE_AMG (0x07): All sensors, no fusion
     *          - BNO055_OPERATION_MODE_IMUPLUS (0x08): IMU fusion (no magnetometer)
     *          - BNO055_OPERATION_MODE_COMPASS (0x09): Compass mode
     *          - BNO055_OPERATION_MODE_M4G (0x0A): M4G fusion mode
     *          - BNO055_OPERATION_MODE_NDOF_FMC_OFF (0x0B): 9DOF without fast mag calibration
     *          - BNO055_OPERATION_MODE_NDOF (0x0C): 9DOF fusion (recommended for rockets)
     */
    bool set_operation_mode(uint8_t mode);
    
    /**
     * @brief Get the current sensor operation mode
     * @param mode Pointer to store the current operation mode
     * @return true if mode retrieved successfully, false otherwise
     */
    bool get_operation_mode(uint8_t* mode);
    
    /**
     * @brief Set the sensor power mode
     * @param mode Power mode to set
     * @return true if power mode set successfully, false otherwise
     * @details Available power modes:
     *          - BNO055_POWER_MODE_NORMAL (0x00): Normal operation
     *          - BNO055_POWER_MODE_LOWPOWER (0x01): Low power mode
     *          - BNO055_POWER_MODE_SUSPEND (0x02): Suspend mode
     */
    bool set_power_mode(uint8_t mode);
    
    /**
     * @brief Get the current sensor power mode
     * @param mode Pointer to store the current power mode
     * @return true if power mode retrieved successfully, false otherwise
     */
    bool get_power_mode(uint8_t* mode);
    
    /**
     * @brief Set the accelerometer power mode
     * @param mode Accelerometer power mode (0x00-0x05)
     * @return true if power mode set successfully, false otherwise
     */
    bool set_accel_power_mode(uint8_t mode);
    
    /**
     * @brief Set the magnetometer power mode
     * @param mode Magnetometer power mode (0x00-0x03)
     * @return true if power mode set successfully, false otherwise
     */
    bool set_mag_power_mode(uint8_t mode);
    
    /**
     * @brief Set the gyroscope power mode
     * @param mode Gyroscope power mode (0x00-0x04)
     * @return true if power mode set successfully, false otherwise
     */
    /**
     * @brief Set the gyroscope power mode
     * @param mode Gyroscope power mode (0x00-0x04)
     * @return true if power mode set successfully, false otherwise
     */
    bool set_gyro_power_mode(uint8_t mode);

    // ========== SENSOR DATA READING FUNCTIONS ==========
    
    /**
     * @brief Get the current accelerometer data
     * @return Accelerometer data structure containing x, y, z acceleration values
     */
    std::vector<float> get_accel();

    /**
     * @brief Get the current magnetometer data
     * @return Magnetometer data structure containing x, y, z magnetic field values
     */
    std::vector<float> get_mag();

    /**
     * @brief Get the current gyroscope data in degrees per second
     * @return Gyroscope data structure containing x, y, z angular velocity values in degrees per second
     */
    std::vector<float> get_gyro_dps();

    /**
     * @brief Get the current gyroscope data in radians per second
     * @return Gyroscope data structure containing x, y, z angular velocity values in radians per second
     */
    std::vector<float> get_gyro_rps();

    /**
     * @brief Get the current Euler angles in degrees
     * @return Euler angles structure containing heading, roll, pitch in degrees
     */
    std::vector<float> get_euler_deg();

    /**
     * @brief Get the current Euler angles in radians
     * @return Euler angles structure containing heading, roll, pitch in radians
     */
    std::vector<float> get_euler_rad();

    /**
     * @brief Get the current quaternion data
     * @return Quaternion data structure containing x, y, z, w values
     */
    std::vector<float> get_quaternion();

    /**
     * @brief Get the current linear acceleration data
     * @return Linear acceleration data structure containing x, y, z acceleration values
     */
    std::vector<float> get_linear_accel();

    /**
     * @brief Get the current gravity vector data
     * @return Gravity vector data structure containing x, y, z acceleration values
     */
    std::vector<float> get_gravity();

    /**
     * @brief Get the current temperature data
     * @return Temperature in degrees Celsius
     */
    float get_temperature();
};
#endif
