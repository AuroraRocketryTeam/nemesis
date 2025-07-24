#include "BNO055SensorInterface.hpp"
#include "const/config.h"

BNO055SensorInterface::BNO055SensorInterface() {}

bool BNO055SensorInterface::init()
{
    return bno055_init(&bno) == BNO055_SUCCESS;
}

int BNO055SensorInterface::check_calibration()
{
    // Return the minimum calibration value between all the sensors
    return (std::min({check_calibration_accel(), check_calibration_mag(), check_calibration_gyro(), check_calibration_sys()}));
}

int BNO055SensorInterface::check_calibration_accel() {
    uint8_t accel_calib = 0;
    
    // Get current accelerometer calibration status from BNO055 registers
    /*bool success = */(bno055_get_accel_calib_stat(&accel_calib) == BNO055_SUCCESS);
    
    return accel_calib;
}

int BNO055SensorInterface::check_calibration_mag() {
    uint8_t mag_calib = 0;
    
    // Get current magnetometer calibration status from BNO055 registers
    /*bool success = */(bno055_get_mag_calib_stat(&mag_calib) == BNO055_SUCCESS);
    
    return mag_calib;
}

int BNO055SensorInterface::check_calibration_gyro() {
    uint8_t gyro_calib = 0;
    
    // Get current gyroscope calibration status from BNO055 registers
    /*bool success = */(bno055_get_gyro_calib_stat(&gyro_calib) == BNO055_SUCCESS);
    
    return gyro_calib;
}

int BNO055SensorInterface::check_calibration_sys() {
    uint8_t sys_calib = 0;
    
    // Get current system calibration status from BNO055 registers
    /*bool success = */(bno055_get_sys_calib_stat(&sys_calib) == BNO055_SUCCESS);
    
    return sys_calib;
}

bool BNO055SensorInterface::selftest_accel() {
    uint8_t selftest_result = 0;
    
    // Get accelerometer self-test result from register 0x36 bit 0
    bool success = (bno055_get_selftest_accel(&selftest_result) == BNO055_SUCCESS);
    
    // Return true if read was successful and test passed (0x01 = pass, 0x00 = fail)
    return (success && selftest_result == 0x01);
}

bool BNO055SensorInterface::selftest_mag() {
    uint8_t selftest_result = 0;
    
    // Get magnetometer self-test result from register 0x36 bit 1
    bool success = (bno055_get_selftest_mag(&selftest_result) == BNO055_SUCCESS);
    
    // Return true if read was successful and test passed (0x01 = pass, 0x00 = fail)
    return (success && selftest_result == 0x01);
}

bool BNO055SensorInterface::selftest_gyro() {
    uint8_t selftest_result = 0;
    
    // Get gyroscope self-test result from register 0x36 bit 2
    bool success = (bno055_get_selftest_gyro(&selftest_result) == BNO055_SUCCESS);
    
    // Return true if read was successful and test passed (0x01 = pass, 0x00 = fail)
    return (success && selftest_result == 0x01);
}

bool BNO055SensorInterface::selftest_mcu() {
    uint8_t selftest_result = 0;
    
    // Get microcontroller self-test result from register 0x36 bit 3
    bool success = (bno055_get_selftest_mcu(&selftest_result) == BNO055_SUCCESS);
    
    // Return true if read was successful and test passed (0x01 = pass, 0x00 = fail)
    return (success && selftest_result == 0x01);
}

bool BNO055SensorInterface::check_system_status() {
    uint8_t sys_stat = 0;
    
    // Get system status from register 0x39
    bool success = (bno055_get_sys_stat_code(&sys_stat) == BNO055_SUCCESS);
    
    // Return true if system is running properly (status 5 or 6)
    return (success && (sys_stat == 5 || sys_stat == 6));
}

bool BNO055SensorInterface::check_system_error() {
    uint8_t sys_error = 0;
    
    // Get system error code from register 0x3A
    bool success = (bno055_get_sys_error_code(&sys_error) == BNO055_SUCCESS);
    
    // Return true if no error (error code 0)
    return (success && sys_error == 0);
}

bool BNO055SensorInterface::check_clock_status() {
    uint8_t clk_stat = 0;
    
    // Get main clock status from register 0x38
    bool success = (bno055_get_stat_main_clk(&clk_stat) == BNO055_SUCCESS);
    
    // Return true if clock is running properly (status 1)
    return (success && clk_stat == 1);
}

uint8_t BNO055SensorInterface::get_system_error_code() {
    uint8_t sys_error = 0;
    bno055_get_sys_error_code(&sys_error);
    return sys_error;
}

uint8_t BNO055SensorInterface::get_system_status_code() {
    uint8_t sys_stat = 0;
    bno055_get_sys_stat_code(&sys_stat);
    return sys_stat;
}

bool BNO055SensorInterface::set_operation_mode(uint8_t mode) {
    return (bno055_set_operation_mode(mode) == BNO055_SUCCESS);
}

bool BNO055SensorInterface::get_operation_mode(uint8_t* mode) {
    return (bno055_get_operation_mode(mode) == BNO055_SUCCESS);
}

bool BNO055SensorInterface::set_power_mode(uint8_t mode) {
    return (bno055_set_power_mode(mode) == BNO055_SUCCESS);
}

bool BNO055SensorInterface::get_power_mode(uint8_t* mode) {
    return (bno055_get_power_mode(mode) == BNO055_SUCCESS);
}

bool BNO055SensorInterface::set_accel_power_mode(uint8_t mode) {
    return (bno055_set_accel_power_mode(mode) == BNO055_SUCCESS);
}

bool BNO055SensorInterface::set_mag_power_mode(uint8_t mode) {
    return (bno055_set_mag_power_mode(mode) == BNO055_SUCCESS);
}

bool BNO055SensorInterface::set_gyro_power_mode(uint8_t mode) {
    return (bno055_set_gyro_power_mode(mode) == BNO055_SUCCESS);
}

std::vector<float> BNO055SensorInterface::get_accel() {
    struct bno055_accel_t accel;
    
    // The function return a boolean value indicating success or failure
    /*bool res = */bno055_read_accel_xyz(&accel);
    
    // Conversion from raw to meterpersecseq
    struct bno055_accel_float_t accel_msq;
    /*bool res2 = */bno055_convert_float_accel_xyz_msq(&accel_msq);
    
    // Convert the bno055_accel_float_t to an std::vector<float>
    std::vector<float> accel_vector = {accel_msq.x, accel_msq.y, accel_msq.z};

    return accel_vector;
}

std::vector<float> BNO055SensorInterface::get_mag() {
    struct bno055_mag_t mag;

    // The function return a boolean value indicating success or failure
    /*bool res = */bno055_read_mag_xyz(&mag);

    // Conversion from raw to microtesla
    struct bno055_mag_float_t mag_mt;
    /*bool res2 = */bno055_convert_float_mag_xyz_uT(&mag_mt);

    // Convert the bno055_mag_float_t to an std::vector<float>
    std::vector<float> mag_vector = {mag_mt.x, mag_mt.y, mag_mt.z};

    return mag_vector;
}

std::vector<float> BNO055SensorInterface::get_gyro_dps() {
    struct bno055_gyro_t gyro;

    // The function return a boolean value indicating success or failure
    /*bool res = */bno055_read_gyro_xyz(&gyro);

    // Conversion from raw to degrees per second (dps)
    struct bno055_gyro_float_t gyro_dps;
    /*bool res2 = */bno055_convert_float_gyro_xyz_dps(&gyro_dps);

    // Convert the bno055_gyro_float_t to an std::vector<float>
    std::vector<float> gyro_dps_vector = {gyro_dps.x, gyro_dps.y, gyro_dps.z};

    return gyro_dps_vector;
}

std::vector<float> BNO055SensorInterface::get_gyro_rps() {
    struct bno055_gyro_t gyro;

    // The function return a boolean value indicating success or failure
    /*bool res = */bno055_read_gyro_xyz(&gyro);

    // Conversion from raw to radians per second (rps)
    struct bno055_gyro_float_t gyro_rps;
    /*bool res2 = */bno055_convert_float_gyro_xyz_rps(&gyro_rps);

    // Convert the bno055_gyro_float_t to an std::vector<float>
    std::vector<float> gyro_rps_vector = {gyro_rps.x, gyro_rps.y, gyro_rps.z};

    return gyro_rps_vector;
}

std::vector<float> BNO055SensorInterface::get_euler_deg() {
    struct bno055_euler_t euler;

    // The function return a boolean value indicating success or failure
    /*bool res = */bno055_read_euler_hrp(&euler);

    // Conversion from raw to degrees
    struct bno055_euler_float_t euler_deg;
    /*bool res2 = */bno055_convert_float_euler_hpr_deg(&euler_deg);

    // Convert the bno055_euler_float_t to an std::vector<float>
    std::vector<float> euler_deg_vector = {euler_deg.h, euler_deg.r, euler_deg.p};

    return euler_deg_vector;
}

std::vector<float> BNO055SensorInterface::get_euler_rad() {
    struct bno055_euler_t euler;
    
    // The function return a boolean value indicating success or failure
    /*bool res = */bno055_read_euler_hrp(&euler);

    // Conversion from raw to radians
    struct bno055_euler_float_t euler_rad;
    /*bool res2 = */bno055_convert_float_euler_hpr_rad(&euler_rad);

    // Convert the bno055_euler_float_t to an std::vector<float>
    std::vector<float> euler_rad_vector = {euler_rad.h, euler_rad.r, euler_rad.p};

    return euler_rad_vector;
}

std::vector<float> BNO055SensorInterface::get_quaternion() {
    struct bno055_quaternion_t quaternion;
    
    /*bool res = */bno055_read_quaternion_wxyz(&quaternion);

    // Convert the bno055_quaternion_t to an std::vector<float>
    std::vector<float> quaternion_vector = {
        static_cast<float>(quaternion.w),
        static_cast<float>(quaternion.x),
        static_cast<float>(quaternion.y),
        static_cast<float>(quaternion.z)
    };

    return quaternion_vector;
}

std::vector<float> BNO055SensorInterface::get_linear_accel() {
    struct bno055_linear_accel_t linear_accel;
    
    /*bool res = */bno055_read_linear_accel_xyz(&linear_accel);

    // Conversion from raw to meter per second squared (m/s^2)
    struct bno055_linear_accel_float_t linear_accel_msq;
    /*bool res2 = */bno055_convert_float_linear_accel_xyz_msq(&linear_accel_msq);

    // Convert the bno055_linear_accel_float_t to an std::vector<float>
    std::vector<float> linear_accel_vector = {linear_accel_msq.x, linear_accel_msq.y, linear_accel_msq.z};

    return linear_accel_vector;
}

std::vector<float> BNO055SensorInterface::get_gravity() {
    struct bno055_gravity_t gravity;
    
    /*bool res = */bno055_read_gravity_xyz(&gravity);

    // Conversion from raw to meter per second squared (m/s^2)
    struct bno055_gravity_float_t gravity_msq;
    /*bool res2 = */bno055_convert_float_gravity_xyz_msq(&gravity_msq);

    // Convert the bno055_gravity_float_t to an std::vector<float>
    std::vector<float> gravity_vector = {gravity_msq.x, gravity_msq.y, gravity_msq.z};

    return gravity_vector;
}

// !!! CHECK THE CONVERTION IN PRACTICE
float BNO055SensorInterface::get_temperature() {
    int8_t raw_temp = 0;
    
    /*bool res = */(bno055_read_temp_data(&raw_temp) == BNO055_SUCCESS);
    
    // Conversion: 1 LSB = 1°C, so just cast to float
    return static_cast<float>(raw_temp);
}

