#include <BNO055Sensor.hpp>

BNO055Sensor::BNO055Sensor()
{
    _bno_interface = BNO055SensorInterface();
}

bool BNO055Sensor::init()
{
    int attempts = 0;
    uint start = millis();
    bool initialized = false;
    
    while (attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS) {
        
        if (_bno_interface.init()) {
            if (_bno_interface.set_operation_mode(BNO055_OPERATION_MODE_NDOF)) {
                initialized = true;
                break;
            } else {
                return false;
            }
        } else {
            return false;
        }
        
        uint end = millis();
        while (end - start < SENSOR_LOOKUP_TIMEOUT) {
            end = millis();
        }
        start = millis();
    }
    
    if (!initialized) {
        return false;
    }
    
    this->setInitialized(true);
    return true;
}


bool BNO055Sensor::updateData()
{
    if (!this->isInitialized())
    {
        return false;
    }

    _data = std::make_shared<BNO055Data>();

    _data->calibration_sys = _bno_interface.check_calibration_sys();
    _data->calibration_gyro = _bno_interface.check_calibration_gyro();
    _data->calibration_accel = _bno_interface.check_calibration_accel();
    _data->calibration_mag = _bno_interface.check_calibration_mag();

    // Get orientation (Euler angles)
    std::vector<float> orientation = _bno_interface.get_euler_deg();
    _data->orientation_x = orientation[0];
    _data->orientation_y = orientation[1];
    _data->orientation_z = orientation[2];

    // Get angular velocity (gyroscope)
    std::vector<float> angular_velocity = _bno_interface.get_gyro_dps();
    _data->angular_velocity_x = angular_velocity[0];
    _data->angular_velocity_y = angular_velocity[1];
    _data->angular_velocity_z = angular_velocity[2];

    // Get linear acceleration
    std::vector<float> linear_accel = _bno_interface.get_linear_accel();
    _data->linear_acceleration_x = linear_accel[0];
    _data->linear_acceleration_y = linear_accel[1];
    _data->linear_acceleration_z = linear_accel[2];

    // Get magnetometer data
    std::vector<float> mag = _bno_interface.get_mag();
    _data->magnetometer_x = mag[0];
    _data->magnetometer_y = mag[1];
    _data->magnetometer_z = mag[2];

    // Get acceleration data
    std::vector<float> accel = _bno_interface.get_accel();
    _data->acceleration_x = accel[0];
    _data->acceleration_y = accel[1];
    _data->acceleration_z = accel[2];

    // Get gravity vector
    std::vector<float> gravity = _bno_interface.get_gravity();
    _data->gravity_x = gravity[0];
    _data->gravity_y = gravity[1];
    _data->gravity_z = gravity[2];

    // Get temperature
    _data->temperature = _bno_interface.get_temperature();

    // Get quaternion data
    std::vector<float> quaternion = _bno_interface.get_quaternion();
    _data->quaternion_w = quaternion[0];
    _data->quaternion_x = quaternion[1];
    _data->quaternion_y = quaternion[2];
    _data->quaternion_z = quaternion[3];

    // Update timestamp
    _data->timestamp = millis();

    return true;
}

bool BNO055Sensor::hardwareTest() {
    bool accel_status = _bno_interface.selftest_accel();
    bool mag_status = _bno_interface.selftest_mag();
    bool gyro_status = _bno_interface.selftest_gyro();
    bool mcu_status = _bno_interface.selftest_mcu();

    // !!! These are not strictly hardware stuff, should we add another test function?
    bool system_status = _bno_interface.check_system_error();
    bool clock_status = _bno_interface.check_clock_status();

    return accel_status && mag_status && gyro_status && mcu_status;
}

std::shared_ptr<BNO055Data> BNO055Sensor::getData() {
    return _data;
}