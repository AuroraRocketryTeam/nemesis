#include <BNO055Sensor.hpp>

BNO055Sensor::BNO055Sensor()
{
    bno_interface = BNO055SensorInterface();
}

bool BNO055Sensor::init()
{
    int attempts = 0;
    uint start = millis();
    bool initialized = false;
    
    while (attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS) {
        
        if (bno_interface.init()) {
            if (bno_interface.set_operation_mode(BNO055_OPERATION_MODE_NDOF)) {
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
    
    this->calibrate();
    this->setInitialized(true);
    return true;
}

std::optional<SensorData> BNO055Sensor::getData()
{
    if (!this->isInitialized())
    {
        return std::nullopt;
    }

    std::vector<float> accelData = bno_interface.get_accel();
    std::vector<float> magData = bno_interface.get_mag();
    std::vector<float> angVelocityData = bno_interface.get_gyro_dps(); // !!! ASK IF IS IT BETTER THE DPS OR RPS
    std::vector<float> orientationData = bno_interface.get_euler_deg();
    std::vector<float> quaternionData = bno_interface.get_quaternion();
    std::vector<float> linearAccelData = bno_interface.get_linear_accel();
    std::vector<float> gravityData = bno_interface.get_gravity();
    float temperature = bno_interface.get_temperature();

    uint8_t systemCal = bno_interface.check_calibration_sys();
    uint8_t gyroCal = bno_interface.check_calibration_gyro();
    uint8_t accelCal = bno_interface.check_calibration_accel();
    uint8_t magCal = bno_interface.check_calibration_mag();

    SensorData data("BNO055");
    data.setData("system_calibration", systemCal);
    data.setData("gyro_calibration", gyroCal);
    data.setData("accel_calibration", accelCal);
    data.setData("mag_calibration", magCal);

    data.setData("orientation", std::map<std::string, float>{
                                    {"x", orientationData[0]},
                                    {"y", orientationData[1]},
                                    {"z", orientationData[2]}});

    data.setData("angular_velocity", std::map<std::string, float>{
                                         {"x", angVelocityData[0]},
                                         {"y", angVelocityData[1]},
                                         {"z", angVelocityData[2]}});

    data.setData("linear_acceleration", std::map<std::string, float>{
                                            {"x", linearAccelData[0]},
                                            {"y", linearAccelData[1]},
                                            {"z", linearAccelData[2]}});

    data.setData("magnetometer", std::map<std::string, float>{
                                     {"x", magData[0]},
                                     {"y", magData[1]},
                                     {"z", magData[2]}});

    data.setData("accelerometer", std::map<std::string, float>{
                                      {"x", accelData[0]},
                                      {"y", accelData[1]},
                                      {"z", accelData[2]},
                                      {"magnitude", sqrt(accelData[0] * accelData[0] + accelData[1] * accelData[1] + accelData[2] * accelData[2])}});

    data.setData("gravity", std::map<std::string, float>{
                                {"x", gravityData[0]},
                                {"y", gravityData[1]},
                                {"z", gravityData[2]}});

    data.setData("board_temperature", temperature);
    data.setData("quaternion", std::map<std::string, double>{
                                   {"w", quaternionData[0]},
                                   {"x", quaternionData[1]},
                                   {"y", quaternionData[2]},
                                   {"z", quaternionData[3]}});

    return data;
}

bool BNO055Sensor::calibrate()
{
    if (!this->isInitialized())
    {
        return false;
    }

    uint start = millis();
    int attempts = 0;
    while (attempts < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        uint end = millis();
        while (end - start < SENSOR_LOOKUP_TIMEOUT)
        {
            end = millis();
        }

        if (bno_interface.check_calibration() >= SUFFICIENT_SENSOR_CALIBRATION) {
            return true;
        }

        attempts++;
    }

    return false;
}

bool BNO055Sensor::hardwareTest() {
    bool accel_status = bno_interface.selftest_accel();
    bool mag_status = bno_interface.selftest_mag();
    bool gyro_status = bno_interface.selftest_gyro();
    bool mcu_status = bno_interface.selftest_mcu();

    // !!! These are not strictly hardware stuff, should we add another test function?
    bool system_status = bno_interface.check_system_error();
    bool clock_status = bno_interface.check_clock_status();

    return accel_status && mag_status && gyro_status && mcu_status;
}

BNO055Sensor::CalibrationStatus BNO055Sensor::getCalibration() {
        CalibrationStatus status;
        status.sys = bno_interface.check_calibration_sys();
        status.gyro = bno_interface.check_calibration_gyro();
        status.accel = bno_interface.check_calibration_accel();
        status.mag = bno_interface.check_calibration_mag();
        return status;
    }    