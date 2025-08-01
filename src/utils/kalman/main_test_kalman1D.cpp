
#include <vector>
#include <tuple>
#include <MPRLSSensor.hpp>
#include <BNO055Sensor.hpp>
#include "./KalmanFilter1D.hpp"
 
// Elements needed to update the kalman filter
unsigned long lastTime = 0;
const int NUM_SAMPLES = 200;
const float STD_TRESHOLD = 0.1f;

KalmanFilter1D* ekf = nullptr;
 
MPRLSSensor* mprls = nullptr;
BNO055Sensor* bno = nullptr;

// Vector to keep track of the position
std::vector<std::tuple<float, float, float>> positionLog;
 
Eigen::Vector3f standard_deviation(const std::vector<Eigen::Vector3f>& readings) {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        mean += reading;
    }
    mean /= readings.size();

    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        Eigen::Vector3f diff = reading - mean;
        variance += diff.cwiseProduct(diff); // componente a componente: (x^2, y^2, z^2)
    }
    variance /= static_cast<float>(readings.size() - 1);

    return variance.cwiseSqrt(); // raíz cuadrada componente a componente
}

// CHECK!!!
float standard_deviation(const std::vector<float>& readings) {
    float mean = 0;
    for (const auto& reading : readings) {
        mean += reading;
    }
    mean /= readings.size();

    float variance = 0;
    for (const auto& reading : readings) {
        float diff = reading - mean;
        variance += (diff*diff);
    }
    variance /= static_cast<float>(readings.size() - 1);

    return sqrt(variance);
}

 void setup()
 {
    Serial.begin(115200);

    Serial.println("Sensors Initialization...");
    mprls = new MPRLSSensor();
    bno = new BNO055Sensor();
    
    mprls->init();
    bno->init();

    Serial.println("Gravity and Magnetometer Readings...");
    Eigen::Vector3f gravity_value, magnetometer_value;
    
    bool calibration = true;
    do {
        // Reading gravity vector samples for kalman filter initialization
        std::vector<Eigen::Vector3f> gravity_readings;
        std::vector<Eigen::Vector3f> magnometer_readings;

        for (int i = 0; i < NUM_SAMPLES; i++) {
            auto bnoDataOpt = bno->getData();
            if (!bnoDataOpt.has_value()) {
                Serial.println("BNO055 data not available");
                return;
            }
            auto bnoData = bnoDataOpt.value();
            
            float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
            float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
            float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);
            
            float bno_magnetometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["x"]);
            float bno_magnetometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["y"]);
            float bno_magnetometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["z"]);
            
            gravity_readings.push_back(Eigen::Vector3f(bno_gravity_x, bno_gravity_y, bno_gravity_z));
            magnometer_readings.push_back(Eigen::Vector3f(bno_magnetometer_x, bno_magnetometer_y, bno_magnetometer_z));

            delay(10); // Small delay to avoid flooding the sensor !!!
        }

        // Calculate the mean of the readings
        Eigen::Vector3f gravity_sum = Eigen::Vector3f::Zero();
        Eigen::Vector3f magnometer_sum = Eigen::Vector3f::Zero();

        for (int i=0; i < NUM_SAMPLES; i++) {
            gravity_sum += gravity_readings[i];
            magnometer_sum += magnometer_readings[i];
        }

        gravity_value = gravity_sum / gravity_readings.size();
        magnetometer_value = magnometer_sum / magnometer_readings.size();

        // Computing the standard deviation of the readings to check for calibration quality
        Eigen::Vector3f gravity_std = standard_deviation(gravity_readings);
        Eigen::Vector3f magnetometer_std = standard_deviation(magnometer_readings);

        if (gravity_std.norm() > STD_TRESHOLD) {
            Serial.println("Calibration failed, gravity variance too high, retrying...");
            calibration = true;
        } else if (magnetometer_std.norm() > STD_TRESHOLD) {
            Serial.println("Calibration failed, magnometer variance too high, retrying...");
            calibration = true;
        } else {
            Serial.println("Calibration successful!");
            calibration = false;
        }
    } while(calibration);
    
    Serial.print("Kalman Filter Initialization");
    ekf = new KalmanFilter1D(gravity_value, magnetometer_value);
 }
 
 void loop()
 {    
    // Evaluating time differences
    unsigned long currentTime = millis();
    float delta_t = (lastTime > 0) ? (currentTime - lastTime) / 1000.0 : 0.0;
    lastTime = currentTime;
    
    // Retrieve data from MPRLS sensor    
    auto mprlsDataOpt = mprls->getData();
    if (!mprlsDataOpt.has_value()) {
        Serial.println("MPRLS data not available");
        return;
    }
    auto mprlsData = mprlsDataOpt.value();
    float mprls_pressure = std::get<float>(mprlsData.getData("pressure").value());

    // Retrieve data from BNO055 sensor
    auto bnoDataOpt = bno->getData();
    if (!bnoDataOpt.has_value()) {
        Serial.println("BNO055 data not available");
        return;
    }
    auto bnoData = bnoDataOpt.value();

    float bno_angVelocity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["x"]);
    float bno_angVelocity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["y"]);
    float bno_angVelocity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["z"]);

    float bno_accelerometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["x"]);
    float bno_accelerometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["y"]);
    float bno_accelerometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["z"]);
    
    /* Currently unused measurements
    float bno_orientation_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["x"]);
    float bno_orientation_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["y"]);
    float bno_orientation_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["z"]);

    
    float bno_linearAccel_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["x"]);
    float bno_linearAccel_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["y"]);
    float bno_linearAccel_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["z"]);
    
    float bno_magnetometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["x"]);
    float bno_magnetometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["y"]);
    float bno_magnetometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["z"]);
    
    float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
    float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
    float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);
    
    float bno_temperature = static_cast<float>(std::get<int8_t>(bnoData.getData("board_temperature").value()));
    
    float bno_quaternion_w = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["w"]);
    float bno_quaternion_x = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["x"]);
    float bno_quaternion_y = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["y"]);
    float bno_quaternion_z = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["z"]);
    */

    float omega[3] = {bno_angVelocity_x, bno_angVelocity_y, bno_angVelocity_z};
    float acc[3] = {bno_accelerometer_x, bno_accelerometer_y, bno_accelerometer_z};
    std::vector<std::vector<float>> measurements = ekf->step(delta_t, omega, acc, mprls_pressure);
    
    std::vector<float> posEKF = measurements[0];
    std::vector<float> velEKF = measurements[1];
    std::vector<float> accEKF = measurements[2];

    Serial.println("Predicted pos: " + String(posEKF[0]) + ", " + String(posEKF[1]) + ", " + String(posEKF[2]));
    delay(10);
 }