void setup() {}
void loop() {}

/***
 * Temporary code for testing EKF with MPRLS and BNO055 sensors (the real main is main_orig.cpp)
 */

/*
#define EKF_N 24
#define EKF_M 24

#include <tinyekf.h>
#include <vector>
#include <tuple>
#include "./sensors/MPRLS/MPRLSSensor.hpp"
#include "./sensors/BNO055/BNO055Sensor.hpp"

// Elements needed to update the kalman filter
unsigned long lastTime = 0;
static const float EPSILON = 1e-4;

static float Q[EKF_N*EKF_N];
static float R[EKF_M*EKF_M];

static float F[EKF_N*EKF_N];
static float H[EKF_M*EKF_N];

static MPRLSSensor mprls;
static BNO055Sensor bno;

ekf_t _ekf;

// Vector to keep track of the position
std::vector<std::tuple<float, float, float>> positionLog;

void setup()
{
    // Initialize the Q matrix (process noise covariance)
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            Q[i * EKF_N + j] = (i == j) ? EPSILON : 0;
        }
    }

    // Initialize the R matrix (measurement noise covariance)
    for (int i = 0; i < EKF_M; i++) {
        for (int j = 0; j < EKF_M; j++) {
            R[i * EKF_M + j] = (i == j) ? EPSILON : 0;
        }
    }

    // Initialize the F matrix (Jacobian of process model)
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            F[i * EKF_N + j] = (i == j) ? 1 : 0;
        }
    }

    // Initialize the H matrix (Jacobian of measurement model)
    for (int i = 0; i < EKF_M; i++) {
        for (int j = 0; j < EKF_N; j++) {
            H[i * EKF_N + j] = (i == j) ? 1 : 0;
        }
    }

    float Pdiag[EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        Pdiag[i] = 1;
    }
    
    mprls.init();
    bno.init();
    
    // Initialize the EKF with the initial covariance diagonal
    ekf_initialize(&_ekf, Pdiag);

    Serial.begin(115200);
}

void loop()
{
    // Evaluating time differences
    unsigned long currentTime = millis();
    float delta_t = (lastTime > 0) ? (currentTime - lastTime)  / 1000.0 : 0.0;
    lastTime = currentTime;

    Serial.println("Test");
    // Retrieve data from MPRLS sensor
    auto mprlsDataOpt = mprls.getData();
    if (!mprlsDataOpt.has_value()) {
        return;
    }
    auto mprlsData = mprlsDataOpt.value();
    float mprls_pressure = std::get<float>(mprlsData.getData("pressure").value());

    // Retrieve data from BNO055 sensor
    auto bnoDataOpt = bno.getData();
    if (!bnoDataOpt.has_value()) {
        return;
    }
    auto bnoData = bnoDataOpt.value();
    
    float bno_orientation_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["x"]);
    float bno_orientation_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["y"]);
    float bno_orientation_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["z"]);

    float bno_angVelocity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["x"]);
    float bno_angVelocity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["y"]);
    float bno_angVelocity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["z"]);

    float bno_linearAccel_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["x"]);
    float bno_linearAccel_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["y"]);
    float bno_linearAccel_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["z"]);
    
    float bno_magnetometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["x"]);
    float bno_magnetometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["y"]);
    float bno_magnetometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["z"]);
    
    float bno_accelerometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["x"]);
    float bno_accelerometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["y"]);
    float bno_accelerometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["z"]);
    
    float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
    float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
    float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);
    
    float bno_temperature = static_cast<float>(std::get<int8_t>(bnoData.getData("board_temperature").value()));
    
    float bno_quaternion_w = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["w"]);
    float bno_quaternion_x = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["x"]);
    float bno_quaternion_y = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["y"]);
    float bno_quaternion_z = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["z"]);
    
    float z[EKF_M] = {
        mprls_pressure, 
        bno_orientation_x, bno_orientation_y, bno_orientation_z, 
        bno_angVelocity_x, bno_angVelocity_y, bno_angVelocity_z, 
        bno_linearAccel_x, bno_linearAccel_y, bno_linearAccel_z, 
        bno_magnetometer_x, bno_magnetometer_y, bno_magnetometer_z, 
        bno_accelerometer_x, bno_accelerometer_y, bno_accelerometer_z, 
        bno_gravity_x, bno_gravity_y, bno_gravity_z, 
        bno_quaternion_w, bno_quaternion_x, bno_quaternion_y, bno_quaternion_z, 
        bno_temperature
    };
    
    // Process model (state remains unchanged)
    float fx[EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        fx[i] = _ekf.x[i];
    }

    // Update model position and velocity with basic kinematic model
    fx[0] += _ekf.x[3] * delta_t + 0.5f * _ekf.x[6] * delta_t * delta_t;
    fx[1] += _ekf.x[4] * delta_t + 0.5f * _ekf.x[7] * delta_t * delta_t;
    fx[2] += _ekf.x[5] * delta_t + 0.5f * _ekf.x[8] * delta_t * delta_t;

    fx[3] += _ekf.x[6] * delta_t;
    fx[4] += _ekf.x[7] * delta_t;
    fx[5] += _ekf.x[8] * delta_t;


    // Run the prediction step of the EKF
    ekf_predict(&_ekf, fx, F, Q);

    // Measurement function h(x) = x (direct observation)
    float hx[EKF_M];
    for (int i = 0; i < EKF_M; i++) {
        hx[i] = _ekf.x[i];
    }

    // Run the update step of the EKF
    ekf_update(&_ekf, z, hx, H, R);

    positionLog.push_back({_ekf.x[0], _ekf.x[1], _ekf.x[2]});
    
    Serial.println("MPRLS Pressure:");
    Serial.println(z[0]);
    Serial.println();

    Serial.println("BNO055 Orientation:");
    Serial.println(z[1]);
    Serial.println(z[2]);
    Serial.println(z[3]);
    Serial.println();

    Serial.println("BNO055 Angular Velocity:");
    Serial.println(z[4]);
    Serial.println(z[5]);
    Serial.println(z[6]);
    Serial.println();
    
    Serial.println("BNO055 Linear Acceleration:");
    Serial.println(z[7]);
    Serial.println(z[8]);
    Serial.println(z[9]);
    Serial.println();

    Serial.println("BNO055 Magnetometer:");
    Serial.println(z[10]);
    Serial.println(z[11]);
    Serial.println(z[12]);
    Serial.println();

    Serial.println("BNO055 Accelerometer:");
    Serial.println(z[13]);
    Serial.println(z[14]);
    Serial.println(z[15]);
    Serial.println();

    Serial.println("BNO055 Gravity:");
    Serial.println(z[16]);
    Serial.println(z[17]);
    Serial.println(z[18]);
    Serial.println();

    Serial.println("BNO055 Quaternion:");
    Serial.println(z[19]);
    Serial.println(z[20]);
    Serial.println(z[21]);
    Serial.println(z[22]);
    Serial.println();

    Serial.println("BNO055 Temperature:");
    Serial.println(z[23]);
    Serial.println();

    Serial.println("EKF Pressure:");
    Serial.println(_ekf.x[0]);
    Serial.println();

    Serial.println("EKF Orientation:");
    Serial.println(_ekf.x[1]);
    Serial.println(_ekf.x[2]);
    Serial.println(_ekf.x[3]);
    Serial.println();

    Serial.println("EKF Angular Velocity:");
    Serial.println(_ekf.x[4]);
    Serial.println(_ekf.x[5]);
    Serial.println(_ekf.x[6]);
    Serial.println();

    Serial.println("EKF Linear Acceleration:");
    Serial.println(_ekf.x[7]);
    Serial.println(_ekf.x[8]);
    Serial.println(_ekf.x[9]);
    Serial.println();

    Serial.println("EKF Magnetometer:");
    Serial.println(_ekf.x[10]);
    Serial.println(_ekf.x[11]);
    Serial.println(_ekf.x[12]);
    Serial.println();

    Serial.println("EKF Accelerometer:");
    Serial.println(_ekf.x[13]);
    Serial.println(_ekf.x[14]);
    Serial.println(_ekf.x[15]);
    Serial.println();

    Serial.println("EKF Gravity:");
    Serial.println(_ekf.x[16]);
    Serial.println(_ekf.x[17]);
    Serial.println(_ekf.x[18]);
    Serial.println();

    Serial.println("EKF Quaternion:");
    Serial.println(_ekf.x[19]);
    Serial.println(_ekf.x[20]);
    Serial.println(_ekf.x[21]);
    Serial.println(_ekf.x[22]);
    Serial.println();

    Serial.println("EKF Temperature:");
    Serial.println(_ekf.x[23]);
    Serial.println();

    delay(1000);
}
*/