#ifndef KALMANFILTER1D_HPP
#define KALMANFILTER1D_HPP

// Macros needed for a conflict between similar macro variables names of Arduino.h and Eigen.h
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif
#ifdef B0
#undef B0
#endif

#define EKF_N 6 // Size of state space [1-position, 1-velocity, 4-quaternion_rot] 
#define EKF_M 8 // Size of measurement space [3-accelerations, 3-angular velocities, 1-GPS, 1-pressure]

#include <tinyekf.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <random>
#include <vector>
#include <ArduinoEigen.h>
#include "esp_task_wdt.h"
#include "config.h"

/**
 * @brief One-dimensional Extended Kalman Filter for sensor fusion.
 * 
 * This class implements a 1D EKF for fusing IMU (accelerometer, gyroscope) and barometer data.
 * The state vector includes position, velocity, and orientation (as quaternion).
 * The filter estimates the vertical position and velocity, as well as orientation, 
 * compensating for sensor biases and gravity.
 */
class KalmanFilter1D {
public:
    /**
     * @brief Construct a new KalmanFilter1D object and perform initial calibration.
     * 
     * @param gravity_value Initial gravity vector (from accelerometer).
     * @param magnometer_value Initial magnetic field vector (from magnetometer).
     */
    KalmanFilter1D(Eigen::Vector3f gravity_value, Eigen::Vector3f magnometer_value);
    
    /**
     * @brief Perform one EKF prediction and update step.
     * 
     * @param dt Time step in seconds.
     * @param omega Gyroscope readings [rad/s].
     * @param accel Accelerometer readings [m/s^2].
     * @param pressure Barometer reading (altitude or pressure).
     */
    void step(float dt, float omega[3], float accel[3], float pressure, float gps);
    
    /**
     * @brief Get the current EKF state vector.
     * 
     * @return Pointer to the state vector.
     */
    float* state();

private:
    // TinyEKF structure for the Extended Kalman Filter
    ekf_t ekf;

    
    // How much we trust the Model, initial covariances of state noise, measurement noise
    const float P0 = 1e-3f;     // position (m²)
    const float V0 = 1e-4f;     // velocity (m²/s²)
    const float q_a = 1e-6f;    // quaternion
    
    // How much we trust the Sensors
    const float A0 = 1e-3f;       // accelerometer (m/s²)
    const float G0 = 5e-4f;       // gyroscope (rad/s)
    const float GPS_Z0 = 3.0f;    // GPS altitude (m²)

    const float SeaLevel = 165.0f;
    const float gps_bias = 3.0f;
    const float h_bias_pressure_sensor = 2.0f;

    // Bias of the sensors (initialized in the constructor functino with calibration data)
    Eigen::Vector3f bias_a; // Bias vector for accelerometer
    Eigen::Vector3f bias_g; // Bias vector for gyroscope
    Eigen::Vector3f gravity; // Gravity vector in ENU coordinates

    // Process noise covariance
    float Q[EKF_N*EKF_N] = {
        P0, 0, 0, 0, 0, 0,
        0, V0, 0, 0, 0, 0,
        0, 0, q_a, 0, 0, 0,
        0, 0, 0, q_a, 0, 0,
        0, 0, 0, 0, q_a, 0,
        0, 0, 0, 0, 0, q_a
    };

    // Measurement noise covariance
    float R[EKF_M*EKF_M];

    // Initially, the acceleration is constantly zero, so it won't change
    float H[EKF_M*EKF_N] = {
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0,
    };
    
    // Measurement Jacobian with input/output relations
    float F[EKF_N*EKF_N];

    /**
     * @brief Calibrate the filter using gravity and magnetometer readings.
     * 
     * @param gravity_readings Gravity vector sample.
     * @param magnetometer_value Magnetometer vector sample.
     * @return Tuple of initial quaternion, accelerometer bias, gyroscope bias.
     */
    std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> calibration(Eigen::Vector3f gravity_value, Eigen::Vector3f magnetometer_value);

    /**
     * @brief EKF process and measurement model.
     * 
     * @param dt Time step.
     * @param fx Output: predicted state.
     * @param hx Output: predicted measurement.
     * @param omega_z Gyroscope readings.
     * @param accel_z Accelerometer readings.
     * @param pressure Barometer reading.
     * @param z_gps GPS altitude.
     */
    void run_model(float dt, float fx[EKF_N], float hx[EKF_M], float omega_z[3], float accel_z[3], float pressure, float z_gps);

    /**
     * @brief Compute the Jacobian of the process model numerically.
     * 
     * @param dt Time step.
     * @param omega_z Gyroscope readings.
     * @param accel_z Accelerometer readings.
     * @param h_pressure_sensor Barometer reading.
     * @param z_gps GPS altitude.
     */
    void computeJacobianF_tinyEKF(float dt, float omega_z[3], float accel_z[3], float h_pressure_sensor, float z_gps);
    
    /*
        SUPPORT FUNCTIONS
    */

    /**
     * @brief Estimate the barometer measurement noise variance.
     * 
     * @param v Barometer reading.
     * @return float Estimated variance.
     */
    float estimateBaroVar(float v);

    /**
     * @brief Convert pressure readings to altitude.
     * 
     * @param pressure Barometer reading.
     * @param seaLevelPressurePa Sea level pressure in Pascals.
     * @param T0 Standard temperature at sea level in Kelvin.
     * @param L Temperature lapse rate in Kelvin per meter.
     * @param g0 Gravitational acceleration at sea level in m/s^2.
     * @param R Universal gas constant in J/(mol·K).
     * @param M Molar mass of Earth's air in kg/mol.
     * @return float Estimated altitude.
     */
    float pressureToAltitude(
        float pressure, 
        float seaLevelPressurePa = 101325.0,
        float T0 = 288.15,
        float L = 0.0065,
        float g0 = 9.80665,
        float R = 8.31447,
        float M = 0.0289644);

    /**
     * @brief Rotate a vector from world to body frame using a quaternion.
     * 
     * @param q Quaternion representing rotation.
     * @param vec_world Vector in world frame.
     * @return Rotated vector in body frame.
     */
    Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world);
};

#endif // KALMANFILTER1D_HPP