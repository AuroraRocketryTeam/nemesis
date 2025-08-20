#include "KalmanFilter1D.hpp"

KalmanFilter1D::KalmanFilter1D(
    Eigen::Vector3f gravity_value, 
    Eigen::Vector3f magnetometer_value) 
{
    //calibration phase
    std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> calibration_data = calibration(gravity_value, magnetometer_value);

    bias_a = std::get<1>(calibration_data);
    bias_g = std::get<2>(calibration_data);

    const float pdiag[EKF_N] = {P0, V0, q_a, q_a, q_a, q_a};
    ekf_initialize(&ekf, pdiag);

    // Position
    ekf.x[0] = 0;

    // Velocity
    ekf.x[1] = 0;

    // Quaternion: received from calibration phase
    ekf.x[2] = std::get<0>(calibration_data).w();
    ekf.x[3] = std::get<0>(calibration_data).x();
    ekf.x[4] = std::get<0>(calibration_data).y();
    ekf.x[5] = std::get<0>(calibration_data).z();
    
    // Inizialize matrices
    float tempR[EKF_M*EKF_M] = {
        A0, 0, 0, 0, 0, 0, 0,
        0, A0, 0, 0, 0, 0, 0,
        0, 0, A0, 0, 0, 0, 0,
        0, 0, 0, G0, 0, 0, 0,
        0, 0, 0, 0, G0, 0, 0,
        0, 0, 0, 0, 0, estimateBaroVar(0), 0,
        0, 0, 0, 0, 0, 0, GPS_Z0
    };
    std::copy(tempR, tempR + EKF_M*EKF_M, R);
}

std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> KalmanFilter1D::calibration(
    Eigen::Vector3f gravity_reading, 
    Eigen::Vector3f magnetometer_reading) 
{
    Eigen::Vector3f TolSTD(0.1, 0.1, 0.1); // Tolerance for standard deviation
    Eigen::Vector3f std(1, 1, 1); // Standard deviation of the gravity readings

    Eigen::Vector3f expected_gravity(0, 0, 9.80537); // Expected gravity vector for specific location (Forlì - 34 m over sea level)
    // Got from: https://www.sensorsone.com/local-gravity-calculator/

    // Rotation Axis: cross product of gravity in local R.F. and Z R.F.
    Eigen::Vector3f z(0, 0, 1);
    Eigen::Vector3f g = gravity.normalized();
    Eigen::Vector3f axis = g.cross(z);
    axis.normalize();

    // Angle: acos of the dot product
    float angle = acos(g.dot(z));

    // Quaternion
    Eigen::Quaternionf q_rot(Eigen::AngleAxisf(angle, axis));

    // Rotate the quaternion to align with north
    // Reading of the angle relative to North
    Eigen::Vector3f north_body = magnetometer_reading.normalized();
    Eigen::Vector3f y_axis_abs(0, 1, 0); // North vector in ENU frame
    Eigen::Vector3f north_abs = q_rot * north_body; // Rotate magntetic north to body frame
    float angle_rad = std::acos(north_abs.dot(y_axis_abs) / north_abs.norm());
    Eigen::Quaternionf q_north(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f(0, 0, 1))); // Quaternion to align with North
    Eigen::Quaternionf initial_quaternion = q_north * q_rot;    // Rotation Abs RF to align with North
    initial_quaternion.normalize();

    // Bias of the accelerometer. gravity is in ENU coordinates, so we need to rotate it to match the sensor's frame of reference.
    Eigen::Quaternionf q_absolute_to_body = initial_quaternion.conjugate();
    Eigen::Vector3f initial_gravity_body = q_absolute_to_body * gravity;
    Eigen::Vector3f expected_gravity_body = q_absolute_to_body * expected_gravity;
    Eigen::Vector3f bias_a = initial_gravity_body - expected_gravity_body;

    // Bias of the gyroscope
    Eigen::Vector3f initial_omega(0, 0, 0); // Mean of various readings
    Eigen::Vector3f bias_w = initial_omega - Eigen::Vector3f(0, 0, 0); // Assuming no rotation
    
    return std::make_tuple(initial_quaternion, bias_a, bias_w);
}

void KalmanFilter1D::step(
    float dt, 
    float omega[3], 
    float accel[3], 
    float pressure,
    float gps) 
{
    // Convert accelerometer readings to Eigen vector
    Eigen::Vector3f accel_z(accel[0], accel[1], accel[2]);
    
    // Convert gyroscope readings from degrees/sec to radians/sec because TinyEKF expects radians
    omega[0] *= (float)M_PI / 180.0f;
    omega[1] *= (float)M_PI / 180.0f;
    omega[2] *= (float)M_PI / 180.0f;
    
    // Update the last element of R with the barometer variance
    R[EKF_M*EKF_M - 1] = estimateBaroVar(ekf.x[1]);
    float h_pressure = pressureToAltitude(pressure) - H_BIAS_PRESSURE_SENSOR - SEA_LEVEL;
    float z_gps = gps - SEA_LEVEL - GPS_BIAS;

    float fx[EKF_N] = {0};
    float hx[EKF_M] = {0};

    computeJacobianF_tinyEKF(dt, omega, accel, h_pressure, z_gps);

    // Set the observation vector z
    float z[EKF_M] = {accel[0], accel[1], accel[2], omega[0], omega[1], omega[2], h_pressure, z_gps};

    run_model(dt, fx, hx, omega, accel, h_pressure, z_gps);

    ekf_predict(&ekf, fx, F, Q);

    ekf_update(&ekf, z, hx, H, R);
}


void KalmanFilter1D::run_model(
    float dt, 
    float fx[EKF_N], 
    float hx[EKF_M], 
    float omega_z[3], 
    float accel_z[3], 
    float h_pressure_sensor,
    float z_gps)
{
    Eigen::Vector3f omega(omega_z[0], omega_z[1], omega_z[2]);
    
    // Build the quaternion rotation from the gyroscope readings:
    // Subtract the gyroscope bias
    Eigen::Vector3f omega_eigen = omega - bias_g;
    
    // Compute the angle of rotation
    float theta = omega_eigen.norm() * dt;

    // Define the axis of rotation and the angle
    Eigen::Vector3f axis = omega_eigen.normalized();

    // Create the quaternion representing the rotation
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)
    Eigen::Quaternionf q_nominal(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);

    // Update the quaternion state
    Eigen::Quaternionf q_rot =  q_nominal*delta_q;
    q_rot.normalize();

    // Acceleration of body --> Intertial frame
    Eigen::Vector3f acc_body(accel_z[0], accel_z[1], accel_z[2]);
    Eigen::Vector3f accel_abs = q_rot * (acc_body - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // Position
    fx[0] = (float)(ekf.x[0] + ekf.x[1]*dt + 0.5f*accel_abs[2]*dt*dt);

    // Velocities
    fx[1] = (float)(ekf.x[1] + accel_abs[2]*dt);

    // Quaternion
    fx[2] = q_rot.w();
    fx[3] = q_rot.x();
    fx[4] = q_rot.y();
    fx[5] = q_rot.z();

    // Measurements
    // Here we have to put the expected measurements of the acceleration /Review for future updates
    hx[0] = acc_body[0];
    hx[1] = acc_body[1];
    hx[2] = acc_body[2];
    hx[3] = omega_z[0];
    hx[4] = omega_z[1];
    hx[5] = omega_z[2];
    hx[6] = fx[0];
    hx[7] = fx[0];
}

void KalmanFilter1D::computeJacobianF_tinyEKF(
    float dt, 
    float omega_z[3], 
    float accel_z[3], 
    float h_pressure_sensor, 
    float z_gps) 
{
    float fx_base[EKF_N];
    float hx_dummy[EKF_M]; // Not used
    std::vector<float> original_state(ekf.x, ekf.x + EKF_N);

    run_model(dt, fx_base, hx_dummy, omega_z, accel_z, h_pressure_sensor, z_gps);

    for (int i = 0; i < EKF_N; ++i) {
        // Perturb state
        float epsilon = 1e-5f * max(fabs(ekf.x[i]), 1.0f);
        if (epsilon < 1e-6f) epsilon = 1e-6f;
        
        ekf.x[i] += epsilon;

        float fx_perturbed[EKF_N];
        run_model(dt, fx_perturbed, hx_dummy, omega_z, accel_z, h_pressure_sensor, z_gps);

        for (int j = 0; j < EKF_N; ++j) {
            F[j * EKF_N + i] = (fx_perturbed[j] - fx_base[j]) / epsilon;
        }

        ekf.x[i] = original_state[i]; // Restore original state
    }
}

float* KalmanFilter1D::state() {
    return ekf.x;
}

/*
        SUPPORT FUNCTIONS
*/

// Pressure to altitude conversion using the barometric formula. 
// seaLevelPressurePa and T0 can be found online for day and location
// h0 is the altitude over sea level for that location
float KalmanFilter1D::pressureToAltitude(
        float pressure, 
        float seaLevelPressurePa,
        float T0,
        float L,
        float g0,
        float R,
        float M) 
{
    float exponent = R * L / (g0 * M);
    float h = T0 / L * (1.0f - std::pow(pressure / seaLevelPressurePa, exponent));
    return h;
}

Eigen::Vector3f KalmanFilter1D::rotateToBody(
    const Eigen::Quaternionf& q, 
    const Eigen::Vector3f& vec_world) 
{
    return q.conjugate() * vec_world;  // Equivalent to R^T * vec
}

float KalmanFilter1D::estimateBaroVar(
    float v) 
{
    float std = (std::abs(v) / 300.0f) * 29.0f + 1.0f;
    return std * std;
}