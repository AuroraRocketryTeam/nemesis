#include "KalmanFilter1D.hpp"

KalmanFilter1D::KalmanFilter1D(Eigen::Vector3f gravity_value, Eigen::Vector3f magnetometer_value) {
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
}

// Pressure to altitude conversion using the barometric formula. 
// seaLevelPressurePa and T0 can be found online for day and location
// h0 is the altitude over sea level for that location
float pressureToAltitude(float pressurePa, float seaLevelPressurePa = 101325.0, float T0 = 288.15, float h0 = 0.0) {
    return T0/0.0065f * (1.0f - std::pow(pressurePa / seaLevelPressurePa, 0.1903f)) - h0;
}

std::vector<std::vector<float>> KalmanFilter1D::step(float dt, float omega[3], float accel[3], float pressure) {
    // Convert accelerometer readings to Eigen vector
    Eigen::Vector3f accel_z(accel[0], accel[1], accel[2]);
    
    // Convert gyroscope readings from degrees/sec to radians/sec because TinyEKF expects radians
    omega[0] *= (float)M_PI / 180.0f;
    omega[1] *= (float)M_PI / 180.0f;
    omega[2] *= (float)M_PI / 180.0f;
    
    float fx[EKF_N] = {0};
    float hx[EKF_M] = {0};
    
    Eigen::Matrix<float,3,4> Hq = computeHqAccelJacobian(
        dt,
        Eigen::Quaternionf(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]),
        accel_z,
        Eigen::Vector3f(omega[0], omega[1], omega[2])
    ); 

    // THIS MIGHT SHADOWS THE CLASS ONE. Jacobian of z (measurement) with respect to the state x (position, velocity, quaternion)
    // H matrix is NOT z = H*x. But z = hx, computed in run_model
    float H[EKF_M*EKF_N] = {
        0, 0, Hq(0,0), Hq(0,1), Hq(0,2), Hq(0,3),
        0, 0, Hq(1,0), Hq(1,1), Hq(1,2), Hq(1,3),
        0, 0, Hq(2,0), Hq(2,1), Hq(2,2), Hq(2,3),
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
    };

    // Estimate the barometer variance based on velocity
    float barvar = (std::abs(std::pow(ekf.x[1], 2.0f))) * 0.35f + R0;  // This is the variance (std)^2
    //float barvar =  std * std;
    R[EKF_M*EKF_M - 1] = barvar; // Update the last element of R with the barometer variance

    R[EKF_M*EKF_M - 1] = barvar; // Update the last element of R with the barometer variance
    float h_pressure = pressureToAltitude(pressure); // Convert pressure to altitude
    
    // Set the observation vector z
    float z[EKF_M] = {accel[0], accel[1], accel[2], omega[0], omega[1], omega[2], h_pressure};

    computeJacobianF_tinyEKF(dt, omega, accel, h_pressure);

    run_model(dt, fx, hx, omega, accel, h_pressure);
    
    ekf_predict(&ekf, fx, F, Q);

    ekf_update(&ekf, z, hx, H, R);

    Eigen::Quaternionf q(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);
    float roll, pitch, yaw;
    quaternionToEulerAngles(q, roll, pitch, yaw);
    
    std::vector<float> posEKF = { ekf.x[0], 0, 0};
    std::vector<float> velEKF = { ekf.x[1], 0, 0};
    
    return {posEKF, velEKF};
}

float* KalmanFilter1D::state() {
    return ekf.x;
}

void KalmanFilter1D::quaternionToEulerAngles(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw) {
    // Extract quaternion components
    float w = q.w();
    float x = q.x();
    float y = q.y();
    float z = q.z();

    // Calculate roll (ϕ)
    roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

    // Calculate pitch (θ)
    pitch = std::asin(2.0f * (w * y - z * x));

    // Calculate yaw (ψ)
    yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> KalmanFilter1D::calibration(Eigen::Vector3f gravity_reading, Eigen::Vector3f magnetometer_reading) {
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

Eigen::Vector3f KalmanFilter1D::rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world) {
    return q.conjugate() * vec_world;  // Equivalent to R^T * vec
}

// Compute H_q^{(a)} numerically
Eigen::Matrix<float, 3, 4> KalmanFilter1D::computeHqAccelJacobian(
    const float dt,
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_z,
    const Eigen::Vector3f& omega,
    float epsilon)
{
    // Update quaternion
    // omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen = omega - bias_g; // Subtract gyroscope bias
    Eigen::Vector3f axis = omega_eigen.normalized();
    float theta = omega_eigen.norm() * dt;
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)

    Eigen::Quaternionf q_rot =  q_nominal * delta_q;
    q_rot.normalize();
    
    // !!! Not sure the current gravity initialization gives correct results
    Eigen::Vector3f accel_world = q_rot * (accel_z - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // std::cout << "Line: " << lineNum << ", Accel: " << accel_abs.transpose() << std::endl; 
    Eigen::Matrix<float,3,4> H = Eigen::Matrix<float,3,4>::Zero();
    Eigen::Vector4f q_vec = q_nominal.coeffs();  // (x, y, z, w)

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4f dq = Eigen::Vector4f::Zero();
        dq(i) = epsilon;

        // Perturb positively
        Eigen::Vector4f q_plus_vec = q_vec + dq;
        Eigen::Quaternionf q_plus(q_plus_vec(3), q_plus_vec(0), q_plus_vec(1), q_plus_vec(2));
        q_plus.normalize();

        // Perturb negatively
        Eigen::Vector4f q_minus_vec = q_vec - dq;
        Eigen::Quaternionf q_minus(q_minus_vec(3), q_minus_vec(0), q_minus_vec(1), q_minus_vec(2));
        q_minus.normalize();

        // Rotate vector under perturbed quaternions
        Eigen::Vector3f a_plus = rotateToBody(q_plus, accel_world);
        Eigen::Vector3f a_minus = rotateToBody(q_minus, accel_world);

        // Central difference
        H.col(i) = (a_plus - a_minus) / (2.0f * epsilon);
    }

    return H;  // size 3x4
}

void KalmanFilter1D::run_model(float dt, float fx[EKF_N], float hx[EKF_M], float omega_z[3], float accel_z[3], float h_pressure_sensor) {
    Eigen::Vector3f omega(omega_z[0], omega_z[1], omega_z[2]);
    
    // Build the quaternion rotation from the gyroscope readings:
    // 1. Subtract the gyroscope bias
    Eigen::Vector3f omega_eigen = omega - bias_g;

    // 2. Define the axis of rotation and the angle
    Eigen::Vector3f axis = omega_eigen.normalized();

    // 3. Compute the angle of rotation
    float theta = omega_eigen.norm() * dt;

    // 4. Create the quaternion representing the rotation
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)
    Eigen::Quaternionf q_nominal(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);

    // 5. Update the quaternion state
    Eigen::Quaternionf q_rot =  q_nominal*delta_q;
    q_rot.normalize();

    // Acceleration of body --> Intertial frame
    Eigen::Vector3f acc_body(accel_z[0], accel_z[1], accel_z[2]);
    Eigen::Vector3f accel_abs = q_rot * (acc_body - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // Position
    fx[0] = (float)(ekf.x[0] + ekf.x[1]*dt);
    
    // Velocities
    fx[1] = (float)(ekf.x[1] + accel_abs[2]*dt);

    // Quaternion
    fx[2] = q_rot.w();
    fx[3] = q_rot.x();
    fx[4] = q_rot.y();
    fx[5] = q_rot.z();

    // Measurements
    // Here we have to put the expected measurements of the acceleration /Review for future updates
    hx[0] = accel_z[0] + bias_a[0];
    hx[1] = accel_z[1] + bias_a[1];
    hx[2] = accel_z[2] + bias_a[2];
    hx[3] = omega_z[0] + bias_g[0];
    hx[4] = omega_z[1] + bias_g[1];
    hx[5] = omega_z[2] + bias_g[2];
    hx[6] = h_pressure_sensor;
}

void KalmanFilter1D::computeJacobianF_tinyEKF(float dt, float omega_z[3], float accel_z[3], float h_pressure_sensor) {
    const float epsilon = 1e-5f;
    float fx_base[EKF_N];
    float hx_dummy[EKF_M]; // Not used
    std::vector<float> original_state(ekf.x, ekf.x + EKF_N);

    run_model(dt, fx_base, hx_dummy, omega_z, accel_z, h_pressure_sensor);

    for (int i = 0; i < EKF_N; ++i) {
        // Perturb state
        ekf.x[i] += epsilon;

        float fx_perturbed[EKF_N];
        run_model(dt, fx_perturbed, hx_dummy, omega_z, accel_z, h_pressure_sensor);

        for (int j = 0; j < EKF_N; ++j) {
            F[j * EKF_N + i] = (fx_perturbed[j] - fx_base[j]) / epsilon;
        }

        ekf.x[i] = original_state[i]; // Restore original state
    }
}