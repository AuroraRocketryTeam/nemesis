#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <random>  // Include for random number generation

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "../lib/Eigen/Geometry"
#include "../lib/Eigen/Dense"

// Get Data from files
// Acceleration
std::vector<float> getAcc(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> acc;
    std::ifstream myfile("../csv_measurements/accFlt.csv");

    int lineNumber = 0;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    acc.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    return acc;
}

// Angular velocity
std::vector<float> getOmega(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> omega;
    std::ifstream myfile("../csv_measurements/wVelFlt.csv");

    int lineNumber = 0;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    omega.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    return omega;
}

// Save values to a CSV file
void saveToCSV(const std::vector<float>& acc, const std::string& filename) {
    // Open in append mode to avoid overwriting previous data
    std::ofstream file(filename, std::ios::app);

    if (file.is_open()) {
        // Make sure the vector has exactly 3 values
        if (acc.size() == 3) {
            file << acc[0] << "," << acc[1] << "," << acc[2] << "\n";
        } else {
            std::cerr << "Error: vector must have exactly 3 elements.\n";
        }

        file.close();
    } else {
        std::cerr << "Unable to open file for writing.\n";
    }
}

// These must be defined before including TinyEKF.h
#define EKF_N 16 // Size of state space [3-positions, 3-velocities, 3-accelerations, 4-quaternion_rot] 
#define EKF_M 6 // Size of observation (measurement) space [3-positions, 3-accelerations, 4-quaternion_rot]

#include "../lib/TinyEKF-master/src/tinyekf.h"

static const float dt = 0.005; // 200 Hz

// initial covariances of state noise, measurement noise
// Q matrix (model) // We obtain this value from a comparison between a model and the real data.
static const float P0 = 1e-4; 
static const float V0 = 1e-4;
static const float q_a = 1e-8;
static const float b_a = 1e-8;
static const float b_g = 1e-8;

// R matrix (measurements)
static const float A0 = 1e-3;
static const float G0 = 1e-5;

static const Eigen::Vector3f gravity(0, 0, -9.81); // Gravity vector in ENU coordinates

// Set fixed process-noise covariance matrix Q, see [1]  ---------------------

static const float Q[EKF_N*EKF_N] = {

    P0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, P0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, P0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, V0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, V0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, V0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, q_a, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, q_a, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, q_a, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, q_a, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b_a, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b_a, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b_a, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b_g, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b_g, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b_g
};

// Set fixed measurement noise covariance matrix R ----------------------------

static const float R[EKF_M*EKF_M] = {
    A0, 0, 0, 0, 0, 0,
    0, A0, 0, 0, 0, 0,
    0, 0, A0, 0, 0, 0,
    0, 0, 0, G0, 0, 0,
    0, 0, 0, 0, G0, 0,
    0, 0, 0, 0, 0, G0

};

// Jacobian matrix
float F[EKF_N*EKF_N] = {
    1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,

    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1

};

// Initially, the acceleration is constantly zero, so it won't change

float H[EKF_M*EKF_N] = {
    
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
};

Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world) {
    return q.conjugate() * vec_world;  // Equivalent to R^T * vec
}

// Compute H_q^{(a)} numerically
Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_world,
    float epsilon = 1e-5)
{
    Eigen::Matrix<float, 3, 4> H;
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
        H.col(i) = (a_plus - a_minus) / (2.0 * epsilon);
    }

    return H;  // size 3x4
}

static void initialize_ekf(ekf_t *ekf) {
    // Position
    ekf -> x[0] = 0;
    ekf -> x[1] = 0;
    ekf -> x[2] = 0;

    // Velocity
    ekf -> x[3] = 0;
    ekf -> x[4] = 0;
    ekf -> x[5] = 0;

    // Quaternion: received from calibration phase
    ekf -> x[6] = 0.9541;
    ekf -> x[7] = -0.0571;
    ekf -> x[8] = 0.2734;
    ekf -> x[9] = -0.1079;
    
    // Accel Bias
    ekf -> x[10] = 0.01;
    ekf -> x[11] = 0.01;
    ekf -> x[12] = 0.01;
    
    // Gyro Bias
    ekf -> x[13] = 0.01;
    ekf -> x[14] = 0.01;
    ekf -> x[15] = 0.01;
}

static void run_model(ekf_t * ekf, float dt, float fx[EKF_N], float hx[EKF_M], float omega_x, float omega_y, float omega_z, float accel_z[3]) {
    
    // Biases
    Eigen::Vector3f bias_a(ekf->x[10], ekf->x[11], ekf->x[12]);
    Eigen::Vector3f bias_g(ekf->x[13], ekf->x[14], ekf->x[15]);

    // Update quaternion
    // omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen(omega_x - bias_g[0], omega_y - bias_g[1], omega_z - bias_g[2]);
    float theta = omega_eigen.norm() * dt;
    Eigen::Vector3f axis = omega_eigen.normalized();
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)
    Eigen::Quaternionf q_nominal(ekf->x[6], ekf->x[7], ekf->x[8], ekf->x[9]);
    Eigen::Quaternionf q_rot =  q_nominal * delta_q;
    q_rot.normalize();

    Eigen::Vector3f acc_body(accel_z[0], accel_z[1], accel_z[2]);
    Eigen::Vector3f accel_abs = q_rot * (acc_body - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // Position
    fx[0] = (float)(ekf->x[0] + ekf->x[3]*dt);
    fx[1] = (float)(ekf->x[1] + ekf->x[4]*dt);
    fx[2] = (float)(ekf->x[2] + ekf->x[5]*dt);
    
    // Velocities
    fx[3] = (float)(ekf->x[3] + accel_abs[0]*dt);
    fx[4] = (float)(ekf->x[4] + accel_abs[1]*dt);
    fx[5] = (float)(ekf->x[5] + accel_abs[2]*dt);

    // Quaternion
    fx[6] = q_rot.w();
    fx[7] = q_rot.x();
    fx[8] = q_rot.y();
    fx[9] = q_rot.z();

    // Define the noise standard deviations for each bias component
    // These values would typically be based on the sensor's datasheet or empirically determined
    const float accel_bias_stddev = 0.000001f;  // Acceleration bias noise (e.g., 1 mG)
    const float gyro_bias_stddev = 0.000001f*M_PI/180;   // Gyroscope bias noise (e.g., 0.1 deg/s)

    std::default_random_engine generator;  // Random number generator
    std::normal_distribution<float> accel_noise(0.0, accel_bias_stddev);  // Gaussian noise for accelerometer bias
    std::normal_distribution<float> gyro_noise(0.0, gyro_bias_stddev);   // Gaussian noise for gyro bias

    // Now we apply the noise to the bias components
    // Accel Bias (BNO055 accelerometer)
    fx[10] = (float)(ekf->x[10]) + accel_noise(generator);  // X-axis accel bias with noise
    fx[11] = (float)(ekf->x[11]) + accel_noise(generator);  // Y-axis accel bias with noise
    fx[12] = (float)(ekf->x[12]) + accel_noise(generator);  // Z-axis accel bias with noise

    // Gyro Bias (BNO055 gyroscope)
    fx[13] = (float)(ekf->x[13]) + gyro_noise(generator);   // X-axis gyro bias with noise
    fx[14] = (float)(ekf->x[14]) + gyro_noise(generator);   // Y-axis gyro bias with noise
    fx[15] = (float)(ekf->x[15]) + gyro_noise(generator);   // Z-axis gyro bias with noise

    // Measurements
    // Here we have to put the expected measurements of the acceleration /Review for future updates
    hx[0] = accel_z[0] + bias_a[0];
    hx[1] = accel_z[1] + bias_a[1];
    hx[2] = accel_z[2] + bias_a[2];
    hx[3] = omega_x + bias_g[0];
    hx[4] = omega_y + bias_g[1];
    hx[5] = omega_z + bias_g[2];
}

void computeJacobianF_tinyEKF(ekf_t* ekf, float dt, float omega_x, float omega_y, float omega_z, float accel_z[3], float F_out[EKF_N * EKF_N]) {
    const float epsilon = 1e-5f;
    float fx_base[EKF_N];
    float hx_dummy[EKF_M]; // Not used
    std::vector<float> original_state(ekf->x, ekf->x + EKF_N);

    run_model(ekf, dt, fx_base, hx_dummy, omega_x, omega_y, omega_z, accel_z);

    for (int i = 0; i < EKF_N; ++i) {
        // Perturb state
        ekf->x[i] += epsilon;

        float fx_perturbed[EKF_N];
        run_model(ekf, dt, fx_perturbed, hx_dummy, omega_x, omega_y, omega_z, accel_z);

        for (int j = 0; j < EKF_N; ++j) {
            F_out[j * EKF_N + i] = (fx_perturbed[j] - fx_base[j]) / epsilon;
        }

        ekf->x[i] = original_state[i]; // Restore original state
    }
}

int main() {
    std::ofstream file("../csv_measurements/accEkf.csv", std::ios::trunc);
    file.close();
    std::ofstream file2("../csv_measurements/velEkf.csv", std::ios::trunc);
    file2.close();
    std::ofstream file3("../csv_measurements/posEkf.csv", std::ios::trunc);
    file3.close();

    int lineNum = 1;
    int totalLines = 819;

    ekf_t ekf = {0};
    const float pdiag[EKF_N] = {P0, P0, P0, V0, V0, V0, q_a, q_a, q_a, q_a, b_a, b_a, b_a, b_g, b_g, b_g};

    ekf_initialize(&ekf, pdiag);
    initialize_ekf(&ekf);

    float accX;
    float accY;
    float accZ;
    float omega_x;
    float omega_y;
    float omega_z;

    while (lineNum <= totalLines) {
        std::vector<float> acc = getAcc(lineNum);           // Get the accelerometer data
        std::vector<float> omega = getOmega(lineNum);       // Get the Angular velocity

        for (auto& value : acc) {
            value *= 9.81; // Convert to m/s^2
        }
        accX = acc[0];
        accY = acc[1];
        accZ = acc[2];
        Eigen::Vector3f accel_z(accX, accY, accZ);
        omega_x = omega[0]*M_PI/180.0; // Convert to rad/s
        omega_y = omega[1]*M_PI/180.0; // Convert to rad/s
        omega_z = omega[2]*M_PI/180.0; // Convert to rad/s

        float fx[EKF_N] = {0};
        float hx[EKF_M] = {0};

        // Biases
        Eigen::Vector3f bias_a(ekf.x[10], ekf.x[11], ekf.x[12]);
        Eigen::Vector3f bias_g(ekf.x[13], ekf.x[14], ekf.x[15]);

        // Update quaternion
        // omega = [wx, wy, wz] in rad/s
        Eigen::Vector3f omega_eigen(omega_x - bias_g[0], omega_y - bias_g[1], omega_z - bias_g[2]);
        Eigen::Vector3f axis = omega_eigen.normalized();
        float theta = omega_eigen.norm() * dt;
        Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)

        Eigen::Quaternionf q_nominal(ekf.x[6], ekf.x[7], ekf.x[8], ekf.x[9]);
        Eigen::Quaternionf q_rot =  q_nominal * delta_q;
        q_rot.normalize();
        
        Eigen::Vector3f accel_abs = q_rot * (accel_z - bias_a) - gravity;  // Equivalent to q * a * q.inverse()

        Eigen::Matrix<float,3,4> Hq = computeHqAccelJacobian(
            Eigen::Quaternionf(ekf.x[6], ekf.x[7], ekf.x[8], ekf.x[9]),
            Eigen::Vector3f(accel_abs[0], accel_abs[1], accel_abs[2])
        );
        

        float H[EKF_M*EKF_N] = {
            
            0, 0, 0, 0, 0, 0, Hq(0,0), Hq(0,1), Hq(0,2), Hq(0,3), 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, Hq(1,0), Hq(1,1), Hq(1,2), Hq(1,3), 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, Hq(2,0), Hq(2,1), Hq(2,2), Hq(2,3), 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
        };
        
        // Set the observation vector z
        float z[EKF_M] = {accX, accY, accZ, omega_x, omega_y, omega_z};

        run_model(&ekf, dt, fx, hx, omega_x, omega_y, omega_z, acc.data());
        
        ekf_predict(&ekf, fx, F, Q);

        ekf_update(&ekf, z, hx, H, R);
        
        lineNum++;  // Move to the next line

        /*
        for (int i = 0; i < EKF_M; ++i) {
            for (int j = 0; j < EKF_N; ++j) {
            std::cout << H[i * EKF_N + j] << " ";
            }
            std::cout << std::endl;
        }*/

        std::vector<float> posEKF = { ekf.x[0], ekf.x[1], ekf.x[2] };
        std::vector<float> velEKF = { ekf.x[3], ekf.x[4], ekf.x[5] };
        std::vector<float> accEKF = { ekf.x[6], ekf.x[7], ekf.x[8] };
        saveToCSV(posEKF, "../csv_measurements/posEkf.csv");
        saveToCSV(velEKF, "../csv_measurements/velEkf.csv");
        saveToCSV(accEKF, "../csv_measurements/accEkf.csv");
    }
    std::cout << "Time: " << lineNum*dt << ": " << ekf.x[0] << ", " << ekf.x[1] << ", " << ekf.x[2] << std::endl;

    return 0;
}