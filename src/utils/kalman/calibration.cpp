/*
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

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>

#include <ArduinoEigen.h>

// Calibration Phase. This function must run when the Launcher is still in the launching position.
// It will also use the magnetometer to align the Z axis with the North. We might not get exact Norht since the readings
// might be modified by the presence of the aluminum frame. It is just to get a rough idea of the North.

Eigen::Vector3f standard_deviation(const std::vector<Eigen::Vector3f>& readings) {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        mean += reading;
    }
    mean /= readings.size();

    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        Eigen::Vector3f diff = reading - mean;
        variance += diff.cwiseProduct(diff); // component by component: (x^2, y^2, z^2)
    }
    variance /= static_cast<float>(readings.size() - 1);

    return variance.cwiseSqrt(); // square root component by component
    // THIS X
}
// STILL WAITING IF WE CAN USE NDOF WITH ACC RANGE 16g
// IF WE USE RAW, WE NEED TO CALIBRATE THE ACCELEROMETER AND GYROSCOPE AND GRAVITY IS INCLUDED IN ACC MEAS.
// IF WE USE NDOF, WE DON'T NEED TO CALIBRATE THE ACCELEROMETER AND GYROSCOPE 

int main() {

    Eigen::Vector3f TolSTD(0.1, 0.1, 0.1); // Tolerance for standard deviation
    Eigen::Vector3f std(1, 1, 1); // Standard deviation of the gravity readings

    // Gravity vector got from the accelerometer
    std::vector<Eigen::Vector3f> gravity_readings;
    std::vector<Eigen::Vector3f> acc_readings;
    std::vector<Eigen::Vector3f> gyro_readings;

    while ((std.array() > TolSTD.array()).any()) {
        gravity_readings.clear();
        acc_readings.clear();
        gyro_readings.clear();
        for (int i = 0; i < 200; ++i) {
            // G Reading
            Eigen::Vector3f reading_g(1.22, -1.03, 9.61); // Replace with actual reading of gravity
            gravity_readings.push_back(reading_g);

            // Acc Reading
            Eigen::Vector3f reading_acc(0.12, -0.09, 0.05); // Replace with actual reading of accelerometer
            acc_readings.push_back(reading_acc);

            // Gyro Reading
            Eigen::Vector3f reading_gyro(0.11, -0.07, 0.11); // Replace with actual reading of gyroscope
            gyro_readings.push_back(reading_gyro);

            // delay(10); // Simulate delay between readings
        }
        std = standard_deviation(gravity_readings);
        if ((std.array() > TolSTD.array()).any()) {
            std::cout << "Standard deviation too high, repeat calibration." << std.array() << std::endl;
            // delay(1000); // Simulate delay before next calibration attempt
        } else {
            std::cout << "Measuring succesful!" << std::endl;
        }
    }
    // TO DO: COMPUTE STANDARD DEVIATION OF THE GRAVITY READINGS, IF TOO HIGH, REPEAT THE CALIBRATION

    // Calculate the mean of the gravity readings
    Eigen::Vector3f gravity_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f acc_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();
    for (const auto& reading : gravity_readings) {
        gravity_sum += reading;
    }
    for (const auto& reading : acc_readings) {
        acc_sum += reading;
    }
    for (const auto& reading : gyro_readings) {
        gyro_sum += reading;
    }
    Eigen::Vector3f gravity = gravity_sum / gravity_readings.size();
    Eigen::Vector3f acc_bias = acc_sum / acc_readings.size();
    Eigen::Vector3f gyro_bias = gyro_sum / gyro_readings.size();
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
    Eigen::Quaternionf initial_quaternion(Eigen::AngleAxisf(angle, axis));
    initial_quaternion.normalize();

    // Bias of the accelerometer. gravity is in ENU coordinates, so we need to rotate it to match the sensor's frame of reference.
    Eigen::Quaternionf q_absolute_to_body = initial_quaternion.conjugate();
    Eigen::Vector3f initial_gravity_body = q_absolute_to_body * gravity;
    Eigen::Vector3f expected_gravity_body = q_absolute_to_body * expected_gravity;
    Eigen::Vector3f bias_g = initial_gravity_body - expected_gravity_body;

    return 0;
}
*/