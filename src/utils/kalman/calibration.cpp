#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>

#include "../lib/Eigen/Dense"
#include "../lib/Eigen/Geometry"

// Calibration Phase. This function must run when the Launcher is still in the launching position.
// It will also use the magnetometer to align the Z axis with the North. We might not get exact Norht since the readings
// might be modified by the presence of the aluminum frame. It is just to get a rough idea of the North.

int main() {

    // Gravity vector got from the accelerometer
    Eigen::Vector3f gravity(-5.0105, -1.7072 , 8.3120);

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
    Eigen::Vector3f magnetometer(21.87, 27.56, -19.75); // Magnetometer reading
    Eigen::Vector3f magnetometer_z_aligned = q_rot * magnetometer;
    float yaw = std::atan2f(magnetometer[1], magnetometer[0]); // Yaw angle in radians
    Eigen::Quaternionf q_yaw(Eigen::AngleAxisf(yaw, Eigen::Vector3f(0, 0, 1)));
    Eigen::Quaternionf initial_quaternion = q_yaw * q_rot;
    initial_quaternion.normalize();

    return 0;
}