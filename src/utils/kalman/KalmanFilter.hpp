#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#define EKF_N 24 // Size of state space
#define EKF_M 6 // Size of observation (measurement) space

#include <tinyekf.h>
#include <cstdint>

class KalmanFilter {
public:
    KalmanFilter();
    void step(const float* z, float delta_t);
    float* state();
    ekf_t _ekf;

private:
    float *Q;  // Process noise covariance
    float *R;  // Measurement noise covariance
    float *F;  // Jacobian with input/output relations
    float *H;  // Measurement Jacobian
    
    const float EPSILON = 1e-4f;
};

#endif // KALMANFILTER_HPP