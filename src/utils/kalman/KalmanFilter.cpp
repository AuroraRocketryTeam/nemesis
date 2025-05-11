#include "KalmanFilter.hpp"
#include <cstring>

KalmanFilter::KalmanFilter() {
    Q = new float[EKF_N * EKF_N]();
    R = new float[EKF_M * EKF_M]();
    F = new float[EKF_N * EKF_N]();
    H = new float[EKF_M * EKF_N]();

    // Q: process noise covariance
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            Q[i * EKF_N + j] = (i == j) ? EPSILON : 0.0f;
            F[i * EKF_N + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    // R: measurement noise covariance
    for (int i = 0; i < EKF_M; i++) {
        for (int j = 0; j < EKF_M; j++) {
            R[i * EKF_M + j] = (i == j) ? EPSILON : 0.0f;
        }
    }
    // H: measurement Jacobian (direct observation)
    for (int i = 0; i < EKF_M; i++) {
        for (int j = 0; j < EKF_M; j++) {
            H[i * EKF_N + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    auto Pdiag = new float[EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        Pdiag[i] = EPSILON;
    }

    ekf_initialize(&_ekf, Pdiag);
}

void KalmanFilter::step(const float* z, float delta_t) {
    // predicted state
    float fx[EKF_N];

    std::memcpy(fx, _ekf.x, EKF_N * sizeof(float));

    // Basic kinematic prediction for position
    fx[0] += _ekf.x[7] * delta_t + 0.5f * _ekf.x[4] * delta_t * delta_t;
    fx[1] += _ekf.x[8] * delta_t + 0.5f * _ekf.x[5] * delta_t * delta_t;
    fx[2] += _ekf.x[9] * delta_t + 0.5f * _ekf.x[6] * delta_t * delta_t;

    fx[3] += _ekf.x[4] * delta_t;
    fx[4] += _ekf.x[5] * delta_t;
    fx[5] += _ekf.x[6] * delta_t;

    // Prediction step
    ekf_predict(&_ekf, fx, F, Q);

    // Measurement prediction (direct: hx = x)
    float hx[EKF_M];
    for (int i = 0; i < EKF_M; i++) {
        hx[i] = _ekf.x[i];
    }

    // Update step with measurement z
    ekf_update(&_ekf, const_cast<float*>(z), hx, H, R);
}

float* KalmanFilter::state() {
    return _ekf.x;
}