#pragma once

#include "BaseTask.hpp"
#include <Nemesis.hpp>
#include "KalmanFilter1D.hpp"

#include <BNO055Sensor.hpp>
#include <MS561101BA03.hpp>
#include <LIS3DHTRSensor.hpp>
#include <GPS.hpp>

/**
 * @brief Class to implement an EKF task.
 * 
 */
class EkfTask : public BaseTask {
public:
    /**
     * @brief Construct a new Ekf Task object
     * 
     * @param model The shared pointer to the rocket model
     * @param modelMutex The semaphore handle to protect access to the model
     * @param kalmanFilter The shared pointer to the Kalman filter instance
     */
    EkfTask(std::shared_ptr<Nemesis> model, 
        SemaphoreHandle_t modelMutex,
        std::shared_ptr<KalmanFilter1D> kalmanFilter) : 
        BaseTask("EkfTask"),
        _model(model),
        _modelMutex(modelMutex),
        _kalmanFilter(kalmanFilter) {}
    ~EkfTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<Nemesis> _model;
    SemaphoreHandle_t _modelMutex;
    std::shared_ptr<KalmanFilter1D> _kalmanFilter;

    uint32_t _lastTimestamp = 0;
};