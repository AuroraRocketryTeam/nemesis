#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "KalmanFilter1D.hpp"

// NEED TO BE PROPERLY INITIALIZED !!!
class EkfTask : public BaseTask {
public:
    EkfTask(std::shared_ptr<SharedSensorData> sensorData, 
        SemaphoreHandle_t sensorDataMutex,
        std::shared_ptr<KalmanFilter1D> kalmanFilter) : 
        BaseTask("EkfTask"),
        sensorData(sensorData),
        sensorDataMutex(sensorDataMutex),
        kalmanFilter(kalmanFilter) {}
    ~EkfTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t sensorDataMutex;
    std::shared_ptr<KalmanFilter1D> kalmanFilter;

    uint32_t lastTimestamp = 0;
};