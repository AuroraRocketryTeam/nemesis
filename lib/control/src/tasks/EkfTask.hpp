#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "KalmanFilter1D.hpp"

// NEED TO BE PROPERLY INITIALIZED !!!
class EkfTask : public BaseTask {
public:
    EkfTask(SharedSensorData* sensorData, 
        SemaphoreHandle_t sensorDataMutex,
        KalmanFilter1D* kalmanFilter) : 
        BaseTask("EkfTask") {}
    ~EkfTask() override;

protected:
    void taskFunction() override;

private:
    SharedSensorData* sensorData;
    SemaphoreHandle_t sensorDataMutex;
    KalmanFilter1D* kalmanFilter;

    uint32_t lastTimestamp = 0;
};