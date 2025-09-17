#pragma once

#include "tasks/BaseTask.hpp"
#include "SharedData.hpp"
#include <ISensor.hpp>

class SensorTask : public BaseTask {
private:
    SharedSensorData* sharedData;
    ISensor* bno055;
    ISensor* baro1;
    ISensor* baro2;
    SemaphoreHandle_t* dataMutex;
    
public:
    SensorTask(SharedSensorData* data, SemaphoreHandle_t* mutex);
    
    void setSensors(ISensor* imu, ISensor* barometer1, ISensor* barometer2);
    
protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
};