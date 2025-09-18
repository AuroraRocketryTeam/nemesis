#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include <ISensor.hpp>
#include <memory>

class SensorTask : public BaseTask {
private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;
    
    // Sensor pointers
    ISensor* bno055;
    ISensor* baro1;
    ISensor* baro2;

public:
    SensorTask(std::shared_ptr<SharedSensorData> sensorData,
               SemaphoreHandle_t mutex);
    
    void setSensors(ISensor* imu, ISensor* barometer1, ISensor* barometer2);

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
};