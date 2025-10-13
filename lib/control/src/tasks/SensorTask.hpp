#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "Logger.hpp"
#include "RocketLogger.hpp"
#include <ISensor.hpp>
#include <memory>

class SensorTask : public BaseTask
{
private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;

    std::shared_ptr<RocketLogger> rocketLogger;
    SemaphoreHandle_t loggerMutex;

    // Use shared pointers for sensors
    ISensor *bno055;
    ISensor *baro1;
    ISensor *baro2;

public:
    SensorTask(std::shared_ptr<SharedSensorData> sensorData,
               SemaphoreHandle_t mutex,
               std::shared_ptr<ISensor> imu = nullptr,
               std::shared_ptr<ISensor> barometer1 = nullptr,
               std::shared_ptr<ISensor> barometer2 = nullptr,
               std::shared_ptr<RocketLogger> rocketLogger = nullptr, 
               SemaphoreHandle_t loggerMutex = nullptr);

    void setSensors(std::shared_ptr<ISensor> imu,
                    std::shared_ptr<ISensor> barometer1,
                    std::shared_ptr<ISensor> barometer2);

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
};