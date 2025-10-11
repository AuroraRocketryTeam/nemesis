#pragma once

#include "ITask.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>
#include <string>
#include <KalmanFilter1D.hpp>
#include "TaskConfig.hpp"
#include "Logger.hpp"
#include "SensorTask.hpp"
#include "EkfTask.hpp"
#include "GpsTask.hpp"
#include "TelemetryTask.hpp"
#include "BarometerTask.hpp"
#include <EspNowTransmitter.hpp>
//#include "LoggingTask.hpp"

class TaskManager {
private:
    std::map<TaskType, std::unique_ptr<ITask>> tasks;
    std::shared_ptr<SharedSensorData> sensorData;
    std::shared_ptr<KalmanFilter1D> kalmanFilter; // Needs to be initialized!!!
    std::shared_ptr<ISensor> bno055;
    std::shared_ptr<ISensor> baro1;
    std::shared_ptr<ISensor> baro2;
    std::shared_ptr<ISensor> gps;
    SemaphoreHandle_t sensorDataMutex;

    //
    std::shared_ptr<bool> isRising;
    std::shared_ptr<float> heightGainSpeed;
    
    // Telemetry
    std::shared_ptr<EspNowTransmitter> espNowTransmitter;
    
public:
    TaskManager(std::shared_ptr<SharedSensorData> sensorData,
            std::shared_ptr<KalmanFilter1D> kalmanFilter,
            std::shared_ptr<ISensor> imu,
            std::shared_ptr<ISensor> barometer1,
            std::shared_ptr<ISensor> barometer2,
            std::shared_ptr<ISensor> gps,
            SemaphoreHandle_t sensorMutex,
            std::shared_ptr<bool> isRising,
            std::shared_ptr<float> heightGainSpeed);
    ~TaskManager();
    
    void initializeTasks();
    bool startTask(TaskType type, const TaskConfig& config);
    void stopTask(TaskType type);
    void stopAllTasks();
    
    bool isTaskRunning(TaskType type) const;
    uint32_t getTaskStackUsage(TaskType type) const;
    
    void printTaskStatus() const;
};