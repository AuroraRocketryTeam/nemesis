#pragma once

#include "ITask.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>
#include <string>
#include <KalmanFilter1D.hpp>
#include "TaskConfig.hpp"
#include "Logger.hpp"
#include "SD-master.hpp"

#include "SensorTask.hpp"
#include "SDLoggingTask.hpp"
#include "EkfTask.hpp"
#include "GpsTask.hpp"
#include "SimulationTask.hpp"
#include "TelemetryTask.hpp"
#include "BarometerTask.hpp"
#include <EspNowTransmitter.hpp>

#define SIMULATION_DATA // Comment this out to use real sensors

class TaskManager {
private:
    std::map<TaskType, std::unique_ptr<ITask>> tasks;
    std::shared_ptr<SharedSensorData> sensorData;
    std::shared_ptr<KalmanFilter1D> kalmanFilter;
    std::shared_ptr<ISensor> bno055;
    std::shared_ptr<ISensor> baro1;
    std::shared_ptr<ISensor> baro2;
    std::shared_ptr<ISensor> gps;
    std::shared_ptr<RocketLogger> rocketLogger;
    SemaphoreHandle_t sensorDataMutex;
    SemaphoreHandle_t loggerMutex;

    std::shared_ptr<SD> sd;
    
    // Telemetry
    std::shared_ptr<EspNowTransmitter> espNowTransmitter;
    
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
            std::shared_ptr<SD> sd,
            std::shared_ptr<RocketLogger> rocketLogger,
            SemaphoreHandle_t loggerMutex);
    
    ~TaskManager();
    
    void initializeTasks();
    bool startTask(TaskType type, const TaskConfig& config);
    void stopTask(TaskType type);
    void stopAllTasks();
    int getRunningTaskCount();
    
    bool isTaskRunning(TaskType type) const;
    uint32_t getTaskStackUsage(TaskType type) const;
    
    void printTaskStatus() const;
};