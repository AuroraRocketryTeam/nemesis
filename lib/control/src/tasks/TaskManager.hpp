#pragma once

#include "ITask.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>
#include <string>
#include <KalmanFilter1D.hpp>
#include "TaskConfig.hpp"

class TaskManager {
private:
    std::map<TaskType, std::unique_ptr<ITask>> tasks;
    std::shared_ptr<SharedSensorData> sensorData;
    std::shared_ptr<KalmanFilter1D> kalmanFilter; // Needs to be initialized!!!
    SemaphoreHandle_t sensorDataMutex;
    
public:
    TaskManager(std::shared_ptr<SharedSensorData> sensorData, 
            std::shared_ptr<KalmanFilter1D> kalmanFilter, 
            SemaphoreHandle_t sensorMutex);
    ~TaskManager();
    
    void initializeTasks();
    bool startTask(TaskType type, const TaskConfig& config);
    void stopTask(TaskType type);
    void stopAllTasks();
    
    bool isTaskRunning(TaskType type) const;
    uint32_t getTaskStackUsage(TaskType type) const;
    
    void printTaskStatus() const;
};