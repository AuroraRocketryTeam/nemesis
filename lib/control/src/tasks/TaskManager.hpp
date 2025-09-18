#pragma once

#include "ITask.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>
#include <string>

enum class TaskType {
    SENSOR,
    EKF,
    APOGEE_DETECTION,
    RECOVERY,
    DATA_COLLECTION,
    TELEMETRY,
    GPS,
    LOGGING
};

class TaskManager {
private:
    std::map<TaskType, std::unique_ptr<ITask>> tasks;
    std::shared_ptr<SharedSensorData> sensorData;
    std::shared_ptr<SharedFilteredData> filteredData;
    SemaphoreHandle_t sensorDataMutex;
    SemaphoreHandle_t filteredDataMutex;
    
public:
    TaskManager(std::shared_ptr<SharedSensorData> sensorData, 
            std::shared_ptr<SharedFilteredData> filteredData, 
            SemaphoreHandle_t sensorMutex, 
            SemaphoreHandle_t filteredMutex);
    ~TaskManager();
    
    void initializeTasks();
    bool startTask(TaskType type, const TaskConfig& config);
    void stopTask(TaskType type);
    void stopAllTasks();
    
    bool isTaskRunning(TaskType type) const;
    uint32_t getTaskStackUsage(TaskType type) const;
    
    void printTaskStatus() const;
};