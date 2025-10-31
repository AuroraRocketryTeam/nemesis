#pragma once

#include "ITask.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>
#include <string>
#include "TaskConfig.hpp"
#include "Logger.hpp"
#include "SD-master.hpp"
#include "Nemesis.hpp"

#include "SensorTask.hpp"
#include "SDLoggingTask.hpp"
#include "EkfTask.hpp"
#include "GpsTask.hpp"
#include "SimulationTask.hpp"
#include "TelemetryTask.hpp"
#include "BarometerTask.hpp"
#include <EspNowTransmitter.hpp>

//#define SIMULATION_DATA // Comment this out to use real sensors

/**
 * @brief Class to manage tasks in the system.
 * 
 */
class TaskManager {    
public:
    /**
     * @brief Construct a new Task Manager object
     * 
     * @param model The shared pointer to the rocket model
     * @param modelMutex The semaphore handle to protect access to the model
     * @param sd The shared pointer to the SD card
     * @param logger The shared pointer to the RocketLogger instance
     */
    TaskManager(std::shared_ptr<Nemesis> model,
            SemaphoreHandle_t modelMutex,
            std::shared_ptr<SD> sd,
            std::shared_ptr<RocketLogger> logger,
            SemaphoreHandle_t loggerMutex);
    
    /**
     * @brief Destroy the Task Manager object
     * 
     */
    ~TaskManager();

    /**
     * @brief Initialize all tasks in the manager
     * 
     */
    void initializeTasks();

    /**
     * @brief Start a task in the manager
     * 
     * @param type The type of task to start
     * @param config The configuration parameters for the task
     * @return true if the task was started successfully, false otherwise
     */
    bool startTask(TaskType type, const TaskConfig& config);
    
    /**
     * @brief Stop a task in the manager
     * 
     * @param type The type of task to stop
     */
    void stopTask(TaskType type);

    /**
     * @brief Stop all running tasks in the manager
     * 
     */
    void stopAllTasks();

    /**
     * @brief Get the count of currently running tasks
     * 
     * @return int The number of running tasks
     */
    int getRunningTaskCount();

    /**
     * @brief Check if a task is currently running
     * 
     * @param type The type of task to check
     * @return true if the task is running, false otherwise
     */
    bool isTaskRunning(TaskType type) const;

    /**
     * @brief Get the stack usage of a task
     * 
     * @param type The type of task to check
     * @return uint32_t The stack usage of the task
     */
    uint32_t getTaskStackUsage(TaskType type) const;
    
    /**
     * @brief Print the status of all tasks
     * 
     */
    void printTaskStatus() const;

private:
    // Map of task type to task instance
    std::map<TaskType, std::unique_ptr<ITask>> _tasks;
    
    // Shared resources
    std::shared_ptr<Nemesis> _model;
    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _modelMutex;
    SemaphoreHandle_t _loggerMutex;

    std::shared_ptr<SD> _sd;
    
    // Telemetry
    std::shared_ptr<EspNowTransmitter> _espNowTransmitter;
};