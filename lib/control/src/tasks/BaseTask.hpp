#pragma once

#include "ITask.hpp"
#include "esp_task_wdt.h"

/**
 * @brief Base class for all tasks in the system, providing common task management functionality.
 *
 * This abstract class implements the ITask interface and provides a foundation for creating
 * FreeRTOS-based tasks with standardized lifecycle management, configuration, and monitoring.
 * Derived classes must implement the pure virtual taskFunction() method to define their
 * specific task behavior.
 *
 * The class handles task creation, starting, stopping, and provides utilities for monitoring
 * task status and stack usage. It uses FreeRTOS primitives for task management and ensures
 * proper cleanup when tasks are destroyed.
 *
 * @note This class is abstract and cannot be instantiated directly.
 * @note Tasks are managed using FreeRTOS TaskHandle_t and follow FreeRTOS conventions.
 */
class BaseTask : public ITask
{
public:
    /**
     * @brief Construct a new Base Task object
     * 
     * @param name The name of the task
     */
    BaseTask(const char *name);
    
    /**
     * @brief Destroy the Base Task object and clean up resources.
     * 
     */
    virtual ~BaseTask();

    /**
     * @brief Start the task with the given configuration.
     * 
     * @param config The task configuration
     * @return true if the task started successfully 
     * @return false if the task failed to start
     */
    bool start(const TaskConfig &config) override;

    /**
     * @brief Stop the task and clean up resources.
     * 
     */
    void stop() override;

    /**
     * @brief Check if the task is currently running.
     * 
     * @return true if the task is running
     * @return false if the task is not running
     */
    bool isRunning() const override { return running; }

    /**
     * @brief Get the name of the task.
     * 
     * @return const char* The name of the task
     */
    const char *getName() const override { return taskName; }
    
    /**
     * @brief Get the high water mark of the task's stack.
     * 
     * @return uint32_t The minimum amount of stack space that remained for the task
     */
    uint32_t getStackHighWaterMark() const override;

protected:
    TaskHandle_t taskHandle;
    TaskConfig config;
    volatile bool running;
    const char *taskName;

    virtual void taskFunction() = 0;
    virtual void onTaskStart() {}
    virtual void onTaskStop() {}

private:
    static void taskWrapper(void *parameter);
    void internalTaskFunction();
};