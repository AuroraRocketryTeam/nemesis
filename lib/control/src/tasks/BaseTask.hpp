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
protected:
    TaskHandle_t taskHandle;
    TaskConfig config;
    volatile bool running;
    const char *taskName;

public:
    BaseTask(const char *name);
    virtual ~BaseTask();

    bool start(const TaskConfig &config) override;
    void stop() override;
    bool isRunning() const override { return running; }
    const char *getName() const override { return taskName; }
    uint32_t getStackHighWaterMark() const override;

protected:
    virtual void taskFunction() = 0;
    virtual void onTaskStart() {}
    virtual void onTaskStop() {}

private:
    static void taskWrapper(void *parameter);
    void internalTaskFunction();
};