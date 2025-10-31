#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>
#include "TaskConfig.hpp"

/**
 * @brief Interface for tasks in the system.
 * 
 */
class ITask
{
public:
    /** 
     * @brief Virtual destructor for proper cleanup.
     * 
     */
    virtual ~ITask() = default;

    /**
     * @brief Start the task with the given configuration.
     * 
     * @param config The task configuration
     * @return true if the task started successfully
     * @return false if the task failed to start
     */
    virtual bool start(const TaskConfig &config) = 0;

    /**
     * @brief Stop the task and clean up resources.
     * 
     */
    virtual void stop() = 0;

    /**
     * @brief Check if the task is currently running.
     * 
     * @return true if the task is running
     * @return false if the task is not running
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Get the name of the task.
     * 
     * @return const char* The name of the task
     */
    virtual const char *getName() const = 0;

    /**
     * @brief Get the stack high water mark of the task.
     * 
     * @return uint32_t The stack high water mark
     */
    virtual uint32_t getStackHighWaterMark() const = 0;

protected:
    /**
     * @brief The main function of the task.
     * 
     */
    virtual void taskFunction() = 0;

    /**
     * @brief Static wrapper function to adapt member function to FreeRTOS task signature.
     * 
     * @param parameter The task parameter (pointer to the task instance)
     */
    static void taskWrapper(void *parameter);
};