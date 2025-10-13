#include "BaseTask.hpp"
#include "esp_heap_caps.h"
#include <Logger.hpp>

BaseTask::BaseTask(const char *name)
    : taskHandle(nullptr), running(false), taskName(name)
{
    LOG_INFO("BaseTask", "Successfully created task: %s", taskName);
}

BaseTask::~BaseTask()
{
    LOG_INFO("BaseTask", "Deleting task: %s", taskName);
    stop();
}

bool BaseTask::start(const TaskConfig &config)
{
    LOG_INFO("BaseTask", "Successfully started task: %s", taskName);
    if (running)
        return false;

    this->config = config;

    // Mark running true early to avoid a race where the created task
    // can start executing before this function sets the flag. If
    // task creation fails we'll revert the flag.
    running = true;

    BaseType_t result;
    if (config.coreId == TaskCore::ANY_CORE)
    {
        result = xTaskCreate(
            taskWrapper,
            config.name,
            config.stackSize,
            this,
            static_cast<UBaseType_t>(config.priority),
            &taskHandle);
    }
    else
    {
        result = xTaskCreatePinnedToCore(
            taskWrapper,
            config.name,
            config.stackSize,
            this,
            static_cast<UBaseType_t>(config.priority),
            &taskHandle,
            static_cast<BaseType_t>(config.coreId));
    }

    if (result == pdPASS)
    {
        LOG_INFO("BaseTask", "%s started on core %d", taskName, static_cast<int>(config.coreId));
        return true;
    }

    // Creation failed: revert running flag
    running = false;
    LOG_ERROR("BaseTask", "Failed to create task %s", taskName);
    return false;
}

void BaseTask::stop()
{
    if (!running)
        return;

    LOG_DEBUG("BaseTask", "Stopping task %s: setting running flag to false", taskName);
    
    // Set flag first to signal task to exit
    running = false;

    // Save task handle locally because the task will set it to nullptr before dying
    TaskHandle_t localHandle = taskHandle;
    
    if (localHandle != nullptr)
    {
        LOG_DEBUG("BaseTask", "Waiting for task %s to exit cooperatively...", taskName);
        
        // Wait for task to finish cooperatively (up to 3 seconds)
        int maxWaits = 60; // 60 * 50ms = 3000ms
        int waits = 0;
        
        while (waits < maxWaits)
        {
            eTaskState taskState = eTaskGetState(localHandle);
            
            // Task has self-deleted or is invalid
            if (taskState == eDeleted || taskState == eInvalid)
            {
                LOG_DEBUG("BaseTask", "Task %s exited after %d waits (%d ms)", taskName, waits, waits * 50);
                break;
            }
            
            vTaskDelay(pdMS_TO_TICKS(50));
            waits++;
        }
        
        // Check final state
        eTaskState finalState = eTaskGetState(localHandle);
        if (finalState != eDeleted && finalState != eInvalid)
        {
            // CRITICAL: Do NOT force delete - this causes mutex deadlock
            // Task must exit on its own or system will hang
            LOG_ERROR("BaseTask", "Task %s FAILED to exit after %d ms - SYSTEM MAY BE UNSTABLE", taskName, maxWaits * 50);
            LOG_ERROR("BaseTask", "Task state: %d (0=Running, 1=Ready, 2=Blocked, 3=Suspended, 4=Deleted)", finalState);
            // Leave taskHandle as-is so we can debug which task is stuck
            return; // Don't set taskHandle to nullptr
        }
    }
    
    taskHandle = nullptr;
    LOG_INFO("BaseTask", "Task %s stopped safely", taskName);
}

void BaseTask::taskWrapper(void *parameter)
{
    BaseTask *task = static_cast<BaseTask *>(parameter);
    if (task)
    {
        task->internalTaskFunction();
    }
    vTaskDelete(NULL);
}

void BaseTask::internalTaskFunction()
{
    // Register with watchdog
    esp_task_wdt_add(NULL);

    LOG_INFO("BaseTask", "[TASK] %s starting on core %d", taskName, xPortGetCoreID());

    onTaskStart();

    try
    {
        taskFunction();
    }
    catch (...)
    {
        LOG_ERROR("BaseTask", "Exception in task %s", taskName);
    }

    onTaskStop();
    
    // Cleanup watchdog BEFORE setting handle to nullptr
    esp_task_wdt_delete(NULL);
    
    // Mark task handle as nullptr so stop() knows we're done
    // This must happen before vTaskDelete to avoid race
    taskHandle = nullptr;
    
    LOG_INFO("BaseTask", "[TASK] %s ended, about to self-delete", taskName);
}

uint32_t BaseTask::getStackHighWaterMark() const
{
    return taskHandle ? uxTaskGetStackHighWaterMark(taskHandle) : 0;
}