#include "BaseTask.hpp"
#include "esp_heap_caps.h"

BaseTask::BaseTask(const char *name)
    : taskHandle(nullptr), running(false), taskName(name)
{
    // Constructor
}

BaseTask::~BaseTask()
{
    stop();
}

bool BaseTask::start(const TaskConfig &config)
{
    if (running)
        return false;

    this->config = config;

    BaseType_t result = xTaskCreatePinnedToCore(
        taskWrapper,
        config.name,
        config.stackSize,
        this,
        static_cast<UBaseType_t>(config.priority),
        &taskHandle,
        static_cast<BaseType_t>(config.coreId));

    if (result == pdPASS)
    {
        running = true;
        Serial.printf("[TASK] %s started on core %d\n", taskName, static_cast<int>(config.coreId));
        return true;
    }

    Serial.printf("[ERROR] Failed to create task %s\n", taskName);
    return false;
}

void BaseTask::stop()
{
    if (!running)
        return;

    running = false;

    if (taskHandle)
    {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    Serial.printf("[TASK] %s stopped\n", taskName);
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

    Serial.printf("[TASK] %s starting on core %d\n", taskName, xPortGetCoreID());

    onTaskStart();

    try
    {
        taskFunction();
    }
    catch (...)
    {
        Serial.printf("[ERROR] Exception in task %s\n", taskName);
    }

    onTaskStop();

    // Cleanup watchdog
    esp_task_wdt_delete(NULL);

    Serial.printf("[TASK] %s ended\n", taskName);
}

uint32_t BaseTask::getStackHighWaterMark() const
{
    return taskHandle ? uxTaskGetStackHighWaterMark(taskHandle) : 0;
}