#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>

enum class TaskType
{
    SENSOR,
    EKF,
    APOGEE_DETECTION,
    RECOVERY,
    DATA_COLLECTION,        // This might be removed if SharedData are used in the corresponding tasks
    TELEMETRY,
    GPS,
    LOGGING
};

enum class TaskPriority
{
    TASK_LOW = 1,
    TASK_MEDIUM = 2,
    TASK_HIGH = 3,
    TASK_CRITICAL = 4,
    TASK_REAL_TIME = 5
};

enum class TaskCore
{
    CORE_0 = 0, // Critical tasks
    CORE_1 = 1, // Non-critical tasks
    ANY_CORE = tskNO_AFFINITY
};

struct TaskConfig
{
    TaskType type;
    const char *name;
    uint32_t stackSize;
    TaskPriority priority;
    TaskCore coreId;
    bool shouldRun;

    /**
     * @brief Construct a new Task Config object
     *
     * @param t TaskType: The type of task @see enum class TaskType
     * @param n const char*: The name of the task
     * @param stack uint32_t: The stack size for the task
     * @param prio TaskPriority: The priority of the task @see enum class TaskPriority
     * @param core TaskCore: The core ID for the task @see enum class TaskCore
     * @param run bool: Flag indicating if the task should run
     */
    TaskConfig(TaskType t = TaskType::SENSOR, const char *n = "Task", uint32_t stack = 2048,
               TaskPriority prio = TaskPriority::TASK_MEDIUM,
               TaskCore core = TaskCore::ANY_CORE, bool run = true)
        : type(t), name(n), stackSize(stack), priority(prio), coreId(core), shouldRun(run) {}
};