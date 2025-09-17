#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>

enum class TaskPriority {
    TASK_LOW = 1,
    TASK_MEDIUM = 2,
    TASK_HIGH = 3,
    TASK_CRITICAL = 4,
    TASK_REAL_TIME = 5
};

enum class TaskCore {
    CORE_0 = 0,  // Critical tasks
    CORE_1 = 1,  // Non-critical tasks
    ANY_CORE = tskNO_AFFINITY
};

struct TaskConfig {
    const char* name;
    uint32_t stackSize;
    TaskPriority priority;
    TaskCore coreId;
    bool shouldRun;
    
    TaskConfig(const char* n = "Task", uint32_t stack = 2048, 
               TaskPriority prio = TaskPriority::TASK_MEDIUM, 
               TaskCore core = TaskCore::ANY_CORE, bool run = true)
        : name(n), stackSize(stack), priority(prio), coreId(core), shouldRun(run) {}
};

class ITask {
public:
    virtual ~ITask() = default;
    
    virtual bool start(const TaskConfig& config) = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
    virtual const char* getName() const = 0;
    virtual uint32_t getStackHighWaterMark() const = 0;
    
protected:
    virtual void taskFunction() = 0;
    static void taskWrapper(void* parameter);
};