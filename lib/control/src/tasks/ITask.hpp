#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>
#include "TaskConfig.hpp"


class ITask
{
public:
    virtual ~ITask() = default;

    virtual bool start(const TaskConfig &config) = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
    virtual const char *getName() const = 0;
    virtual uint32_t getStackHighWaterMark() const = 0;

protected:
    virtual void taskFunction() = 0;
    static void taskWrapper(void *parameter);
};