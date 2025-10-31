#pragma once

#include "BaseTask.hpp"
#include <Nemesis.hpp>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <GPS.hpp>
#include "Logger.hpp"
#include <RocketLogger.hpp>

class GpsTask : public BaseTask
{
public:
    GpsTask(std::shared_ptr<Nemesis> model,
            SemaphoreHandle_t modelMutex,
            std::shared_ptr<RocketLogger> logger, 
            SemaphoreHandle_t loggerMutex
        );

    ~GpsTask() override
    {
        stop();
    }

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
private:
    std::shared_ptr<Nemesis> _model;
    SemaphoreHandle_t _modelMutex;

    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _loggerMutex;
};