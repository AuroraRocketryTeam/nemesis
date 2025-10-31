#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "Logger.hpp"
#include "RocketLogger.hpp"
#include <Nemesis.hpp>
#include <ISensor.hpp>
#include <memory>

class SensorTask : public BaseTask
{
private:
    std::shared_ptr<Nemesis> model;
    SemaphoreHandle_t modelMutex;

    std::shared_ptr<RocketLogger> logger;
    SemaphoreHandle_t loggerMutex;

public:
    SensorTask(std::shared_ptr<Nemesis> model,
               SemaphoreHandle_t modelMutex,
               std::shared_ptr<RocketLogger> logger, 
               SemaphoreHandle_t loggerMutex);

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
};