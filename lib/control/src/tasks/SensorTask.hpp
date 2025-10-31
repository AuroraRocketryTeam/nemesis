#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "Logger.hpp"
#include "RocketLogger.hpp"
#include <Nemesis.hpp>
#include <ISensor.hpp>
#include <memory>

/**
 * @brief Class to implement a Sensor task.
 * 
 */
class SensorTask : public BaseTask
{
public:
    /**
     * @brief Construct a new Sensor Task object
     * 
     * @param model The shared pointer to the rocket model
     * @param modelMutex The semaphore handle to protect access to the model
     * @param logger The shared pointer to the RocketLogger instance
     * @param loggerMutex The semaphore handle to protect access to the logger
     */
    SensorTask(std::shared_ptr<Nemesis> model,
               SemaphoreHandle_t modelMutex,
               std::shared_ptr<RocketLogger> logger, 
               SemaphoreHandle_t loggerMutex);

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;


private:
    std::shared_ptr<Nemesis> model;
    SemaphoreHandle_t modelMutex;

    std::shared_ptr<RocketLogger> logger;
    SemaphoreHandle_t loggerMutex;
};