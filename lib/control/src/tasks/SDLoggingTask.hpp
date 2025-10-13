#pragma once

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BaseTask.hpp"
#include "RocketLogger.hpp"
#include <config.h>
#include <SD-master.hpp>
#include <Logger.hpp>

class SDLoggingTask : public BaseTask {
public:
    SDLoggingTask(std::shared_ptr<RocketLogger> rocketLogger = nullptr, 
               SemaphoreHandle_t loggerMutex = nullptr,
               std::shared_ptr<SD> sdCard = nullptr);
    
        ~SDLoggingTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<RocketLogger> rocketLogger;
    SemaphoreHandle_t loggerMutex;

    std::shared_ptr<SD> sdCard;
    bool sdInitialized = false;
    int file_counter = 0;

};