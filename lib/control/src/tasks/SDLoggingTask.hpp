#pragma once

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BaseTask.hpp"
#include "RocketLogger.hpp"
#include <config.h>
#include <SD-master.hpp>
#include <Logger.hpp>

/**
 * @brief Class to implement an SD logging task.
 * 
 */
class SDLoggingTask : public BaseTask {
public:
    /**
     * @brief Construct a new SDLoggingTask object
     * 
     * @param logger The shared pointer to the RocketLogger instance
     * @param loggerMutex The semaphore handle to protect access to the logger
     * @param sdCard The shared pointer to the SD card instance
     */
    SDLoggingTask(std::shared_ptr<RocketLogger> logger, 
               SemaphoreHandle_t loggerMutex,
               std::shared_ptr<SD> sdCard);
    
    ~SDLoggingTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<RocketLogger> logger;
    SemaphoreHandle_t loggerMutex;

    std::shared_ptr<SD> sdCard;
    bool sdInitialized = false;
    int file_counter = 0;

};