#include "SDLoggingTask.hpp"
#include <stdexcept>
#include "esp_task_wdt.h"

SDLoggingTask::SDLoggingTask(std::shared_ptr<RocketLogger> rocketLogger, 
                               SemaphoreHandle_t loggerMutex,
                               std::shared_ptr<SD> sdCard)
    : BaseTask("SDLoggingTask"),
      rocketLogger(rocketLogger),
      loggerMutex(loggerMutex),
      sdCard(sdCard)
{
    sdInitialized = static_cast<bool>(sdCard);
    
    if (!sdInitialized) {
        LOG_INFO("SDLoggingTask", "SD card Failed!");
        if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            //rocketLogger->logError("SD card initialization failed!");
            xSemaphoreGive(loggerMutex);
        } else {
            LOG_ERROR("SDLoggingTask", "Failed to acquire mutex for logging SD error");
        }
    } else {
        LOG_INFO("SDLoggingTask", "SD card initialized successfully.");
        if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            //rocketLogger->logInfo("SD card initialized successfully.");
            xSemaphoreGive(loggerMutex);
        } else {
            LOG_ERROR("SDLoggingTask", "Failed to acquire mutex for logging SD success");
        }
    }
}

SDLoggingTask::~SDLoggingTask() {
    stop();
}

void SDLoggingTask::taskFunction() {
    LOG_INFO("SDLoggingTask", "Task started, batch size: %d", BATCH_SIZE);
    
    while (running) {
        esp_task_wdt_reset();
        
        // Early exit check
        if (!running) break;
        
        size_t currentLogCount = rocketLogger->getLogCount();
        
        if (currentLogCount >= BATCH_SIZE) {
            LOG_INFO("SDLoggingTask", "Batch size reached (%zu >= %d), processing...", currentLogCount, BATCH_SIZE);
            
            if (sdInitialized && running) { // Check running before SD operations
                
                // Create a unique filename for this batch, checking if file already exists
                // note: the check should not be useful, but if the system somehow restarts, the check could prevent overwriting
                LOG_INFO("SDCard", "Creating unique filename for batch...");
                String filename;
                int currentFileCounter = file_counter;
                
                // Add task handle to filename to prevent conflicts between multiple instances
                do {
                    LOG_INFO("SDCard", "Checking filename: JSON_data_%d_%p.json", currentFileCounter, (void*)xTaskGetCurrentTaskHandle());
                    filename = "JSON_data_" + String(currentFileCounter) + ".json";
                    currentFileCounter++;
                    LOG_INFO("SDCard", "Filename to check: %s", filename.c_str());
                } while (sdCard->fileExists(filename.c_str()) && running);
                
                if (!running) break; // Exit if task stopped during filename generation
                
                LOG_INFO("SDCard", "Unique filename determined: %s", filename.c_str());
                // Update file_counter to the next available number
                file_counter = currentFileCounter;

                std::string dataToWrite = "";
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Convert the entire batch of JSON data to a string
                    try {
                        dataToWrite = rocketLogger->getJSONAll().dump();
                        rocketLogger->clearData();
                    } catch (const std::exception& e) {
                        LOG_ERROR("SDLoggingTask", "JSON serialization failed: %s", e.what());
                        xSemaphoreGive(loggerMutex);
                        continue; // Skip this batch
                    }
                    xSemaphoreGive(loggerMutex);
                } else {
                    LOG_ERROR("SDLoggingTask", "Failed to acquire logger mutex for SD write");
                    continue; // Skip this iteration if we can't get the mutex
                }
                
                if (!running) break; // Exit if task stopped during data preparation
                
                LOG_INFO("SDLoggingTask", "Data to write size: %zu bytes", dataToWrite.length());

                // Write the string to the SD card with error handling
                try {
                    sdCard->openFile(filename.c_str());
                    LOG_INFO("SDLoggingTask", "File opened: %s", filename.c_str());
                    
                    if (!sdCard->writeFile(filename.c_str(), dataToWrite)) {
                        LOG_ERROR("SDLoggingTask", "Failed to write batch to file: %s", filename.c_str());
                    } else {
                        LOG_INFO("SDLoggingTask", "Successfully wrote batch to file: %s (size: %zu bytes)", filename.c_str(), dataToWrite.length());
                    }
                    
                    LOG_INFO("SDLoggingTask", "Closing file...");
                    sdCard->closeFile();
                } catch (const std::exception& e) {
                    LOG_ERROR("SDLoggingTask", "SD card operation failed: %s", e.what());
                    try {
                        sdCard->closeFile(); // Attempt to close file on error
                    } catch (...) {
                        // Ignore close errors
                    }
                }
            } else if (!sdInitialized) {
                // SD not available, just clear the data to prevent memory buildup
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    rocketLogger->clearData();
                    xSemaphoreGive(loggerMutex);
                } else {
                    LOG_ERROR("SDLoggingTask", "Failed to acquire logger mutex for clearing data (SD not initialized)");
                }
            }
        } else {
            LOG_INFO("SDLoggingTask", "Current log count (%zu) below batch size (%d), waiting...", currentLogCount, BATCH_SIZE);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    LOG_INFO("SDLoggingTask", "Task exiting");
}