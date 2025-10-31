#include "SensorTask.hpp"
#include "esp_task_wdt.h"

SensorTask::SensorTask(std::shared_ptr<Nemesis> model,
                       SemaphoreHandle_t modelMutex,
                       std::shared_ptr<RocketLogger> logger, 
                       SemaphoreHandle_t loggerMutex)
    : BaseTask("SensorTask"), 
      model(model), 
      modelMutex(modelMutex),
      logger(logger), 
      loggerMutex(loggerMutex)
{
    LOG_INFO("Sensor", "SensorTask constructor initialized");
}

void SensorTask::onTaskStart()
{
    LOG_INFO("Sensor", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Sensor", "Model pointer: %s", model ? "OK" : "NULL");
}

void SensorTask::onTaskStop()
{
    LOG_INFO("Sensor", "Task stopped");
}

void SensorTask::taskFunction()
{
    uint32_t loopCount = 0;

    while (running)
    {
        // CRITICAL: Reset the watchdog every loop (watchdog created in BaseTask)
        esp_task_wdt_reset();
        LOG_INFO("SensorTask", "READING SENSORS");
        
        // Check running flag early to exit quickly during shutdown
        if (!running) break;
        
        // Update sensors through the model with mutex protection
        if (model && xSemaphoreTake(modelMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            model->updateBNO055();
            model->updateMS561101BA03_1();
            model->updateMS561101BA03_2();
            model->updateLIS3DHTR();
            
            xSemaphoreGive(modelMutex);
            LOG_DEBUG("Sensor", "Updated all sensors");
        }
        else
        {
            LOG_WARNING("Sensor", "Failed to take model mutex");
        }
        
        if (!running) break;

        // Log memory usage every 10 loops
        if (loopCount % 10 == 0)
        {
            uint32_t freeHeap = ESP.getFreeHeap();
            LOG_INFO("Sensor", "L%lu: Stack HwM:%u, Heap=%u, Memory=%u",
                          loopCount, uxTaskGetStackHighWaterMark(NULL), freeHeap,
                          heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                          
            // Check for low memory condition
            if (freeHeap < 50000) { // Warning threshold
                LOG_WARNING("Sensor", "LOW MEMORY WARNING: Only %u bytes free heap remaining!", freeHeap);
            }
        }

        // Log sensor data every 3 loops if logger is available
        if (logger && loopCount % 3 == 0 && xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // Log sensor data through the model
            if (model)
            {
                auto bnoData = model->getBNO055Data();
                auto ms56Data1 = model->getMS561101BA03Data_1();
                auto ms56Data2 = model->getMS561101BA03Data_2();
                auto lis3dhData = model->getLIS3DHTRData();
                
                if (bnoData) {
                    logger->logSensorData(bnoData);
                }
                if (ms56Data1) {
                    logger->logSensorData(ms56Data1);
                }
                if (ms56Data2) {
                    logger->logSensorData(ms56Data2);
                }
                if (lis3dhData) {
                    logger->logSensorData(lis3dhData);
                }
            }
            
            xSemaphoreGive(loggerMutex);
            
            // Log current RocketLogger memory usage for monitoring
            LOG_INFO("Sensor", "RocketLogger entries logged");
        }

        loopCount++;
        
        // Shorter delay to exit faster (split into smaller chunks)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}