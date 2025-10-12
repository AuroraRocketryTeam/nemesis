#include "SensorTask.hpp"
#include "esp_task_wdt.h"

SensorTask::SensorTask(std::shared_ptr<SharedSensorData> sensorData,
                       SemaphoreHandle_t mutex,
                       std::shared_ptr<ISensor> imu,
                       std::shared_ptr<ISensor> barometer1,
                       std::shared_ptr<ISensor> barometer2,
                       std::shared_ptr<RocketLogger> rocketLogger, 
                       SemaphoreHandle_t loggerMutex)
    : BaseTask("SensorTask"), 
    sensorData(sensorData), dataMutex(mutex),
    rocketLogger(rocketLogger), loggerMutex(loggerMutex)
{
    bno055 = imu.get();
    baro1 = barometer1.get();
    baro2 = barometer2.get();

    LOG_INFO("Sensor", "Raw pointers: IMU=%p, B1=%p, B2=%p",
             static_cast<void *>(bno055),
             static_cast<void *>(baro1),
             static_cast<void *>(baro2));
}

void SensorTask::setSensors(std::shared_ptr<ISensor> imu,
                            std::shared_ptr<ISensor> barometer1,
                            std::shared_ptr<ISensor> barometer2)
{
    bno055 = imu.get();
    baro1 = barometer1.get();
    baro2 = barometer2.get();
}

void SensorTask::onTaskStart()
{
    LOG_INFO("Sensor", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Sensor", "Sensors: IMU=%s, Baro1=%s, Baro2=%s",
             bno055 ? "OK" : "NULL",
             baro1 ? "OK" : "NULL",
             baro2 ? "OK" : "NULL");
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
        
        if (bno055)
        {
            auto imuData = bno055->getData();
            if (imuData && running) // Check before mutex
            {
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->imuData = *imuData;
                    LOG_DEBUG("Sensor", "Read IMU data");
                    xSemaphoreGive(dataMutex);
                }
                else
                {
                    LOG_WARNING("Sensor", "Failed to take data mutex for IMU");
                }
            }
        }
        
        if (!running) break; // Check between sensors
        
        if (baro1)
        {
            auto baroData1 = baro1->getData();
            if (baroData1 && running)
            {
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->baroData1 = *baroData1;
                    LOG_DEBUG("Sensor", "Read Baro1 data");
                    LOG_DEBUG("Sensor", "Baro1 pressure: %.2f hPa", std::get<float>(baroData1->getData("pressure").value()));
                    xSemaphoreGive(dataMutex);
                }
                else
                {
                    LOG_WARNING("Sensor", "Failed to take data mutex for Baro1");
                }
            }
        }
        
        if (!running) break;
            if (baro2)
            {
            auto baroData2 = baro2->getData();
                if (baroData2 && running)
                {
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        sensorData->baroData2 = *baroData2;
                        LOG_DEBUG("Sensor", "Read Baro2 data");
                        xSemaphoreGive(dataMutex);
                    }
                    else
                    {
                    LOG_WARNING("Sensor", "Failed to take data mutex for Baro2");
                }
           }
        }
        
        if (!running) break;
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            sensorData->timestamp = esp_timer_get_time();
            LOG_DEBUG("Sensor", "Updated timestamp");
            xSemaphoreGive(dataMutex);
        }

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

        // Adding the data to the RocketLogger - REDUCED FREQUENCY TO PREVENT MEMORY EXHAUSTION
        // Only log every 50 loops (every ~5 seconds) instead of every loop
        if (loopCount % 50 == 0 && xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            auto timestampData = SensorData("Timestamp");
            timestampData.setData("timestamp", static_cast<int>(millis()));
            rocketLogger->logSensorData(timestampData);

            // Create a copy of sensorData under mutex to avoid pointer issues
            auto imuCopy = sensorData->imuData;
            auto baro1Copy = sensorData->baroData1;
            auto baro2Copy = sensorData->baroData2;
            
            rocketLogger->logSensorData(imuCopy);
            rocketLogger->logSensorData(baro1Copy);
            rocketLogger->logSensorData(baro2Copy);

            xSemaphoreGive(loggerMutex);
            
            // Log current RocketLogger memory usage for monitoring
            LOG_INFO("Sensor", "RocketLogger entries: %d", rocketLogger->getLogCount());
        }

        loopCount++;
        
        // Shorter delay to exit faster (split into smaller chunks)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}