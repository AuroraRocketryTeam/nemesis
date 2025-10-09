#include "SensorTask.hpp"
#include "esp_task_wdt.h"

SensorTask::SensorTask(std::shared_ptr<SharedSensorData> sensorData,
                       SemaphoreHandle_t mutex,
                       std::shared_ptr<ISensor> imu,
                       std::shared_ptr<ISensor> barometer1,
                       std::shared_ptr<ISensor> barometer2)
    : BaseTask("SensorTask"), sensorData(sensorData), dataMutex(mutex)
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
            LOG_INFO("Sensor", "L%lu: Stack HwM:%u, Heap=%u, Memory=%u",
                          loopCount, uxTaskGetStackHighWaterMark(NULL), ESP.getFreeHeap(),
                          heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        }
        loopCount++;
        
        // Shorter delay to exit faster (split into smaller chunks)
        for (int i = 0; i < 10 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // 10x10ms = 100ms total, but check every 10ms
        }
    }
}