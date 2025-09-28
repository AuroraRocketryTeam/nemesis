#include "SensorTask.hpp"
#include "esp_task_wdt.h"

SensorTask::SensorTask(std::shared_ptr<SharedSensorData> sensorData,
                       SemaphoreHandle_t mutex,
                       std::shared_ptr<ISensor> imu,
                       std::shared_ptr<ISensor> barometer1,
                       std::shared_ptr<ISensor> barometer2)
    : BaseTask("SensorTask"), sensorData(sensorData), dataMutex(mutex),
      bno055(imu), baro1(barometer1), baro2(barometer2)
{
}

void SensorTask::setSensors(std::shared_ptr<ISensor> imu,
                            std::shared_ptr<ISensor> barometer1,
                            std::shared_ptr<ISensor> barometer2)
{
    bno055 = imu;
    baro1 = barometer1;
    baro2 = barometer2;
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
    esp_task_wdt_add(NULL);
    uint32_t loopCount = 0;
    uint32_t errorCount = 0;

    while (running)
    {
        esp_task_wdt_reset();
        bool anyErrors = false;

        // IMU reading
        if (bno055)
        {
            auto data = bno055->getData();
            if (data.has_value())
            {
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    sensorData->imuData = data.value();
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                anyErrors = true;
            }
        }

        // Barometer 1 reading
        if (baro1)
        {
            auto data = baro1->getData();
            if (data.has_value())
            {
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    sensorData->baroData1 = data.value();
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                anyErrors = true;
            }
        }

        // Barometer 2 reading
        if (baro2)
        {
            auto data = baro2->getData();
            if (data.has_value())
            {
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    sensorData->baroData2 = data.value();
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                anyErrors = true;
            }
        }

        // Count errors for health monitoring
        if (anyErrors)
        {
            errorCount++;
        }

        // Minimal status logging every 100 loops
        if (loopCount % 100 == 0)
        {
            LOG_DEBUG("Sensor", "L%lu: Errors=%lu, Stack=%u",
                      loopCount, errorCount, uxTaskGetStackHighWaterMark(taskHandle));

            // Reset error count periodically
            errorCount = 0;
        }

        loopCount++;
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz sensor reading
    }

    esp_task_wdt_delete(NULL);
    LOG_INFO("Sensor", "Task function exiting");
}