#include "GpsTask.hpp"

GpsTask::~GpsTask()
{
    stop();
}

void GpsTask::taskFunction()
{
    esp_task_wdt_add(NULL);

    while (running)
    {
        esp_task_wdt_reset();
        if (gps)
        {
            auto gpsData = gps->getData();
            // Write to shared data
            if (gpsData)
            {
                LOG_INFO("GpsTask", "Got GPS data");
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    sensorData->gpsData = *gpsData;
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                LOG_WARNING("GpsTask", "No GPS data available");
            }
        }
        else
        {
            LOG_WARNING("GpsTask", "No GPS sensor available");
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }

    esp_task_wdt_delete(NULL);
    LOG_INFO("GpsTask", "Task exiting");
}

void GpsTask::onTaskStart()
{
    LOG_INFO("GpsTask", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("GpsTask", "GPS: %s", gps ? "OK" : "NULL");
}

void GpsTask::onTaskStop()
{
    LOG_INFO("GpsTask", "Task stopped");
}