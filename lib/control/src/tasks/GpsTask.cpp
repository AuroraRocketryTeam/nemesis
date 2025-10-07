#include "GpsTask.hpp"

void GpsTask::taskFunction()
{
    const TickType_t mutexTimeout = pdMS_TO_TICKS(10);
    unsigned long loopCounter = 0;
    while (running)
    {
        esp_task_wdt_reset(); // Reset watchdog created in BaseTask
        
        // Check early for fast exit
        if (!running) break;

        if (gps)
        {
            auto gpsData = gps->getData();
            if (gpsData && running) // Check before mutex
            {
                if (dataMutex && xSemaphoreTake(dataMutex, mutexTimeout) == pdTRUE)
                {
                    sensorData->gpsData = *gpsData;
                    xSemaphoreGive(dataMutex);
                    if ((loopCounter & 0x0F) == 0)
                        LOG_INFO("GpsTask", "GPS update stored");
                }
                else
                {
                    if ((loopCounter & 0x0F) == 0)
                        LOG_WARNING("GpsTask", "Failed to take data mutex");
                }
            }
            else
            {
                // GPS fix or data might be temporarily unavailable; log sparsely
                if ((loopCounter & 0x3F) == 0)
                    LOG_WARNING("GpsTask", "No GPS data available");
            }
        }
        else
        {
            // Sensor missing - this is likely a configuration issue; log less frequently
            if ((loopCounter & 0x3F) == 0)
                LOG_WARNING("GpsTask", "No GPS sensor available");
        }

        loopCounter++;
        
        // Split 200ms delay into 20x10ms chunks for faster exit (still 5 Hz)
        for (int i = 0; i < 20 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
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