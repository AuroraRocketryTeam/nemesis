#include "GpsTask.hpp"

void GpsTask::taskFunction()
{
    while (running)
    {
        esp_task_wdt_reset();
        if (gps)
        {
            auto gpsData = gps->getData();
            // Write to shared data
            if (gpsData.has_value())
            {
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->gpsData = gpsData.value();
                    LOG_INFO("GpsTask", "Got GPS data");
                    xSemaphoreGive(dataMutex);
                }
                else
                {
                    LOG_WARNING("GpsTask", "Failed to take data mutex");
                }
                
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    auto timestampData = SensorData("Timestamp");
                    timestampData.setData("timestamp", static_cast<int>(millis()));
                    rocketLogger->logSensorData(timestampData);
                    
                    rocketLogger->logSensorData("GPS", gpsData.value());
                    xSemaphoreGive(loggerMutex);
                } else {
                    LOG_WARNING("GpsTask", "Failed to take logger mutex");
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
        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz
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