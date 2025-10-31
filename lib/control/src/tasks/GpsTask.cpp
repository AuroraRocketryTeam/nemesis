#include "GpsTask.hpp"

GpsTask::GpsTask(std::shared_ptr<Nemesis> model,
            SemaphoreHandle_t modelMutex,
            std::shared_ptr<RocketLogger> logger, 
            SemaphoreHandle_t loggerMutex
        )
        : BaseTask("GpsTask"),
          _model(model),
          _modelMutex(modelMutex),
          _logger(logger),
          _loggerMutex(loggerMutex)
    {
        LOG_INFO("GpsTask", "Initialized GPS task");
    }

void GpsTask::taskFunction()
{
    const TickType_t mutexTimeout = pdMS_TO_TICKS(10);
    unsigned long loopCounter = 0;
    while (running)
    {
        esp_task_wdt_reset();

        _model->updateGPS();
        std::shared_ptr<GPSData> gpsData = nullptr;
        
        if (_modelMutex && xSemaphoreTake(_modelMutex, mutexTimeout) == pdTRUE)
        {
            gpsData = _model->getGPSData();

            LOG_INFO("GpsTask", "Got GPS data");
            xSemaphoreGive(_modelMutex);
        }
        else
        {
            if ((loopCounter & 0x0F) == 0)
                LOG_WARNING("GpsTask", "Failed to take data mutex");
        }
        
        if (xSemaphoreTake(_loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Only log GPS data every 10 loops (every ~2 seconds) to reduce memory pressure
            if ((loopCounter % 10) == 0) {

                _logger->logSensorData(gpsData);

                // Log current RocketLogger memory usage for monitoring
                if ((loopCounter % 50) == 0) {
                    LOG_INFO("GpsTask", "RocketLogger entries: %d", _logger->getLogCount());
                }
            }
            xSemaphoreGive(_loggerMutex);
        } else {
            LOG_WARNING("GpsTask", "Failed to take logger mutex");
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
}

void GpsTask::onTaskStop()
{
    LOG_INFO("GpsTask", "Task stopped");
}