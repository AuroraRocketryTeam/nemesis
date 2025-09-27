#include "SensorTask.hpp"
#include "esp_task_wdt.h"

SensorTask::SensorTask(std::shared_ptr<SharedSensorData> sensorData, SemaphoreHandle_t mutex)
    : BaseTask("SensorTask"), sensorData(sensorData), dataMutex(mutex),
      bno055(nullptr), baro1(nullptr), baro2(nullptr) {
}

void SensorTask::setSensors(ISensor* imu, ISensor* barometer1, ISensor* barometer2) {
    bno055 = imu;
    baro1 = barometer1;
    baro2 = barometer2;
}

void SensorTask::onTaskStart() {
    Serial.printf("[SENSOR] Task started with stack: %u bytes\n", config.stackSize);
}

void SensorTask::onTaskStop() {
    Serial.println("[SENSOR] Task stopped");
}

void SensorTask::taskFunction()
{
    esp_task_wdt_init(20, true);
    esp_task_wdt_add(NULL);
    uint32_t loopCount = 0;

    while (running)
    {
        esp_task_wdt_reset(); // Reset the watchdog timer

        // IMU reading
        if (imu)
        {
            unsigned long startTime = millis();
            auto data = imu->getData();
            unsigned long readTime = millis() - startTime;

            // Only log every few iterations to reduce output volume
            bool detailedLog = (loopCount % 10 == 0);

            if (detailedLog)
            {
                LOG_DEBUG("Sensor", "IMU data read (took %lu ms)", readTime);
            }

            if (data.has_value())
            {
                if (detailedLog)
                {
                    LOG_DEBUG("Sensor", "Sensor data read");
                }

                // Store the data (always do this, regardless of logging)
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->imuData = data.value();
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                LOG_WARNING("Sensor", "No IMU data available");
            }
        }

        esp_task_wdt_reset();

        // Barometer 1 reading
        if (baro1) {
            auto data = baro1->getData();
            if (data.has_value())
            {
                if (loopCount % 10 == 0)
                {
                    LOG_DEBUG("Sensor", "Sensor data read");
                }

                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->baroData1 = data.value();
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                LOG_WARNING("Sensor", "No Baro1 data available");
            }
        }

        esp_task_wdt_reset();

        // Barometer 2 reading  
        if (baro2) {
            auto data = baro2->getData();
            if (data.has_value())
            {
                if (loopCount % 10 == 0)
                {
                    LOG_DEBUG("Sensor", "Sensor data read");
                }

                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->baroData2 = data.value();
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                LOG_WARNING("Sensor", "No Baro2 data available");
            }
        }

        esp_task_wdt_reset();

        // Only log stack usage periodically
        if (loopCount % 50 == 0)
        {
            LOG_INFO("Sensor", "Stack HWM: %u bytes", uxTaskGetStackHighWaterMark(taskHandle));
        }

        loopCount++;
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}