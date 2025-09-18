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

void SensorTask::taskFunction() {
    while (true) {
        // IMU reading
        if (bno055) {
            auto data = bno055->getData();
            if (data.has_value() && dataMutex &&
                xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sensorData->imuData = data.value();
                xSemaphoreGive(dataMutex);
            }
        }

        // Barometer 1 reading
        if (baro1) {
            auto data = baro1->getData();
            if (data.has_value() && dataMutex &&
                xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sensorData->baroData1 = data.value();
                xSemaphoreGive(dataMutex);
            }
        }

        // Barometer 2 reading  
        if (baro2) {
            auto data = baro2->getData();
            if (data.has_value() && dataMutex &&
                xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sensorData->baroData2 = data.value();
                xSemaphoreGive(dataMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
    }
}