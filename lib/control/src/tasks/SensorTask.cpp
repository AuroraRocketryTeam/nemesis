#include "SensorTask.hpp"
#include "esp_task_wdt.h"

SensorTask::SensorTask(SharedSensorData* data, SemaphoreHandle_t* mutex)
    : BaseTask("SensorTask"), sharedData(data), dataMutex(mutex),
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
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    unsigned long loopCount = 0;
    
    while (running) {
        esp_task_wdt_reset();
        
        // Collect sensor data
        if (bno055) {
            auto data = bno055->getData();
            if (data.has_value() && dataMutex && 
                xSemaphoreTake(*dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedData->imuData = data.value();
                sharedData->timestamp = millis();
                xSemaphoreGive(*dataMutex);
            }
        }
        
        if (baro1) {
            auto data = baro1->getData();
            if (data.has_value() && dataMutex && 
                xSemaphoreTake(*dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedData->baroData1 = data.value();
                xSemaphoreGive(*dataMutex);
            }
        }
        
        if (baro2) {
            auto data = baro2->getData();
            if (data.has_value() && dataMutex && 
                xSemaphoreTake(*dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedData->baroData2 = data.value();
                xSemaphoreGive(*dataMutex);
            }
        }
        
        // Debug output every 1000 loops
        if (loopCount % 1000 == 0) {
            Serial.printf("[SENSOR] Loop %lu, Stack HWM: %u, Free heap: %u\n",
                         loopCount, getStackHighWaterMark(), ESP.getFreeHeap());
        }
        
        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}