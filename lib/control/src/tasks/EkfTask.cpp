#include "EkfTask.hpp"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Logger.hpp>

EkfTask::~EkfTask() {
    stop();
}

void EkfTask::taskFunction() {
    const TickType_t mutexTimeout = pdMS_TO_TICKS(10);
    unsigned long loopCounter = 0;
    while (running)
    {
        esp_task_wdt_reset();
        
        // Check early exit
        if (!running) break;

        std::shared_ptr<BNO055Data> imuDataCopy = std::make_shared<BNO055Data>();
        std::shared_ptr<MS561101BA03Data> baro1DataCopy = std::make_shared<MS561101BA03Data>();
        std::shared_ptr<MS561101BA03Data> baro2DataCopy = std::make_shared<MS561101BA03Data>();
        std::shared_ptr<LIS3DHTRData> lisDataCopy = std::make_shared<LIS3DHTRData>();
        std::shared_ptr<GPSData> gpsDataCopy = std::make_shared<GPSData>();
        uint32_t dataTimeStamp = 0;

        // Try to take the mutex quickly; if unavailable, skip this cycle to avoid deadlock
        if (xSemaphoreTake(_modelMutex, mutexTimeout) == pdTRUE)
        {
            imuDataCopy = _model->getBNO055Data();
            baro1DataCopy = _model->getMS561101BA03Data_1();
            baro2DataCopy = _model->getMS561101BA03Data_2();
            lisDataCopy = _model->getLIS3DHTRData();
            gpsDataCopy = _model->getGPSData();
            dataTimeStamp = millis();

            xSemaphoreGive(_modelMutex);
        }
        else
        {
            // Couldn't get sensor snapshot this cycle; sleep a short while and try again
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        if (!running) break; // Check after mutex operation

        // Safely extract sensor values using optionals and variant checks
        float omega[3] = {0.0f, 0.0f, 0.0f};
        float accel[3] = {0.0f, 0.0f, 0.0f};
        float baro1Pressure = 0.0f;
        float baro2Pressure = 0.0f;
        float gpsAltitude = 0.0f;

        auto orientation_x = imuDataCopy->orientation_x;
        auto orientation_y = imuDataCopy->orientation_y;
        auto orientation_z = imuDataCopy->orientation_z;

        auto acceleration_x = lisDataCopy->acceleration_x;
        auto acceleration_y = lisDataCopy->acceleration_y;
        auto acceleration_z = lisDataCopy->acceleration_z;

        auto pressure1 = baro1DataCopy->pressure;
        auto pressure2 = baro2DataCopy->pressure;

        auto altitude = gpsDataCopy->altitude;

        if (!running) break; // Check before heavy computation

        // Compute elapsed time in seconds. On first run, set a small default dt.
        float elapsed = 0.01f;
        if (_lastTimestamp != 0 && dataTimeStamp > _lastTimestamp)
        {
            elapsed = static_cast<float>(dataTimeStamp - _lastTimestamp) / 1000.0f;
        }

        // Run the EKF step
        _kalmanFilter->step(elapsed,
                           omega,
                           accel,
                           (baro1Pressure + baro2Pressure) / 2.0f,
                           gpsAltitude);

        // Check for filter stability periodically (not every loop)
        if ((loopCounter++ & 0x0F) == 0)
        {
            float *currentState = _kalmanFilter->state();
            bool diverged = false;
            for (int j = 0; j < EKF_N; j++)
            {
                if (!isfinite(currentState[j]) || fabs(currentState[j]) > 1e6f)
                {
                    diverged = true;
                    break;
                }
            }
            if (diverged)
            {
                LOG_WARNING("EkfTask", "EKF diverged or produced invalid state.");
            }
        }
        LOG_DEBUG("EkfTask", "EKF State: pos=%.2f vel=%.2f q=[%.3f, %.3f, %.3f, %.3f]",
                  _kalmanFilter->state()[0],
                  _kalmanFilter->state()[1],
                  _kalmanFilter->state()[2],
                  _kalmanFilter->state()[3],
                  _kalmanFilter->state()[4],
                  _kalmanFilter->state()[5]);
        _lastTimestamp = dataTimeStamp;

        // Split delay for faster exit response (4x5ms = 20ms total)
        for (int i = 0; i < 4 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}