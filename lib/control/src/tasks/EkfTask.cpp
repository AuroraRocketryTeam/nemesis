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

        SensorData imuCopy("imu");
        SensorData baro1Copy("baro1");
        SensorData baro2Copy("baro2");
        SensorData gpsCopy("gps");
        uint32_t dataTimeStamp = 0;

        // Try to take the mutex quickly; if unavailable, skip this cycle to avoid deadlock
        if (xSemaphoreTake(sensorDataMutex, mutexTimeout) == pdTRUE)
        {
            imuCopy = sensorData->imuData;
            baro1Copy = sensorData->baroData1;
            baro2Copy = sensorData->baroData2;
            gpsCopy = sensorData->gpsData;
            dataTimeStamp = sensorData->timestamp;

            xSemaphoreGive(sensorDataMutex);
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

        bool validInputs = true;

        auto orientOpt = imuCopy.getData("orientation");
        if (orientOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(orientOpt.value()))
        {
            const auto &orientationMap = std::get<std::map<std::string, float>>(orientOpt.value());
            omega[0] = orientationMap.count("x") ? orientationMap.at("x") : 0.0f;
            omega[1] = orientationMap.count("y") ? orientationMap.at("y") : 0.0f;
            omega[2] = orientationMap.count("z") ? orientationMap.at("z") : 0.0f;
        }
        else
        {
            validInputs = false;
        }

        auto accelOpt = imuCopy.getData("accelerometer");
        if (accelOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(accelOpt.value()))
        {
            const auto &accelMap = std::get<std::map<std::string, float>>(accelOpt.value());
            accel[0] = accelMap.count("x") ? accelMap.at("x") : 0.0f;
            accel[1] = accelMap.count("y") ? accelMap.at("y") : 0.0f;
            accel[2] = accelMap.count("z") ? accelMap.at("z") : 0.0f;
        }
        else
        {
            validInputs = false;
        }

        auto b1Opt = baro1Copy.getData("pressure");
        if (b1Opt.has_value() && std::holds_alternative<float>(b1Opt.value()))
        {
            baro1Pressure = std::get<float>(b1Opt.value());
        }
        else
        {
            validInputs = false;
        }

        auto b2Opt = baro2Copy.getData("pressure");
        if (b2Opt.has_value() && std::holds_alternative<float>(b2Opt.value()))
        {
            baro2Pressure = std::get<float>(b2Opt.value());
        }
        else
        {
            validInputs = false;
        }

        auto gpsOpt = gpsCopy.getData("altitude");
        if (gpsOpt.has_value() && std::holds_alternative<float>(gpsOpt.value()))
        {
            gpsAltitude = std::get<float>(gpsOpt.value());
        }
        else
        {
            // GPS may be temporarily unavailable; allow EKF to run with barometer only
            gpsAltitude = 0.0f;
        }

        if (!validInputs)
        {
            // Skip this cycle rather than passing garbage to the EKF
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        if (!running) break; // Check before heavy computation

        // Compute elapsed time in seconds. On first run, set a small default dt.
        float elapsed = 0.01f;
        if (lastTimestamp != 0 && dataTimeStamp > lastTimestamp)
        {
            elapsed = static_cast<float>(dataTimeStamp - lastTimestamp) / 1000.0f;
        }

        // Run the EKF step
        kalmanFilter->step(elapsed,
                           omega,
                           accel,
                           (baro1Pressure + baro2Pressure) / 2.0f,
                           gpsAltitude);

        // Check for filter stability periodically (not every loop)
        if ((loopCounter++ & 0x0F) == 0)
        {
            float *currentState = kalmanFilter->state();
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
                  kalmanFilter->state()[0],
                  kalmanFilter->state()[1],
                  kalmanFilter->state()[2],
                  kalmanFilter->state()[3],
                  kalmanFilter->state()[4],
                  kalmanFilter->state()[5]);
        lastTimestamp = dataTimeStamp;

        // Split delay for faster exit response (4x5ms = 20ms total)
        for (int i = 0; i < 4 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}