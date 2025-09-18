#include "EkfTask.hpp"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

EkfTask::~EkfTask() {
    stop();
}

void EkfTask::taskFunction() {
    SensorData imuCopy = SensorData("imu");
    SensorData baro1Copy = SensorData("baro1");
    SensorData baro2Copy = SensorData("baro2");
    SensorData gpsCopy = SensorData("gps");
    uint32_t dataTimeStamp;
    
    // Create a local copy of the sensors data using the mutex
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
        imuCopy = sensorData->imuData;
        baro1Copy = sensorData->baroData1;
        baro2Copy = sensorData->baroData2;
        gpsCopy = sensorData->gpsData;
        dataTimeStamp = sensorData->timestamp;

        xSemaphoreGive(sensorDataMutex);
    }

    // Extract sensor values
    auto orientationMap = std::get<std::map<std::string, float>>(imuCopy.getData("orientation").value());
    float omega[3] = {
        orientationMap["x"],
        orientationMap["y"],
        orientationMap["z"]
    };

    auto accelMap = std::get<std::map<std::string, float>>(imuCopy.getData("accelerometer").value());
    float accel[3] = {
        accelMap["x"],
        accelMap["y"],
        accelMap["z"]
    };

    auto baro1Pressure = std::get<float>(baro1Copy.getData("pressure").value());
    auto baro2Pressure = std::get<float>(baro2Copy.getData("pressure").value());
    auto gpsAltitude = std::get<float>(gpsCopy.getData("altitude").value());

    auto elapsed = (dataTimeStamp - lastTimestamp) / 1000.0f;

    // Run the EKF step
    kalmanFilter->step(elapsed,
                      omega,
                      accel,
                      (baro1Pressure + baro2Pressure) / 2,
                      gpsAltitude);

    // Check for filter stability
    float* currentState = kalmanFilter->state();

    for (int j = 0; j < EKF_N; j++) {
        if (!isfinite(currentState[j]) || abs(currentState[j]) > 1e6) {
            // NOTIFY ERROR THROUGH LOGGER !!!
            break;
        }
    }

    lastTimestamp = dataTimeStamp;
}