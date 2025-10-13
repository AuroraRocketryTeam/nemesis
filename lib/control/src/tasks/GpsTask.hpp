#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <GPS.hpp>
#include "Logger.hpp"
#include <RocketLogger.hpp>

class GpsTask : public BaseTask
{
public:
    GpsTask(std::shared_ptr<SharedSensorData> sensorData,
            SemaphoreHandle_t sensorDataMutex,
            std::shared_ptr<ISensor> gps,
            std::shared_ptr<RocketLogger> rocketLogger, 
            SemaphoreHandle_t loggerMutex
        )
        : BaseTask("GpsTask"),
          sensorData(sensorData),
          dataMutex(sensorDataMutex),
          gps(gps ? gps.get() : nullptr),
          rocketLogger(rocketLogger),
          loggerMutex(loggerMutex)
    {
        LOG_INFO("GpsTask", "Initialized with GPS: %s", gps ? "OK" : "NULL");
    }

    ~GpsTask() override
    {
        stop();
    }

    void setGps(std::shared_ptr<ISensor> gps)
    {
        this->gps = gps.get();
        LOG_INFO("GpsTask", "Updated GPS: %s", gps ? "OK" : "NULL");
    }

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;
    ISensor *gps;

    std::shared_ptr<RocketLogger> rocketLogger;
    SemaphoreHandle_t loggerMutex;
};