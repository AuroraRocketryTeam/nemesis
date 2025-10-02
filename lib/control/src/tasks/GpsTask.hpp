#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <GPS.hpp>
#include "Logger.hpp"

class GpsTask : public BaseTask
{
public:
    GpsTask(std::shared_ptr<SharedSensorData> sensorData,
            SemaphoreHandle_t sensorDataMutex, std::shared_ptr<ISensor> gps) : BaseTask("GpsTask"),
                                                                               sensorData(sensorData),
                                                                               dataMutex(sensorDataMutex),
                                                                               gps(gps)
    {
        LOG_INFO("GpsTask", "Initialized with GPS: %s", gps ? "OK" : "NULL");
        
    }
    ~GpsTask() override;

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;
    std::shared_ptr<ISensor> gps;
};