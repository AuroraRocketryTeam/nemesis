#pragma once


#include "SD-master.hpp"
#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "Logger.hpp"
#include "RocketLogger.hpp"
#include <ISensor.hpp>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

// If this variable is not commented, the format for old simulation of mission analysis (August 2025), 
// is expected, otherwise the one obtained for Euroc 2025 is expected
//#define OLD_DATA

class SimulationTask : public BaseTask {
private:
    bool started = false;

    // Shared static variables for simulation state
    static SD sdManager;
    static std::string csvFilePath;
    static uint32_t filePosition;
    static bool fileInitialized;
    // Used to calculate the elapsed time from the first task start
    static unsigned long startTime;
    // The first time it'll skip the first line (header)
    static bool firstTime;

    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;
    std::shared_ptr<RocketLogger> rocketLogger;
    SemaphoreHandle_t loggerMutex;

public:
    SimulationTask(
        const std::string& csvFilePath,
        std::shared_ptr<SharedSensorData> sensorData,
        SemaphoreHandle_t mutex,
        std::shared_ptr<RocketLogger> rocketLogger,
        SemaphoreHandle_t loggerMutex);
    ~SimulationTask();
    void onTaskStart() override;
    void onTaskStop() override;
    void taskFunction() override;
    void reset();
    
};