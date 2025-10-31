#pragma once


#include "SD-master.hpp"
#include "BaseTask.hpp"
#include <Nemesis.hpp>
#include "Logger.hpp"
#include "RocketLogger.hpp"
#include <ISensor.hpp>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <Nemesis.hpp>

// If this variable is not commented, the format for old simulation of mission analysis (August 2025), 
// is expected, otherwise the one obtained for Euroc 2025 is expected
//#define OLD_DATA

/**
 * @brief Class to implement a simulation task.
 * 
 */
class SimulationTask : public BaseTask {
public:
    /**
     * @brief Construct a new Simulation Task object
     * 
     * @param csvFilePath The path to the CSV file containing the simulation data
     * @param model The shared pointer to the rocket model
     * @param modelMutex The semaphore handle to protect access to the model
     * @param logger The shared pointer to the RocketLogger instance
     * @param loggerMutex The semaphore handle to protect access to the logger
     */
    SimulationTask(
        const std::string& csvFilePath,
        std::shared_ptr<Nemesis> model,
        SemaphoreHandle_t modelMutex,
        std::shared_ptr<RocketLogger> logger,
        SemaphoreHandle_t loggerMutex);

    /**
     * @brief Destroy the Simulation Task object
     * 
     */
    ~SimulationTask();
    
    void onTaskStart() override;
    void onTaskStop() override;
    void taskFunction() override;
    
    /**
     * @brief Reset the simulation task to its initial state
     * 
     */
    void reset();

private:
    bool _started = false;

    // Shared static variables for simulation state
    static SD _sdManager;
    static std::string _csvFilePath;
    static uint32_t _filePosition;
    static bool _fileInitialized;
    // Used to calculate the elapsed time from the first task start
    static unsigned long _startTime;
    // The first time it'll skip the first line (header)
    static bool _firstTime;

    std::shared_ptr<Nemesis> _model;
    SemaphoreHandle_t _modelMutex;
    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _loggerMutex;
    
};