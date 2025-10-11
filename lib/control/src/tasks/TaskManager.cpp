#include "TaskManager.hpp"
#include <config.h>

TaskManager::TaskManager(std::shared_ptr<SharedSensorData> sensorData,
                            std::shared_ptr<KalmanFilter1D> kalmanFilter,
                            std::shared_ptr<ISensor> imu,
                            std::shared_ptr<ISensor> barometer1,
                            std::shared_ptr<ISensor> barometer2,
                            std::shared_ptr<ISensor> gps,
                            SemaphoreHandle_t sensorMutex,
                            std::shared_ptr<bool> isRising,
                            std::shared_ptr<float> heightGainSpeed) : sensorData(sensorData), kalmanFilter(kalmanFilter),
                                                          bno055(imu), baro1(barometer1), baro2(barometer2), 
                                                          gps(gps), sensorDataMutex(sensorMutex), isRising(isRising),
                                                          heightGainSpeed(heightGainSpeed)
{
    LOG_INFO("TaskMgr", "Initialized with sensors: IMU=%s, Baro1=%s, Baro2=%s, GPS=%s",
             imu ? "OK" : "NULL",
             barometer1 ? "OK" : "NULL",
             barometer2 ? "OK" : "NULL",
             gps ? "OK" : "NULL");

    // Initialize ESP-NOW transmitter
    uint8_t peerMac[] = ESPNOW_PEER_MAC;
    espNowTransmitter = std::make_shared<EspNowTransmitter>(peerMac, ESPNOW_CHANNEL);

    // Initialize transmitter
    ResponseStatusContainer initResult = espNowTransmitter->init();
    if (initResult.getCode() != 0)
    {
        LOG_ERROR("TaskMgr", "Failed to initialize ESP-NOW: %s", initResult.getDescription().c_str());
    }
    else
    {
        LOG_INFO("TaskMgr", "ESP-NOW transmitter initialized successfully");
    }
}

TaskManager::~TaskManager()
{
    stopAllTasks();
    LOG_INFO("TaskManager", "Destroyed");
}

void TaskManager::initializeTasks()
{
    LOG_INFO("TaskManager", "Creating task instances...");

    // Create all task instances but don't start them yet
    // Create SensorTask with shared pointers
    tasks[TaskType::SENSOR] = std::make_unique<SensorTask>(
        sensorData,
        sensorDataMutex,
        bno055, // Pass the shared pointers
        baro1,
        baro2);
    tasks[TaskType::GPS] = std::make_unique<GpsTask>(sensorData, sensorDataMutex, gps);
    tasks[TaskType::EKF] = std::make_unique<EkfTask>(sensorData, sensorDataMutex, kalmanFilter);

    // Create TelemetryTask with ESP-NOW transmitter
    tasks[TaskType::TELEMETRY] = std::make_unique<TelemetryTask>(
        sensorData,
        sensorDataMutex,
        espNowTransmitter,
        TELEMETRY_INTERVAL_MS);

    tasks[TaskType::BAROMETER] = std::make_unique<BarometerTask>(
        sensorData,
        sensorDataMutex,
        baro1,
        baro2,
        isRising,
        heightGainSpeed);

    // tasks[TaskType::LOGGING] = std::make_unique<LoggingTask>(sensorData, sensorDataMutex);
    // tasks[TaskType::APOGEE_DETECTION] = std::make_unique<ApogeeDetectionTask>(filteredData, filteredDataMutex);
    // tasks[TaskType::RECOVERY] = std::make_unique<RecoveryTask>(sharedData.get(), dataMutex);
    // tasks[TaskType::DATA_COLLECTION] = std::make_unique<DataCollectionTask>(sharedData.get(), dataMutex);

    LOG_INFO("TaskManager", "Created %d task instances", tasks.size());
}

bool TaskManager::startTask(TaskType type, const TaskConfig &config)
{
    LOG_INFO("TaskManager", "Starting task type %d with config: %s",
             static_cast<int>(type), config.name);

    // Check if task exists
    auto it = tasks.find(type);
    if (it == tasks.end())
    {
        LOG_ERROR("TaskManager", "ERROR: Task type %d not found", static_cast<int>(type));
        return false;
    }

    // Check if already running
    if (it->second->isRunning())
    {
        LOG_WARNING("TaskManager", "WARNING: Task %s already running", config.name);
        return true; // Not an error, just already running
    }

    // Check available memory before starting
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < config.stackSize + 1024)
    { // Reserve 1KB buffer
        LOG_ERROR("TaskManager", "ERROR: Insufficient memory. Free: %u, Need: %u",
                  freeHeap, config.stackSize + 1024);
        return false;
    }

    // Start the task
    bool result = it->second->start(config);
    if (result)
    {
        LOG_INFO("TaskManager", "Successfully started task: %s", config.name);
    }
    else
    {
        LOG_ERROR("TaskManager", "FAILED to start task: %s", config.name);
    }

    return result;
}

void TaskManager::stopTask(TaskType type)
{
    if (tasks.find(type) == tasks.end())
    {
        LOG_WARNING("TaskManager", "Task type %d not found", static_cast<int>(type));
        return;
    }

    auto &task = tasks[type];
    if (task && task->isRunning())
    {
        LOG_INFO("TaskManager", "Stopping task: %s", task->getName());

        // Record pre-stop status
        LOG_DEBUG("TaskManager", "Pre-stop: %s isRunning=%d, StackHWM=%u, FreeHeap=%u",
                  task->getName(), task->isRunning(), task->getStackHighWaterMark(), ESP.getFreeHeap());

        // Request cooperative stop - BaseTask::stop() now handles waiting
        task->stop();

        // Quick verification
        if (task->isRunning())
        {
            LOG_ERROR("TaskManager", "Task %s still running after stop()", task->getName());
        }
        else
        {
            LOG_INFO("TaskManager", "Task %s stopped - FreeHeap=%u", task->getName(), ESP.getFreeHeap());
        }
    }
}

void TaskManager::stopAllTasks()
{
    LOG_INFO("TaskManager", "Stopping all tasks...");

    // Iterate over a snapshot of task types to avoid iterator invalidation during stops
    std::vector<TaskType> taskTypes;
    for (const auto &p : tasks)
        taskTypes.push_back(p.first);

    for (auto type : taskTypes)
    {
        auto it = tasks.find(type);
        if (it != tasks.end())
        {
            auto &task = it->second;
            if (task && task->isRunning())
            {
                LOG_DEBUG("TaskManager", "About to stop task type %d (%s)", static_cast<int>(type), task->getName());
                stopTask(type);
            }
        }
    }

    // Minimal settle delay since BaseTask::stop() already waits
    vTaskDelay(pdMS_TO_TICKS(50));

    LOG_INFO("TaskManager", "All tasks stopped - FreeHeap: %u bytes", ESP.getFreeHeap());
}

bool TaskManager::isTaskRunning(TaskType type) const
{
    auto it = tasks.find(type);
    return (it != tasks.end()) && it->second->isRunning();
}

uint32_t TaskManager::getTaskStackUsage(TaskType type) const
{
    auto it = tasks.find(type);
    if (it != tasks.end())
    {
        return it->second->getStackHighWaterMark();
    }
    return 0;
}

void TaskManager::printTaskStatus() const
{
    LOG_INFO("TaskManager", "\n=== TASK STATUS ===");
    LOG_INFO("TaskManager", "Free heap: %u bytes", ESP.getFreeHeap());
    LOG_INFO("TaskManager", "Task Status:");

    const char *taskNames[] = {
        "SENSOR", "EKF", "APOGEE_DETECTION", "RECOVERY",
        "DATA_COLLECTION", "TELEMETRY", "GPS", "LOGGING"};

    int index = 0;
    for (const auto &[type, task] : tasks)
    {
        if (task)
        {
            LOG_INFO("TaskManager", "  %s: %s (Stack HWM: %u)",
                     taskNames[index],
                     task->isRunning() ? "RUNNING" : "STOPPED",
                     task->getStackHighWaterMark());
        }
        index++;
    }
    LOG_INFO("TaskManager", "=================");
}