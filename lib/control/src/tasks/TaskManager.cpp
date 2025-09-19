#include "TaskManager.hpp"
#include "SensorTask.hpp"
#include "TelemetryTask.hpp"
#include "LoggingTask.hpp"
#include "EkfTask.hpp"
// #include "tasks/GpsTask.hpp"
// ... include other specific task headers

TaskManager::TaskManager(std::shared_ptr<SharedSensorData> data, SemaphoreHandle_t *mutex)
    : sharedData(data), dataMutex(mutex)
{
    Serial.println("[TaskManager] Initialized");
}

TaskManager::~TaskManager()
{
    stopAllTasks();
    Serial.println("[TaskManager] Destroyed");
}

void TaskManager::initializeTasks()
{
    Serial.println("[TaskManager] Creating task instances...");

    // Create all task instances but don't start them yet
    tasks[TaskType::SENSOR] = std::make_unique<SensorTask>(sharedData.get(), dataMutex);
    tasks[TaskType::TELEMETRY] = std::make_unique<TelemetryTask>(sharedData.get(), dataMutex);
    tasks[TaskType::LOGGING] = std::make_unique<LoggingTask>(sharedData.get(), dataMutex);
    tasks[TaskType::EKF] = std::make_unique<EkfTask>(sharedData.get(), dataMutex);

    Serial.printf("[TaskManager] Created %d task instances\n", tasks.size());
}

bool TaskManager::startTask(TaskType type, const TaskConfig &config)
{
    Serial.printf("[TaskManager] Starting task type %d with config: %s\n",
                  static_cast<int>(type), config.name);

    // Check if task exists
    auto it = tasks.find(type);
    if (it == tasks.end())
    {
        Serial.printf("[TaskManager] ERROR: Task type %d not found\n", static_cast<int>(type));
        return false;
    }

    // Check if already running
    if (it->second->isRunning())
    {
        Serial.printf("[TaskManager] WARNING: Task %s already running\n", config.name);
        return true; // Not an error, just already running
    }

    // Check available memory before starting
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < config.stackSize + 1024)
    { // Reserve 1KB buffer
        Serial.printf("[TaskManager] ERROR: Insufficient memory. Free: %u, Need: %u\n",
                      freeHeap, config.stackSize + 1024);
        return false;
    }

    // Start the task
    bool result = it->second->start(config);
    if (result)
    {
        Serial.printf("[TaskManager] Successfully started task: %s\n", config.name);
    }
    else
    {
        Serial.printf("[TaskManager] FAILED to start task: %s\n", config.name);
    }

    return result;
}

void TaskManager::stopTask(TaskType type)
{
    auto it = tasks.find(type);
    if (it != tasks.end() && it->second->isRunning())
    {
        Serial.printf("[TaskManager] Stopping task: %s\n", it->second->getName());
        it->second->stop();

        // Add delay to ensure FreeRTOS has time to clean up task resources
        // This is especially important when switching cores
        vTaskDelay(pdMS_TO_TICKS(50));

        // Optional: Second verification that task is fully stopped
        for (int i = 0; i < 5 && it->second->isRunning(); i++)
        {
            Serial.printf("[TaskManager] Waiting for %s to fully stop...\n", it->second->getName());
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void TaskManager::stopAllTasks()
{
    Serial.println("[TaskManager] Stopping all tasks...");

    // First pass: Request all tasks to stop
    for (auto &[type, task] : tasks)
    {
        if (task && task->isRunning())
        {
            Serial.printf("[TaskManager] Stopping task: %s\n", task->getName());
            task->stop();
        }
    }

    // Wait a bit for tasks to finish initial shutdown
    vTaskDelay(pdMS_TO_TICKS(100));

    // Second pass: Verify all tasks have stopped with extra wait time if needed
    bool allTasksStopped = false;
    for (int attempt = 0; attempt < 5 && !allTasksStopped; attempt++)
    {
        allTasksStopped = true;

        for (auto &[type, task] : tasks)
        {
            if (task && task->isRunning())
            {
                allTasksStopped = false;
                Serial.printf("[TaskManager] Still waiting for %s to stop (attempt %d)...\n",
                              task->getName(), attempt + 1);
            }
        }

        if (!allTasksStopped)
        {
            // Additional wait time for stubborn tasks
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    // Final verification
    for (auto &[type, task] : tasks)
    {
        if (task && task->isRunning())
        {
            Serial.printf("[TaskManager] WARNING: %s failed to stop properly\n", task->getName());
        }
    }

    Serial.println("[TaskManager] All tasks stopped");
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
    Serial.println("\n=== TASK STATUS ===");
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.println("Task Status:");

    const char *taskNames[] = {
        "SENSOR", "EKF", "APOGEE_DETECTION", "RECOVERY",
        "DATA_COLLECTION", "TELEMETRY", "GPS", "LOGGING"};

    int index = 0;
    for (const auto &[type, task] : tasks)
    {
        if (task)
        {
            Serial.printf("  %s: %s (Stack HWM: %u)\n",
                          taskNames[index],
                          task->isRunning() ? "RUNNING" : "STOPPED",
                          task->getStackHighWaterMark());
        }
        index++;
    }
    Serial.println("==================\n");
}