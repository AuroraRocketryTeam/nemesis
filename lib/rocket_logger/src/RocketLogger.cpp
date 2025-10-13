#include "RocketLogger.hpp"
#include <new>

// Destructor - clean up all dynamically allocated memory
RocketLogger::~RocketLogger() {
    clearData();
}

// Override logInfo to log informational messages
void RocketLogger::logInfo(const std::string& message) {
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 5000) {
        return;
    }
    
    ILoggable* logMessage = new(std::nothrow) LogMessage("RocketLogger", message);
    if (logMessage) {
        this->logDataList.push_back(LogData("INFO", logMessage));
    }
}

// Override logWarning to log warning messages
void RocketLogger::logWarning(const std::string& message) {
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 5000) { 
        return;
    }
    
    ILoggable* logMessage = new(std::nothrow) LogMessage("RocketLogger", message);
    if (logMessage) {
        this->logDataList.push_back(LogData("WARNING", logMessage));
    }
}

// Override logError to log error messages
void RocketLogger::logError(const std::string& message) {
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 5000) { 
        return;
    }
    
    ILoggable* logMessage = new(std::nothrow) LogMessage("RocketLogger", message);
    if (logMessage) {
        this->logDataList.push_back(LogData("ERROR", logMessage));
    }
}

// Override logData to store sensor data
void RocketLogger::logSensorData(const SensorData sensorData) {
    // Prevent memory exhaustion by limiting log entries
    const size_t MAX_LOG_ENTRIES = 1000;
    
    if (logDataList.size() >= MAX_LOG_ENTRIES) {
        LOG_WARNING("RocketLogger", "Log buffer full (%zu entries), clearing oldest entries", logDataList.size());
        // Clear half the entries to avoid frequent clears
        size_t entriesToRemove = MAX_LOG_ENTRIES / 2;
        for (size_t i = 0; i < entriesToRemove && !logDataList.empty(); i++) {
            delete logDataList[i].getData();
            logDataList.erase(logDataList.begin());
        }
        LOG_INFO("RocketLogger", "Cleared %zu entries, now have %zu entries", entriesToRemove, logDataList.size());
    }
    
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 10000) {
        LOG_ERROR("RocketLogger", "Insufficient memory for logging (%u bytes free), skipping log entry", freeHeap);
        return;
    }
    
    ILoggable* logSensorData = new(std::nothrow) LogSensorData(sensorData.getSensorName(), sensorData);
    if (!logSensorData) {
        LOG_ERROR("RocketLogger", "Failed to allocate memory for LogSensorData");
        return;
    }
    
    this->logDataList.push_back(LogData("SENSOR_DATA", logSensorData));
}

void RocketLogger::logSensorData(const std::string& sensorName, const SensorData sensorData) {
    // Prevent memory exhaustion by limiting log entries
    const size_t MAX_LOG_ENTRIES = 1000;
    
    if (logDataList.size() >= MAX_LOG_ENTRIES) {
        LOG_WARNING("RocketLogger", "Log buffer full (%zu entries), clearing oldest entries", logDataList.size());
        // Clear half the entries to avoid frequent clears
        size_t entriesToRemove = MAX_LOG_ENTRIES / 2;
        for (size_t i = 0; i < entriesToRemove && !logDataList.empty(); i++) {
            delete logDataList[i].getData();
            logDataList.erase(logDataList.begin());
        }
        LOG_INFO("RocketLogger", "Cleared %zu entries, now have %zu entries", entriesToRemove, logDataList.size());
    }
    
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 10000) {
        LOG_ERROR("RocketLogger", "Insufficient memory for logging (%u bytes free), skipping log entry", freeHeap);
        return;
    }
    
    ILoggable* logSensorData = new(std::nothrow) LogSensorData(sensorName, sensorData);
    if (!logSensorData) {
        LOG_ERROR("RocketLogger", "Failed to allocate memory for LogSensorData");
        return;
    }
    
    this->logDataList.push_back(LogData("SENSOR_DATA", logSensorData));
}

// Function to get all logged sensor data as a JSON list
json RocketLogger::getJSONAll() const {
    json jsonDataList = json::array();
    for (const auto& sensorData : this->logDataList) {
        jsonDataList.push_back(sensorData.toJSON());  // Convert each sensor data to JSON
    }
    return jsonDataList;
}

// Clear logged sensor data
void RocketLogger::clearData() {
    size_t initialCount = this->getLogCount();
    
    // Print deleting n elements
    LOG_INFO("RocketLogger", "Clearing %zu log entries...", initialCount);

    // Delete all dynamically allocated LogSensorData objects
    for (auto& logData : this->logDataList) {
        delete logData.getData();
    }

    this->logDataList.clear();
    
    size_t finalCount = this->getLogCount();
    LOG_INFO("RocketLogger", "Clear complete. Before: %zu, After: %zu entries", initialCount, finalCount);
}