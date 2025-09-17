#include "BaseTask.hpp"
#include "esp_heap_caps.h"

BaseTask::BaseTask(const char* name)
    : taskHandle(nullptr), running(false), taskName(name) {
    // Constructor
}

BaseTask::~BaseTask() {
    stop();
}

bool BaseTask::start(const TaskConfig& config) {
    if (running) return false;
    
    this->config = config;
    
    BaseType_t result = xTaskCreatePinnedToCore(
        taskWrapper,
        config.name,
        config.stackSize,
        this,
        static_cast<UBaseType_t>(config.priority),
        &taskHandle,
        static_cast<BaseType_t>(config.coreId)
    );
    
    if (result == pdPASS) {
        running = true;
        Serial.printf("[TASK] %s started on core %d\n", taskName, static_cast<int>(config.coreId));
        return true;
    }
    
    Serial.printf("[ERROR] Failed to create task %s\n", taskName);
    return false;
}
