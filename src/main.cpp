#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <variant>
#include <config.h>
#include <pins.h>

// Include your interfaces and implementations
#include <ISensor.hpp>
#include <ILogger.hpp>
#include <ITransmitter.hpp>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <MS561101BA03.hpp>
// #include <GPSSensor.hpp> // Assuming you have this
#include <RocketLogger.hpp>
#include <E220LoRaTransmitter.hpp>
#include <KalmanFilter1D.hpp>
#include <RocketFSM.hpp>
#include <Logger.hpp> // Make sure this path is correct

#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"

// Add global variables for health monitoring
static uint32_t lastFreeHeap = 0;
static uint32_t maxHeapUsed = 0;
static unsigned long systemStartTime = 0;
static uint32_t crashCounter = 0;

// Type definitions
using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// Global component instances - these match the extern declarations in RocketFSM.cpp
ISensor *bno055 = nullptr;
ISensor *baro1 = nullptr;
ISensor *baro2 = nullptr;
ISensor *gps = nullptr;
ILogger *rocketLogger = nullptr;
ITransmitter<TransmitDataType> *loraTransmitter = nullptr;
KalmanFilter1D *ekf = nullptr;

// Hardware serial for LoRa
HardwareSerial loraSerial(LORA_SERIAL);

// FSM instance
RocketFSM rocketFSM;

// Utility functions
void tcaSelect(uint8_t bus);
void initializeComponents();
void printSystemInfo();
void createHealthMonitorTask();


void testFSMTransitions(RocketFSM &fsm)
{
    Serial.println("\n\n=== STARTING AUTOMATED FSM TEST ===");
    Serial.println("Testing all state transitions with automatic timeouts");
    Serial.println("Will cycle through all states, observing task execution\n");

    // Il test inizia automaticamente quando fsm.start() viene chiamato
    // Le transizioni sono tutte temporizzate nei metodi is*() modificati

    // Rendi i log più visibili
    fsm.start();

    // Use FreeRTOS timing for more reliable 1-second intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second
    
    RocketState lastLoggedState = RocketState::INACTIVE;
    unsigned long testStartTime = millis();
    const unsigned long MAX_TEST_DURATION = 300000; // 5 minutes max
    
    while (millis() - testStartTime < MAX_TEST_DURATION)
    {
        RocketState currentState = fsm.getCurrentState();
        
        // Only log when state changes or every 10 seconds
        static unsigned long lastPeriodicLog = 0;
        bool stateChanged = (currentState != lastLoggedState);
        bool periodicLog = (millis() - lastPeriodicLog > 10000);
        
        if (stateChanged || periodicLog)
        {
            LOG_INFO("Test", "State: %s (runtime: %lu ms, uptime: %.1f min)",
                     fsm.getStateString(currentState).c_str(), 
                     millis(),
                     (millis() - testStartTime) / 60000.0);
            
            if (periodicLog)
                lastPeriodicLog = millis();
            
            lastLoggedState = currentState;
        }

        // Termina il test quando raggiungiamo lo stato RECOVERED
        if (currentState == RocketState::RECOVERED)
        {
            LOG_INFO("Test", "=== FSM TEST COMPLETED SUCCESSFULLY ===");
            LOG_INFO("Test", "All states were visited in the correct order!");
            LOG_INFO("Test", "Total test duration: %.1f minutes", (millis() - testStartTime) / 60000.0);
            while(true) vTaskDelay(pdMS_TO_TICKS(1000)); // Halt here
        }

        // Use FreeRTOS delay for precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    // Check for timeout
    if (millis() - testStartTime >= MAX_TEST_DURATION)
    {
        LOG_WARNING("Test", "FSM test timed out after 5 minutes in state: %s", 
                    fsm.getStateString(fsm.getCurrentState()).c_str());
    }
}

void setup()
{
    // Initialize basic hardware
    Serial.begin(115200);
    while (!Serial && millis() < 5000)
    {
        delay(10); // Wait for serial or timeout after 5 seconds
    }

    Serial.println("\n=== Aurora Rocketry Flight Software ===");
    Serial.println("Initializing system...");

    // Initialize pins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // Signal initialization start
    digitalWrite(LED_RED, HIGH);

    // Initialize I2C
    Wire.begin();
    Serial.println("I2C initialized");

    // Initialize components
    initializeComponents();

    // Print system information
    printSystemInfo();

    // Initialize and start FSM
    Serial.println("\n=== Initializing Flight State Machine ===");
    rocketFSM.init();

    // Give system a moment to stabilize
    delay(1000);
    testFSMTransitions(rocketFSM);
    delay(1000);
    // Start FSM tasks
    rocketFSM.start();

    // Signal successful initialization
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);

    LOG_INFO("Main", "=== System initialization complete ===");
    LOG_INFO("Main", "FSM is now running with FreeRTOS tasks");
    LOG_INFO("Main", "Monitor output for task execution and state transitions");

    createHealthMonitorTask();

    LOG_INFO("Main", "System crash monitoring enabled");
}

void loop()
{
    // Main loop is kept minimal since everything runs in FreeRTOS tasks
    static unsigned long lastHeartbeat = 0;
    static bool ledState = false;

    // Heartbeat every 2 seconds
    if (millis() - lastHeartbeat > 2000)
    {
        lastHeartbeat = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);

        // Optional: Print current state periodically
        static RocketState lastLoggedState = RocketState::INACTIVE;
        RocketState currentState = rocketFSM.getCurrentState();

        if (currentState != lastLoggedState)
        {
            Serial.printf("\n[MAIN] Current FSM State: %s (Runtime: %lu ms)\n",
                          rocketFSM.getStateString(currentState).c_str(), millis());
            lastLoggedState = currentState;
        }
    }

    // Small delay to prevent watchdog issues
    delay(100);
}

void initializeComponents()
{
    Serial.println("\n--- Initializing Components ---");

    // Initialize logger first
    rocketLogger = new RocketLogger();
    if (rocketLogger)
    {
        rocketLogger->logInfo("RocketLogger initialized");
        Serial.println("✓ Logger initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize logger");
    }

    // Initialize sensors
    Serial.println("Initializing sensors...");

    // Initialize BNO055 (IMU)
    bno055 = new BNO055Sensor();
    if (bno055 && bno055->init())
    {
        Serial.println("✓ BNO055 (IMU) initialized");
        if (rocketLogger)
            rocketLogger->logInfo("BNO055 sensor initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize BNO055");
        if (rocketLogger)
            rocketLogger->logError("Failed to initialize BNO055");
    }

    // Initialize MPRLS sensors (barometers)
    // tcaSelect(I2C_MULTIPLEXER_MPRLS1);
    baro1 = new MS561101BA03(0x76);
    if (baro1 && baro1->init())
    {
        Serial.println("✓ MPRLS1 (Barometer 1) initialized");
        if (rocketLogger)
            rocketLogger->logInfo("MPRLS1 sensor initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize MPRLS1");
        if (rocketLogger)
            rocketLogger->logError("Failed to initialize MPRLS1");
    }

    // tcaSelect(I2C_MULTIPLEXER_MPRLS2);
    baro2 = new MS561101BA03(0x77);
    if (baro2 && baro2->init())
    {
        Serial.println("✓ MPRLS2 (Barometer 2) initialized");
        if (rocketLogger)
            rocketLogger->logInfo("MPRLS2 sensor initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize MPRLS2");
        if (rocketLogger)
            rocketLogger->logError("Failed to initialize MPRLS2");
    }

    // Initialize GPS
    // gps = new GPSSensor(); // Uncomment when GPS sensor is available
    // if (gps && gps->init()) {
    //     Serial.println("✓ GPS initialized");
    //     if (rocketLogger) rocketLogger->logInfo("GPS sensor initialized");
    // } else {
    //     Serial.println("✗ Failed to initialize GPS");
    //     if (rocketLogger) rocketLogger->logError("Failed to initialize GPS");
    // }

    // Initialize LoRa transmitter
    Serial.println("Initializing LoRa transmitter...");
    loraSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, LORA_RX, LORA_TX);

    loraTransmitter = new E220LoRaTransmitter(loraSerial, LORA_AUX, LORA_M0, LORA_M1);
    if (loraTransmitter)
    {
        auto transmitterStatus = loraTransmitter->init();
        // Log transmitter status based on your implementation
        Serial.println("✓ LoRa transmitter initialized");
        if (rocketLogger)
            rocketLogger->logInfo("LoRa transmitter initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize LoRa transmitter");
        if (rocketLogger)
            rocketLogger->logError("Failed to initialize LoRa transmitter");
    }

    // Initialize Kalman Filter
    Serial.println("Initializing Kalman Filter...");
    Eigen::Vector3f gravity(0, 0, GRAVITY);
    Eigen::Vector3f magnetometer(1, 0, 0); // Placeholder values
    ekf = new KalmanFilter1D(gravity, magnetometer);
    if (ekf)
    {
        Serial.println("✓ Kalman Filter initialized");
        if (rocketLogger)
            rocketLogger->logInfo("Kalman Filter initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize Kalman Filter");
        if (rocketLogger)
            rocketLogger->logError("Failed to initialize Kalman Filter");
    }

    Serial.println("--- Component initialization complete ---");
}

void printSystemInfo()
{
    Serial.println("\n--- System Information ---");
    Serial.printf("ESP32 Chip: %s\n", ESP.getChipModel());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Total Heap: %d bytes\n", ESP.getHeapSize());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM Total: %d bytes\n", ESP.getPsramSize());
    Serial.printf("PSRAM Free: %d bytes\n", ESP.getFreePsram());
    Serial.printf("Flash Size: %d bytes\n", ESP.getFlashChipSize());
    Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());

    // FreeRTOS information
    LOG_INFO("SysInfo", "FreeRTOS running on %d cores", portNUM_PROCESSORS);
    LOG_INFO("SysInfo", "Tick rate: %d Hz", configTICK_RATE_HZ);
    LOG_INFO("SysInfo", "--- End System Information ---");
}

/**
 * @brief Create a Health Monitor Task object
 *  Basic control
    stop                    // Stop FSM
    start                   // Start FSM

    Force state transitions (useful for testing)
    force LAUNCH            // Jump directly to launch state
    force APOGEE            // Jump directly to apogee detection
    force RECOVERED         // Jump directly to recovery state

    System diagnostics
    health                  // Get immediate health report
    memory                  // Show detailed memory statistics
    help                    // Show command list
 *
 */
void createHealthMonitorTask()
{
    xTaskCreatePinnedToCore(
        [](void *param)
        {
            LOG_INFO("HealthMon", "Health Monitor Task started");

            // Initialize baseline values
            systemStartTime = millis();
            lastFreeHeap = ESP.getFreeHeap();
            maxHeapUsed = ESP.getHeapSize() - lastFreeHeap;

            TickType_t xLastWakeTime = xTaskGetTickCount();
            uint32_t healthCheckCounter = 0;

            while (true)
            {
                healthCheckCounter++;

                // === MEMORY ANALYSIS ===
                uint32_t freeHeap = ESP.getFreeHeap();
                uint32_t minFreeHeap = ESP.getMinFreeHeap();
                uint32_t maxAllocHeap = ESP.getMaxAllocHeap();
                size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
                uint32_t heapUsed = ESP.getHeapSize() - freeHeap;

                // Track maximum heap usage
                if (heapUsed > maxHeapUsed)
                {
                    maxHeapUsed = heapUsed;
                }

                // Calculate memory fragmentation
                double fragmentation = 0.0f;
                if (freeHeap > 0)
                {
                    fragmentation = 100.0f * (1.0f - (float)largestBlock / freeHeap);
                }

                // Memory leak detection
                int32_t heapDelta = (int32_t)lastFreeHeap - (int32_t)freeHeap;
                lastFreeHeap = freeHeap;

                // === TASK ANALYSIS ===
                UBaseType_t taskCount = uxTaskGetNumberOfTasks();
                static UBaseType_t lastTaskCount = 0;

                // Get current FSM state
                RocketState currentState = rocketFSM.getCurrentState();
                static RocketState lastState = RocketState::INACTIVE;
                static unsigned long stateChangeTime = millis();

                if (currentState != lastState)
                {
                    unsigned long timeInState = millis() - stateChangeTime;
                    LOG_INFO("HealthMon", "State change: %s -> %s (was in previous state for %lu ms)",
                             rocketFSM.getStateString(lastState).c_str(),
                             rocketFSM.getStateString(currentState).c_str(),
                             timeInState);
                    lastState = currentState;
                    stateChangeTime = millis();
                }

                // === CPU AND SYSTEM ANALYSIS ===
                uint32_t cpuFreq = ESP.getCpuFreqMHz();

                // Check for system anomalies
                bool criticalCondition = false;
                String alertMsg = "";

                // Critical memory condition
                if (freeHeap < 8192)
                { // Less than 8KB
                    criticalCondition = true;
                    alertMsg += "CRITICAL_MEMORY ";
                    LOG_ERROR("HealthMon", "CRITICAL: Only %u bytes free heap remaining!", freeHeap);
                }

                // High fragmentation
                if (fragmentation > 30.0)
                {
                    criticalCondition = true;
                    alertMsg += "HIGH_FRAGMENTATION ";
                    LOG_WARNING("HealthMon", "High memory fragmentation: %.1f%%", fragmentation);
                }

                // Significant memory leak
                if (heapDelta > 1024 && heapDelta > 0)
                { // Lost more than 1KB
                    alertMsg += "MEMORY_LEAK ";
                    LOG_WARNING("HealthMon", "Possible memory leak: %d bytes lost", heapDelta);
                }

                // Task count anomaly
                if (taskCount != lastTaskCount)
                {
                    if (taskCount > lastTaskCount + 2)
                    {
                        alertMsg += "TASK_LEAK ";
                        LOG_WARNING("HealthMon", "Task count increased unexpectedly: %u -> %u",
                                    lastTaskCount, taskCount);
                    }
                    lastTaskCount = taskCount;
                }

                // CPU frequency anomaly
                if (cpuFreq < 240)
                { // ESP32 should run at 240MHz
                    alertMsg += "LOW_CPU_FREQ ";
                    LOG_WARNING("HealthMon", "CPU frequency reduced to %u MHz", cpuFreq);
                }

                // === WATCHDOG MONITORING ===
                esp_task_wdt_reset(); // Reset our own watchdog

                // === PERIODIC DETAILED REPORTS ===
                bool detailedReport = (healthCheckCounter % 12 == 0); // Every 60 seconds
                bool summaryReport = (healthCheckCounter % 4 == 0);   // Every 20 seconds

                if (detailedReport || criticalCondition)
                {
                    LOG_INFO("HealthMon", "=== DETAILED HEALTH REPORT  ===");
                    LOG_INFO("HealthMon", "Memory: Free=%u, Min=%u, MaxAlloc=%u, Used=%u, MaxUsed=%u",
                             freeHeap, minFreeHeap, maxAllocHeap, heapUsed, maxHeapUsed);
                    LOG_INFO("HealthMon", "Fragmentation: %.1f%%, Largest block: %u bytes",
                             fragmentation, largestBlock);
                    LOG_INFO("HealthMon", "Tasks: Count=%u, CPU=%uMHz, State=%s",
                             taskCount, cpuFreq, rocketFSM.getStateString(currentState).c_str());
                    LOG_INFO("HealthMon", "Phase: %d, Time in current state: %lu ms",
                             static_cast<int>(rocketFSM.getCurrentPhase()),
                             millis() - stateChangeTime);

                    // PSRAM info if available
                    if (ESP.getPsramSize() > 0)
                    {
                        LOG_INFO("HealthMon", "PSRAM: Total=%d, Free=%d",
                                 ESP.getPsramSize(), ESP.getFreePsram());
                    }

                    if (criticalCondition)
                    {
                        LOG_ERROR("HealthMon", "ALERTS: %s", alertMsg.c_str());

                        // Take emergency action for critical conditions
                        if (freeHeap < 4096)
                        { // Less than 4KB - emergency stop
                            LOG_ERROR("HealthMon", "EMERGENCY: Stopping FSM due to critical memory shortage");
                            rocketFSM.stop();

                            // Try to free some memory
                            LOG_ERROR("HealthMon", "Attempting emergency cleanup...");
                            // Force garbage collection if possible
                            vTaskDelay(pdMS_TO_TICKS(100));

                            crashCounter++;
                            if (crashCounter > 3)
                            {
                                LOG_ERROR("HealthMon", "Multiple critical conditions - restarting system");
                                esp_restart();
                            }
                        }
                    }
                    LOG_INFO("HealthMon", "=== END HEALTH REPORT ===");
                }
                else if (summaryReport)
                {
                    LOG_INFO("HealthMon", "Health: Free=%uKB, Frag=%.1f%%, Tasks=%u, State=%s%s",
                             freeHeap / 1024, fragmentation, taskCount,
                             rocketFSM.getStateString(currentState).c_str(),
                             alertMsg.length() > 0 ? " [ALERTS]" : "");
                }

                // === PERFORMANCE MONITORING ===
                // Check if any task is consuming too much CPU time
                static TickType_t lastTickCount = 0;
                TickType_t currentTicks = xTaskGetTickCount();
                TickType_t ticksDelta = currentTicks - lastTickCount;
                lastTickCount = currentTicks;

                if (ticksDelta > pdMS_TO_TICKS(6000))
                { // If we're delayed by more than 1 second
                    LOG_WARNING("HealthMon", "Health monitor delayed by %lu ms - system may be overloaded",
                                pdTICKS_TO_MS(ticksDelta - pdMS_TO_TICKS(5000)));
                }

                // === STACK MONITORING ===
                UBaseType_t healthMonitorStack = uxTaskGetStackHighWaterMark(NULL);
                if (healthMonitorStack < 256)
                { // Less than 256 bytes stack remaining
                    LOG_ERROR("HealthMon", "Health monitor stack low: %u bytes remaining", healthMonitorStack);
                }

                // Sleep until next check (every 5 seconds)
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
            }
        },
        "HealthMonitor",
        3072, // Increased stack size for detailed reporting
        nullptr,
        3, // High priority - higher than monitoring tasks
        nullptr,
        0 // Run on Core 0 (opposite of SystemMonitor)
    );
}

// Also update monitorTasks() to be lighter since HealthMonitor handles most of this
void monitorTasks()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Reduced frequency logging - HealthMonitor handles detailed monitoring
        static unsigned long lastStatusPrint = 0;
        if (millis() - lastStatusPrint > 30000) // Every 30 seconds instead of 10
        {
            lastStatusPrint = millis();
            LOG_INFO("Monitor", "Quick status: Heap=%uKB, Tasks=%u",
                     ESP.getFreeHeap() / 1024, uxTaskGetNumberOfTasks());
        }

        // Monitor for manual commands via Serial (keep this functionality)
        if (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            command.trim();

            if (command == "stop")
            {
                Serial.println("Manual FSM stop requested");
                rocketFSM.stop();
            }
            else if (command == "start")
            {
                Serial.println("Manual FSM start requested");
                rocketFSM.start();
            }
            else if (command == "health")
            {
                // Force immediate detailed health report
                LOG_INFO("Monitor", "Manual health check requested");
                Logger::debugMemory("Manual health check");
                LOG_INFO("Monitor", "Current state: %s, Tasks: %u",
                         rocketFSM.getStateString(rocketFSM.getCurrentState()).c_str(),
                         uxTaskGetNumberOfTasks());
            }
            else if (command.startsWith("force "))
            {
                String stateName = command.substring(6);
                stateName.toUpperCase();

                if (stateName == "LAUNCH")
                {
                    rocketFSM.forceTransition(RocketState::LAUNCH);
                }
                else if (stateName == "APOGEE")
                {
                    rocketFSM.forceTransition(RocketState::APOGEE);
                }
                else if (stateName == "RECOVERED")
                {
                    rocketFSM.forceTransition(RocketState::RECOVERED);
                }
                else
                {
                    Serial.println("Unknown state: " + stateName);
                }
            }
            else if (command == "help")
            {
                SemaphoreHandle_t serialMutex = Logger::getSerialMutex();
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    Serial.println("Available commands:");
                    Serial.println("  stop - Stop FSM");
                    Serial.println("  start - Start FSM");
                    Serial.println("  health - Show detailed health status");
                    Serial.println("  force <STATE> - Force transition to state");
                    Serial.println("  memory - Show memory stats");
                    Serial.println("  help - Show this help");
                    xSemaphoreGive(serialMutex);
                }
            }
            else if (command == "memory")
            {
                Logger::debugMemory("Manual request");
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000)); // 2 second monitoring
    }
}
void tcaSelect(uint8_t bus)
{
    if (bus > 7)
        return;

    Wire.beginTransmission(0x70); // TCA9548A address
    Wire.write(1 << bus);
    Wire.endTransmission();
}