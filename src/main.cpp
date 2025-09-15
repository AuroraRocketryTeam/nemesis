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
#include <KalmanFilter.hpp>
#include <RocketFSM.hpp>

// Type definitions
using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// Global component instances - these match the extern declarations in RocketFSM.cpp
ISensor *bno055 = nullptr;
ISensor *baro1 = nullptr;
ISensor *baro2 = nullptr;
ISensor *gps = nullptr;
ILogger *rocketLogger = nullptr;
ITransmitter<TransmitDataType> *loraTransmitter = nullptr;
KalmanFilter *ekf = nullptr;

// Hardware serial for LoRa
HardwareSerial loraSerial(LORA_SERIAL);

// FSM instance
RocketFSM rocketFSM;

// Utility functions
void tcaSelect(uint8_t bus);
void initializeComponents();
void printSystemInfo();
void monitorTasks();

void testFSMTransitions(RocketFSM &fsm)
{
    Serial.println("\n\n=== STARTING AUTOMATED FSM TEST ===");
    Serial.println("Testing all state transitions with automatic timeouts");
    Serial.println("Will cycle through all states, observing task execution\n");

    // Il test inizia automaticamente quando fsm.start() viene chiamato
    // Le transizioni sono tutte temporizzate nei metodi is*() modificati

    // Rendi i log più visibili
    fsm.start();

    // Mostra lo stato ogni secondo
    for (int i = 0; i < 50; i++)
    {
        RocketState currentState = fsm.getCurrentState();
        Serial.printf("[TEST] Current state: %s (runtime: %lu ms)\n",
                      fsm.getStateString(currentState).c_str(), millis());

        // Termina il test quando raggiungiamo lo stato RECOVERED
        if (currentState == RocketState::RECOVERED)
        {
            Serial.println("\n=== FSM TEST COMPLETED SUCCESSFULLY ===");
            Serial.println("All states were visited in the correct order!");
            break;
        }

        delay(1000);
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

    Serial.println("=== System initialization complete ===");
    Serial.println("FSM is now running with FreeRTOS tasks");
    Serial.println("Monitor output for task execution and state transitions\n");

    // Create a task to monitor system status
    xTaskCreatePinnedToCore(
        [](void *param)
        {
            while (true)
            {
                monitorTasks();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        },
        "SystemMonitor",
        2048,
        nullptr,
        1,
        nullptr,
        1 // Run on Core 1
    );
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
    tcaSelect(I2C_MULTIPLEXER_MPRLS1);
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

    tcaSelect(I2C_MULTIPLEXER_MPRLS2);
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
    Eigen::Vector3f gravity(0, 0, -9.81f);
    Eigen::Vector3f magnetometer(1, 0, 0); // Placeholder values
    ekf = new KalmanFilter(gravity, magnetometer);
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
    Serial.printf("FreeRTOS running on %d cores\n", portNUM_PROCESSORS);
    Serial.printf("Tick rate: %d Hz\n", configTICK_RATE_HZ);
    Serial.println("--- End System Information ---");
}

void monitorTasks()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Print system status every 10 seconds
        static unsigned long lastStatusPrint = 0;
        if (millis() - lastStatusPrint > 10000)
        {
            lastStatusPrint = millis();

            Serial.println("\n--- System Status ---");
            Serial.printf("Runtime: %lu ms\n", millis());
            Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
            Serial.printf("Min Free Heap: %u bytes\n", ESP.getMinFreeHeap());
            Serial.printf("Current State: %s\n", rocketFSM.getStateString(rocketFSM.getCurrentState()).c_str());
            Serial.printf("Current Phase: %d\n", static_cast<int>(rocketFSM.getCurrentPhase()));

            // Task information
            UBaseType_t taskCount = uxTaskGetNumberOfTasks();
            Serial.printf("Active tasks: %u\n", taskCount);

            Serial.println("--- End Status ---\n");
        }

        // Check for emergency conditions
        if (ESP.getFreeHeap() < 10000)
        { // Less than 10KB free
            Serial.println("[WARNING] Low memory condition detected!");
            if (rocketLogger)
            {
                rocketLogger->logWarning("Low memory condition");
            }
        }

        // Monitor for manual commands via Serial
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
            else if (command.startsWith("force "))
            {
                // Example: "force LAUNCH" to force transition to LAUNCH state
                String stateName = command.substring(6);
                stateName.toUpperCase();

                // Map string to state (add more as needed)
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
                Serial.println("Available commands:");
                Serial.println("  stop - Stop FSM");
                Serial.println("  start - Start FSM");
                Serial.println("  force <STATE> - Force transition to state");
                Serial.println("  help - Show this help");
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // 1Hz monitoring
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