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
#include <MS561101BA03.hpp>
#include <LIS3DHTRSensor.hpp>
#include <GPS.hpp>
#include <RocketLogger.hpp>
#include <E220LoRaTransmitter.hpp>
#include <KalmanFilter1D.hpp>
#include <RocketFSM.hpp>

// Type definitions
using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// Global component instances - these match the extern declarations in RocketFSM.cpp
ISensor *bno055 = nullptr;
ISensor *baro1 = nullptr;
ISensor *baro2 = nullptr;
ISensor *accl = nullptr;
ISensor *gps = nullptr;
ILogger *rocketLogger = nullptr;
ITransmitter<TransmitDataType> *loraTransmitter = nullptr;
KalmanFilter1D *ekf = nullptr;

// Hardware serial for LoRa
HardwareSerial loraSerial(LORA_SERIAL);

// FSM instance
RocketFSM rocketFSM;

// Utility functions
void testFSMTransitions(RocketFSM &fsm);
void tcaSelect(uint8_t bus);
void initializeComponents();
void sensorsCalibration();
void initializeKalman();
void printSystemInfo();
void monitorTasks();

void setup()
{
    // Initialize basic hardware
    Serial.begin(SERIAL_BAUD_RATE);
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

    // Checking sensors calibration
    sensorsCalibration();

    // Initialize kalman
    initializeKalman();

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

    // Initialize barometers
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

    // Initialize accelerometer
    accl = new LIS3DHTRSensor();
    if (accl && accl->init())
    {
        Serial.println("✓ LIS3DHTR (Accelerometer) initialized");
        if (rocketLogger)
            rocketLogger->logInfo("LIS3DHTR sensor initialized");
    }
    else
    {
        Serial.println("✗ Failed to initialize LIS3DHTR");
        if (rocketLogger)
            rocketLogger->logError("Failed to initialize LIS3DHTR");
    }

    // Initialize GPS
    gps = new GPS();
    if (gps && gps->init()) {
        Serial.println("✓ GPS initialized");
        if (rocketLogger) rocketLogger->logInfo("GPS sensor initialized");
    } else {
        Serial.println("✗ Failed to initialize GPS");
        if (rocketLogger) rocketLogger->logError("Failed to initialize GPS");
    }

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
}

// Function to check sensors calibration
void sensorsCalibration()
{
    Serial.println("Calibrating BNO055 sensors...");
    
    if (bno055)
    {
        auto bnoData = bno055->getData();
        
        if (!bnoData.has_value()) {
            Serial.println("BNO055 not initialized, skipping calibration.");
            return;
        }
        
        auto sensorData = bnoData.value();
        auto gyro_cal_opt = sensorData.getData("gyro_calibration");
        auto accel_cal_opt = sensorData.getData("accel_calibration");
        auto mag_cal_opt = sensorData.getData("mag_calibration");
        
        if (!gyro_cal_opt.has_value() || 
            !accel_cal_opt.has_value() || !mag_cal_opt.has_value()) {
            Serial.println("Could not read calibration status, skipping calibration.");
            return;
        }
        
        auto gyro_cal = std::get<uint8_t>(gyro_cal_opt.value());
        auto accel_cal = std::get<uint8_t>(accel_cal_opt.value());
        auto mag_cal = std::get<uint8_t>(mag_cal_opt.value());
        
        // Find minimum calibration status
        uint8_t min_calibration = std::min({gyro_cal, accel_cal, mag_cal});
        
        if (min_calibration < 3)
        {
            Serial.println("Calibrating BNO055's gyro...");
            if (rocketLogger)
                rocketLogger->logInfo("Calibrating BNO055's gyro...");
            do
            {
                sensorData = bno055->getData().value();
                gyro_cal_opt = sensorData.getData("gyro_calibration");
                gyro_cal = std::get<uint8_t>(gyro_cal_opt.value());

                Serial.printf("Current Gyro calibration status: %d/3\n", gyro_cal);
                Serial.println("1. Move the sensor in a figure-8 pattern for few seconds.");
                Serial.println("2. Rotate the sensor around all axes.");
                Serial.println("3. Keep the sensor still on a flat surface.");
            } while(gyro_cal < 3);

            Serial.println("Calibrating BNO055's accel...");
            if (rocketLogger)
                rocketLogger->logInfo("Calibrating BNO055's accel...");
            do
            {
                sensorData = bno055->getData().value();
                accel_cal_opt = sensorData.getData("accel_calibration");
                accel_cal = std::get<uint8_t>(accel_cal_opt.value());

                Serial.printf("Current Accel calibration status: %d/3\n", accel_cal);
                Serial.println("1. Move the sensor in a figure-8 pattern for few seconds.");
                Serial.println("2. Rotate the sensor around all axes.");
                Serial.println("3. Keep the sensor still on a flat surface.");
            } while(accel_cal < 3);

            Serial.println("Calibrating BNO055's mag...");
            if (rocketLogger)
                rocketLogger->logInfo("Calibrating BNO055's mag...");
            do
            {
                sensorData = bno055->getData().value();
                mag_cal_opt = sensorData.getData("mag_calibration");
                mag_cal = std::get<uint8_t>(mag_cal_opt.value());

                Serial.printf("Current Mag calibration status: %d/3\n", mag_cal);
                Serial.println("1. Move the sensor in a figure-8 pattern for few seconds.");
                Serial.println("2. Rotate the sensor around all axes.");
                Serial.println("3. Keep the sensor still on a flat surface.");
            } while(mag_cal < 3);

            Serial.println("BNO055 calibration complete.");
            if (rocketLogger)
                rocketLogger->logInfo("BNO055 calibration complete");
        }
        else
        {
            Serial.println("BNO055 already calibrated.");
        }
    }
    else
    {
        Serial.println("BNO055 not initialized, skipping calibration.");
    }

    Serial.println("Sensor calibration complete.");
}

// Utility function to calculate mean sensor readings
Eigen::Vector3f calculateMean(const std::vector<Eigen::Vector3f>& readings) {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        mean += reading;
    }
    mean /= readings.size();
    return mean;
}

// Utility function to calculate standard deviation of sensor readings
Eigen::Vector3f calculateStandardDeviation(const std::vector<Eigen::Vector3f>& readings) {
    Eigen::Vector3f mean = calculateMean(readings);
    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        Eigen::Vector3f diff = reading - mean;
        variance += diff.cwiseProduct(diff);
    }
    variance /= static_cast<float>(readings.size() - 1);
    return variance.cwiseSqrt();
}

// Finish implementation !!!
void initializeKalman()
{
    // Store calibration samples
    std::vector<Eigen::Vector3f> accelSamples;
    std::vector<Eigen::Vector3f> magSamples;
    Serial.println("Collecting calibration samples for Kalman Filter...");
    if (rocketLogger)
        rocketLogger->logInfo("Collecting calibration samples for Kalman Filter...");

    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++)
    {
        if (accl)
        {
            auto acclDataOpt = accl->getData();
            if (acclDataOpt.has_value())
            {
                auto acclData = acclDataOpt.value();
                // LIS3DHTR sensor uses "accel_x", "accel_y", "accel_z" keys
                auto x_opt = acclData.getData("accel_x");
                auto y_opt = acclData.getData("accel_y");
                auto z_opt = acclData.getData("accel_z");
                
                if (x_opt.has_value() && y_opt.has_value() && z_opt.has_value())
                {
                    float x = std::get<float>(x_opt.value());
                    float y = std::get<float>(y_opt.value());
                    float z = std::get<float>(z_opt.value());

                    accelSamples.push_back(Eigen::Vector3f(x, y, z));
                }
            }
        }

        if (bno055)
        {
            auto bnoDataOpt = bno055->getData();
            if (bnoDataOpt.has_value())
            {
                auto bnoData = bnoDataOpt.value();
                auto mag_opt = bnoData.getData("magnetometer");
                if (mag_opt.has_value())
                {
                    auto magMap = std::get<std::map<std::string, float>>(mag_opt.value());

                    magSamples.push_back(Eigen::Vector3f(magMap["x"], magMap["y"], magMap["z"]));
                }
            }
        }

        delay(50);
    }

    // Evaluate quality of collected samples through stddev and STD_THRESHOLD
    auto accelStdDev = calculateStandardDeviation(accelSamples);
    auto magStdDev = calculateStandardDeviation(magSamples);

    if (accelStdDev.norm() > STD_THRESHOLD) {
        rocketLogger->logWarning("High accelerometer noise during calibration: " + std::to_string(accelStdDev.norm()));
    }

    if (magStdDev.norm() > STD_THRESHOLD) {
        rocketLogger->logWarning("High magnetometer noise during calibration: " + std::to_string(magStdDev.norm()));
    }

    auto accelMean = calculateMean(accelSamples);
    auto magMean = calculateMean(magSamples);

    // Initialize Kalman Filter
    Serial.println("Initializing Kalman Filter...");
    ekf = new KalmanFilter1D(accelMean, magMean);
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