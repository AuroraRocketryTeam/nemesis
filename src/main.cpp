#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <variant>
#include <config.h>
#include <pins.h>
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
#include <Logger.hpp>
#include <LEDManager.h>

// Type definitions
using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// Global component instances - these match the extern declarations in RocketFSM.cpp
std::shared_ptr<ISensor> bno055 = nullptr;
std::shared_ptr<ISensor> baro1 = nullptr;
std::shared_ptr<ISensor> baro2 = nullptr;
std::shared_ptr<ISensor> gps = nullptr;
std::shared_ptr<ISensor> accl = nullptr;

ILogger *rocketLogger = nullptr;
std::shared_ptr<KalmanFilter1D> ekf = nullptr;

// FSM instance
std::unique_ptr<RocketFSM> rocketFSM;

// Utility functions
void testFSMTransitions(RocketFSM &fsm);
void initializeComponents();
void sensorsCalibration();
void initializeKalman();
void printSystemInfo();
void monitorTasks();

void showOutputLED(LEDColor color, uint8_t duration_ms = 250, uint8_t pause_ms = 100, uint8_t times = 1);

void setup()
{
    // Initialize basic hardware
    Serial.begin(SERIAL_BAUD_RATE);

    LOG_INFO("Main", "\n=== Aurora Rocketry Flight Software ===");
    LOG_INFO("Main", "Initializing system...");

    // Initialize pins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(MAIN_ACTUATOR_PIN, OUTPUT);
    pinMode(DROGUE_ACTUATOR_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    // Signal initialization start
    digitalWrite(LED_RED, HIGH);

    // Deactivate actuators and set LEDs to known state
    digitalWrite(MAIN_ACTUATOR_PIN, LOW);
    digitalWrite(DROGUE_ACTUATOR_PIN, LOW);

    analogWrite(LED_RED_PIN, 100);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 100);

    // Initialize I2C
    Wire.begin();
    LOG_INFO("Main", "I2C initialized");

    // Initialize components
    initializeComponents();

    // Checking sensors calibration
    sensorsCalibration();

    // Initialize kalman
    initializeKalman();

    // Print system information
    printSystemInfo();

    // Initialize and start FSM
    LOG_INFO("Main", "=== System initialization complete ===");
    LOG_INFO("Main", "\n=== Initializing Flight State Machine ===");
    rocketFSM = std::make_unique<RocketFSM>(bno055, baro1, baro2, accl, gps, ekf);
    rocketFSM->init();

    // Give system a moment to stabilize
    delay(1000);
    testFSMTransitions(*rocketFSM);
    delay(1000);
    // Start FSM tasks
    rocketFSM->start();

    // Signal successful initialization
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
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
        RocketState currentState = rocketFSM->getCurrentState();

        if (currentState != lastLoggedState)
        {
            LOG_INFO("Main", "Current FSM State: %s (Runtime: %lu ms)",
                     rocketFSM->getStateString(currentState), millis());
            lastLoggedState = currentState;
        }
    }

    // Small delay to prevent watchdog issues
    delay(100);
}

void testFSMTransitions(RocketFSM &fsm)
{
    LOG_INFO("Test", "\n\n=== STARTING AUTOMATED FSM TEST ===");
    LOG_INFO("Test", "Testing all state transitions with automatic timeouts");
    LOG_INFO("Test", "Will cycle through all states, observing task execution\n");

    // Il test inizia automaticamente quando fsm.start() viene chiamato
    // Le transizioni sono tutte temporizzate nei metodi di transizione definiti nella classe RocketFSM

    fsm.start();
    // Use FreeRTOS timing for more reliable 1-second intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second

    RocketState lastLoggedState = RocketState::INACTIVE;
    unsigned long testStartTime = millis();
    while (true)
    {
        RocketState currentState = fsm.getCurrentState();

        // Only log when state changes or every 10 seconds
        static unsigned long lastPeriodicLog = 0;
        bool stateChanged = (currentState != lastLoggedState);
        bool periodicLog = (millis() - lastPeriodicLog > 10000);

        if (stateChanged || periodicLog)
        {
            LOG_INFO("Test", "State: %s (runtime: %lu ms, uptime: %.1f sec)",
                     fsm.getStateString(currentState),
                     millis(),
                     (millis() - testStartTime) / 1000.0);

            if (periodicLog)
                lastPeriodicLog = millis();

            lastLoggedState = currentState;
        }

        // Termina il test quando raggiungiamo lo stato RECOVERED
        if (currentState == RocketState::RECOVERED)
        {
            LOG_INFO("Test", "=== FSM TEST COMPLETED SUCCESSFULLY ===");
            LOG_INFO("Test", "All states were visited in the correct order!");
            LOG_INFO("Test", "Total test duration: %.1f seconds", (millis() - testStartTime) / 1000.0);
            while (true)
            {
                Serial.readString();
            }
        }
        // Use FreeRTOS delay for precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void initializeComponents()
{
    LOG_INFO("Init", "\n--- Initializing Components ---");

    // Initialize sensors
    LOG_INFO("Init", "Initializing sensors...");

    // Initialize BNO055 (IMU)
    bno055 = std::make_shared<BNO055Sensor>();
    if (bno055 && bno055->init())
    {
        LOG_INFO("Init", "✓ BNO055 (IMU) initialized");
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize BNO055");
    }

    // Initialize barometers
    baro1 = std::make_shared<MS561101BA03>(MS56_I2C_ADDR_1);
    if (baro1 && baro1->init())
    {
        LOG_INFO("Init", "✓ Barometer 1 initialized");
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize Barometer 1");
    }

    baro2 = std::make_shared<MS561101BA03>(MS56_I2C_ADDR_2);
    if (baro2 && baro2->init())
    {
        LOG_INFO("Init", "✓ Barometer 2 initialized");
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize Barometer 2");
    }

    // Initialize accelerometer
    accl = std::make_shared<LIS3DHTRSensor>();
    if (accl && accl->init())
    {
        LOG_INFO("Init", "✓ LIS3DHTR (Accelerometer) initialized");
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize LIS3DHTR");
    }

    // Initialize GPS
    gps = std::make_shared<GPS>();
    if (gps)
    {
        LOG_INFO("Init", "✓ GPS object created");
        if (gps->init())
        {
            LOG_INFO("Init", "✓ GPS initialized");
        }
        else
        {
            LOG_ERROR("Init", "✗ Failed to initialize GPS");
        }
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize GPS");
    }
}

// Function to check sensors calibration
void sensorsCalibration()
{
    LOG_INFO("BNO055", "Calibrating BNO055 sensors...");

    if (bno055)
    {
        auto bnoData = bno055->getData();

        if (!bnoData.has_value())
        {
            LOG_WARNING("BNO055", "BNO055 not initialized, skipping calibration.");
            return;
        }

        auto sensorData = bnoData.value();
        auto gyro_cal_opt = sensorData.getData("gyro_calibration");
        auto accel_cal_opt = sensorData.getData("accel_calibration");
        auto mag_cal_opt = sensorData.getData("mag_calibration");

        if (!gyro_cal_opt.has_value() ||
            !accel_cal_opt.has_value() || !mag_cal_opt.has_value())
        {
            LOG_WARNING("BNO055", "Could not read calibration status, skipping calibration.");
            return;
        }

        auto gyro_cal = std::get<uint8_t>(gyro_cal_opt.value());
        auto accel_cal = std::get<uint8_t>(accel_cal_opt.value());
        auto mag_cal = std::get<uint8_t>(mag_cal_opt.value());

        // Find minimum calibration status
        uint8_t min_calibration = std::min({gyro_cal, accel_cal, mag_cal});

        if (min_calibration < IMU_MINIMUM_CALIBRATION)
        {
            LOG_INFO("BNO055", "Calibrating BNO055's gyro...");
            do
            {
                sensorData = bno055->getData().value();
                gyro_cal_opt = sensorData.getData("gyro_calibration");
                gyro_cal = std::get<uint8_t>(gyro_cal_opt.value());

                LOG_INFO("BNO055", "Current Gyro calibration status: %d/3", gyro_cal);
                LOG_INFO("BNO055", "Keep the sensor still on a flat surface.");
            } while (gyro_cal < IMU_MINIMUM_CALIBRATION);

            LOG_INFO("BNO055", "Calibrating BNO055's accel...");
            do
            {
                sensorData = bno055->getData().value();
                auto accel_opt = sensorData.getData("accelerometer");
                auto accelMap = std::get<std::map<std::string, float>>(accel_opt.value());
                auto acceleration_x = accelMap["x"];
                auto acceleration_y = accelMap["y"];
                auto acceleration_z = accelMap["z"];
                LOG_INFO("BNO055", "Acceleration: x=%.2f, y=%.2f, z=%.2f m/s^2",
                         (double)acceleration_x,
                         (double)acceleration_y,
                         (double)acceleration_z);

                accel_cal_opt = sensorData.getData("accel_calibration");
                accel_cal = std::get<uint8_t>(accel_cal_opt.value());

                LOG_INFO("BNO055", "Current Accel calibration status: %d/3", accel_cal);
                LOG_INFO("BNO055", "Place the sensor in these 6 standing positions for about 5 seconds each\n(Positive = +9.8 m/s^2, Negative = -9.8 m/s^2):\n1. +X\n2. -X\n3. +Y\n4. -Y\n5. +Z\n6. -Z\n");
                delay(100);
            } while (accel_cal < IMU_MINIMUM_CALIBRATION);

            LOG_INFO("BNO055", "Calibrating BNO055's mag...");
            do
            {
                sensorData = bno055->getData().value();
                mag_cal_opt = sensorData.getData("mag_calibration");
                mag_cal = std::get<uint8_t>(mag_cal_opt.value());

                LOG_INFO("BNO055", "Current Mag calibration status: %d/3", mag_cal);
                LOG_INFO("BNO055", "Move the sensor in a figure-8 pattern for few seconds.");
            } while (mag_cal < IMU_MINIMUM_CALIBRATION);
            LOG_INFO("BNO055", "BNO055 calibration complete.");
        }
        else
        {
            LOG_INFO("BNO055", "BNO055 already calibrated.");
        }
    }
    else
    {
        LOG_WARNING("BNO055", "BNO055 not initialized, skipping calibration.");
    }
    if (gps)
    {
        LOG_INFO("GPS", "Checking GPS lock...");

        bool gpsLocked = false;
        unsigned long startTime = millis();

        while (!gpsLocked && (millis() - startTime < GPS_FIX_TIMEOUT_MS))
        {
            auto gpsDataOpt = gps->getData();
            if (gpsDataOpt.has_value())
            {
                LOG_INFO("GPS", "Getting GPS data...");
                auto gpsData = gpsDataOpt.value();
                auto fix_opt = gpsData.getData("fix");
                auto satellites_opt = gpsData.getData("satellites");
                if (fix_opt.has_value())
                {
                    uint8_t fix = std::get<uint8_t>(fix_opt.value());
                    LOG_INFO("GPS", "Fix value: %d", fix);
                    if (fix >= GPS_MIN_FIX)
                    {
                        gpsLocked = true;
                        LOG_INFO("GPS", "GPS lock acquired. Satellites: %d", std::get<uint8_t>(satellites_opt.value()));
                    }
                }
            }
            delay(GPS_FIX_LOOKUP_INTERVAL_MS);
        }

        if (!gpsLocked)
        {
            LOG_ERROR("GPS", "GPS lock not acquired within timeout period.");
        }
    }

    LOG_INFO("Calibration", "Sensor calibration complete.");
}

// Utility function to calculate mean sensor readings
Eigen::Vector3f calculateMean(const std::vector<Eigen::Vector3f> &readings)
{
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto &reading : readings)
    {
        mean += reading;
    }
    mean /= readings.size();
    return mean;
}

// Utility function to calculate standard deviation of sensor readings
Eigen::Vector3f calculateStandardDeviation(const std::vector<Eigen::Vector3f> &readings)
{
    Eigen::Vector3f mean = calculateMean(readings);
    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto &reading : readings)
    {
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
    LOG_INFO("EKF", "Collecting calibration samples for Kalman Filter...");

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

    if (accelStdDev.norm() > STD_THRESHOLD)
    {
        LOG_WARNING("EKF", "High accelerometer noise during calibration: %.3f", static_cast<double>(accelStdDev.norm()));
    }

    if (magStdDev.norm() > STD_THRESHOLD)
    {
        LOG_WARNING("EKF", "High magnetometer noise during calibration: %.3f", static_cast<double>(magStdDev.norm()));
    }

    auto accelMean = calculateMean(accelSamples);
    auto magMean = calculateMean(magSamples);

    // Initialize Kalman Filter
    LOG_INFO("Init", "Initializing Kalman Filter...");
    ekf = std::make_shared<KalmanFilter1D>(accelMean, magMean);
    if (ekf)
    {
        LOG_INFO("Init", "✓ Kalman Filter initialized");
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize Kalman Filter");
    }

    LOG_INFO("Init", "--- Component initialization complete ---");
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
            Serial.printf("Current State: %s\n", rocketFSM->getStateString(rocketFSM->getCurrentState()));
            Serial.printf("Current Phase: %d\n", static_cast<int>(rocketFSM->getCurrentPhase()));

            // Task information
            UBaseType_t taskCount = uxTaskGetNumberOfTasks();
            Serial.printf("Active tasks: %u\n", taskCount);

            Serial.println("--- End Status ---\n");
        }

        // Check for emergency conditions
        if (ESP.getFreeHeap() < 10000)
        { // Less than 10KB free
            LOG_WARNING("Memory", "Low memory condition detected!");
        }

        // Monitor for manual commands via Serial
        if (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            command.trim();

            if (command == "stop")
            {
                LOG_INFO("FSM", "Manual FSM stop requested");
                rocketFSM->stop();
            }
            else if (command == "start")
            {
                LOG_INFO("FSM", "Manual FSM start requested");
                rocketFSM->start();
            }
            else if (command.startsWith("force "))
            {
                // Example: "force LAUNCH" to force transition to LAUNCH state
                String stateName = command.substring(6);
                stateName.toUpperCase();

                // Map string to state (add more as needed)
                if (stateName == "LAUNCH")
                {
                    rocketFSM->forceTransition(RocketState::LAUNCH);
                }
                else if (stateName == "APOGEE")
                {
                    rocketFSM->forceTransition(RocketState::APOGEE);
                }
                else if (stateName == "RECOVERED")
                {
                    rocketFSM->forceTransition(RocketState::RECOVERED);
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
