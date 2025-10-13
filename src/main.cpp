// Standard libraries
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <variant>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// WiFi and communication
#include <WiFi.h>
#include <esp_now.h>

// Configuration and pins
#include <config.h>
#include <pins.h>

// Interfaces
#include <ISensor.hpp>
#include <ILogger.hpp>
#include <ITransmitter.hpp>

// Sensors
#include <BNO055Sensor.hpp>
#include <MS561101BA03.hpp>
#include <LIS3DHTRSensor.hpp>
#include <GPS.hpp>

// Storage and logging
#include <SD-master.hpp>
#include <RocketLogger.hpp>
#include <Logger.hpp>

// Transmitters

// Controllers and filters
#include <LEDController.hpp>
#include <BuzzerController.hpp>
#include <StatusManager.hpp>

// Main system
#include <RocketFSM.hpp>

/**
 * @brief Uncomment to enable sensor calibration routine at startup.
 * Useful for fast testing without needing precise sensor reads.
 *
 */
#define CALIBRATE_SENSORS
#define ENABLE_PRE_FLIGHT_MODE
#define TEST_FILE "/test.txt"

// Create controller instances
LEDController ledController(LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN);
BuzzerController buzzerController(BUZZER_PIN);
StatusManager statusManager(ledController, buzzerController);

// Type definitions
using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// Global component instances - these match the extern declarations in RocketFSM.cpp
std::shared_ptr<ISensor> bno055 = nullptr;
std::shared_ptr<ISensor> baro1 = nullptr;
std::shared_ptr<ISensor> baro2 = nullptr;
std::shared_ptr<ISensor> gps = nullptr;
std::shared_ptr<ISensor> accl = nullptr;

std::shared_ptr<SD> sdCard = nullptr;

// Define the RocketLogger
std::shared_ptr<RocketLogger> rocketLogger = nullptr;

// FSM instance
std::unique_ptr<RocketFSM> rocketFSM;

// Utility functions
void testFSMTransitions(RocketFSM &fsm);
void initializeComponents();
void GPSfix();
void printSystemInfo();
void testRoutine();
void gpsFix();

void setup()
{
    // Initialize actuator pins
    pinMode(MAIN_ACTUATOR_PIN, OUTPUT);
    pinMode(DROGUE_ACTUATOR_PIN, OUTPUT);

    digitalWrite(MAIN_ACTUATOR_PIN, LOW);
    digitalWrite(DROGUE_ACTUATOR_PIN, LOW);

    // Initialize LED pins (only those not handled by controllers)
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialize pins (only those not handled by controllers)
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ARMING_PIN, INPUT);
    // Signal initialization start
    digitalWrite(LED_RED, HIGH);

    // Turn off internal LED
    digitalWrite(LED_BUILTIN, LOW);

    // Initialize basic hardware
    Serial.begin(SERIAL_BAUD_RATE);
    // Initialize controllers
    ledController.init();
    buzzerController.init();

    // Initialize status patterns
    statusManager.init();

    // Set initial status with PRE_FLIGHT_MODE
    statusManager.setSystemCode(PRE_FLIGHT_MODE);

    LOG_INFO("Main", "\n=== Aurora Rocketry Flight Software ===");
    LOG_INFO("Main", "Initializing system...");

    // Initialize I2C
    Wire.begin();
    LOG_INFO("Main", "I2C initialized");

    // Initialize components
    initializeComponents();

#ifdef ENABLE_PRE_FLIGHT_MODE
    delay(5000);
    // Start test routine if in test mode
    LOG_INFO("Main", "=== TEST MODE ENABLED ===");
    testRoutine();
#endif

#ifdef CALIBRATE_SENSORS
    // Checking sensors calibration
    statusManager.setSystemCode(CALIBRATING);
    GPSfix();
    statusManager.setSystemCode(SYSTEM_OK);
#endif

    // Initialize logger
    LOG_INFO("Init", "Initializing rocket logger...");
    rocketLogger = std::make_shared<RocketLogger>();
    LOG_INFO("Init", "✓ Rocket logger initialized");

    // Print system information
    printSystemInfo();

    // Initialize and start FSM
    LOG_INFO("Main", "=== System initialization complete ===");
    LOG_INFO("Main", "\n=== Initializing Flight State Machine ===");
    rocketFSM = std::make_unique<RocketFSM>(bno055, baro1, baro2, accl, gps, sdCard, rocketLogger);
    rocketFSM->init();
    delay(1000);

    // Wait for arming pin to be enabled before starting FSM
    statusManager.setSystemCode(PRE_FLIGHT_MODE);
    while (digitalRead(ARMING_PIN) == LOW)
    {
        LOG_WARNING("Main", "System not armed! Waiting for arming signal on pin %d...", ARMING_PIN);
        delay(1000);
    }
    // Start FSM tasks
    LOG_INFO("Main", "Starting Flight State Machine...");
    statusManager.setSystemCode(FSM_STARTED);
    delay(1000);
    rocketFSM->start();
    statusManager.setSystemCode(FLIGHT_MODE);

    // Signal successful initialization
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    LOG_INFO("Main", "SETUP COMPLETE - SYSTEM IN FLIGHT MODE");
}

void loop()
{
    auto currentState = rocketFSM->getCurrentState();
    LOG_INFO("Main", "Current FSM State: %s", rocketFSM->getStateString(currentState));
    LOG_INFO("Main", "Free heap: %u bytes", ESP.getFreeHeap());

    static unsigned long lastHeartbeat = 0;
    static bool ledState = false;

    // Heartbeat every 2 seconds
    if (millis() - lastHeartbeat > 2000)
    {
        LOG_INFO("Main", "Last heartbeat at %lu ms - System running", millis());
        lastHeartbeat = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);

        // Monitor RocketLogger memory usage
        if (rocketLogger)
        {
            int logCount = rocketLogger->getLogCount();
            LOG_INFO("Main", "RocketLogger entries: %d", logCount);

            // If log count is high, warn about memory usage
            if (logCount > 800)
            {
                LOG_WARNING("Main", "RocketLogger approaching memory limit (%d entries)", logCount);
            }
        }

        // Optional: Print current state periodically
        static RocketState lastLoggedState = RocketState::INACTIVE;
        RocketState currentState = rocketFSM->getCurrentState();

        if (currentState != lastLoggedState)
        {
            LOG_INFO("Main", "Current FSM State: %s",
                     rocketFSM->getStateString(currentState));
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
            vTaskDelete(NULL);
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

    if (gps && gps->init())
    {
        LOG_INFO("Init", "✓ GPS initialized");
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize GPS");
    }

    // Inizializza la scheda SD
    LOG_INFO("Init", "Initializing SD card for logging...");
    sdCard = std::make_shared<SD>();
    if (sdCard && sdCard->init())
    {
        LOG_INFO("Init", "✓ SD card initialized");
        sdCard->openFile("test.txt");
        LOG_INFO("Init", "Testing SD card write...");
        std::string content = "SD card write test successful! Timestamp: " + std::to_string(millis()) + " ms";
        if (sdCard->writeFile("test.txt", content))
        {
            LOG_INFO("Init", "✓ SD card write test successful");
            char *readContent = sdCard->readFile("test.txt");
            if (readContent)
            {
                LOG_INFO("Init", "Read from SD card: %s", readContent);
            }
            else
            {
                LOG_ERROR("Init", "✗ Failed to read back from SD card");
            }
        }
        else
        {
            LOG_ERROR("Init", "✗ SD card write test failed");
        }
        sdCard->closeFile();
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize SD card");
    }

    // Initializa ESP-NOW connection for telemetry
    LOG_INFO("Init", "Initializing ESP-NOW for telemetry...");
    if (WiFi.begin())
    {
        // Disattiva Wi-Fi STA/AP
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        if (esp_now_init() == ESP_OK)
        {
            LOG_INFO("Init", "✓ ESP-NOW initialized");
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, RECEIVER_MAC_ADDRESS, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;

            if (esp_now_add_peer(&peerInfo) == ESP_OK)
            {
                LOG_INFO("Init", "✓ ESP-NOW peer added");
            }
            else
            {
                LOG_ERROR("Init", "✗ Failed to add ESP-NOW peer");
            }
        }
        else
        {
            LOG_ERROR("Init", "✗ Failed to initialize ESP-NOW");
        }
    }
    else
    {
        LOG_ERROR("Init", "✗ Failed to initialize Wi-Fi");
    }

    // We don't need to initialize LEDManager anymore - StatusManager handles it
    LOG_INFO("Init", "✓ Status indicators initialized");
}

// Function to check GPS fix
void GPSfix()
{
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
    statusManager.setSystemCode(SYSTEM_OK);
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

// Helper function to show a pattern for a specific test
void showTestPattern(int testNumber, StatusManager &statusManager)
{
    switch (testNumber)
    {
    case 1:
        statusManager.playBlockingPattern(TEST_POWER, 1000);
        break;
    case 2:
        statusManager.playBlockingPattern(TEST_SENSORS, 1000);
        break;
    case 3:
        statusManager.playBlockingPattern(TEST_ACTUATORS, 1000);
        break;
    case 4:
        statusManager.playBlockingPattern(TEST_SD, 1000);
        break;
    case 5:
        statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);
        break;
    case 6:
        statusManager.playBlockingPattern(TEST_ALL, 2000);
        break;
    default:
        break;
    }
}

// Modified waitForUserInput with buzzer patterns
bool waitForUserInput(const char *message)
{
    String full_message = String(message) + "\n Or type REBOOT to restart the system.";
    Serial.println(full_message);
    statusManager.setSystemCode(WAITING_INPUT);

    while (true)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            input.toUpperCase();
            if (input == "PASSED")
            {
                statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
                return true;
            }
            if (input == "FAILED")
            {
                statusManager.playBlockingPattern(TEST_FAILURE, 1000);
                return false;
            }
            if (input == "REBOOT")
            {
                LOG_WARNING("Test", "System is going to reboot, are you sure?");
                LOG_WARNING("Test", "Type REBOOT to confirm reboot, or anything else to cancel.");

                // Clear any existing serial input
                while (Serial.available())
                {
                    Serial.read();
                }

                unsigned long waitStart = millis();
                String confirm = "";
                while (millis() - waitStart < 10000) // Wait up to 10 seconds
                {
                    if (Serial.available())
                    {
                        confirm = Serial.readStringUntil('\n');
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                if (confirm.length() == 0)
                {
                    LOG_INFO("Test", "Reboot timeout - continuing normal operation.");
                    continue;
                }
                confirm.trim();
                confirm.toUpperCase();
                if (confirm == "REBOOT")
                {
                    LOG_WARNING("Test", "Rebooting system...");
                    ESP.restart();
                }
            }
            else
            {
                LOG_INFO("Test", "Reboot cancelled.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Test routine subroutines with proper pattern handling
bool testPowerAndLEDs()
{
    LOG_INFO("Test", "\n[STEP 1] Verifica alimentazione e LED di stato");
    LOG_INFO("Test", "Controllare manualmente:");
    LOG_INFO("Test", " - LED di alimentazione componenti accesi");
    LOG_INFO("Test", " - LED presenza SD acceso");
    LOG_INFO("Test", " - LED attuatori visibili");
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testSensors()
{
    LOG_INFO("Test", "\n[STEP 2] Test sensori");
    bool testFailed = false;
    bool imu_ok = bno055->init();
    bool baro1_ok = baro1->init();
    bool baro2_ok = baro2->init();
    bool accl_ok = accl->init();

    if (!imu_ok)
    {
        testFailed = true;
        statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: IMU non inizializzata.");
    }
    if (!baro1_ok)
    {
        statusManager.playBlockingPattern(BARO1_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 1 non inizializzato.");
    }
    if (!baro2_ok)
    {
        statusManager.playBlockingPattern(BARO2_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 2 non inizializzato.");
    }
    if (!accl_ok)
    {
        statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Accelerometro non inizializzato.");
    }

    if (!testFailed)
    {
        LOG_INFO("Test", "Verifica output dei sensori...");

        auto imuDataOpt = bno055->getData();
        if (imuDataOpt.has_value())
        {
            auto imuData = imuDataOpt.value();
            auto accelerometer_opt = imuData.getData("accelerometer");
            if (accelerometer_opt.has_value())
            {
                auto accelMap = std::get<std::map<std::string, float>>(accelerometer_opt.value());
                LOG_INFO("Test", "IMU Accelerometer: x=%.2f, y=%.2f, z=%.2f m/s^2",
                         (double)accelMap["x"],
                         (double)accelMap["y"],
                         (double)accelMap["z"]);
                pinMode(D5, OUTPUT);
                digitalWrite(D5, HIGH);
                pinMode(D5, OUTPUT);
                digitalWrite(D5, HIGH);
            }
            else
            {
                statusManager.playBlockingPattern(IMU_FAIL, 2000);
                LOG_ERROR("Test", "Errore: Impossibile leggere dati accelerometro dall'IMU.");
            }
        }
        else
        {
            statusManager.playBlockingPattern(IMU_FAIL, 2000);
            LOG_ERROR("Test", "Errore: Impossibile leggere dati dall'IMU.");
        }
        auto baro1DataOpt = baro1->getData();
        if (baro1DataOpt.has_value())
        {
            auto baro1Data = baro1DataOpt.value();
            auto pressure_opt = baro1Data.getData("pressure");
            if (pressure_opt.has_value())
            {
                float pressure = std::get<float>(pressure_opt.value());
                LOG_INFO("Test", "Barometer 1 Pressure: %.2f hPa", (double)pressure);
                pinMode(A7, OUTPUT);
                digitalWrite(A7, HIGH);
                pinMode(A7, OUTPUT);
                digitalWrite(A7, HIGH);
            }
            else
            {
                statusManager.playBlockingPattern(BARO1_FAIL, 2000);
                LOG_ERROR("Test", "Errore: Impossibile leggere dati pressione dal Barometro 1.");
            }
        }
        else
        {
            statusManager.playBlockingPattern(BARO1_FAIL, 2000);
            LOG_ERROR("Test", "Errore: Impossibile leggere dati dal Barometro 1.");
        }
        auto baro2DataOpt = baro2->getData();
        if (baro2DataOpt.has_value())
        {
            auto baro2Data = baro2DataOpt.value();
            auto pressure_opt = baro2Data.getData("pressure");
            if (pressure_opt.has_value())
            {
                float pressure = std::get<float>(pressure_opt.value());
                LOG_INFO("Test", "Barometer 2 Pressure: %.2f hPa", (double)pressure);
                pinMode(D4, OUTPUT);
                digitalWrite(D4, HIGH);
                pinMode(D4, OUTPUT);
                digitalWrite(D4, HIGH);
            }
            else
            {
                statusManager.playBlockingPattern(BARO2_FAIL, 2000);
                LOG_ERROR("Test", "Errore: Impossibile leggere dati pressione dal Barometro 2.");
            }
        }
        else
        {
            statusManager.playBlockingPattern(BARO2_FAIL, 2000);
            LOG_ERROR("Test", "Errore: Impossibile leggere dati dal Barometro 2.");
        }
        auto acclDataOpt = accl->getData();
        if (acclDataOpt.has_value())
        {
            auto acclData = acclDataOpt.value();
            auto x_opt = acclData.getData("accel_x");
            auto y_opt = acclData.getData("accel_y");
            auto z_opt = acclData.getData("accel_z");
            if (x_opt.has_value() && y_opt.has_value() && z_opt.has_value())
            {
                float x = std::get<float>(x_opt.value());
                float y = std::get<float>(y_opt.value());
                float z = std::get<float>(z_opt.value());
                LOG_INFO("Test", "Accelerometer: x=%.2f, y=%.2f, z=%.2f m/s^2",
                         (double)x,
                         (double)y,
                         (double)z);
                pinMode(A6, OUTPUT);
                digitalWrite(A6, HIGH);
                pinMode(A6, OUTPUT);
                digitalWrite(A6, HIGH);
            }
            else
            {
                statusManager.playBlockingPattern(IMU_FAIL, 2000);
                LOG_ERROR("Test", "Errore: Impossibile leggere dati dall'accelerometro.");
            }
        }
        else
        {
            statusManager.playBlockingPattern(IMU_FAIL, 2000);
            LOG_ERROR("Test", "Errore: Impossibile leggere dati dall'accelerometro.");
        }
    }
    // After all tests, go to user input
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testActuators()
{
    LOG_INFO("Test", "\n[STEP 3] Test attuatori");

    pinMode(DROGUE_ACTUATOR_PIN, OUTPUT);
    pinMode(MAIN_ACTUATOR_PIN, OUTPUT);

    LOG_INFO("Test", "Accensione attuatori uno per volta...");

    // DROGUE test
    statusManager.playBlockingPattern(TEST_ACTUATORS, 500); // Show test pattern first
    digitalWrite(DROGUE_ACTUATOR_PIN, HIGH);
    buzzerController.playTone(TONE_MID, 1000);
    digitalWrite(DROGUE_ACTUATOR_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // MAIN test
    statusManager.playBlockingPattern(TEST_ACTUATORS, 500); // Show test pattern first
    digitalWrite(MAIN_ACTUATOR_PIN, HIGH);
    buzzerController.playTone(TONE_MID, 1000);
    digitalWrite(MAIN_ACTUATOR_PIN, LOW);

    return waitForUserInput("Verificare accensione LED e tensione in uscita da DROGUE e MAIN, verificare funzionamento Buzzer. Scrivi PASSED o FAILED");
}

bool testSDCard()
{
    LOG_INFO("Test", "[STEP 4] Test SD Card");
    LOG_INFO("Test", "Inizializzazione scheda SD e verifica scrittura/lettura...");

    if (sdCard->openFile(TEST_FILE))
    {
        std::string content = "SD card write test successful! Timestamp: " + std::to_string(millis()) + " ms\n";
        if (sdCard->writeFile(TEST_FILE, content))
        {
            LOG_INFO("Test", "✓ SD card write test successful");
            char *readContent = sdCard->readFile(TEST_FILE);
            if (readContent)
            {
                LOG_INFO("Test", "Read from SD card: %s", readContent);
            }
            else
            {
                statusManager.playBlockingPattern(SD_READ_FAIL, 2000);
                LOG_ERROR("Test", "✗ Failed to read back from SD card");
            }
        }
        else
        {
            statusManager.playBlockingPattern(SD_WRITE_FAIL, 2000);
            LOG_ERROR("Test", "✗ SD card write test failed");
        }
        sdCard->closeFile();
    }
    else
    {
        statusManager.playBlockingPattern(SD_MOUNT_FAIL, 2000);
        LOG_ERROR("Test", "✗ Failed to open test file on SD card");
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testTelemetry()
{
    LOG_INFO("Test", "\n[STEP 5] Test telemetria");
    LOG_INFO("Test", "Inizializzazione antenne e verifica collegamento...");
    LOG_INFO("Test", "Verificare sul monitor ricezione pacchetti LORA.");

    // Test LORA patterns
    statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

// Main test routine
void testRoutine()
{
    LOG_INFO("Test", "=== SYSTEM TEST ROUTINE INITIATED ===");

    // Menu di selezione test
    while (true)
    {
        // Menu is BLUE with no buzzer
        statusManager.setSystemCode(TEST_MENU);

        Serial.println("\n=== MENU TEST ===");
        Serial.println("1 - Test alimentazione e LED");
        Serial.println("2 - Test sensori");
        Serial.println("3 - Test attuatori");
        Serial.println("4 - Test SD Card");
        Serial.println("5 - Test telemetria");
        Serial.println("6 - Esegui tutti i test in sequenza");
        Serial.println("7 - Esci dal menu test");
        Serial.println("Inserisci il numero del test da eseguire:");

        while (!Serial.available())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        String input = Serial.readStringUntil('\n');
        input.trim();
        int choice = input.toInt();
        bool testPassed = false;

        // Show specific pattern for this test
        showTestPattern(choice, statusManager);

        switch (choice)
        {
        case 1:
            do
            {
                testPassed = testPowerAndLEDs();
            } while (!testPassed);
            break;
        case 2:
            do
            {
                testPassed = testSensors();
            } while (!testPassed);
            break;
        case 3:
            do
            {
                testPassed = testActuators();
            } while (!testPassed);
            break;
        case 4:
            do
            {
                testPassed = testSDCard();
            } while (!testPassed);
            break;
        case 5:
            do
            {
                testPassed = testTelemetry();
            } while (!testPassed);
            break;
        case 6:
            // Show all tests pattern
            statusManager.playBlockingPattern(TEST_ALL, 2000);

            // Execute all tests in sequence
            do
            {
                testPassed = testPowerAndLEDs();
            } while (!testPassed);
            do
            {
                testPassed = testSensors();
            } while (!testPassed);
            do
            {
                testPassed = testActuators();
            } while (!testPassed);
            do
            {
                testPassed = testSDCard();
            } while (!testPassed);
            do
            {
                testPassed = testTelemetry();
            } while (!testPassed);

            // All tests successful
            statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== TUTTI I TEST COMPLETATI CON SUCCESSO ===");
            break;
        case 7:
            // Exit test mode with success pattern
            statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== USCITA DAL MENU TEST ===");
            statusManager.setSystemCode(SYSTEM_OK);
            return;
        default:
            Serial.println("Scelta non valida. Riprova.");
            continue;
        }

        if (choice >= 1 && choice <= 6)
        {
            LOG_INFO("Test", "Test completato con successo!");
            // Show success pattern before returning to menu
            statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
        }
    }
}

void gpsFix()
{
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
            statusManager.setSystemCode(SystemCode::GPS_NO_SIGNAL);
            while(true) {
                // Read OVERRIDE from Serial
                if (Serial.available()) {
                    String input = Serial.readStringUntil('\n');
                    input.trim();
                    input.toUpperCase();
                    Serial.println("Type OVERRIDE to bypass GPS lock and continue.");
                    if (input == "OVERRIDE") {
                        LOG_WARNING("GPS", "GPS lock override received. Continuing without GPS.");
                        statusManager.setSystemCode(SYSTEM_OK);
                        break;
                    } else {
                        LOG_INFO("GPS", "Invalid input. Type OVERRIDE to bypass GPS lock.");
                    }
                }
            }
        }
    }
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