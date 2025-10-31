#pragma once
#include "LEDController.hpp"
#include "BuzzerController.hpp"
#include <map>

// System status codes
enum SystemCode
{
    SYSTEM_OK = 0,
    PRE_FLIGHT_MODE,
    CALIBRATING,
    WAITING_INPUT,
    FLIGHT_MODE,
    FSM_STARTED,

    IMU_FAIL = 20,
    IMU_DATA_INVALID,
    BARO1_FAIL,
    BARO1_DATA_INVALID,
    BARO2_FAIL,
    BARO2_DATA_INVALID,
    GPS_NO_SIGNAL,
    GPS_DATA_INVALID,

    SD_MOUNT_FAIL = 40,
    SD_WRITE_FAIL,
    SD_READ_FAIL,

    LORA_INIT_FAIL = 50,
    LORA_CONFIG_FAIL,
    LORA_TX_FAIL,

    MEMORY_ERROR = 80,
    TASK_FAIL,
    MUTEX_ERROR,

    POWER_LOW_WARNING = 90,
    UNKNOWN_ERROR = 99,

    // Additional flight status codes
    LAUNCH_READY = 100,
    LAUNCH_DETECTED,
    APOGEE_DETECTED,
    DROGUE_DEPLOY,
    MAIN_DEPLOY,
    LANDING_DETECTED,
    RECOVERY_COMPLETE,

    // Test mode status codes
    TEST_MENU = 200,
    TEST_POWER,
    TEST_SENSORS,
    TEST_ACTUATORS,
    TEST_SD,
    TEST_TELEMETRY,
    TEST_ALL,
    TEST_SUCCESS,
    TEST_FAILURE
};

// Combined pattern definition
struct StatusPattern
{
    LEDPattern ledPattern;
    BuzzerPattern buzzerPattern;
    BuzzerTone buzzerTone;
    bool buzzerSync; // Whether to sync buzzer with LED
};

/**
 * @brief Class to manage system status using LED and Buzzer controllers.
 * 
 */
class StatusManager
{
private:
    // Controllers
    LEDController &ledController;
    BuzzerController &buzzerController;

    // Current status
    SystemCode currentCode;

    // Pattern mapping
    std::map<SystemCode, StatusPattern> codeToPattern;

    // Task handles for special pattern handling
    TaskHandle_t statusTaskHandle;

    // Internal task context
    struct TaskContext
    {
        StatusManager *instance;
        SystemCode code;
        StatusPattern pattern;
    };

    // Static task function for special patterns
    static void statusTaskFunction(void *param);

public:
    // Constructor
    StatusManager(LEDController &ledController, BuzzerController &buzzerController);

    // Initialize the pattern map
    void init();

    // Set system status
    void setSystemCode(SystemCode code);

    // Get current system code
    SystemCode getCurrentCode() const { return currentCode; }

    // Get pattern for a system code
    const StatusPattern &getPattern(SystemCode code) const;

    // Calculate duration of a pattern
    uint32_t getPatternDuration(SystemCode code) const;

    // Stop any running patterns
    void stopAllPatterns();

    // Play a blocking pattern (waits for completion)
    void playBlockingPattern(SystemCode code, uint32_t minDuration = 0);
};