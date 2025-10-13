#include "StatusManager.hpp"
#include <Arduino.h>

StatusManager::StatusManager(LEDController &ledController, BuzzerController &buzzerController)
    : ledController(ledController), buzzerController(buzzerController),
      currentCode(UNKNOWN_ERROR), statusTaskHandle(nullptr)
{
}

void StatusManager::init()
{
    // Initialize pattern map with all patterns from specification
    codeToPattern = {
        // Standard status patterns
        {SYSTEM_OK, {{ART_LED_GREEN, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_SUCCESS, TONE_SUCCESS, false}},
        {PRE_FLIGHT_MODE, {{ART_LED_BLUE, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {CALIBRATING, {{ART_LED_YELLOW, ART_LED_OFF, 1000, 1000, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {WAITING_INPUT, {{ART_LED_CYAN, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_MID, false}},
        {FLIGHT_MODE, {{ART_LED_GREEN, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {FSM_STARTED, {{ART_LED_ORANGE, ART_LED_OFF, 500, 500, 0, 0}, BUZZER_CONTINUOUS, TONE_MID, true}},

        // Sensor hardware failures
        {IMU_FAIL, {{ART_LED_RED, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {IMU_DATA_INVALID, {{ART_LED_RED, ART_LED_OFF, 200, 200, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},
        {BARO1_FAIL, {{ART_LED_RED, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {BARO1_DATA_INVALID, {{ART_LED_RED, ART_LED_OFF, 200, 200, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},
        {BARO2_FAIL, {{ART_LED_RED, ART_LED_OFF, 800, 400, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {BARO2_DATA_INVALID, {{ART_LED_RED, ART_LED_OFF, 800, 400, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},
        {GPS_NO_SIGNAL, {{ART_LED_RED, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {GPS_DATA_INVALID, {{ART_LED_RED, ART_LED_OFF, 200, 200, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},

        // SD card errors
        {SD_MOUNT_FAIL, {{ART_LED_RED, ART_LED_OFF, 300, 300, 3, 0}, BUZZER_WARNING, TONE_ERROR, false}},
        {SD_WRITE_FAIL, {{ART_LED_RED, ART_LED_OFF, 800, 400, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {SD_READ_FAIL, {{ART_LED_RED, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},

        // // LoRa errors
        // {LORA_INIT_FAIL, {{ART_LED_RED, ART_LED_YELLOW, 400, 400, 0, 0}, BUZZER_WARNING, TONE_ERROR, false}},
        // {LORA_CONFIG_FAIL, {{ART_LED_RED, ART_LED_CYAN, 400, 400, 0, 0}, BUZZER_WARNING, TONE_ERROR, false}},
        // {LORA_TX_FAIL, {{ART_LED_RED, ART_LED_BLUE, 400, 400, 0, 0}, BUZZER_WARNING, TONE_ERROR, false}},

        // // System errors
        // {MEMORY_ERROR, {{ART_LED_YELLOW, ART_LED_RED, 300, 300, 0, 0}, BUZZER_SOS, TONE_ERROR, false}},
        // {TASK_FAIL, {{ART_LED_YELLOW, ART_LED_OFF, 200, 200, 3, 0}, BUZZER_TRIPLE_BEEP, TONE_ERROR, false}},
        // {MUTEX_ERROR, {{ART_LED_YELLOW, ART_LED_OFF, 300, 300, 4, 0}, BUZZER_WARNING, TONE_ERROR, false}},

        // // Power and unknown errors
        // {POWER_LOW_WARNING, {{ART_LED_YELLOW, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_WARNING, TONE_ERROR, true}},
        // {UNKNOWN_ERROR, {{ART_LED_MAGENTA, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SOS, TONE_ERROR, false}},

        // // Flight status indicators
        // {LAUNCH_READY, {{ART_LED_GREEN, ART_LED_BLUE, 500, 500, 0, 0x01}, BUZZER_COUNTDOWN, TONE_MID, false}},
        // {LAUNCH_DETECTED, {{ART_LED_WHITE, ART_LED_OFF, 100, 100, 0, 0x03}, BUZZER_CONTINUOUS, TONE_HIGH, true}},
        // {APOGEE_DETECTED, {{ART_LED_CYAN, ART_LED_OFF, 100, 100, 0, 0x07}, BUZZER_SUCCESS, TONE_SUCCESS, false}},
        // {DROGUE_DEPLOY, {{ART_LED_YELLOW, ART_LED_OFF, 300, 300, 0, 0x0F}, BUZZER_SHORT_BEEP, TONE_MID, true}},
        // {MAIN_DEPLOY, {{ART_LED_GREEN, ART_LED_OFF, 300, 300, 0, 0x1F}, BUZZER_DOUBLE_BEEP, TONE_MID, true}},
        // {LANDING_DETECTED, {{ART_LED_BLUE, ART_LED_GREEN, 500, 500, 0, 0x3F}, BUZZER_SUCCESS, TONE_SUCCESS, false}},
        // {RECOVERY_COMPLETE, {{ART_LED_GREEN, ART_LED_OFF, 0, 0, 0, 0x7F}, BUZZER_SUCCESS, TONE_SUCCESS, false}},

        // Test mode specific patterns
        {TEST_MENU, {{ART_LED_BLUE, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},                    // Fixed blue for menu
        {TEST_POWER, {{ART_LED_WHITE, ART_LED_OFF, 0, 0, 0, 0x01}, BUZZER_SHORT_BEEP, TONE_MID, false}},        // White for power test
        {TEST_SENSORS, {{ART_LED_RED, ART_LED_OFF, 0, 0, 0, 0x02}, BUZZER_SHORT_BEEP, TONE_MID, false}},        // Red for sensor test
        {TEST_ACTUATORS, {{ART_LED_YELLOW, ART_LED_OFF, 0, 0, 0, 0x04}, BUZZER_SHORT_BEEP, TONE_MID, false}},   // Yellow for actuator test
        {TEST_SD, {{ART_LED_ORANGE, ART_LED_OFF, 0, 0, 0, 0x08}, BUZZER_SHORT_BEEP, TONE_MID, false}},           // Green for SD test
        {TEST_TELEMETRY, {{ART_LED_MAGENTA, ART_LED_OFF, 0, 0, 0, 0x10}, BUZZER_SHORT_BEEP, TONE_MID, false}},  // Magenta for telemetry
        {TEST_ALL, {{ART_LED_WHITE, ART_LED_BLUE, 500, 500, 0, 0x1F}, BUZZER_DOUBLE_BEEP, TONE_MID, false}},    // All tests
        {TEST_SUCCESS, {{ART_LED_GREEN, ART_LED_OFF, 200, 200, 3, 0x3F}, BUZZER_SUCCESS, TONE_SUCCESS, false}}, // Success pattern
        {TEST_FAILURE, {{ART_LED_RED, ART_LED_OFF, 200, 200, 3, 0}, BUZZER_ERROR, TONE_ERROR, false}}           // Failure pattern
    };
}

void StatusManager::setSystemCode(SystemCode code)
{
    // Stop current patterns
    stopAllPatterns();

    // Update current code
    currentCode = code;

    // Find pattern for this code
    if (codeToPattern.find(code) == codeToPattern.end())
    {
        return; // Pattern not found
    }

    StatusPattern pattern = codeToPattern[code];

    // Special cases that need task-based coordination
    if (pattern.buzzerSync)
    {
        // Create task context for coordinated patterns
        TaskContext *ctx = new TaskContext{
            .instance = this,
            .code = code,
            .pattern = pattern};

        // Start coordination task
        xTaskCreatePinnedToCore(
            statusTaskFunction,
            "StatusTask",
            2048,
            ctx,
            1,
            &statusTaskHandle,
            1 // Run on core 1
        );
    }
    else
    {
        // Start LED pattern
        ledController.startPattern(pattern.ledPattern);

        // Start buzzer pattern
        buzzerController.startPattern(pattern.buzzerPattern, pattern.buzzerTone);
    }
}

void StatusManager::stopAllPatterns()
{
    // Stop synchronized task if running
    if (statusTaskHandle != nullptr)
    {
        vTaskDelete(statusTaskHandle);
        statusTaskHandle = nullptr;
    }

    // Stop individual controllers
    ledController.stopPattern();
    buzzerController.stopPattern();
}

const StatusPattern &StatusManager::getPattern(SystemCode code) const
{
    static StatusPattern empty{};
    auto it = codeToPattern.find(code);
    if (it != codeToPattern.end())
        return it->second;
    return empty;
}

uint32_t StatusManager::getPatternDuration(SystemCode code) const
{
    const StatusPattern &pattern = getPattern(code);

    if (pattern.ledPattern.times > 0)
    {
        // Fixed number of blinks
        return (pattern.ledPattern.duration + pattern.ledPattern.pause) * pattern.ledPattern.times + 200;
    }
    else if (pattern.ledPattern.duration == 0 && pattern.ledPattern.pause == 0)
    {
        // Solid color
        return 0; // Infinite
    }
    else
    {
        // Continuous blinking - default to showing at least a few cycles
        return (pattern.ledPattern.duration + pattern.ledPattern.pause) * 3;
    }
}

void StatusManager::playBlockingPattern(SystemCode code, uint32_t minDuration)
{
    // Set the pattern
    setSystemCode(code);

    // Get pattern duration
    uint32_t duration = getPatternDuration(code);

    // If pattern is continuous (duration = 0) or minDuration is specified, use minDuration
    if (duration == 0 || (minDuration > 0 && minDuration > duration))
    {
        duration = minDuration;
    }

    // Wait for pattern to complete
    if (duration > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(duration));
    }
    else
    {
        // For infinite patterns, just show for a reasonable time
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void StatusManager::statusTaskFunction(void *param)
{
    // Extract context
    TaskContext *ctx = static_cast<TaskContext *>(param);
    StatusManager *instance = ctx->instance;
    StatusPattern pattern = ctx->pattern;
    SystemCode code = ctx->code;

    // Free context memory
    delete ctx;

    // Handle special case patterns that need coordination

    // Special patterns like SD_MOUNT_FAIL, MEMORY_ERROR, MUTEX_ERROR
    if (code == SD_MOUNT_FAIL)
    {
        while (true)
        {
            // 1-2 sequence with synchronized buzzer
            // 1 blink
            instance->ledController.setColor(ART_LED_RED);
            if (pattern.buzzerSync)
                instance->buzzerController.playToneFreq(pattern.buzzerTone);
            vTaskDelay(pdMS_TO_TICKS(300));
            instance->ledController.setOff();
            if (pattern.buzzerSync)
                instance->buzzerController.stopTone();
            vTaskDelay(pdMS_TO_TICKS(300));

            // Pause
            vTaskDelay(pdMS_TO_TICKS(500));

            // 2 blinks
            for (int i = 0; i < 2; i++)
            {
                instance->ledController.setColor(ART_LED_RED);
                if (pattern.buzzerSync)
                    instance->buzzerController.playToneFreq(pattern.buzzerTone);
                vTaskDelay(pdMS_TO_TICKS(300));
                instance->ledController.setOff();
                if (pattern.buzzerSync)
                    instance->buzzerController.stopTone();
                vTaskDelay(pdMS_TO_TICKS(300));
            }

            // Long pause
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Default synchronized behavior
    while (true)
    {
        // LED on
        instance->ledController.setColor(pattern.ledPattern.color1);

        // Buzzer on if synchronized
        if (pattern.buzzerSync)
            instance->buzzerController.playToneFreq(pattern.buzzerTone);

        // Hold for duration
        vTaskDelay(pdMS_TO_TICKS(pattern.ledPattern.duration));

        // LED off
        instance->ledController.setOff();

        // Buzzer off if synchronized
        if (pattern.buzzerSync)
            instance->buzzerController.stopTone();

        // Pause
        vTaskDelay(pdMS_TO_TICKS(pattern.ledPattern.pause));
    }

    // Task is done
    instance->statusTaskHandle = nullptr;
    vTaskDelete(NULL);
}