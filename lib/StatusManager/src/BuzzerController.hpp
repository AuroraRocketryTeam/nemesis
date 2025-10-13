#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Buzzer tones (frequencies in Hz)
enum BuzzerTone
{
    TONE_OFF = 0,
    TONE_LOW = 220,    // Low A
    TONE_MID = 440,    // Middle A
    TONE_HIGH = 880,   // High A
    TONE_ALERT = 1760, // Very high A
    TONE_ERROR = 330,  // E - creates dissonance for error sounds
    TONE_SUCCESS = 587 // D - pleasant for success sounds
};

// Buzzer patterns
enum BuzzerPattern
{
    BUZZER_OFF,
    BUZZER_CONTINUOUS,  // Constant tone
    BUZZER_SHORT_BEEP,  // Single short beep
    BUZZER_DOUBLE_BEEP, // Two short beeps
    BUZZER_TRIPLE_BEEP, // Three short beeps
    BUZZER_SOS,         // SOS pattern (... --- ...)
    BUZZER_WARNING,     // Alternating high/low tone
    BUZZER_COUNTDOWN,   // Descending tones
    BUZZER_SUCCESS,     // Rising tone sequence
    BUZZER_ERROR        // Error pattern
};

// Buzzer Pattern Definition
struct BuzzerSequence
{
    BuzzerTone tone;   // Frequency
    uint16_t duration; // Duration in ms
    uint16_t pause;    // Pause after the tone
};

class BuzzerController
{
private:
    // Buzzer pin
    uint8_t buzzerPin;

    // Current active pattern
    TaskHandle_t buzzerTaskHandle;

    // Internal task context structure
    struct TaskContext
    {
        BuzzerController *instance;
        BuzzerPattern pattern;
        BuzzerTone baseTone; // Base tone for the pattern
    };

    // Static task function for FreeRTOS
    static void buzzerTaskFunction(void *param);

    // Internal pattern implementation
    void playPatternInternal(BuzzerPattern pattern, BuzzerTone baseTone = TONE_MID);

public:
    // Constructor & Destructor
    BuzzerController(uint8_t buzzerPin);
    ~BuzzerController();

    // Initialize pins
    void init();

    // Control buzzer
    void startPattern(BuzzerPattern pattern, BuzzerTone baseTone = TONE_MID);
    void stopPattern();

    // Simple direct controls
    void playTone(BuzzerTone frequency, uint16_t duration);
    void playSequence(const BuzzerSequence *sequence, uint8_t count, bool repeat = false);
    void setOff();

    // Play a single tone
    void playToneFreq(uint16_t frequency);
    void stopTone();
};