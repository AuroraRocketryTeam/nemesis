#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Available colors
enum LEDColor
{
    ART_LED_OFF,
    ART_LED_RED,
    ART_LED_GREEN,
    ART_LED_BLUE,
    ART_LED_YELLOW,
    ART_LED_CYAN,
    ART_LED_MAGENTA,
    ART_LED_WHITE,
    ART_LED_ORANGE // Simulated with PWM values
};

// LED pattern definition
struct LEDPattern
{
    LEDColor color1;
    LEDColor color2;   // For alternating colors
    uint16_t duration; // ms on time
    uint16_t pause;    // ms pause time
    uint8_t times;     // Number of blinks (0 = continuous)
    uint8_t auxLeds;   // Bitmask for additional LEDs
};

class LEDController
{
private:
    // LED pins
    uint8_t redPin;
    uint8_t greenPin;
    uint8_t bluePin;

    // Additional status LEDs
    uint8_t statusLedCount;
    uint8_t *statusLedPins;

    // Current active pattern
    TaskHandle_t ledTaskHandle;

    // Internal task context structure
    struct TaskContext
    {
        LEDController *instance;
        LEDPattern pattern;
    };

    // RGB LED control
    void setRGB(uint8_t r, uint8_t g, uint8_t b);

    // Status LEDs control
    void setStatusLeds(uint8_t mask);

    // LED pattern display helper
    void showOutputLED(LEDColor color, uint16_t duration_ms, uint16_t pause_ms, uint8_t times);

    // Static task function for FreeRTOS
    static void ledTaskFunction(void *param);

public:
    // Constructor & Destructor
    LEDController(uint8_t redPin, uint8_t greenPin, uint8_t bluePin);
    ~LEDController();

    // Add additional status LEDs
    void addStatusLeds(uint8_t *pins, uint8_t count);

    // Initialize pins
    void init();

    // Control LED pattern
    void startPattern(const LEDPattern &pattern);
    void stopPattern();

    // Simple direct controls
    void setColor(LEDColor color);
    void setOff();

    // Get RGB values for a color
    static void getRGBValues(LEDColor color, uint8_t &r, uint8_t &g, uint8_t &b);
};