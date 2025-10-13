#include "Logger.hpp"
#include "esp_heap_caps.h"
#include <Arduino.h>

// Serial mutex for thread-safe printing
static SemaphoreHandle_t serialMutex = nullptr;

namespace Logger
{

    void init()
    {
        if (serialMutex == nullptr)
        {
            serialMutex = xSemaphoreCreateMutex();
        }
    }

    void log(LogLevel level, const char *tag, const char *format, ...)
    {
        if (serialMutex == nullptr)
        {
            init();
        }

        // Use shorter timeout to detect deadlocks faster
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            char buffer[256];
            va_list args;
            va_start(args, format);

            // Add timestamp, log level, and tag
            unsigned long timestamp = millis();
            const char *levelStr = "";
            switch (level)
            {
            case LogLevel::ERROR:
                levelStr = "ERROR";
                break;
            case LogLevel::WARNING:
                levelStr = "WARN ";
                break;
            case LogLevel::INFO:
                levelStr = "INFO ";
                break;
            case LogLevel::DEBUG:
                levelStr = "DEBUG";
                break;
            case LogLevel::TRACE:
                levelStr = "TRACE";
                break;
            }

            // Print header with timestamp and tag
            Serial.printf("[%8lu][%s][%s] ", timestamp, levelStr, tag);

            // Print the actual message
            vsnprintf(buffer, sizeof(buffer), format, args);
            Serial.println(buffer);

            va_end(args);
            xSemaphoreGive(serialMutex);
        }
        else
        {
            // DEADLOCK DETECTION: Mutex timeout - likely a task was deleted while holding it
            // Print directly without mutex as emergency fallback (not thread-safe but better than hanging)
            Serial.printf("[DEADLOCK] Failed to acquire log mutex for: %s\n", tag);
        }
    }

    void debugMemory(const char *location)
    {
        if (serialMutex == nullptr)
        {
            init();
        }

        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            size_t freeHeap = ESP.getFreeHeap();
            size_t maxAlloc = ESP.getMaxAllocHeap();
            size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

            float fragmentation = 100.0f * (1.0f - (float)largestBlock / freeHeap);

            Serial.printf("\n=== MEMORY DEBUG [%s] ===\n", location);
            Serial.printf("Free heap: %u bytes\n", freeHeap);
            Serial.printf("Min free heap: %u bytes\n", ESP.getMinFreeHeap());
            Serial.printf("Max alloc heap: %u bytes\n", maxAlloc);
            Serial.printf("Fragmentation: %.2f%%\n", fragmentation);
            Serial.printf("Largest free block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            Serial.printf("==========================\n\n");

            if (fragmentation > 20.0f)
            {
                Serial.println("[WARNING] High memory fragmentation detected!");
            }
            if (freeHeap < 10000)
            {
                Serial.println("[WARNING] Low free heap memory!");
            }
            if (maxAlloc < 5000)
            {
                Serial.println("[WARNING] Low maximum allocatable block!");
            }
            if (largestBlock < 2000)
            {
                Serial.println("[WARNING] Very small largest free block!");
            }
            xSemaphoreGive(serialMutex);
        }
    }

    SemaphoreHandle_t getSerialMutex()
    {
        if (serialMutex == nullptr)
        {
            init();
        }
        return serialMutex;
    }
}