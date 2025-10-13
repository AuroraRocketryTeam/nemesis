#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace Logger
{
    // Log levels for structured logging
    enum class LogLevel
    {
        ERROR,
        WARNING,
        INFO,
        DEBUG,
        TRACE
    };

    /// @brief Initialize the logger (creates the serial mutex if not already created).
    void init();

    /// @brief Thread-safe log function with printf-style formatting.
    /// @param level Log severity level
    /// @param tag Identifier for the log source
    /// @param format printf-style format string
    /// @param ... Arguments for format string
    void log(LogLevel level, const char *tag, const char *format, ...);

    /// @brief Prints memory usage statistics with fragmentation info.
    /// @param location Identifier string for the debug location
    void debugMemory(const char *location);

    /// @brief Returns the Serial print mutex handle (creates it if not initialized).
    /// @return SemaphoreHandle_t The mutex handle
    SemaphoreHandle_t getSerialMutex();
}

#define LOG_ERROR(tag, format, ...) Logger::log(Logger::LogLevel::ERROR, tag, format, ##__VA_ARGS__)
#define LOG_WARNING(tag, format, ...) Logger::log(Logger::LogLevel::WARNING, tag, format, ##__VA_ARGS__)
#define LOG_INFO(tag, format, ...) Logger::log(Logger::LogLevel::INFO, tag, format, ##__VA_ARGS__)
#define LOG_DEBUG(tag, format, ...) Logger::log(Logger::LogLevel::DEBUG, tag, format, ##__VA_ARGS__)
#define LOG_TRACE(tag, format, ...) Logger::log(Logger::LogLevel::TRACE, tag, format, ##__VA_ARGS__)