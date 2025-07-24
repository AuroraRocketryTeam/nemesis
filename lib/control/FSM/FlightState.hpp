#pragma once
#include <functional>
/**
 * @brief Enum representing all possible rocket states
 */
enum class RocketState
{
    // Initial state
    INACTIVE,

    // Pre-flight phase
    CALIBRATING,
    READY_FOR_LAUNCH,

    // Flight phase
    LAUNCH,
    ACCELERATED_FLIGHT,
    BALLISTIC_FLIGHT,
    APOGEE,

    // Recovery phase
    STABILIZATION,
    DECELERATION,
    LANDING,
    RECOVERED
};

/**
 * @brief Enum representing flight phases for better organization
 */
enum class FlightPhase
{
    PRE_FLIGHT,
    FLIGHT,
    RECOVERY
};
