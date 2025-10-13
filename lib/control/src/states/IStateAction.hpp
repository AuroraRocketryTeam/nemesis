#pragma once

#include "FlightState.hpp"
#include <functional>

/**
 * @brief Interface for state-specific actions and behaviors
 *
 * This interface defines the contract for objects that handle state-specific
 * logic including entry/exit actions and periodic updates during state execution.
 */
class IStateAction
{
public:
    virtual ~IStateAction() = default;

    /**
     * @brief Called when entering this state
     *
     * Use this to:
     * - Initialize state-specific resources
     * - Start state-specific tasks
     * - Log state entry
     * - Set up hardware configurations
     */
    virtual void onEntry() {}

    /**
     * @brief Called when exiting this state
     *
     * Use this to:
     * - Clean up state-specific resources
     * - Stop state-specific tasks
     * - Log state exit
     * - Save state data
     */
    virtual void onExit() {}

    /**
     * @brief Called periodically while in this state
     *
     * Use this for:
     * - Periodic state-specific processing
     * - Condition monitoring
     * - Data updates
     */
    virtual void onUpdate() {}

    /**
     * @brief Get the state this action handles
     * @return The RocketState this action is associated with
     */
    virtual RocketState getState() const = 0;
};

/**
 * @brief Interface for transition condition evaluation
 *
 * This interface defines conditions that can trigger automatic state transitions.
 * Implementations should evaluate specific conditions (time-based, sensor-based, etc.)
 * and return whether the transition should occur.
 */
class ITransitionCondition
{
public:
    virtual ~ITransitionCondition() = default;

    /**
     * @brief Evaluate whether the transition condition is met
     *
     * This method should:
     * - Check relevant sensors, timers, or other data
     * - Return true if transition should occur
     * - Be lightweight and non-blocking
     * - Be thread-safe
     *
     * @return true if condition is satisfied, false otherwise
     */
    virtual bool isConditionMet() = 0;

    /**
     * @brief Get a human-readable name for this condition
     *
     * Used for logging and debugging purposes.
     *
     * @return C-string describing this condition (e.g., "Calibration Timeout")
     */
    virtual const char *getConditionName() const = 0;
};