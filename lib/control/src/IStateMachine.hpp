#pragma once

#include "FlightState.hpp"
#include <functional>

/**
 * @brief Interface for a finite state machine managing rocket flight phases
 *
 * This interface defines the contract for a state machine that manages the complete
 * lifecycle of a rocket flight, from initialization through recovery. Implementations
 * should handle state transitions, event processing, and task management in a
 * thread-safe manner suitable for real-time embedded systems.
 *
 * State Machine Behavior:
 * - States represent distinct phases of rocket operation (INACTIVE, CALIBRATING, etc.)
 * - Events trigger state transitions when specific conditions are met
 * - Transitions can be automatic (time-based, sensor-based) or event-driven
 * - Each state may have associated tasks that run concurrently
 *
 * Thread Safety:
 * - All methods must be thread-safe for use in multi-core environments
 * - State transitions should be atomic operations
 * - Event handling should be non-blocking where possible
 *
 * @see RocketState for available states
 * @see FSMEvent for triggerable events
 * @see FlightPhase for high-level flight phases
 */
class IStateMachine
{
public:
    virtual ~IStateMachine() = default;

    /**
     * @brief Initialize the state machine and its components
     *
     * This method should:
     * - Set up internal data structures (event queues, mutexes, etc.)
     * - Initialize the state machine in the INACTIVE state
     * - Configure state actions and transition rules
     * - Prepare task management systems
     * - Allocate necessary memory resources
     *
     * Should be called once before start() and must complete successfully
     * before the state machine can be used.
     *
     * @throws May throw/return error if initialization fails
     * @note Must be called from main thread before starting tasks
     */
    virtual void init() = 0;

    /**
     * @brief Start the state machine and begin processing
     *
     * This method should:
     * - Create and start the main FSM task
     * - Begin event processing loop
     * - Start state-specific tasks based on current state
     * - Enable automatic transition checking
     * - Activate watchdog timers if used
     *
     * The state machine will begin in INACTIVE state and should automatically
     * transition to CALIBRATING if conditions are met.
     *
     * @note Should only be called after successful init()
     * @note May be called multiple times (should handle already-running case)
     */
    virtual void start() = 0;

    /**
     * @brief Stop the state machine and clean up resources
     *
     * This method should:
     * - Signal all tasks to stop execution
     * - Wait for tasks to terminate gracefully
     * - Clean up FreeRTOS objects (tasks, queues, mutexes)
     * - Reset internal state to stopped condition
     * - Preserve current state for potential restart
     *
     * Should be safe to call multiple times and should not block indefinitely.
     * After calling stop(), the state machine can be restarted with start().
     *
     * @note Should handle case where FSM is already stopped
     * @note Should be safe to call from any thread
     */
    virtual void stop() = 0;

    /**
     * @brief Send an event to trigger state transitions
     *
     * Events are the primary mechanism for triggering state changes. This method
     * should:
     * - Queue the event for processing by the main FSM task
     * - Return immediately without blocking (asynchronous)
     * - Handle queue full conditions gracefully
     * - Support both targeted and general events
     *
     * Event Processing:
     * - Events are processed by the main FSM task in FIFO order
     * - Each event is matched against current state transition rules
     * - Valid transitions execute immediately, invalid ones are ignored
     * - FORCE_TRANSITION events bypass normal transition rules
     *
     * @param event The event type to send (START_CALIBRATION, LAUNCH_DETECTED, etc.)
     * @param targetState For FORCE_TRANSITION events, the destination state
     * @param eventData Optional data payload for the event (may be nullptr)
     * @return true if event was queued successfully, false if queue full or invalid
     *
     * @note Thread-safe, can be called from any task
     * @note Non-blocking operation
     */
    virtual bool sendEvent(FSMEvent event, RocketState targetState = RocketState::INACTIVE, void *eventData = nullptr) = 0;

    /**
     * @brief Get the current state of the state machine
     *
     * Returns the current state atomically. Implementation should:
     * - Use appropriate locking to ensure atomic read
     * - Return immediately without blocking
     * - Be safe to call from any thread at any time
     *
     * @return Current RocketState (INACTIVE, CALIBRATING, LAUNCH, etc.)
     * @note Thread-safe getter
     * @note Should never block or fail
     */
    virtual RocketState getCurrentState() = 0;

    /**
     * @brief Get the current high-level flight phase
     *
     * Flight phases group related states for easier high-level logic:
     * - PRE_FLIGHT: INACTIVE, CALIBRATING, READY_FOR_LAUNCH
     * - FLIGHT: LAUNCH, ACCELERATED_FLIGHT, BALLISTIC_FLIGHT, APOGEE
     * - RECOVERY: STABILIZATION, DECELERATION, LANDING, RECOVERED
     *
     * Implementation should map the current state to its corresponding phase.
     *
     * @return Current FlightPhase based on current state
     * @note Thread-safe, derived from getCurrentState()
     */
    virtual FlightPhase getCurrentPhase() = 0;

    /**
     * @brief Force an immediate transition to the specified state
     *
     * This bypasses normal transition rules and should be used with caution.
     * Typical use cases:
     * - Emergency abort sequences
     * - Testing and debugging
     * - Manual override conditions
     * - Recovery from error states
     *
     * Implementation should:
     * - Execute the transition immediately if possible
     * - Handle state cleanup (exit actions, task stopping)
     * - Start new state (entry actions, task starting)
     * - Log the forced transition for debugging
     * - Be thread-safe
     *
     * @param newState The state to transition to immediately
     * @warning Bypasses normal safety checks - use carefully
     * @note Should be used sparingly, primarily for emergency/test scenarios
     */
    virtual void forceTransition(RocketState newState) = 0;

    /**
     * @brief Check if the state machine has completed its mission
     *
     * Returns true when the rocket has reached a terminal state (usually RECOVERED)
     * and no further state transitions are expected. Implementation should:
     * - Return true for terminal states (RECOVERED, or error states)
     * - Return false for all active states
     * - Consider mission abort conditions
     *
     * This can be used by higher-level systems to determine when mission
     * operations are complete.
     *
     * @return true if mission is complete, false if still active
     * @note Useful for determining when to shut down systems
     */
    virtual bool isFinished() = 0;
};