#pragma once

#include "IStateAction.hpp"
#include "FlightState.hpp"
#include <vector>
#include <memory>
/**
 * @brief Represents a state transition rule in the finite state machine
 *
 * A Transition defines how the state machine can move from one state to another.
 * It encapsulates the source state, destination state, triggering event, and
 * optional conditions that must be met for the transition to occur.
 *
 * Transition Types:
 * - Event-driven: Triggered by explicit FSMEvent (e.g., LAUNCH_DETECTED)
 * - Automatic: Triggered by conditions (e.g., timeout, sensor thresholds)
 * - Hybrid: Event-driven with additional condition checks
 */
struct Transition
{
    RocketState fromState;                           // Source state for this transition
    RocketState toState;                             // Destination state for this transition
    FSMEvent triggerEvent;                           // Event that triggers this transition
    std::shared_ptr<ITransitionCondition> condition; // Optional condition for this transition to occur (nullptr if none)

    /**
     * @brief Constructor for Transition
     *
     * @param from Source state
     * @param to Destination state
     * @param event Triggering event
     * @param cond Optional condition (nullptr if none)
     */
    Transition(RocketState from, RocketState to, FSMEvent event, std::shared_ptr<ITransitionCondition> cond = nullptr)
        : fromState(from), toState(to), triggerEvent(event), condition(cond) {}
};

/**
 * @brief Manages state transitions and transition rules for the rocket state machine
 *
 * The TransitionManager is responsible for:
 * - Storing and organizing all possible state transitions
 * - Finding valid transitions based on current state and events
 * - Evaluating automatic transition conditions
 * - Providing a centralized location for transition logic
 *
 * This class separates transition logic from the main state machine implementation,
 * making it easier to modify transition rules without affecting other components.
 *
 * @note Transitions are evaluated in the order they were added
 * @see Transition for individual transition rules
 * @see ITransitionCondition for automatic transition conditions
 */
class TransitionManager
{
private:
    std::vector<Transition> transitions; // Collection of all defined transitions

public:
    /** @brief Adds a new transition to the manager
     * Transitions are stored in the order they are added. When multiple transitions could apply,
     * the first matching transition in the list will be used.
     *
     * @param transition The Transition object to add
     * @note Duplicated transitions are allowed but may lead to ambiguous and unexpected behavior
     */
    void addTransition(const Transition &transition);
    /**
     * @brief Find a valid transition based on current state and event
     *
     * Searched through all defined transitions to find one that matches:
     *
     * Current state == fromState;
     * Event == triggerEvent;
     * Condition (if any) is met
     *
     * @param currentState the current state of the FSM
     * @param event the event that was received
     * @return std::optional<RocketState> The next state if a valid transition is found, otherwise std::nullopt
     *
     * @note If multiple transitions match, the first one found is returned
     */
    std::optional<RocketState> findTransition(RocketState currentState, FSMEvent event);

    /** @brief Checks for automatic transitions based on elapsed time
     *
     * Evaluates all transitions from the current state that have conditions
     * defined, regardless of their trigger event. This enables automatic
     * transitions based on time, sensor data, or other criteria.
     * 
     * Typical automatic conditions are: 
     * 
     * Timeouts, Sensor thresholds (e.g. velocity or altitude for apogee detection), System status (e.g. battery level, calibration status)
     * 
     * @param currentState the current state of the FSM
     * @param stateStartTime the time when the current state was entered
     * @return true if an automatic condition was met and should be processed, false otherwise.
     */
    bool checkAutomaticTransitions(RocketState currentState, unsigned long stateStartTime);
};