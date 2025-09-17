#include "TransitionManager.hpp"
#include <Arduino.h>
void TransitionManager::addTransition(const Transition &transition)
{
    transitions.push_back(transition);
}

std::optional<RocketState> TransitionManager::findTransition(RocketState currentState, FSMEvent event)
{
    // Iterate through all transitions to find a match
    for (const auto &transition : transitions)
    {
        // Check if the transition matches current state and event
        if (transition.fromState == currentState && transition.triggerEvent == event)
        {
            // If there's a condition, check if it's met
            if (transition.condition != nullptr)
            {
                if (transition.condition->isConditionMet())
                {
                    return transition.toState;
                }
            }
            else
            {
                // No condition required, transition is valid
                return transition.toState;
            }
        }
    }

    // No valid transition found
    return std::nullopt;
}

bool TransitionManager::checkAutomaticTransitions(RocketState currentState, unsigned long stateStartTime)
{
    // Calculate elapsed time in current state
    unsigned long elapsedTime = millis() - stateStartTime;

    // Check all transitions from the current state that have conditions
    for (const auto &transition : transitions)
    {
        if (transition.fromState == currentState && transition.condition != nullptr)
        {
            // Check if the automatic condition is met
            if (transition.condition->isConditionMet())
            {
                return true;
            }
        }
    }

    return false;
}