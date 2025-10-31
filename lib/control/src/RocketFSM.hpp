#pragma once

#include "IStateMachine.hpp"
#include "tasks/TaskManager.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"
#include <Logger.hpp>
#include "RocketLogger.hpp"
#include "config.h"
#include <SD-master.hpp>
#include <memory>
#include <map>
#include <Nemesis.hpp>

/**
 * @brief Enumeration for the different rocket states.
 * 
 */
class RocketFSM : public IStateMachine
{
public:
    /**
     * @brief Construct a new Rocket FSM object
     * 
     * @param model The shared pointer to the rocket model
     * @param sd The shared pointer to the SD card
     * @param logger The shared pointer to the RocketLogger instance
     */
    RocketFSM(std::shared_ptr<Nemesis> model,
              std::shared_ptr<SD> sd,
              std::shared_ptr<RocketLogger> logger
            );
    
    /**
     * @brief Destroy the Rocket FSM object
     * 
     */
    ~RocketFSM();

    // IStateMachine interface
    void init() override;
    void start() override;
    void stop() override;

    /**
     * @brief Send an event to the FSM
     * 
     * @param event The event to send
     * @param targetState The target state to transition to
     * @param eventData Optional data associated with the event
     * @return true if the event was processed successfully, false otherwise
     */
    bool sendEvent(FSMEvent event, RocketState targetState = RocketState::INACTIVE, void *eventData = nullptr) override;

    /**
     * @brief Get the current state of the FSM
     * 
     * @return RocketState The current state
     */
    RocketState getCurrentState() override;

    /**
     * @brief Get the current flight phase of the FSM
     * 
     * @return FlightPhase The current flight phase
     */
    FlightPhase getCurrentPhase() override;

    /**
     * @brief Force a transition to a new state
     * 
     * @param newState The new state to transition to
     */
    void forceTransition(RocketState newState) override;

    /**
     * @brief Check if the FSM has finished its current task
     * 
     * @return true if the FSM is finished, false otherwise
     */
    bool isFinished() override;

    // Utility methods
    const char* getStateString(RocketState state) const;

private:
    void setupStateActions();
    void setupTransitions();
    void transitionTo(RocketState newState);
    void processEvent(const FSMEventData &eventData);
    void checkTransitions();

    // FreeRTOS task function
    static void fsmTaskWrapper(void *parameter);
    void fsmTask();

    // Core FSM components
    std::unique_ptr<TaskManager> _taskManager;
    std::unique_ptr<TransitionManager> _transitionManager;
    std::map<RocketState, std::unique_ptr<StateAction>> _stateActions;

    // FreeRTOS components
    TaskHandle_t _fsmTaskHandle;
    QueueHandle_t _eventQueue;
    SemaphoreHandle_t _stateMutex;

    // State management
    RocketState _currentState;
    RocketState _previousState;
    unsigned long _stateStartTime;
    volatile bool _isRunning;
    volatile bool _isTransitioning;

    // Shared data
    std::shared_ptr<Nemesis> _model;
    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _modelMutex;
    SemaphoreHandle_t _loggerMutex;

    std::shared_ptr<SD> _sd;

    // Important timers and tresholds
    const unsigned long LAUNCH_TO_BALLISTIC_THRESHOLD = 6000;
    const unsigned long LAUNCH_TO_APOGEE_THRESHOLD = 27000; //24850 + 2150 = 27000
    unsigned long _launchDetectionTime = 0;
};