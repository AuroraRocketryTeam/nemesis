#pragma once

#include "IStateMachine.hpp"
#include "tasks/TaskManager.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>

class RocketFSM : public IStateMachine
{
private:
    // Core FSM components
    std::unique_ptr<TaskManager> taskManager;
    std::unique_ptr<TransitionManager> transitionManager;
    std::map<RocketState, std::unique_ptr<StateAction>> stateActions;

    // FreeRTOS components
    TaskHandle_t fsmTaskHandle;
    QueueHandle_t eventQueue;
    SemaphoreHandle_t stateMutex;

    // State management
    RocketState currentState;
    RocketState previousState;
    unsigned long stateStartTime;
    volatile bool isRunning;

    // Shared data
    std::shared_ptr<SharedSensorData> sharedData;

public:
    RocketFSM();
    ~RocketFSM();

    // IStateMachine interface
    void init() override;
    void start() override;
    void stop() override;
    bool sendEvent(FSMEvent event, RocketState targetState = RocketState::INACTIVE, void *eventData = nullptr) override;
    RocketState getCurrentState() override;
    FlightPhase getCurrentPhase() override;
    void forceTransition(RocketState newState) override;
    bool isFinished() override;

    // Utility methods
    String getStateString(RocketState state) const;

private:
    void setupStateActions();
    void setupTransitions();
    void transitionTo(RocketState newState);
    void processEvent(const FSMEventData &eventData);
    void checkTransitions();

    // FreeRTOS task function
    static void fsmTaskWrapper(void *parameter);
    void fsmTask();
};