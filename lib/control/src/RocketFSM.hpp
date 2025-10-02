#pragma once

#include "IStateMachine.hpp"
#include "tasks/TaskManager.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"
#include "SharedData.hpp"
#include "KalmanFilter1D.hpp"
#include "Logger.hpp"
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
    volatile bool isTransitioning;

    // Shared data
    std::shared_ptr<SharedSensorData> sharedData;
    std::shared_ptr<KalmanFilter1D> kalmanFilter;
    SemaphoreHandle_t sensorDataMutex;
    std::shared_ptr<ISensor> bno055;
    std::shared_ptr<ISensor> baro1;
    std::shared_ptr<ISensor> baro2;
    std::shared_ptr<ISensor> accl;
    std::shared_ptr<ISensor> gps;

public:
    RocketFSM(std::shared_ptr<ISensor> imu,
              std::shared_ptr<ISensor> barometer1,
              std::shared_ptr<ISensor> barometer2,
              std::shared_ptr<ISensor> accelerometer,
              std::shared_ptr<ISensor> gpsModule,
              std::shared_ptr<KalmanFilter1D> kf
            );
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