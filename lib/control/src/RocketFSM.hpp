#pragma once

#include "IStateMachine.hpp"
#include "tasks/TaskManager.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"
#include <SharedData.hpp>
#include <Logger.hpp>
#include "RocketLogger.hpp"
#include "config.h"
#include <SD-master.hpp>
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
    std::shared_ptr<RocketLogger> logger;
    SemaphoreHandle_t sensorDataMutex;
    SemaphoreHandle_t loggerMutex;
    std::shared_ptr<ISensor> bno055;
    std::shared_ptr<ISensor> baro1;
    std::shared_ptr<ISensor> baro2;
    std::shared_ptr<ISensor> accl;
    std::shared_ptr<ISensor> gps;
    // Rising Flag
    std::shared_ptr<bool> isRising=std::make_shared<bool>(true);
    std::shared_ptr<float> heightGainSpeed=std::make_shared<float>(0.0f);
    std::shared_ptr<float> currentHeight=std::make_shared<float>(0.0f);

    std::shared_ptr<SD> sd;

    // Important timers and tresholds
    const unsigned long LAUNCH_TO_BALLISTIC_THRESHOLD = 7000; //5300;
    const unsigned long LAUNCH_TO_APOGEE_THRESHOLD = 27000; //24850 + 500 = 25350
    unsigned long launchDetectionTime = 0;


public:
    RocketFSM(std::shared_ptr<ISensor> imu,
              std::shared_ptr<ISensor> barometer1,
              std::shared_ptr<ISensor> barometer2,
              std::shared_ptr<ISensor> accelerometer,
              std::shared_ptr<ISensor> gpsModule,
              std::shared_ptr<SD> sd,
              std::shared_ptr<RocketLogger> logger
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
};