#include "RocketFSM.hpp"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include <Arduino.h>

#define __DEBUG__

// Debug macro that only prints when __DEBUG__ is defined
#ifdef __DEBUG__
#define DEBUG_PRINT(x) Serial.println(x)
#define DEBUG_PRINTF(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(format, ...)
#endif

// Add this utility method to RocketFSM class header
void debugMemory(const char *location)
{
    DEBUG_PRINTF("\n=== MEMORY DEBUG [%s] ===\n", location);
    DEBUG_PRINTF("Free heap: %u bytes\n", ESP.getFreeHeap());
    DEBUG_PRINTF("Min free heap: %u bytes\n", ESP.getMinFreeHeap());
    DEBUG_PRINTF("Max alloc heap: %u bytes\n", ESP.getMaxAllocHeap());
    DEBUG_PRINTF("Largest free block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    DEBUG_PRINTF("==========================\n\n");
}
// Event queue size
static const size_t EVENT_QUEUE_SIZE = 10;

RocketFSM::RocketFSM(std::shared_ptr<ISensor> imu,
                     std::shared_ptr<ISensor> barometer1,
                     std::shared_ptr<ISensor> barometer2,
                     std::shared_ptr<ISensor> gpsModule)
    : fsmTaskHandle(nullptr), eventQueue(nullptr), stateMutex(nullptr),
      currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE),
      stateStartTime(0), isRunning(false), isTransitioning(false),
      bno055(imu), baro1(barometer1), baro2(barometer2), gps(gpsModule)
{
    LOG_INFO("FSM", "Constructor called");
    LOG_INFO("FSM", "Sensors received: IMU=%s, Baro1=%s, Baro2=%s, GPS=%s",
             bno055 ? "OK" : "NULL",
             baro1 ? "OK" : "NULL",
             baro2 ? "OK" : "NULL",
             gps ? "OK" : "NULL");

    // Initialize shared data
    sharedData = std::make_shared<SharedSensorData>();

    // Initialize Kalman filter
    Eigen::Vector3f gravity(0.0f, 0.0f, -9.81f);
    Eigen::Vector3f magnetometer(0.0f, 1.0f, 0.0f);
    kalmanFilter = std::make_shared<KalmanFilter1D>(gravity, magnetometer);

    LOG_INFO("FSM", "Constructor completed");
}

RocketFSM::~RocketFSM()
{
    LOG_INFO("RocketFSM", "Destructor called");
    stop();

    // Clean up FreeRTOS objects
    if (eventQueue)
    {
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
    }

    if (stateMutex)
    {
        vSemaphoreDelete(stateMutex);
        stateMutex = nullptr;
    }

    LOG_INFO("RocketFSM", "Destructor completed");
}

void RocketFSM::init()
{
    LOG_INFO("RocketFSM", "Initializing...");

    // Initialize watchdog timer
    esp_task_wdt_init(10, true); // 10 second timeout

    // Create FreeRTOS objects
    eventQueue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(FSMEventData));
    if (!eventQueue)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create event queue");
        return;
    }

    stateMutex = xSemaphoreCreateMutex();
    if (!stateMutex)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create state mutex");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        return;
    }
    sensorDataMutex = xSemaphoreCreateMutex();
    if (!sensorDataMutex)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create sensor data mutex");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        vSemaphoreDelete(stateMutex);
        stateMutex = nullptr;
        return;
    }
    // Initialize managers
    taskManager = std::make_unique<TaskManager>(sharedData, kalmanFilter, bno055, baro1, baro2, gps, sensorDataMutex);
    taskManager->initializeTasks();

    transitionManager = std::make_unique<TransitionManager>();

    // Setup state machine configuration
    setupStateActions();
    setupTransitions();

    // Initialize state
    currentState = RocketState::INACTIVE;
    previousState = RocketState::INACTIVE;
    stateStartTime = millis();

    LOG_INFO("RocketFSM", "Initialization complete - Free heap: %u bytes", ESP.getFreeHeap());
}

void RocketFSM::start()
{
    LOG_INFO("RocketFSM", "Starting...");

    if (isRunning)
    {
        LOG_WARNING("RocketFSM", "Already running");
        return;
    }

    // Create main FSM task
    BaseType_t result = xTaskCreate(
        fsmTaskWrapper,
        "FSM_Task",
        3072, // Stack size
        this, // Parameter
        2,    // Priority
        &fsmTaskHandle);

    if (result == pdPASS)
    {
        isRunning = true;
        LOG_INFO("RocketFSM", "Started successfully");
    }
    else
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create FSM task");
    }
}

void RocketFSM::stop()
{
    LOG_INFO("RocketFSM", "Stopping...");

    if (!isRunning)
    {
        LOG_WARNING("RocketFSM", "Already stopped");
        return;
    }

    isRunning = false;

    // Stop all tasks
    if (taskManager)
    {
        taskManager->stopAllTasks();
    }

    // Wait for main task to finish
    if (fsmTaskHandle)
    {
        vTaskDelete(fsmTaskHandle);
        fsmTaskHandle = nullptr;
    }

    LOG_INFO("RocketFSM", "Stopped");
}

bool RocketFSM::sendEvent(FSMEvent event, RocketState targetState, void *eventData)
{
    if (!eventQueue)
    {
        LOG_ERROR("RocketFSM", "ERROR: Event queue not initialized");
        return false;
    }

    FSMEventData eventMsg(event, targetState, eventData);

    BaseType_t result = xQueueSend(eventQueue, &eventMsg, pdMS_TO_TICKS(100));
    if (result == pdPASS)
    {
        LOG_INFO("RocketFSM", "Event %d sent successfully", static_cast<int>(event));
        return true;
    }
    else
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to send event %d (queue full?)", static_cast<int>(event));
        return false;
    }
}

RocketState RocketFSM::getCurrentState()
{
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        RocketState state = currentState;
        xSemaphoreGive(stateMutex);
        return state;
    }
    return currentState; // Fallback without mutex
}

FlightPhase RocketFSM::getCurrentPhase()
{
    RocketState state = getCurrentState();

    switch (state)
    {
    case RocketState::INACTIVE:
    case RocketState::CALIBRATING:
    case RocketState::READY_FOR_LAUNCH:
        return FlightPhase::PRE_FLIGHT;

    case RocketState::LAUNCH:
    case RocketState::ACCELERATED_FLIGHT:
    case RocketState::BALLISTIC_FLIGHT:
    case RocketState::APOGEE:
        return FlightPhase::FLIGHT;

    case RocketState::STABILIZATION:
    case RocketState::DECELERATION:
    case RocketState::LANDING:
    case RocketState::RECOVERED:
        return FlightPhase::RECOVERY;

    default:
        return FlightPhase::PRE_FLIGHT;
    }
}

void RocketFSM::forceTransition(RocketState newState)
{
    LOG_INFO("RocketFSM", "Force transition to %s", getStateString(newState).c_str());
    sendEvent(FSMEvent::FORCE_TRANSITION, newState);
}

bool RocketFSM::isFinished()
{
    return getCurrentState() == RocketState::RECOVERED;
}

String RocketFSM::getStateString(RocketState state) const
{
    switch (state)
    {
    case RocketState::INACTIVE:
        return "INACTIVE";
    case RocketState::CALIBRATING:
        return "CALIBRATING";
    case RocketState::READY_FOR_LAUNCH:
        return "READY_FOR_LAUNCH";
    case RocketState::LAUNCH:
        return "LAUNCH";
    case RocketState::ACCELERATED_FLIGHT:
        return "ACCELERATED_FLIGHT";
    case RocketState::BALLISTIC_FLIGHT:
        return "BALLISTIC_FLIGHT";
    case RocketState::APOGEE:
        return "APOGEE";
    case RocketState::STABILIZATION:
        return "STABILIZATION";
    case RocketState::DECELERATION:
        return "DECELERATION";
    case RocketState::LANDING:
        return "LANDING";
    case RocketState::RECOVERED:
        return "RECOVERED";
    default:
        return "UNKNOWN";
    }
}

void RocketFSM::setupStateActions()
{
    LOG_INFO("RocketFSM", "Setting up state actions...");

    // INACTIVE state
    stateActions[RocketState::INACTIVE] = std::make_unique<StateAction>(RocketState::INACTIVE);
    stateActions[RocketState::INACTIVE]->setEntryAction([this]()
                                                        { LOG_INFO("RocketFSM", "Entering INACTIVE"); });

    // CALIBRATING state
    stateActions[RocketState::CALIBRATING] = std::make_unique<StateAction>(RocketState::CALIBRATING);
    stateActions[RocketState::CALIBRATING]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering CALIBRATING"); })
        .setExitAction([this]()
                       { LOG_INFO("RocketFSM", "Exiting CALIBRATING"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));
    // READY_FOR_LAUNCH state
    stateActions[RocketState::READY_FOR_LAUNCH] = std::make_unique<StateAction>(RocketState::READY_FOR_LAUNCH);
    stateActions[RocketState::READY_FOR_LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering READY_FOR_LAUNCH"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Ready", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Ready", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));

    // LAUNCH state
    stateActions[RocketState::LAUNCH] = std::make_unique<StateAction>(RocketState::LAUNCH);
    stateActions[RocketState::LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LAUNCH"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Launch", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Launch", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));
    ;
    stateActions[RocketState::ACCELERATED_FLIGHT] = std::make_unique<StateAction>(RocketState::ACCELERATED_FLIGHT);
    stateActions[RocketState::ACCELERATED_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering ACCELERATED_FLIGHT"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Accel", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Accel", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::EKF, "Ekf_Calib", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true));
    ;
    ;
    // BALLISTIC_FLIGHT state
    stateActions[RocketState::BALLISTIC_FLIGHT] = std::make_unique<StateAction>(RocketState::BALLISTIC_FLIGHT);
    stateActions[RocketState::BALLISTIC_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering BALLISTIC_FLIGHT"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Ballistic", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));

    stateActions[RocketState::APOGEE] = std::make_unique<StateAction>(RocketState::APOGEE);
    stateActions[RocketState::APOGEE]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering APOGEE"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Apogee", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true));

    stateActions[RocketState::STABILIZATION] = std::make_unique<StateAction>(RocketState::STABILIZATION);
    stateActions[RocketState::STABILIZATION]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering STABILIZATION"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Stabilization", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));

    stateActions[RocketState::DECELERATION] = std::make_unique<StateAction>(RocketState::DECELERATION);
    stateActions[RocketState::DECELERATION]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering DECELERATION"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Deceleration", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true));

    stateActions[RocketState::LANDING] = std::make_unique<StateAction>(RocketState::LANDING);
    stateActions[RocketState::LANDING]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LANDING"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Landing", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_0, true));
    stateActions[RocketState::RECOVERED] = std::make_unique<StateAction>(RocketState::RECOVERED);
    stateActions[RocketState::RECOVERED]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering RECOVERED"); })
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_PostFlight", 4096, TaskPriority::TASK_LOW, TaskCore::CORE_1, true));
    LOG_INFO("RocketFSM", "State actions setup complete");
}

void RocketFSM::setupTransitions()
{
    LOG_INFO("RocketFSM", "Setting up transitions...");

    // Basic event-driven transitions
    transitionManager->addTransition(Transition(
        RocketState::INACTIVE,
        RocketState::CALIBRATING,
        FSMEvent::START_CALIBRATION));

    transitionManager->addTransition(Transition(
        RocketState::CALIBRATING,
        RocketState::READY_FOR_LAUNCH,
        FSMEvent::CALIBRATION_COMPLETE));

    transitionManager->addTransition(Transition(
        RocketState::READY_FOR_LAUNCH,
        RocketState::LAUNCH,
        FSMEvent::LAUNCH_DETECTED));

    transitionManager->addTransition(Transition(
        RocketState::LAUNCH,
        RocketState::ACCELERATED_FLIGHT,
        FSMEvent::LIFTOFF_STARTED));

    transitionManager->addTransition(Transition(
        RocketState::ACCELERATED_FLIGHT,
        RocketState::BALLISTIC_FLIGHT,
        FSMEvent::ACCELERATION_COMPLETE));

    transitionManager->addTransition(Transition(
        RocketState::BALLISTIC_FLIGHT,
        RocketState::APOGEE,
        FSMEvent::APOGEE_REACHED));

    transitionManager->addTransition(Transition(
        RocketState::APOGEE,
        RocketState::STABILIZATION,
        FSMEvent::DROGUE_READY));

    transitionManager->addTransition(Transition(
        RocketState::STABILIZATION,
        RocketState::DECELERATION,
        FSMEvent::STABILIZATION_COMPLETE));

    transitionManager->addTransition(Transition(
        RocketState::DECELERATION,
        RocketState::LANDING,
        FSMEvent::DECELERATION_COMPLETE));

    transitionManager->addTransition(Transition(
        RocketState::LANDING,
        RocketState::RECOVERED,
        FSMEvent::LANDING_COMPLETE));

    // Emergency abort transition from any state to INACTIVE
    for (int state = static_cast<int>(RocketState::INACTIVE); state <= static_cast<int>(RocketState::RECOVERED); ++state)
    {
        transitionManager->addTransition(Transition(
            static_cast<RocketState>(state),
            RocketState::INACTIVE,
            FSMEvent::EMERGENCY_ABORT));
    }

    // Add more transitions as needed...

    LOG_INFO("RocketFSM", "Transitions setup complete");
}

void RocketFSM::transitionTo(RocketState newState)
{
    if (newState == currentState)
    {
        return;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        LOG_INFO("RocketFSM", "[TRANSITION] %s -> %s",
                 getStateString(currentState).c_str(),
                 getStateString(newState).c_str());

        // Execute exit action for current state
        if (stateActions[currentState])
        {
            stateActions[currentState]->onExit();
        }

        // Stop current tasks
        taskManager->stopAllTasks();

        // Update state
        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();

        // Execute entry action for new state
        if (stateActions[currentState])
        {
            stateActions[currentState]->onEntry();
        }

        // Start new tasks
        if (stateActions[currentState])
        {
            for (const auto &taskConfig : stateActions[currentState]->getTaskConfigs())
            {
                taskManager->startTask(taskConfig.type, taskConfig);
            }
        }

        xSemaphoreGive(stateMutex);

        LOG_INFO("RocketFSM", "[TRANSITION] Complete - Free heap: %u bytes", ESP.getFreeHeap());
    }
    else
    {
        LOG_ERROR("RocketFSM", "[TRANSITION] ERROR: Failed to acquire state mutex");
    }
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    debugMemory("Before processEvent");
    LOG_INFO("RocketFSM", "Processing event %d in state %s",
             static_cast<int>(eventData.event),
             getStateString(currentState).c_str());

    if (eventData.event == FSMEvent::FORCE_TRANSITION)
    {
        LOG_INFO("RocketFSM", "Force transition to %s",
                 getStateString(eventData.targetState).c_str());
        transitionTo(eventData.targetState);
        return;
    }

    // Find valid transition
    auto newState = transitionManager->findTransition(currentState, eventData.event);
    if (newState.has_value())
    {
        transitionTo(newState.value());
    }
    else
    {
        LOG_ERROR("RocketFSM", "No valid transition for event %d in state %s",
                  static_cast<int>(eventData.event),
                  getStateString(currentState).c_str());
    }
}

void RocketFSM::checkTransitions()
{
    // Check for automatic transitions based on conditions
    if (transitionManager->checkAutomaticTransitions(currentState, stateStartTime))
    {
        // Automatic transition was triggered
        return;
    }

    // State-specific automatic transition logic
    switch (currentState)
    {
    case RocketState::INACTIVE:
        // Automatically start calibration
        sendEvent(FSMEvent::START_CALIBRATION);
        break;

    case RocketState::CALIBRATING:
        // Check for calibration timeout (example: 5 seconds)
        if (millis() - stateStartTime > 5000)
        {
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        break;

    case RocketState::READY_FOR_LAUNCH:
        // Check for launch detection (example: 4 seconds timeout for testing)
        if (millis() - stateStartTime > 4000)
        {
            sendEvent(FSMEvent::LAUNCH_DETECTED);
        }
        break;
    case RocketState::LAUNCH:
        // Check if accelerated flight has begun (leaving launch pad, wait ... ms, as an example we'll wait 1s)
        if (millis() - stateStartTime > 1000)
        {
            sendEvent(FSMEvent::LIFTOFF_STARTED);
        }
        break;
    case RocketState::ACCELERATED_FLIGHT:
        // Check if ballistic flight has begun (acceleration becomes <= 0, for now just a timeout of 6s)
        if (millis() - stateStartTime > 2000)
        {
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }
        break;
    case RocketState::BALLISTIC_FLIGHT:
        // Check if apogee has arrived (vertical velocity <= 0, for now just a timeout of 3s)
        if (millis() - stateStartTime > 3000)
        {
            sendEvent(FSMEvent::APOGEE_REACHED);
        }
        break;
    case RocketState::APOGEE:
        // Delay after drogue chute is opened (wait ... ms, for now 1s for testing)
        if (millis() - stateStartTime > 1000)
        {
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;
    case RocketState::STABILIZATION:
        // Check if altitude is < 450m, for now a timeout of 3s for testing
        if (millis() - stateStartTime > 3000)
        {
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
        break;
    case RocketState::DECELERATION:
        // Check for touchdown, for now a timeout of 5s for testing
        if (millis() - stateStartTime > 5000)
        {
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        break;
    case RocketState::LANDING:
        // Check for user signal, for now a timeout of 3s for testing
        if (millis() - stateStartTime > 3000)
        {
            sendEvent(FSMEvent::LANDING_COMPLETE);
        }
        break;
    case RocketState::RECOVERED:
        break;
        // Add more automatic transition logic as needed...

    default:
        break;
    }
}

void RocketFSM::fsmTaskWrapper(void *parameter)
{
    esp_task_wdt_add(NULL);

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->fsmTask();
    }

    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

void RocketFSM::fsmTask()
{
    LOG_INFO("RocketFSM", "Main task started");

    FSMEventData eventData(FSMEvent::NONE);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long loopCounter = 0;

    while (isRunning)
    {
        esp_task_wdt_reset();

        // Check for automatic transitions
        checkTransitions();

        // Process events from queue
        if (xQueueReceive(eventQueue, &eventData, pdMS_TO_TICKS(50)) == pdPASS)
        {
            processEvent(eventData);
        }

        // Periodic status output
        if (loopCounter % 20 == 0)
        {
            LOG_INFO("RocketFSM", "Loop %lu: State=%s, Time in state=%lu ms, Free heap=%u",
                     loopCounter,
                     getStateString(currentState).c_str(),
                     millis() - stateStartTime,
                     ESP.getFreeHeap());
        }

        loopCounter++;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // 20Hz
    }

    LOG_INFO("RocketFSM", "Main task ended");
}
