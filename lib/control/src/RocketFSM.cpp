#include "RocketFSM.hpp"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include <Arduino.h>

// Event queue size
static const size_t EVENT_QUEUE_SIZE = 10;

RocketFSM::RocketFSM()
    : fsmTaskHandle(nullptr), eventQueue(nullptr), stateMutex(nullptr), currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE), stateStartTime(0), isRunning(false)
{
    Serial.println("[RocketFSM] Constructor called");

    // Initialize shared data
    sharedData = std::make_unique<SharedSensorData>();

    Serial.println("[RocketFSM] Constructor completed");
}

RocketFSM::~RocketFSM()
{
    Serial.println("[RocketFSM] Destructor called");
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

    Serial.println("[RocketFSM] Destructor completed");
}

void RocketFSM::init()
{
    Serial.println("[RocketFSM] Initializing...");

    // Initialize watchdog timer
    esp_task_wdt_init(10, true); // 10 second timeout

    // Create FreeRTOS objects
    eventQueue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(FSMEventData));
    if (!eventQueue)
    {
        Serial.println("[RocketFSM] ERROR: Failed to create event queue");
        return;
    }

    stateMutex = xSemaphoreCreateMutex();
    if (!stateMutex)
    {
        Serial.println("[RocketFSM] ERROR: Failed to create state mutex");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        return;
    }

    // Initialize managers
    taskManager = std::make_unique<TaskManager>(sharedData, &stateMutex);
    taskManager->initializeTasks();

    transitionManager = std::make_unique<TransitionManager>();

    // Setup state machine configuration
    setupStateActions();
    setupTransitions();

    // Initialize state
    currentState = RocketState::INACTIVE;
    previousState = RocketState::INACTIVE;
    stateStartTime = millis();

    Serial.printf("[RocketFSM] Initialization complete - Free heap: %u bytes\n", ESP.getFreeHeap());
}

void RocketFSM::start()
{
    Serial.println("[RocketFSM] Starting...");

    if (isRunning)
    {
        Serial.println("[RocketFSM] Already running");
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
        Serial.println("[RocketFSM] Started successfully");
    }
    else
    {
        Serial.println("[RocketFSM] ERROR: Failed to create FSM task");
    }
}

void RocketFSM::stop()
{
    Serial.println("[RocketFSM] Stopping...");

    if (!isRunning)
    {
        Serial.println("[RocketFSM] Already stopped");
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

    Serial.println("[RocketFSM] Stopped");
}

bool RocketFSM::sendEvent(FSMEvent event, RocketState targetState, void *eventData)
{
    if (!eventQueue)
    {
        Serial.println("[RocketFSM] ERROR: Event queue not initialized");
        return false;
    }

    FSMEventData eventMsg(event, targetState, eventData);

    BaseType_t result = xQueueSend(eventQueue, &eventMsg, pdMS_TO_TICKS(100));
    if (result == pdPASS)
    {
        Serial.printf("[RocketFSM] Event %d sent successfully\n", static_cast<int>(event));
        return true;
    }
    else
    {
        Serial.printf("[RocketFSM] ERROR: Failed to send event %d (queue full?)\n", static_cast<int>(event));
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
    Serial.printf("[RocketFSM] Force transition to %s\n", getStateString(newState).c_str());
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
    Serial.println("[RocketFSM] Setting up state actions...");

    // INACTIVE state
    stateActions[RocketState::INACTIVE] = std::make_unique<StateAction>(RocketState::INACTIVE);
    stateActions[RocketState::INACTIVE]->setEntryAction([this]()
                                                        { Serial.println("[STATE] Entering INACTIVE"); });

    // CALIBRATING state
    stateActions[RocketState::CALIBRATING] = std::make_unique<StateAction>(RocketState::CALIBRATING);
    stateActions[RocketState::CALIBRATING]
        ->setEntryAction([this]()
                         { Serial.println("[STATE] Entering CALIBRATING"); })
        .setExitAction([this]()
                       { Serial.println("[STATE] Exiting CALIBRATING"); })
        .addTask(TaskConfig("Sensor_Calib", 2048, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig("Logging_Calib", 1536, TaskPriority::TASK_LOW, TaskCore::CORE_1, true));

    // READY_FOR_LAUNCH state
    stateActions[RocketState::READY_FOR_LAUNCH] = std::make_unique<StateAction>(RocketState::READY_FOR_LAUNCH);
    stateActions[RocketState::READY_FOR_LAUNCH]
        ->setEntryAction([this]()
                         { Serial.println("[STATE] Entering READY_FOR_LAUNCH"); })
        .addTask(TaskConfig("Sensor_Ready", 2048, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig("Telemetry_Ready", 2048, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        .addTask(TaskConfig("Logging_Ready", 1536, TaskPriority::TASK_LOW, TaskCore::CORE_1, true));

    // LAUNCH state
    stateActions[RocketState::LAUNCH] = std::make_unique<StateAction>(RocketState::LAUNCH);
    stateActions[RocketState::LAUNCH]
        ->setEntryAction([this]()
                         { Serial.println("[STATE] Entering LAUNCH"); })
        .addTask(TaskConfig("Sensor_Launch", 2048, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig("DataCollection_Launch", 2048, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true));

    // BALLISTIC_FLIGHT state
    stateActions[RocketState::BALLISTIC_FLIGHT] = std::make_unique<StateAction>(RocketState::BALLISTIC_FLIGHT);
    stateActions[RocketState::BALLISTIC_FLIGHT]
        ->setEntryAction([this]()
                         { Serial.println("[STATE] Entering BALLISTIC_FLIGHT"); })
        .addTask(TaskConfig("Sensor_Ballistic", 2048, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig("EKF_Ballistic", 2048, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig("ApogeeDetection", 1536, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig("DataCollection_Ballistic", 2048, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    // Add more states as needed...

    Serial.println("[RocketFSM] State actions setup complete");
}

void RocketFSM::setupTransitions()
{
    Serial.println("[RocketFSM] Setting up transitions...");

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

    // Add more transitions as needed...

    Serial.println("[RocketFSM] Transitions setup complete");
}

// Helper function to map task configs to task types
TaskType RocketFSM::mapConfigToTaskType(const TaskConfig &config)
{
    String name = String(config.name);

    if (name.indexOf("Sensor") >= 0)
        return TaskType::SENSOR;
    if (name.indexOf("Telemetry") >= 0)
        return TaskType::TELEMETRY;
    if (name.indexOf("Logging") >= 0)
        return TaskType::LOGGING;
    if (name.indexOf("GPS") >= 0)
        return TaskType::GPS;
    if (name.indexOf("EKF") >= 0)
        return TaskType::EKF;
    if (name.indexOf("ApogeeDetection") >= 0)
        return TaskType::APOGEE_DETECTION;
    if (name.indexOf("Recovery") >= 0)
        return TaskType::RECOVERY;
    if (name.indexOf("DataCollection") >= 0)
        return TaskType::DATA_COLLECTION;

    return TaskType::SENSOR; // Default fallback
}

void RocketFSM::transitionTo(RocketState newState)
{
    if (newState == currentState)
    {
        return;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.printf("\n[TRANSITION] %s -> %s\n",
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
                TaskType taskType = mapConfigToTaskType(taskConfig);
                taskManager->startTask(taskType, taskConfig);
            }
        }

        xSemaphoreGive(stateMutex);

        Serial.printf("[TRANSITION] Complete - Free heap: %u bytes\n", ESP.getFreeHeap());
    }
    else
    {
        Serial.println("[TRANSITION] ERROR: Failed to acquire state mutex");
    }
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    Serial.printf("[RocketFSM] Processing event %d in state %s\n",
                  static_cast<int>(eventData.event),
                  getStateString(currentState).c_str());

    if (eventData.event == FSMEvent::FORCE_TRANSITION)
    {
        Serial.printf("[RocketFSM] Force transition to %s\n",
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
        Serial.printf("[RocketFSM] No valid transition for event %d in state %s\n",
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
    Serial.println("[RocketFSM] Main task started");

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
            Serial.printf("[FSM] Loop %lu: State=%s, Time in state=%lu ms, Free heap=%u\n",
                          loopCounter,
                          getStateString(currentState).c_str(),
                          millis() - stateStartTime,
                          ESP.getFreeHeap());
        }

        loopCounter++;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // 20Hz
    }

    Serial.println("[RocketFSM] Main task ended");
}
