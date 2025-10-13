#include "RocketFSM.hpp"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include <Arduino.h>
#include <pins.h>

// Event queue size
static const size_t EVENT_QUEUE_SIZE = 10;

RocketFSM::RocketFSM(std::shared_ptr<ISensor> imu,
                     std::shared_ptr<ISensor> barometer1,
                     std::shared_ptr<ISensor> barometer2,
                     std::shared_ptr<ISensor> accelerometer,
                     std::shared_ptr<ISensor> gpsModule,
                     std::shared_ptr<SD> sd,
                    std::shared_ptr<RocketLogger> logger)
    : fsmTaskHandle(nullptr), eventQueue(nullptr), stateMutex(nullptr),
      currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE),
      stateStartTime(0), isRunning(false), isTransitioning(false),
      bno055(imu), baro1(barometer1), baro2(barometer2), accl(accelerometer), gps(gpsModule), sd(sd), logger(logger)
{
    LOG_INFO("FSM", "Constructor called");
    LOG_INFO("FSM", "Sensors received: IMU=%s, Baro1=%s, Baro2=%s, GPS=%s, SD=%s",
             bno055 ? "OK" : "NULL",
             baro1 ? "OK" : "NULL",
             baro2 ? "OK" : "NULL",
             gps ? "OK" : "NULL",
             sd ? "OK" : "NULL");

    // Initialize shared data
    sharedData = std::make_shared<SharedSensorData>();

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

    if (sensorDataMutex)
    {
        vSemaphoreDelete(sensorDataMutex);
        sensorDataMutex = nullptr;
    }

    if (loggerMutex)
    {
        vSemaphoreDelete(loggerMutex);
        loggerMutex = nullptr;
    }

    LOG_INFO("RocketFSM", "Destructor completed");
}

void RocketFSM::init()
{
    LOG_INFO("RocketFSM", "Initializing...");

    // Initialize watchdog timer
    esp_task_wdt_init(60000, true); // 60 second timeout

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
    loggerMutex = xSemaphoreCreateMutex();
    if (!loggerMutex)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create logger mutex");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        vSemaphoreDelete(stateMutex);
        stateMutex = nullptr;
        vSemaphoreDelete(sensorDataMutex);
        sensorDataMutex = nullptr;
        return;
    }
    // Initialize managers
    LOG_INFO("RocketFSM", "Initializing TaskManager...");
    taskManager = std::make_unique<TaskManager>(
        sharedData,     // sensorData
        bno055,         // imu
        baro1,          // barometer1
        baro2,          // barometer2
        gps,            // gpsModule
        sensorDataMutex,// sensorMutex
        sd,             // sdCard
        logger,         // logger
        loggerMutex,   // loggerMutex
        isRising,       // barometer Rising flag
        heightGainSpeed, // height gain speed in m/s
        currentHeight   // height which the rocket is at
    );
    LOG_INFO("RocketFSM", "INITIALIZING TASKS...");
    taskManager->initializeTasks();
    LOG_INFO("RocketFSM", "INITIALIZING TRANSITION MANAGER...");
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

    // CRITICAL: Set isRunning BEFORE creating task to prevent race condition
    // where the new task starts before this flag is set
    isRunning = true;

    // Create main FSM task
    BaseType_t result = xTaskCreate(
        fsmTaskWrapper,
        "FSM_Task",
        4096, // Stack size
        this, // Parameter
        2,    // Priority
        &fsmTaskHandle);

    if (result == pdPASS)
    {
        LOG_INFO("RocketFSM", "Started successfully");
    }
    else
    {
        // Task creation failed: revert the flag
        isRunning = false;
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

    // Signal task to stop
    isRunning = false;

    // Stop all managed tasks first
    if (taskManager)
    {
        taskManager->stopAllTasks();
    }

    // Wait cooperatively for main FSM task to finish (up to 3 seconds)
    if (fsmTaskHandle)
    {
        LOG_DEBUG("RocketFSM", "Waiting for FSM task to exit cooperatively...");


        TaskHandle_t localHandle = fsmTaskHandle;
        int maxWaits = 60; // 60 * 50ms = 3000ms
        int waits = 0;


        while (waits < maxWaits)
        {
            eTaskState taskState = eTaskGetState(localHandle);


            // Task has self-deleted or is invalid
            if (taskState == eDeleted || taskState == eInvalid)
            {
                LOG_DEBUG("RocketFSM", "FSM task exited after %d ms", waits * 50);
                break;
            }


            vTaskDelay(pdMS_TO_TICKS(50));
            waits++;
        }


        // Check final state
        eTaskState finalState = eTaskGetState(localHandle);
        if (finalState != eDeleted && finalState != eInvalid)
        {
            // CRITICAL: Do NOT force delete - this causes mutex deadlock
            LOG_ERROR("RocketFSM", "FSM task FAILED to exit after %d ms - SYSTEM MAY BE UNSTABLE", maxWaits * 50);
            LOG_ERROR("RocketFSM", "Task state: %d (0=Running, 1=Ready, 2=Blocked, 3=Suspended, 4=Deleted)", finalState);
            // Leave handle as-is for debugging
            return;
        }


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

    BaseType_t result = xQueueSend(eventQueue, &eventMsg, 0);
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
    LOG_INFO("RocketFSM", "Force transition to %s", getStateString(newState));
    sendEvent(FSMEvent::FORCE_TRANSITION, newState);
}

bool RocketFSM::isFinished()
{
    return getCurrentState() == RocketState::RECOVERED;
}

const char *RocketFSM::getStateString(RocketState state) const
{
    // Use const char* instead of String to avoid heap allocation
    static const char *stateStrings[] = {
        "INACTIVE", "CALIBRATING", "READY_FOR_LAUNCH", "LAUNCH",
        "ACCELERATED_FLIGHT", "BALLISTIC_FLIGHT", "APOGEE",
        "STABILIZATION", "DECELERATION", "LANDING", "RECOVERED"};

    int index = static_cast<int>(state);
    if (index >= 0 && index < 11)
    {
        return stateStrings[index];
    }
    return "UNKNOWN";
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
        #ifdef SIMULATION_DATA
        // This state should be deleted eventually, also commenting out the simulation part, as it would just burn samples from the file
        //.addTask(TaskConfig(TaskType::SIMULATION, "Simulation_1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Launch", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "Started_SD_Logging", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));
    // READY_FOR_LAUNCH state
    stateActions[RocketState::READY_FOR_LAUNCH] = std::make_unique<StateAction>(RocketState::READY_FOR_LAUNCH);
    stateActions[RocketState::READY_FOR_LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering READY_FOR_LAUNCH"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_2", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "Started_SD_Logging_2", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask((TaskConfig(TaskType::TELEMETRY, "Telemetry_Ready", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true)));

    // LAUNCH state
    stateActions[RocketState::LAUNCH] = std::make_unique<StateAction>(RocketState::LAUNCH);
    stateActions[RocketState::LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LAUNCH"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_3", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Launch_3", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Launch", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Launch", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    stateActions[RocketState::ACCELERATED_FLIGHT] = std::make_unique<StateAction>(RocketState::ACCELERATED_FLIGHT);
    stateActions[RocketState::ACCELERATED_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering ACCELERATED_FLIGHT"); })
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Accel", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_4", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Accel_4", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Accel", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    stateActions[RocketState::BALLISTIC_FLIGHT] = std::make_unique<StateAction>(RocketState::BALLISTIC_FLIGHT);
    stateActions[RocketState::BALLISTIC_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering BALLISTIC_FLIGHT"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_5", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Ballistic_5", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Ballistic", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Ballistic", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    stateActions[RocketState::APOGEE] = std::make_unique<StateAction>(RocketState::APOGEE);
    stateActions[RocketState::APOGEE]
        ->setEntryAction([this]()
                         {
                             LOG_INFO("RocketFSM", "Entering APOGEE");
                             digitalWrite(DROGUE_ACTUATOR_PIN, HIGH); // Activate drogue deployment
                             tone(BUZZER_PIN, 1000, 500);             // Sound buzzer at 1kHz for 500ms
                         })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_6", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Apogee", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Apogee_6", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Apogee", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    stateActions[RocketState::STABILIZATION] = std::make_unique<StateAction>(RocketState::STABILIZATION);
    stateActions[RocketState::STABILIZATION]
        ->setExitAction([this]()
                         {
                             LOG_INFO("RocketFSM", "Exiting STABILIZATION");
                             digitalWrite(MAIN_ACTUATOR_PIN, HIGH); // Activate main deployment
                             tone(BUZZER_PIN, 1000, 500);           // Sound buzzer at 1kHz for 500ms
                         })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_7", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Stabilization", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Stabilization_7", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Stabilization", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    stateActions[RocketState::DECELERATION] = std::make_unique<StateAction>(RocketState::DECELERATION);
    stateActions[RocketState::DECELERATION]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering DECELERATION"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_8", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Deceleration", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Deceleration_8", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Deceleration", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    stateActions[RocketState::LANDING] = std::make_unique<StateAction>(RocketState::LANDING);
    stateActions[RocketState::LANDING]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LANDING"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_9", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Landing", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Landing_9", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Landing", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    stateActions[RocketState::RECOVERED] = std::make_unique<StateAction>(RocketState::RECOVERED);
    stateActions[RocketState::RECOVERED]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering RECOVERED"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_10", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_PostFlight_10", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_PostFlight", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
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
    //play buzzer
    tone(BUZZER_PIN, 2000, 100);
    if (isTransitioning)
    {
        LOG_WARNING("RocketFSM", "Already transitioning, ignoring");
        return;
    }

    if (newState == currentState)
    {
        return;
    }

    isTransitioning = true;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        LOG_INFO("RocketFSM", "[TRANSITION] %s -> %s",
                 getStateString(currentState),
                 getStateString(newState));

        // Execute exit action for current state
        if (stateActions[currentState])
        {
            stateActions[currentState]->onExit();
        }

        // Stop current tasks
        if (taskManager)
        {
            try
            {
                taskManager->stopAllTasks();
            }
            catch (const std::exception &e)
            {
                LOG_ERROR("RocketFSM", "[TRANSITION] ERROR: Exception while stopping tasks: %s", e.what());
            }
        }

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
    isTransitioning = false;
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    LOG_INFO("RocketFSM", "Processing event %d in state %s",
             static_cast<int>(eventData.event),
             getStateString(currentState));

    if (eventData.event == FSMEvent::FORCE_TRANSITION)
    {
        LOG_INFO("RocketFSM", "Force transition to %s",
                 getStateString(eventData.targetState));
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
        LOG_WARNING("RocketFSM", "No valid transition for event %d in state %s",
                    static_cast<int>(eventData.event),
                    getStateString(currentState));
    }
}

void RocketFSM::checkTransitions()
{
    // Check for automatic transitions quickly (fast paths first)
    if (transitionManager->checkAutomaticTransitions(currentState, stateStartTime))
    {
        return; // automatic transition handled
    }

    // Cache some persistent values across calls to avoid repeated allocations
    static bool haveAccel = false;
    static float accelZ = 0.0f;
    static unsigned long launchHighSince = 0;
    static unsigned long decelSince = 0;

    // Get accelerometer data before switch statement
    auto accOpt = sharedData->imuData.getData("accelerometer");

    // Fast state-based checks
    switch (currentState)
    {
    case RocketState::INACTIVE:
        // Kick off calibration immediately
        sendEvent(FSMEvent::START_CALIBRATION);
        break;

    case RocketState::CALIBRATING:
        if (millis() - stateStartTime > 5000U)
        {
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        // Calibration timeout fallback
        break;
    case RocketState::READY_FOR_LAUNCH:
        try {
            if (accOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(accOpt.value()))
            {
                const auto &accMap = std::get<std::map<std::string, float>>(accOpt.value());
                auto accVal = accMap.at("magnitude");
            
                if (accVal > LIFTOFF_ACCELERATION_THRESHOLD)
                {
                    if (launchHighSince == 0)
                    {
                        launchHighSince = millis();
                    }
                    else if (millis() - launchHighSince > static_cast<unsigned long>(LIFTOFF_TIMEOUT_MS))
                    {
                        this->launchDetectionTime = millis();
                        sendEvent(FSMEvent::LAUNCH_DETECTED);
                        launchHighSince = 0;
                    }
                }
            } else {
                LOG_INFO("RocketFSM", "READY_FOR_LAUNCH: No accelerometer data available");
            }
        } catch (const std::exception& e) {
            LOG_ERROR("RocketFSM", "READY_FOR_LAUNCH: Exception occurred: %s", e.what());
        }
        
        break;

    case RocketState::LAUNCH:
        // After a short delay consider liftoff started (rocket left the launch pad and is accelerating)
        LOG_INFO("RocketFSM", "Now: %lu, stateStartTime: %lu, LAUNCH: elapsed=%lu ms", millis(), stateStartTime, millis() - stateStartTime);
        sendEvent(FSMEvent::LIFTOFF_STARTED);
        break;

    case RocketState::ACCELERATED_FLIGHT:
        if (millis() - launchDetectionTime >= LAUNCH_TO_BALLISTIC_THRESHOLD)
        {
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }

        //LOG_INFO("ACC_FLIGHT", "Checking condition");
        /*if (accOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(accOpt.value()))
        {
            const auto &accMap = std::get<std::map<std::string, float>>(accOpt.value());
            auto accVal = accMap.at("magnitude");

            LOG_INFO("RocketFSM", "ACCELERATED_FLIGHT: accelZ=%.3f", accVal);
            // Detect sustained deceleration to switch to ballistic
            if (accVal <= 0)
            {
                if (decelSince == 0)
                {
                    decelSince = millis();
                }
                else if (millis() - decelSince > 200U)
                {
                    sendEvent(FSMEvent::ACCELERATION_COMPLETE);
                    decelSince = 0;
                }
            }
            else
            {
                decelSince = 0;
            }
        } */
        break;

    case RocketState::BALLISTIC_FLIGHT:
    {
        //LOG_INFO("RocketFSM", "BALLISTIC_FLIGHT: is rising = %u", *isRising);
        if(!*isRising){
            sendEvent(FSMEvent::APOGEE_REACHED);
        }

        break;
    }

    case RocketState::APOGEE:
        if (millis() - stateStartTime > DROGUE_APOGEE_TIMEOUT)
        {
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;

    case RocketState::STABILIZATION:
        LOG_INFO("RocketFSM", "STABILIZATION: altitude=%.3f", *currentHeight);
        if (*currentHeight < MAIN_ALTITUDE_THRESHOLD)
        {
            LOG_INFO("RocketFSM", "STABILIZATION: condition met (altitude=%.3f, elapsed=%lu ms)", *currentHeight, millis() - stateStartTime);
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
    
        break;

    case RocketState::DECELERATION:
        // In DECELERATION state, vertical velocity in heightGainSpeed will still be tracked, but it should be negative (falling)
        // !!! choose if chenge the control to be with negative values or to invert the value here
        
        if (*currentHeight < TOUCHDOWN_ALTITUDE_THRESHOLD)
        {
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        
        break;

    case RocketState::LANDING:
        if (millis() - stateStartTime > 2000U)
        {
            sendEvent(FSMEvent::LANDING_COMPLETE);
        }
        break;

    case RocketState::RECOVERED:
        // Nothing to do
        break;

    default:
        break;
    }
}

void RocketFSM::fsmTaskWrapper(void *parameter)
{

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->fsmTask();
    }

    vTaskDelete(NULL);
}

void RocketFSM::fsmTask()
{
    LOG_INFO("RocketFSM", "Main task started");

    FSMEventData eventData(FSMEvent::NONE);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long loopCounter = 0;
    esp_task_wdt_add(NULL);

    while (isRunning)
    {
        esp_task_wdt_reset();

        if (isTransitioning)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        // Check for automatic transitions
        checkTransitions();

        // Process events from queue
        if (eventQueue && xQueueReceive(eventQueue, &eventData, pdMS_TO_TICKS(20)) == pdPASS)
        {
            if (!isTransitioning)
            {
                processEvent(eventData);
            }
        }

        loopCounter++;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz
    }
    esp_task_wdt_delete(NULL);

    LOG_INFO("RocketFSM", "Main task ended");
}
