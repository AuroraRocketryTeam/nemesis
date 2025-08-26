// #include <Arduino.h>
// #include <ILogger.hpp>
// #include <ITransmitter.hpp>
// #include <ISensor.hpp>
// #include <BNO055Sensor.hpp>
// #include <MS561101BA03.hpp>
// #include <GPS.hpp>
// #include <RocketFSM.hpp>
// #include <RocketLogger.hpp>
// #include <E220LoRaTransmitter.hpp>
// #include <HardwareSerial.h>
// #include <config.h>
// #include <pins.h>
// #include <KalmanFilter.hpp>

// // using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// ISensor *bno055;
// ISensor *baro1;
// ISensor *baro2;
// ISensor *gps;
// ILogger *rocketLogger;
// ITransmitter<TransmitDataType> *loraTransmitter;
// HardwareSerial loraSerial(LORA_SERIAL);
// RocketFSM rocketFSM;
// KalmanFilter *ekf;

// void setup()
// {
//     Serial.begin(115200);

//     // Initialize your sensors, logger, etc.
//     //loraSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, LORA_RX, LORA_TX);
//     bno055 = new BNO055Sensor();
//     baro1 = new MS561101BA03(); // Example
//     baro2 = new MS561101BA03(); // Example
//     gps = new GPS();
//     rocketLogger = new RocketLogger();
//     //loraTransmitter = new E220LoRaTransmitter(Serial2, 17); // Example
//     //loraTransmitter->init();

//     // Initialize and start FSM
//     rocketFSM.init();
//     rocketFSM.start();

//     Serial.println("System initialized with FreeRTOS FSM");
// }

// void loop()
// {
//     // Main loop can be minimal since FSM runs in its own tasks
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     // Optional: Monitor FSM state
//     static RocketState lastState = RocketState::INACTIVE;
//     RocketState currentState = rocketFSM.getCurrentState();

//     if (currentState != lastState)
//     {
//         Serial.println("State: " + rocketFSM.getStateString(currentState));
//         lastState = currentState;
//     }
// }