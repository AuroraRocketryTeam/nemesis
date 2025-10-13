// #pragma once

// #include <Arduino.h>
// #include <BNO055Sensor.hpp>
// #include <MPRLSSensor.hpp>
// #include <LIS3DHTRSensor.hpp>
// #include <MS561101BA03.hpp>
// #include <GPS.hpp>
// #include <Termoresistenze.hpp>
// #include <config.h>


// // State machine for actuator sequencing
// enum class ActuatorState {
//     MAIN_ON,
//     MAIN_PAUSE,
//     DROGUE_ON,
//     DROGUE_PAUSE
// };

// class Nemesis {
// public:
//     Nemesis();
//     void init();
//     void run();

//     BNO055Sensor bno;
//     //MPRLSSensor mprls;
//     LIS3DHTRSensor lis3dh;
//     MS561101BA03 ms56_1;
//     MS561101BA03 ms56_2;
//     GPS gps;
//     //Termoresistenze termoresistenze;

//     RocketLogger rocketLogger;

// private:
//     // Timing variables for actuators and buzzer
//     unsigned long actuatorsTimer;
//     unsigned long actuatorsDuration;
//     unsigned long actuatorsDelay;
//     bool toggleStateMainAct;
//     bool toggleStateDrogueAct;
//     ActuatorState currentActuatorState;

//     unsigned long buzzerTimer;
//     unsigned long buzzerDuration;
//     unsigned long buzzerInterval;
//     bool buzzerPlaying;

//     void readSensors();
//     void controlActuators();
//     void controlBuzzer();
//     void readBattery();
//     void logData();
// };