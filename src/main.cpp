#include <Arduino.h>
#include <Wire.h>
#include "BNO055/BNO055Printer.hpp"
#include "sensors/ISensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include <global/config.h>
#include <global/pins.h>

/**
 * To use this file change the extension from .disabled to .cpp and change the
 * extension of src/main.cpp to .disabled. After finished put everything back as
 * it was before.
 */

ISensor* bno;
BNO055Printer* bnoPrinter;

void setup(void) {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        delay(100); // Wait for the serial port to connect.
    }
    
    Serial.println(F("Test BNO055 sensor con ESP32"));
    // I2C initialization with defined pins.
    Wire.begin();
    
    bno = new BNO055Sensor();
    if (!bno->init()) {
        Serial.println(F("Could not initialize bno"));
        exit(1);
    }
    bnoPrinter = new BNO055Printer(bno);
    delay(1000);
}
  
void loop(void) {
    // bnoPrinter->displayCalibrationStatus();

    // bnoPrinter->displayOrientation();

    // bnoPrinter->displayAccelleration();

    // bnoPrinter->displayGyroscope();

    bnoPrinter->displayMagnetometer();

    delay(100);
}
