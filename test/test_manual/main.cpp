#include <Arduino.h>
#include <Wire.h>
#include "sensors/ISensor.hpp"
#include "sensors/MPRLS/MPRLSSensor.hpp"
#include "MPLRS/MPRLSPrinter.hpp"
#include <global/config.h>
#include <global/pins.h>

/**
 * To use this file change the extension from .disabled to .cpp and change the
 * extension of src/main.cpp to .disabled. After finished put everything back as
 * it was before.
 */

ISensor* mprls;
MPRLSPrinter* mprlsPrinter;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        delay(100); // Wait for the serial port to connect.
    }

    Serial.println(F("Test MPRLS sensor con ESP32"));
    // I2C initialization with defined pins.
    Wire.begin();

    mprls = new MPRLSSensor();

    if (!mprls->init()) {
        Serial.println(F("Could not initialize MPRLS sensor"));
        exit(1);
    }

    mprlsPrinter = new MPRLSPrinter(mprls);
    delay(1000);
}

void loop() {
    mprlsPrinter->displayPressure();
    delay(1000);
}

